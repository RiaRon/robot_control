"""Descriptions and state ownership for the split OpenArm + LEAP Hand bringup (fake only).

Same layout as the OpenArm + RH56F1 split bringup (rh56f1_split_description):
one controller manager per hardware owner,

    arms        /controller_manager               14 arm joints
    right hand  /leap_right/controller_manager    16 right-hand joints
    left hand   /leap_left/controller_manager     16 left-hand joints

and every description handed to a manager is the whole integrated model plus only
the ros2_control block that manager owns. The integrated model for the one
robot_state_publisher has no ros2_control block.

The model is the canonical RL asset of the urdf repo
(``generated/rl/openarm_leap_bi_rl.urdf``, branch feat/leap-hand), so the ROS
model and the Isaac asset are one file. Nothing here edits a joint origin, axis,
limit or inertial; only mesh URIs are re-rooted at this checkout. Joint names are
canonical (``r_aj_1``, ``r_hj_index_1``); every LEAP joint is actuated (no mimic),
so each hand owns all 16. There is no real hand backend yet.
"""

from __future__ import annotations

import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from rh56f1_description import default_manifest_for
from rh56f1_split_description import (
    FAKE_PLUGIN,
    controller_manager,
    prime_fake_trajectory_controllers,
    ros2_control_blocks,
)

SIDES = ("right", "left")
ARM_NAMESPACE = ""
HAND_NAMESPACE = {"right": "leap_right", "left": "leap_left"}
ARM_STATE_TOPIC = "/openarm/joint_states"
HAND_STATE_TOPIC = {side: f"/{ns}/joint_states" for side, ns in HAND_NAMESPACE.items()}
MERGED_STATE_TOPIC = "/joint_states"
DISPLAY_STATE_TOPIC = "/openarm_leap/display_joint_states"
SOURCE_STATUS_TOPIC = "/openarm_leap/joint_state_sources"
#: Jazzy controller_manager reads its description from a topic only. The model's
#: robot_state_publisher owns /robot_description (no ros2_control), so the arms
#: manager listens here instead; each hand manager uses /<namespace>/robot_description.
ARM_DESCRIPTION_TOPIC = "/openarm_leap/arms/robot_description"
ARM_CONTROLLERS = {"right": "right_joint_trajectory_controller",
                   "left": "left_joint_trajectory_controller"}
FAKE_HAND_CONTROLLERS = {"right": "right_hand_trajectory_controller",
                         "left": "left_hand_trajectory_controller"}
CANONICAL_RELATIVE = Path("urdf/generated/rl/openarm_leap_bi_rl.urdf")
_PREFIX = {"right": "r", "left": "l"}
ARM_JOINTS, HAND_JOINTS = 14, 16
_TRAJECTORY_PARAMS = {
    "command_interfaces": ["position"],
    "state_interfaces": ["position"],
    "state_publish_rate": 50.0,
    "action_monitor_rate": 20.0,
    "allow_partial_joints_goal": False,
}


def default_canonical_urdf(launch_file: str) -> str:
    """The canonical LEAP asset, found relative to *launch_file*: the first parent
    directory holding ``urdf/generated/rl/openarm_leap_bi_rl.urdf`` (kuku_lab, from
    robot_control's source tree or its ros_ws/install). $KUKU_LAB_ROOT overrides it.
    """
    import os

    roots = [Path(os.environ["KUKU_LAB_ROOT"])] if os.environ.get("KUKU_LAB_ROOT") else []
    roots += Path(launch_file).resolve().parents
    return next((str(r / CANONICAL_RELATIVE) for r in roots if (r / CANONICAL_RELATIVE).is_file()), "")


def device_joints(manifest: Path) -> dict[str, list[str]]:
    """Canonical joints each device owns, in manifest ``control_joint_order``."""
    order = yaml.safe_load(Path(manifest).read_text()).get("control_joint_order")
    if not isinstance(order, list) or not order:
        raise ValueError(f"{manifest}: no control_joint_order")
    owned = {
        "arms": [n for n in order if n.startswith(("r_aj_", "l_aj_"))],
        "right": [n for n in order if n.startswith("r_hj_")],
        "left": [n for n in order if n.startswith("l_hj_")],
    }
    if len(owned["arms"]) != ARM_JOINTS or any(len(owned[s]) != HAND_JOINTS for s in SIDES):
        raise ValueError(f"{manifest}: expected {ARM_JOINTS} arm and 2x{HAND_JOINTS} hand "
                         "joints in control_joint_order")
    return owned


def canonical_model(source_urdf: Path) -> ET.Element:
    """The canonical model with its mesh URIs re-rooted at this checkout.

    The generator writes absolute ``file://`` URIs of the machine it ran on; each is
    replaced by the longest tail of its path that exists under this urdf repo.
    """
    source_urdf = Path(source_urdf).resolve()
    if not source_urdf.is_file():
        raise FileNotFoundError(f"canonical LEAP URDF not found: {source_urdf}")
    repo = source_urdf.parents[2]  # <urdf repo>/generated/rl/<asset>.urdf
    root = ET.parse(source_urdf).getroot()
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename") or ""
        if not uri.startswith("file://"):
            continue
        parts = Path(uri[len("file://"):]).parts
        local = next((repo.joinpath(*parts[i:]) for i in range(1, len(parts))
                      if repo.joinpath(*parts[i:]).is_file()), None)
        if local is not None:
            mesh.set("filename", local.as_uri())
    return root


def _to_text(root: ET.Element) -> str:
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def model_description(source_urdf: Path) -> str:
    """The integrated model for the one robot_state_publisher: no ros2_control."""
    root = canonical_model(source_urdf)
    root.set("name", "openarm_leap_fake_model")
    for block in root.findall("ros2_control"):
        root.remove(block)
    return _to_text(root)


def _fake_description(source_urdf: Path, name: str, joints: list[str]) -> str:
    root = canonical_model(source_urdf)
    root.set("name", name)
    block = ET.SubElement(root, "ros2_control", {"name": name, "type": "system"})
    hardware = ET.SubElement(block, "hardware")
    ET.SubElement(hardware, "plugin").text = FAKE_PLUGIN
    ET.SubElement(hardware, "param", {"name": "fake_sensor_commands"}).text = "false"
    ET.SubElement(hardware, "param", {"name": "state_following_offset"}).text = "0.0"
    for joint_name in joints:
        joint = ET.SubElement(block, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        # Fake-only start pose; 0 is inside every arm and LEAP joint limit. Not a physical pose.
        ET.SubElement(state, "param", {"name": "initial_value"}).text = "0.0"
    return _to_text(root)


def fake_arm_description(source_urdf: Path, manifest: Path) -> str:
    """Integrated model + the 14 arm joints on GenericSystem, no hand resource."""
    return _fake_description(source_urdf, "openarm_leap_arms_fake", device_joints(manifest)["arms"])


def fake_hand_description(side: str, source_urdf: Path, manifest: Path) -> str:
    """Integrated model + one hand's 16 joints on GenericSystem, nothing else."""
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")
    return _fake_description(source_urdf, f"leap_{side}_hand_fake", device_joints(manifest)[side])


def _controllers(description: str, controllers: dict[str, list[str]]) -> dict:
    """Wildcarded params file: joint_state_broadcaster + one trajectory controller per group.

    Each controller's joints must be exactly resources of *description*.
    """
    resources = [j for block in ros2_control_blocks(description).values() for j in block["joints"]]
    manager = {"update_rate": 100,
               "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"}}
    params = {"/**/controller_manager": {"ros__parameters": manager}}
    for name, joints in controllers.items():
        if not joints or any(j not in resources for j in joints):
            raise ValueError(f"{name}: joints {joints} are not resources of the description")
        manager[name] = {"type": "joint_trajectory_controller/JointTrajectoryController"}
        params[f"/**/{name}"] = {"ros__parameters": {"joints": list(joints), **copy.deepcopy(_TRAJECTORY_PARAMS)}}
    return params


def fake_arm_controllers(manifest: Path, description: str) -> dict:
    arms = device_joints(manifest)["arms"]
    return _controllers(description, {
        ARM_CONTROLLERS[side]: [n for n in arms if n.startswith(f"{_PREFIX[side]}_aj_")]
        for side in ("left", "right")})


def fake_hand_controllers(side: str, manifest: Path, description: str) -> dict:
    return _controllers(description, {FAKE_HAND_CONTROLLERS[side]: device_joints(manifest)[side]})


#: The head's pan/tilt joints are revolute but no device drives or reports them; without a
#: state robot_state_publisher publishes no TF below head_base (RViz RobotModel error). They
#: get the merger's marked display placeholder (0) from a topic nobody publishes.
HEAD_STATE_TOPIC = "/openarm_leap/head/joint_states"


def undriven_joints(manifest: Path) -> list[str]:
    """Movable joints no device owns (the head's pan/tilt), from the manifest."""
    data = yaml.safe_load(Path(manifest).read_text())
    owned = set(data["control_joint_order"]) | set(data.get("fixed_joint_order") or [])
    return [n for n in data["kinematic_joint_order"] if n not in owned]


def state_sources(manifest: Path) -> list[dict]:
    """The merger's inputs: which topic owns which joints."""
    owned = device_joints(manifest)
    sources = [{"id": "openarm", "topic": ARM_STATE_TOPIC, "joints": owned["arms"],
                "display_placeholder": False}]
    for side in SIDES:
        # Fingers may be drawn at 0 while this hand has no fresh state; marked, never measured.
        sources.append({"id": HAND_NAMESPACE[side], "topic": HAND_STATE_TOPIC[side],
                        "joints": owned[side], "display_placeholder": True})
    head = undriven_joints(manifest)
    if head:
        sources.append({"id": "head", "topic": HEAD_STATE_TOPIC, "joints": head,
                        "display_placeholder": True})
    return sources


def description_nodes(description: str, namespace: str, topic: str, manager_node: dict):
    """The controller manager (``manager_node``: Node kwargs) fed by a latched description.

    Jazzy's ros2_control_node takes no robot_description parameter: both nodes are
    remapped onto *topic*, where robot_description_publisher.py latches *description*.
    """
    from launch_ros.actions import Node

    remap = [("robot_description", topic)]
    publisher = Node(package="openarm_bringup", executable="robot_description_publisher.py",
                     namespace=namespace, output="both",
                     parameters=[{"robot_description": description}], remappings=remap)
    manager = Node(package="controller_manager", executable="ros2_control_node",
                   namespace=namespace, output="both",
                   **{**manager_node, "remappings": remap + manager_node.get("remappings", [])})
    return [publisher, manager]
