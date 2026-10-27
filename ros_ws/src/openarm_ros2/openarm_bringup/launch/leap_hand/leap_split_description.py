"""Descriptions and state ownership for the split OpenArm + LEAP Hand bringup.

Same layout as the OpenArm + RH56F1 split bringup (../rh56f1):
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
so each hand owns all 16. Real hands: the ROBOTIS Dynamixel plugin (real_hand_description).
"""

from __future__ import annotations

import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # split_common

from split_common import (
    FAKE_PLUGIN,
    controller_manager,
    default_manifest_for,
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


RUNTIMES = ("fake", "real")


def arm_source_names(manifest: Path) -> dict[str, str]:
    """canonical arm joint -> OpenArmHW name (``r_aj_1`` -> ``openarm_right_joint1``)."""
    mapping = yaml.safe_load(Path(manifest).read_text()).get("source_to_canonical_joints") or {}
    arms = set(device_joints(manifest)["arms"])  # not the fixed r_aj_base mounts
    names = {c: s for s, c in mapping.items() if c in arms}
    if len(names) != ARM_JOINTS:
        raise ValueError(f"{manifest}: expected {ARM_JOINTS} arm source names")
    return names


def _runtime_model(source_urdf: Path, manifest: Path | None, runtime: str) -> ET.Element:
    """Canonical model; for real, the arm joints carry their OpenArmHW names (the
    plugin names its joints itself), so states, TF and commands all agree."""
    if runtime not in RUNTIMES:
        raise ValueError(f"runtime must be one of {RUNTIMES}, got {runtime!r}")
    root = canonical_model(source_urdf)
    if runtime == "real":
        names = arm_source_names(manifest or default_manifest_for(source_urdf))
        for joint in root.findall("joint"):
            joint.set("name", names.get(joint.get("name"), joint.get("name")))
    return root


def model_description(source_urdf: Path, manifest: Path | None = None, runtime: str = "fake") -> str:
    """The integrated model for the one robot_state_publisher: no ros2_control."""
    root = _runtime_model(source_urdf, manifest, runtime)
    root.set("name", f"openarm_leap_{runtime}_model")
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


REAL_PLUGIN = "dynamixel_hardware_interface/DynamixelHardware"
#: config/leap_hand/leap_hand_real.yaml, same relative place in the source tree and the install.
REAL_SETTINGS = Path(__file__).resolve().parents[2] / "config" / "leap_hand" / "leap_hand_real.yaml"


def load_real_settings(path: Path | None = None) -> dict:
    return yaml.safe_load(Path(path or REAL_SETTINGS).read_text())


def motor_joints(side: str, manifest: Path, swap: dict | None = None) -> list[tuple[int, str]]:
    """(motor ID, canonical joint) of one hand, by ID: ``leap_<side>_<id>`` in the manifest.

    *swap* ({id: id}, from ``motor_id_swap`` in leap_hand_real.yaml) exchanges the joints
    of motor pairs, for a hand wired differently from the manifest.
    """
    mapping = yaml.safe_load(Path(manifest).read_text()).get("source_to_canonical_joints") or {}
    prefix = f"leap_{side}_"
    by_id = {int(source[len(prefix):]): joint for source, joint in mapping.items()
             if source.startswith(prefix) and source[len(prefix):].isdigit()}
    for a, b in (swap or {}).items():
        a, b = int(a), int(b)
        if a not in by_id or b not in by_id:
            raise ValueError(f"motor_id_swap {a}<->{b}: no such motor")
        by_id[a], by_id[b] = by_id[b], by_id[a]
    motors = sorted(by_id.items())
    if sorted(j for _, j in motors) != sorted(device_joints(manifest)[side]) \
            or [i for i, _ in motors] != list(range(HAND_JOINTS)):
        raise ValueError(f"{manifest}: leap_{side}_0..{HAND_JOINTS - 1} must map onto the "
                         f"{side} hand's {HAND_JOINTS} control joints")
    return motors


def real_hand_description(side: str, source_urdf: Path, manifest: Path, settings: dict) -> str:
    """Integrated model + one hand's 16 joints on the ROBOTIS Dynamixel plugin.

    One transmission per motor, identity matrices: joint ``k`` is the motor with ID ``k``.
    The plugin turns torque off at init (``disable_torque_at_init``) and enables it only
    when the component is activated.
    """
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")
    port = str((settings.get(side) or {}).get("port") or "")
    if not port:
        raise ValueError(f"leap_hand_real.yaml: {side}.port is not set (the hand's U2D2)")
    motors = motor_joints(side, manifest, settings.get("motor_id_swap"))
    n = len(motors)
    identity = ", ".join("1" if r == c else "0" for r in range(n) for c in range(n))
    root = canonical_model(source_urdf)
    name = f"leap_{side}_hand_real"
    root.set("name", name)
    block = ET.SubElement(root, "ros2_control", {"name": name, "type": "system"})
    hardware = ET.SubElement(block, "hardware")
    ET.SubElement(hardware, "plugin").text = REAL_PLUGIN
    for key, value in {
        "port_name": port,
        "baud_rate": settings.get("baud_rate", 4000000),
        "error_timeout_ms": settings.get("error_timeout_ms", 500),
        "dynamixel_model_folder": "/param/dxl_model",
        "number_of_joints": n,
        "number_of_transmissions": n,
        "transmission_to_joint_matrix": identity,
        "joint_to_transmission_matrix": identity,
        "disable_torque_at_init": "true",
        "dynamixel_state_pub_msg_name": "dxl_state",
        "get_dynamixel_data_srv_name": "get_dxl_data",
        "reboot_dxl_srv_name": "reboot_dxl",
    }.items():
        ET.SubElement(hardware, "param", {"name": key}).text = str(value)
    for _, joint_name in motors:
        joint = ET.SubElement(block, "joint", {"name": joint_name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        for interface in ("position", "velocity", "effort"):
            ET.SubElement(joint, "state_interface", {"name": interface})
    for motor_id, _ in motors:
        gpio = ET.SubElement(block, "gpio", {"name": f"leap_{side}_dxl{motor_id}"})
        ET.SubElement(gpio, "param", {"name": "type"}).text = "dxl"
        ET.SubElement(gpio, "param", {"name": "ID"}).text = str(motor_id)
        ET.SubElement(gpio, "command_interface", {"name": "Goal Position"})
        for interface in ("Present Position", "Present Velocity", "Present Current"):
            ET.SubElement(gpio, "state_interface", {"name": interface})
        for item, value in (settings.get("motor") or {}).items():
            ET.SubElement(gpio, "param", {"name": item}).text = str(value)
    return _to_text(root)


def real_hand_controllers(side: str, manifest: Path, description: str, start_inactive: bool) -> dict:
    """The fake hand's controllers, plus the component's initial state."""
    params = _controllers(description, {FAKE_HAND_CONTROLLERS[side]: device_joints(manifest)[side]})
    if start_inactive:
        manager = params["/**/controller_manager"]["ros__parameters"]
        manager["hardware_components_initial_state"] = {"inactive": [f"leap_{side}_hand_real"]}
    return params


REAL_ARM_SETTINGS = REAL_SETTINGS.with_name("openarm_leap_arms_real.yaml")
REAL_ARM_PLUGIN = "openarm_hardware/OpenArmHW"


def _control_gains() -> dict:
    """openarm_description config/arm/v10/control_gains.yaml (installed, else this source tree)."""
    relative = Path("config/arm/v10/control_gains.yaml")
    try:
        from ament_index_python.packages import get_package_share_directory

        path = Path(get_package_share_directory("openarm_description")) / relative
    except Exception:  # not sourced: the source tree (ros_ws/src/openarm_description)
        path = Path(__file__).resolve().parents[4] / "openarm_description" / relative
    return yaml.safe_load(path.read_text())


def real_arm_sides(settings: dict) -> list[str]:
    sides = [s for s in SIDES if (settings.get(s) or {}).get("can_interface")]
    if not sides:
        raise ValueError("openarm_leap_arms_real.yaml: no arm enabled (set right/left can_interface)")
    return sides


def real_arm_description(source_urdf: Path, manifest: Path, settings: dict) -> str:
    """Integrated model (arm joints renamed) + one OpenArmHW block per enabled arm."""
    sides = real_arm_sides(settings)
    gains = _control_gains()
    root = _runtime_model(source_urdf, manifest, "real")
    root.set("name", "openarm_leap_arms_real")
    limits = {j.get("name"): j.find("limit") for j in root.findall("joint")}
    for side in sides:
        block = ET.SubElement(root, "ros2_control", {"name": f"openarm_leap_{side}_arm", "type": "system"})
        hardware = ET.SubElement(block, "hardware")
        ET.SubElement(hardware, "plugin").text = REAL_ARM_PLUGIN
        params = {"can_interface": settings[side]["can_interface"], "arm_prefix": f"{side}_",
                  "hand": "false", "can_fd": str(bool(settings.get("can_fd", True))).lower()}
        params.update({k: str(v).lower() if isinstance(v, bool) else v
                       for k, v in (settings.get("safety") or {}).items()})
        for i in range(1, 8):
            params[f"kp{i}"] = gains[f"joint{i}"]["kp"]
        for i in range(1, 8):
            params[f"kd{i}"] = gains[f"joint{i}"]["kd"]
        for key, value in params.items():
            ET.SubElement(hardware, "param", {"name": key}).text = str(value)
        for i in range(1, 8):  # OpenArmHW exports openarm_<side>_joint1..7 in this order
            name = f"openarm_{side}_joint{i}"
            joint = ET.SubElement(block, "joint", {"name": name})
            position = ET.SubElement(joint, "command_interface", {"name": "position"})
            ET.SubElement(position, "param", {"name": "min"}).text = limits[name].get("lower")
            ET.SubElement(position, "param", {"name": "max"}).text = limits[name].get("upper")
            for interface in ("velocity", "effort"):
                ET.SubElement(joint, "command_interface", {"name": interface})
            for interface in ("position", "velocity", "effort", "temperature_rotor", "temperature_mos"):
                ET.SubElement(joint, "state_interface", {"name": interface})
    return _to_text(root)


def real_arm_controllers(description: str, settings: dict) -> dict:
    """Broadcaster + the fake bringup's controller names over the real joint names."""
    sides = real_arm_sides(settings)
    params = _controllers(description, {
        ARM_CONTROLLERS[s]: [f"openarm_{s}_joint{i}" for i in range(1, 8)] for s in sides})
    manager = params["/**/controller_manager"]["ros__parameters"]
    manager["update_rate"] = int(settings.get("update_rate", 750))
    if settings.get("start_inactive", True):
        manager["hardware_components_initial_state"] = {"inactive": [f"openarm_leap_{s}_arm" for s in sides]}
    for side in sides:
        params[f"/**/{ARM_CONTROLLERS[side]}"]["ros__parameters"]["state_interfaces"] = ["position", "velocity"]
    return params


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


def state_sources(manifest: Path, runtime: str = "fake") -> list[dict]:
    """The merger's inputs: which topic owns which joints."""
    owned = device_joints(manifest)
    arms = owned["arms"]
    if runtime == "real":
        names = arm_source_names(manifest)
        arms = [names[n] for n in arms]
    sources = [{"id": "openarm", "topic": ARM_STATE_TOPIC, "joints": arms,
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
