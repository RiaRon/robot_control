"""Descriptions and state ownership for the split OpenArm + RH56F1 bringup.

Three controller managers, one per hardware owner, as in the OpenArm-Tesollo
bringup (arms under ``/controller_manager``, each hand under its own
namespace):

    arms        /controller_manager                 14 arm joints
    right hand  /rh56f1_right/controller_manager     6 right-hand actuators
    left hand   /rh56f1_left/controller_manager      6 left-hand actuators

Every description handed to a controller manager is the whole integrated model
(both hands' geometry, so the palm frames and the mounted hands stay where the
canonical URDF puts them) plus only the ros2_control block that manager owns.
The integrated model for robot_state_publisher has no ros2_control block at
all. Geometry, joint origins, axes, limits and mimic relations always come from
the canonical URDF through ``rh56f1_description.build_canonical_variant`` (fake,
canonical joint names) or ``rh56f1_real_description.build_runtime_variant``
(real, the manifest's source names); nothing here edits them.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from rh56f1_description import (
    FAKE_HAND_CONTROLLERS,
    build_canonical_variant,
    default_manifest_for,
    fake_hand_controller_params,
    render_control_description,
)

RUNTIMES = ("fake", "real")
SIDES = ("right", "left")
_PREFIX = {"right": "r", "left": "l"}
FAKE_PLUGIN = "mock_components/GenericSystem"

#: Arms: the root controller manager, as in openarm.bimanual.launch.py.
ARM_NAMESPACE = ""
#: Each hand's controller manager lives in its own namespace, as dg5f_right does.
HAND_NAMESPACE = {"right": "rh56f1_right", "left": "rh56f1_left"}
#: Each device's own joint_state_broadcaster output.
ARM_STATE_TOPIC = "/openarm/joint_states"
HAND_STATE_TOPIC = {side: f"/{ns}/joint_states" for side, ns in HAND_NAMESPACE.items()}
#: Measured joints of every fresh device, each forwarded once with its own stamp.
MERGED_STATE_TOPIC = "/joint_states"
#: What robot_state_publisher draws: the measured joints plus, only while a hand
#: has no fresh state, an explicitly marked display pose for its fingers.
DISPLAY_STATE_TOPIC = "/openarm_rh56f1/display_joint_states"
SOURCE_STATUS_TOPIC = "/openarm_rh56f1/joint_state_sources"
DISPLAY_PLACEHOLDER_FRAME = "display_placeholder"


def controller_manager(namespace: str) -> str:
    return f"/{namespace}/controller_manager" if namespace else "/controller_manager"


def _check_runtime(runtime: str) -> None:
    if runtime not in RUNTIMES:
        raise ValueError(f"runtime must be one of {RUNTIMES}, got {runtime!r}")


def _manifest_order(manifest: Path) -> list[str]:
    order = yaml.safe_load(Path(manifest).read_text()).get("control_joint_order")
    if not isinstance(order, list) or not order:
        raise ValueError(f"{manifest}: no control_joint_order")
    return order


def _canonical_to_source(manifest: Path) -> dict[str, str]:
    from rh56f1_real_description import load_canonical_to_source

    return load_canonical_to_source(manifest)


def device_joints(runtime: str, manifest: Path) -> dict[str, list[str]]:
    """Runtime joint names each device owns, in manifest order.

    ``arms`` is the right arm then the left arm (the manifest's order); each
    hand has its six actuators. Fake names are canonical; real names are the
    manifest's source names (``openarm_right_joint1``,
    ``rh56f1_right_right_thumb_1_joint``). Passive/mimic joints belong to no
    device.
    """
    _check_runtime(runtime)
    order = _manifest_order(manifest)
    rename = {} if runtime == "fake" else _canonical_to_source(manifest)
    owned = {
        "arms": [n for n in order if n.startswith(("r_aj_", "l_aj_"))],
        "right": [n for n in order if n.startswith("r_hj_")],
        "left": [n for n in order if n.startswith("l_hj_")],
    }
    if len(owned["arms"]) != 14 or any(len(owned[s]) != 6 for s in SIDES):
        raise ValueError(f"{manifest}: expected 14 arm and 2x6 hand joints in control_joint_order")
    return {device: [rename.get(n, n) for n in names] for device, names in owned.items()}


def _model_root(runtime: str, source_urdf: Path, manifest: Path) -> ET.Element:
    if runtime == "fake":
        return build_canonical_variant(source_urdf, "both")
    from rh56f1_real_description import build_runtime_variant

    root, _, _ = build_runtime_variant(source_urdf, "both", manifest)
    return root


def model_description(runtime: str, source_urdf: Path, manifest: Path | None = None) -> str:
    """The integrated model for the one robot_state_publisher: no ros2_control."""
    _check_runtime(runtime)
    manifest = Path(manifest) if manifest else default_manifest_for(source_urdf)
    root = _model_root(runtime, Path(source_urdf), manifest)
    root.set("name", f"openarm_rh56f1_{runtime}_model")
    for block in root.findall("ros2_control"):
        root.remove(block)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def fake_arm_description(source_urdf: Path, wrapper_xacro: Path) -> str:
    """Integrated model + the 14 arm joints on GenericSystem, nothing else.

    This is the existing fake overlay with ``rh56f1_state_policy:=inactive``:
    hand geometry kept, no hand resource exported.
    """
    return render_control_description(
        source_urdf=Path(source_urdf), wrapper_xacro=Path(wrapper_xacro),
        hand_configuration="both", state_policy="inactive", use_fake_hardware=True,
    )


def fake_hand_description(side: str, source_urdf: Path, manifest: Path | None = None) -> str:
    """Integrated model + one hand's six actuators on GenericSystem, nothing else."""
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")
    manifest = Path(manifest) if manifest else default_manifest_for(source_urdf)
    root = build_canonical_variant(Path(source_urdf), "both")
    root.set("name", f"openarm_rh56f1_{side}_hand_fake")
    joints = device_joints("fake", manifest)[side]
    block = ET.SubElement(root, "ros2_control", {"name": f"rh56f1_{side}_hand_fake", "type": "system"})
    hardware = ET.SubElement(block, "hardware")
    ET.SubElement(hardware, "plugin").text = FAKE_PLUGIN
    ET.SubElement(hardware, "param", {"name": "state_following_offset"}).text = "0.0"
    for name in joints:
        joint = ET.SubElement(block, "joint", {"name": name})
        ET.SubElement(joint, "command_interface", {"name": "position"})
        state = ET.SubElement(joint, "state_interface", {"name": "position"})
        # Fake-only start pose (open). Not a measured or a physical pose.
        ET.SubElement(state, "param", {"name": "initial_value"}).text = "0.0"
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)


def fake_hand_controllers(side: str, manifest: Path, description: str) -> dict:
    """Controller-manager parameters for one fake hand, as a params-file dict.

    The trajectory controller and its joints are the Stage 6 fake ones
    (``right_hand_trajectory_controller`` ...), joints from the manifest.
    Keys are wildcarded like dg5f_right_controller.yaml, so the file is the
    same whatever namespace the manager runs in.
    """
    hand = fake_hand_controller_params(Path(manifest), side, description)
    name = FAKE_HAND_CONTROLLERS[side]
    values = dict(hand[name]["ros__parameters"])
    controller_type = values.pop("type")
    return {
        "/**/controller_manager": {"ros__parameters": {
            "update_rate": 100,
            "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
            name: {"type": controller_type},
        }},
        f"/**/{name}": {"ros__parameters": values},
    }


def real_arm_description(
    source_urdf: Path,
    manifest: Path | None,
    wrapper_xacro: Path,
    *,
    right_can_interface: str,
    left_can_interface: str,
    can_fd: bool,
    enable_right_arm: bool,
    enable_left_arm: bool,
    arm_test_double: bool = False,
) -> tuple[str, dict]:
    """Integrated model + the arms' OpenArmHW blocks only, and the device plan.

    Same renderer, safety file and startup policy as the integrated real
    bringup (rh56f1_real_description), with the hands' hardware left out.
    """
    from rh56f1_real_description import (
        build_runtime_variant,
        device_plan,
        render_real_control_description,
    )

    manifest = Path(manifest) if manifest else default_manifest_for(source_urdf)
    _, contract, _ = build_runtime_variant(Path(source_urdf), "both", manifest)
    plan = device_plan(
        contract, hand_configuration="arm_only", enable_right_arm=enable_right_arm,
        enable_left_arm=enable_left_arm, right_hand_transport="", left_hand_transport="",
        allow_mock_hands=False,
    )
    description = render_real_control_description(
        source_urdf=Path(source_urdf), manifest=manifest, wrapper_xacro=Path(wrapper_xacro),
        hand_configuration="both", hardware_hand_configuration="arm_only",
        right_can_interface=right_can_interface, left_can_interface=left_can_interface,
        can_fd=can_fd, enable_right_arm=enable_right_arm, enable_left_arm=enable_left_arm,
        right_hand_transport="", left_hand_transport="",
        right_hand_port="", left_hand_port="",
        right_hand_baudrate=0, left_hand_baudrate=0,
        right_hand_device_id=0, left_hand_device_id=0,
        arm_test_double=arm_test_double,
    )
    return description, plan


def ros2_control_blocks(description: str) -> dict[str, dict]:
    """name -> {plugin, joints, command interfaces} of every ros2_control block."""
    blocks = {}
    for block in ET.fromstring(description).findall("ros2_control"):
        plugin = block.find("hardware/plugin")
        joints = block.findall("joint")
        blocks[block.get("name")] = {
            "plugin": None if plugin is None else (plugin.text or "").strip(),
            "joints": [joint.get("name") for joint in joints],
            "commands": [
                f"{joint.get('name')}/{command.get('name')}"
                for joint in joints for command in joint.findall("command_interface")
            ],
        }
    return blocks


def state_sources(runtime: str, manifest: Path) -> list[dict]:
    """The merger's inputs: which topic owns which joints."""
    owned = device_joints(runtime, manifest)
    sources = [{"id": "openarm", "topic": ARM_STATE_TOPIC, "joints": owned["arms"],
                "display_placeholder": False}]
    for side in SIDES:
        sources.append({
            "id": HAND_NAMESPACE[side], "topic": HAND_STATE_TOPIC[side],
            "joints": owned[side],
            # Fingers may be drawn at the open pose while this hand has no
            # fresh state; marked, never on /joint_states.
            "display_placeholder": True,
        })
    return sources
