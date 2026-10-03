"""Runtime description for the REAL OpenArm + RH56F1 bringup.

Geometry, origins, axes, limits and mimic relations come only from the
canonical URDF (via rh56f1_description.build_canonical_variant). The real
runtime then uses the manifest's source joint names, the way the OpenArm arm
bringup already does: OpenArmHW exports openarm_{right,left}_joint1..7, and the
hands use the manifest's rh56f1_* names. Only joint names change, in a
temporary variant; link names (so r_hl_palm_sensor / l_hl_palm_sensor) and the
authoritative files are untouched. Canonical names stay at the policy
boundary (robot_control profile + CanonicalInterface).
"""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rh56f1_description import HAND_CONFIGURATIONS, build_canonical_variant  # noqa: E402

#: Canonical actuator order the rh56f1_hardware plugin binds by name.
HAND_ACTUATORS = ("thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "pinky_1")
ARM_JOINTS = tuple(f"aj_{i}" for i in range(1, 8))
SIDES = {"right": "r", "left": "l"}
HAND_TRANSPORTS = ("mock", "rs485")


def default_manifest_for(canonical_urdf: Path) -> Path:
    canonical_urdf = Path(canonical_urdf)
    return canonical_urdf.with_name(canonical_urdf.stem + "_manifest.yaml")


def load_canonical_to_source(manifest: Path) -> dict[str, str]:
    """canonical joint name -> source joint name, from source_to_canonical_joints."""
    data = yaml.safe_load(Path(manifest).read_text())
    source_to_canonical = data.get("source_to_canonical_joints")
    if not isinstance(source_to_canonical, dict) or not source_to_canonical:
        raise ValueError(f"{manifest}: no source_to_canonical_joints")
    canonical_to_source = {c: s for s, c in source_to_canonical.items()}
    if len(canonical_to_source) != len(source_to_canonical):
        raise ValueError(f"{manifest}: source_to_canonical_joints is not one-to-one")
    return canonical_to_source


def apply_source_joint_names(root: ET.Element, canonical_to_source: dict[str, str]) -> ET.Element:
    """Rename joints (and mimic references) that have a manifest source name."""
    joint_names = {joint.get("name") for joint in root.findall("joint")}
    clashes = joint_names & set(canonical_to_source.values())
    if clashes:
        raise ValueError(f"source names already used as joint names: {sorted(clashes)}")
    for joint in root.findall("joint"):
        name = joint.get("name")
        if name in canonical_to_source:
            joint.set("name", canonical_to_source[name])
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") in canonical_to_source:
            mimic.set("joint", canonical_to_source[mimic.get("joint")])
    return root


def _limit(root: ET.Element, joint_name: str) -> dict[str, float]:
    for joint in root.findall("joint"):
        if joint.get("name") == joint_name:
            limit = joint.find("limit")
            if limit is None or limit.get("lower") is None or limit.get("upper") is None:
                raise ValueError(f"joint {joint_name} has no position limits")
            return {"lower": float(limit.get("lower")), "upper": float(limit.get("upper"))}
    raise ValueError(f"joint {joint_name} is not in the runtime description")


def control_contract(
    runtime_root: ET.Element,
    canonical_to_source: dict[str, str],
    hand_configuration: str,
) -> dict:
    """Names and position limits of every commanded joint, from the runtime URDF."""
    enabled_hands = {"arm_only": (), "left": ("left",), "right": ("right",),
                     "both": ("right", "left")}[hand_configuration]
    contract: dict = {"arms": {}, "hands": {}}
    for side, prefix in SIDES.items():
        arm = []
        for joint in ARM_JOINTS:
            canonical = f"{prefix}_{joint}"
            name = canonical_to_source[canonical]
            arm.append({"canonical": canonical, "name": name, **_limit(runtime_root, name)})
        contract["arms"][side] = arm
        if side in enabled_hands:
            hand = {}
            for actuator in HAND_ACTUATORS:
                canonical = f"{prefix}_hj_{actuator}"
                name = canonical_to_source[canonical]
                hand[actuator] = {"canonical": canonical, "name": name,
                                  **_limit(runtime_root, name)}
            contract["hands"][side] = hand
    return contract


def build_runtime_variant(
    source_urdf: Path, hand_configuration: str, manifest: Path | None = None
) -> tuple[ET.Element, dict, dict[str, str]]:
    manifest = Path(manifest) if manifest else default_manifest_for(source_urdf)
    canonical_to_source = load_canonical_to_source(manifest)
    root = build_canonical_variant(source_urdf, hand_configuration)
    apply_source_joint_names(root, canonical_to_source)
    return root, control_contract(root, canonical_to_source, hand_configuration), canonical_to_source


def enabled_hands(hand_configuration: str) -> tuple[str, ...]:
    if hand_configuration not in HAND_CONFIGURATIONS:
        raise ValueError(
            f"unsupported hand_configuration {hand_configuration!r}; "
            f"choose {', '.join(HAND_CONFIGURATIONS)}"
        )
    return {"arm_only": (), "left": ("left",), "right": ("right",),
            "both": ("right", "left")}[hand_configuration]


def device_plan(
    contract: dict,
    *,
    hand_configuration: str,
    enable_right_arm: bool,
    enable_left_arm: bool,
    right_hand_transport: str,
    left_hand_transport: str,
    allow_mock_hands: bool,
) -> dict:
    """Components, controllers and read-only broadcasters of the real launch.

    Each device has its own broadcaster(s), so a hand that fails to configure
    (e.g. transport rs485, not implemented) takes nothing else down. Real-device
    joints go to /joint_states; a mock hand, refused unless explicitly allowed,
    publishes on its own broadcaster's local topic instead.
    """
    transports = {"right": right_hand_transport, "left": left_hand_transport}
    plan = {"components": [], "controllers": [], "broadcasters": {},
            "real_joints": [], "hand_joints": [], "mock_hand_joints": []}
    arm_joints = []
    for side, enabled in (("right", enable_right_arm), ("left", enable_left_arm)):
        if enabled:
            plan["components"].append(f"openarm_rh56f1_{side}_arm")
            plan["controllers"].append(f"rh56f1_{side}_arm_controller")
            arm_joints += [entry["name"] for entry in contract["arms"][side]]
    if arm_joints:
        plan["broadcasters"]["joint_state_broadcaster"] = {
            "joints": arm_joints, "interfaces": ["position", "velocity", "effort"],
            "use_local_topics": False, "active_at_launch": True}
        plan["real_joints"] += arm_joints
    for side in enabled_hands(hand_configuration):
        transport = transports[side]
        if transport not in HAND_TRANSPORTS:
            raise ValueError(
                f"{side}_hand_transport must be one of {HAND_TRANSPORTS} when that hand is "
                f"enabled (there is no default); got {transport!r}"
            )
        mock = transport == "mock"
        if mock and not allow_mock_hands:
            raise ValueError(
                f"{side}_hand_transport:=mock in the REAL launch requires "
                "allow_mock_hands_in_real_launch:=true; its state is simulated"
            )
        names = [contract["hands"][side][key]["name"] for key in HAND_ACTUATORS]
        plan["components"].append(f"rh56f1_{side}_hand")
        plan["controllers"].append(f"rh56f1_{side}_hand_controller")
        # A hand component starts unconfigured (see controller_params), so its
        # broadcasters are loaded inactive and activated after it is configured.
        plan["broadcasters"][f"rh56f1_{side}_hand_state_broadcaster"] = {
            "joints": names, "interfaces": ["position", "velocity", "effort"],
            "use_local_topics": mock, "active_at_launch": False}
        plan["broadcasters"][f"rh56f1_{side}_hand_status_broadcaster"] = {
            "joints": names, "interfaces": list(HAND_STATUS_INTERFACES),
            "use_local_topics": True, "active_at_launch": False}
        plan["hand_joints"] += names
        plan["mock_hand_joints" if mock else "real_joints"] += names
    return plan


HAND_STATUS_INTERFACES = ("comm_ok", "fault", "command_clamped", "is_mock", "writes")


def controller_params(static: dict, plan: dict) -> dict:
    """The static controller YAML narrowed to the plan, plus broadcaster parameters."""
    params = yaml.safe_load(yaml.safe_dump(static))
    manager = params["controller_manager"]["ros__parameters"]
    for name in [n for n in manager if n.endswith("_controller")]:
        if name not in plan["controllers"]:
            manager.pop(name)
            params.pop(name, None)
    for name, spec in plan["broadcasters"].items():
        manager[name] = {"type": "joint_state_broadcaster/JointStateBroadcaster"}
        params[name] = {"ros__parameters": {
            "joints": list(spec["joints"]), "interfaces": list(spec["interfaces"]),
            "use_local_topics": bool(spec["use_local_topics"])}}
    # Nothing starts active. Humble's controller_manager aborts the whole
    # ros2_control_node when a component misses its initial state at startup,
    # so a hand (whose configure opens its transport and can fail) starts
    # unconfigured and is connected by an explicit operator step; the arms,
    # whose configure cannot fail, start configured but inactive: state read,
    # motors not enabled, no command.
    initial = {}
    arms = [c for c in plan["components"] if c.startswith("openarm_")]
    hands = [c for c in plan["components"] if c.startswith("rh56f1_")]
    if arms:
        initial["inactive"] = arms
    if hands:
        initial["unconfigured"] = hands
    if initial:
        manager["hardware_components_initial_state"] = initial
    return params


def _replace_arm_plugins_with_test_double(document: ET.Element) -> None:
    """Swap OpenArmHW for GenericSystem (same joints and interfaces).

    Only for the isolated runtime probe, which has no SocketCAN; the real
    launch file never asks for it.
    """
    for component in document.findall("ros2_control"):
        plugin = component.find("hardware/plugin")
        if plugin is not None and plugin.text.strip() == "openarm_hardware/OpenArmHW":
            plugin.text = "mock_components/GenericSystem"
    document.set("name", document.get("name", "") + "_TEST_DOUBLE")


def _inject_hardware_params(document: ET.Element, extra: dict[str, dict[str, str]]) -> None:
    for component in document.findall("ros2_control"):
        params = extra.get(component.get("name"))
        if not params:
            continue
        hardware = component.find("hardware")
        for key, value in params.items():
            existing = hardware.find(f"param[@name='{key}']")
            if existing is None:
                existing = ET.SubElement(hardware, "param", {"name": key})
            existing.text = str(value)


def render_real_control_description(
    *,
    source_urdf: Path,
    wrapper_xacro: Path,
    hand_configuration: str,
    right_can_interface: str,
    left_can_interface: str,
    can_fd: bool,
    enable_right_arm: bool,
    enable_left_arm: bool,
    right_hand_transport: str,
    left_hand_transport: str,
    right_hand_port: str,
    left_hand_port: str,
    right_hand_baudrate: int,
    left_hand_baudrate: int,
    right_hand_device_id: int,
    left_hand_device_id: int,
    manifest: Path | None = None,
    arm_test_double: bool = False,
    extra_hardware_params: dict[str, dict[str, str]] | None = None,
    hardware_hand_configuration: str | None = None,
) -> str:
    """Render the runtime description: source-named variant + real control overlay.

    ``hand_configuration`` selects the geometry; ``hardware_hand_configuration``
    (default: the same) selects which hands get a ros2_control block. The split
    bringup passes ``both`` and ``arm_only``: the mounted hands and their palm
    frames stay in the model while only the arms' hardware is loaded.
    """
    hardware_hands = (
        hand_configuration if hardware_hand_configuration is None
        else hardware_hand_configuration
    )
    enabled_hands(hardware_hands)  # validates the name
    if not set(enabled_hands(hardware_hands)) <= set(enabled_hands(hand_configuration)):
        raise ValueError(
            f"hand hardware {hardware_hands!r} needs hand geometry that "
            f"{hand_configuration!r} removes"
        )
    transports = {"right": right_hand_transport, "left": left_hand_transport}
    for side in enabled_hands(hardware_hands):
        if transports[side] not in HAND_TRANSPORTS:
            raise ValueError(
                f"{side}_hand_transport must be one of {HAND_TRANSPORTS} when that hand is "
                f"enabled (there is no default); got {transports[side]!r}"
            )

    import xacro

    variant, contract, _ = build_runtime_variant(source_urdf, hand_configuration, manifest)
    ET.indent(variant, space="  ")
    temporary: list[Path] = []
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", encoding="utf-8", delete=False
        ) as handle:
            handle.write(ET.tostring(variant, encoding="unicode", xml_declaration=True))
            temporary.append(Path(handle.name))
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", encoding="utf-8", delete=False
        ) as handle:
            yaml.safe_dump(contract, handle, sort_keys=False)
            temporary.append(Path(handle.name))

        document = xacro.process_file(
            str(wrapper_xacro),
            mappings={
                "runtime_urdf": str(temporary[0]),
                "control_contract": str(temporary[1]),
                "control_xacro": str(
                    wrapper_xacro.parent.parent
                    / "ros2_control"
                    / "openarm_rh56f1.bimanual.real.ros2_control.xacro"
                ),
                "right_can_interface": right_can_interface,
                "left_can_interface": left_can_interface,
                "can_fd": "true" if can_fd else "false",
                "enable_right_arm": "true" if enable_right_arm else "false",
                "enable_left_arm": "true" if enable_left_arm else "false",
                "hand_configuration": hardware_hands,
                "right_hand_transport": right_hand_transport,
                "left_hand_transport": left_hand_transport,
                "right_hand_port": right_hand_port,
                "left_hand_port": left_hand_port,
                "right_hand_baudrate": str(right_hand_baudrate),
                "left_hand_baudrate": str(left_hand_baudrate),
                "right_hand_device_id": str(right_hand_device_id),
                "left_hand_device_id": str(left_hand_device_id),
            },
        )
    finally:
        for path in temporary:
            path.unlink(missing_ok=True)

    root = ET.fromstring(document.toxml())
    if arm_test_double:
        _replace_arm_plugins_with_test_double(root)
    if extra_hardware_params:
        _inject_hardware_params(root, extra_hardware_params)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True)
