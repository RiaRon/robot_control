"""Canonical OpenArm/RH56F1 description adapter for fake control bringup.

The checked-in canonical URDF remains the only source for geometry, joint
origins, axes, limits, and mimic relationships.  This adapter only removes
disabled hand subtrees, resolves the canonical asset's legacy mesh prefix, and
asks Xacro to append the fake-only ros2_control overlay.
"""

from __future__ import annotations

import copy
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import yaml


HAND_CONFIGURATIONS = ("arm_only", "left", "right", "both")
RH56F1_STATE_POLICIES = ("inactive", "parked", "fake_commandable")
#: Policies whose selected hands export the six actuator interfaces.
HAND_RESOURCE_POLICIES = ("parked", "fake_commandable")
#: Fake-only test controllers, spawned only under ``fake_commandable``.
FAKE_HAND_CONTROLLERS = {
    "right": "right_hand_trajectory_controller",
    "left": "left_hand_trajectory_controller",
}
LEGACY_MESH_ROOT = "file:///home/user/rl_ws/urdf/"
_HAND_LINK_PREFIX = {"left": "l_hl_", "right": "r_hl_"}
_HAND_JOINT_PREFIX = {"left": "l_hj_", "right": "r_hj_"}


def _enabled_sides(hand_configuration: str) -> set[str]:
    if hand_configuration not in HAND_CONFIGURATIONS:
        raise ValueError(
            f"unsupported hand_configuration {hand_configuration!r}; "
            f"choose {', '.join(HAND_CONFIGURATIONS)}"
        )
    if hand_configuration == "both":
        return {"left", "right"}
    if hand_configuration == "arm_only":
        return set()
    return {hand_configuration}


def default_manifest_for(canonical_urdf: Path) -> Path:
    """``<stem>_manifest.yaml`` next to the canonical URDF."""

    canonical_urdf = Path(canonical_urdf)
    return canonical_urdf.with_name(canonical_urdf.stem + "_manifest.yaml")


def ros2_control_joint_names(description: str) -> list[str]:
    """Joint resources declared by the rendered ros2_control overlay, in order."""

    root = ET.fromstring(description)
    return [joint.get("name") for joint in root.findall("ros2_control/joint")]


def fake_hand_controller_params(
    manifest: Path, hand_configuration: str, description: str
) -> dict:
    """Parameters of the fake-only hand trajectory controllers.

    Each controller's joints are the manifest ``control_joint_order`` entries
    of its side's hand, in that order; nothing is listed here.  They must be
    exactly the hand resources of the rendered description, so a passive or
    mimic joint can never become a controller joint.
    """

    order = yaml.safe_load(Path(manifest).read_text()).get("control_joint_order")
    if not isinstance(order, list) or not order:
        raise ValueError(f"{manifest}: no control_joint_order")
    resources = ros2_control_joint_names(description)
    params = {}
    for side in ("right", "left"):
        if side not in _enabled_sides(hand_configuration):
            continue
        prefix = _HAND_JOINT_PREFIX[side]
        joints = [name for name in order if name.startswith(prefix)]
        described = [name for name in resources if name.startswith(prefix)]
        if joints != described:
            raise ValueError(
                f"{side} hand: manifest control_joint_order {joints} does not "
                f"match the ros2_control hand resources {described}"
            )
        params[FAKE_HAND_CONTROLLERS[side]] = {
            "ros__parameters": {
                "type": "joint_trajectory_controller/JointTrajectoryController",
                "joints": joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position"],
                "state_publish_rate": 50.0,
                "action_monitor_rate": 20.0,
                "allow_partial_joints_goal": False,
            }
        }
    return params


def build_canonical_variant(
    source_urdf: Path, hand_configuration: str
) -> ET.Element:
    """Return a selected-hand copy of the canonical integrated model."""

    source_urdf = Path(source_urdf).resolve()
    if not source_urdf.is_file():
        raise FileNotFoundError(f"canonical RH56F1 URDF not found: {source_urdf}")

    enabled_sides = _enabled_sides(hand_configuration)
    root = copy.deepcopy(ET.parse(source_urdf).getroot())
    removed_links = {
        link.get("name")
        for side, prefix in _HAND_LINK_PREFIX.items()
        if side not in enabled_sides
        for link in root.findall("link")
        if (link.get("name") or "").startswith(prefix)
    }

    for link in list(root.findall("link")):
        if link.get("name") in removed_links:
            root.remove(link)
    for joint in list(root.findall("joint")):
        parent = joint.find("parent")
        child = joint.find("child")
        if (
            parent is not None
            and parent.get("link") in removed_links
            or child is not None
            and child.get("link") in removed_links
        ):
            root.remove(joint)

    # The canonical file was generated in another workspace.  Resolve only
    # that historical prefix; all kinematic and dynamic values stay untouched.
    canonical_repo = source_urdf.parents[2]
    local_mesh_root = canonical_repo.as_uri().rstrip("/") + "/"
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename and filename.startswith(LEGACY_MESH_ROOT):
            mesh.set(
                "filename", local_mesh_root + filename[len(LEGACY_MESH_ROOT) :]
            )

    root.set("name", f"openarm_rh56f1_{hand_configuration}_control")
    return root


def render_control_description(
    *,
    source_urdf: Path,
    wrapper_xacro: Path,
    hand_configuration: str,
    state_policy: str,
    use_fake_hardware: bool,
) -> str:
    """Render canonical geometry plus the fake-only ros2_control overlay."""

    if state_policy not in RH56F1_STATE_POLICIES:
        raise ValueError(
            f"unsupported RH56F1 state policy {state_policy!r}; "
            f"choose {', '.join(RH56F1_STATE_POLICIES)}"
        )
    if not use_fake_hardware:
        raise ValueError(
            "RH56F1 control integration has no real hardware backend; "
            "set use_fake_hardware:=true"
        )

    import xacro

    variant = build_canonical_variant(source_urdf, hand_configuration)
    ET.indent(variant, space="  ")
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".urdf", encoding="utf-8", delete=False
        ) as temporary:
            temporary.write(
                ET.tostring(variant, encoding="unicode", xml_declaration=True)
            )
            temporary_path = Path(temporary.name)

        document = xacro.process_file(
            str(wrapper_xacro),
            mappings={
                "canonical_urdf": str(temporary_path),
                "control_xacro": str(
                    wrapper_xacro.parent.parent
                    / "ros2_control"
                    / "openarm_rh56f1.bimanual.ros2_control.xacro"
                ),
                "hand_configuration": hand_configuration,
                "rh56f1_state_policy": state_policy,
                "use_fake_hardware": "true",
            },
        )
        return document.toprettyxml(indent="  ")
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
