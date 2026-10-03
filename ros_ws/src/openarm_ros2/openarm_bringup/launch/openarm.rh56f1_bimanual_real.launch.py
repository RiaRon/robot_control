# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Real-hardware bimanual OpenArm + RH56F1 bringup.

Read docs/rh56f1-real-bringup.md before running this against hardware.

What launching does (controller_manager hardware_components_initial_state):
  - Arm components start configured but INACTIVE: state is read, no motor is
    enabled, no command is written. The arm joint_state_broadcaster is active.
  - Hand components start UNCONFIGURED: their transport is not opened until
    the operator runs `ros2 control set_hardware_component_state
    rh56f1_<side>_hand inactive`. (Humble aborts ros2_control_node if any
    component fails its initial transition, so a missing hand must not be
    connected at startup.) Their broadcasters are loaded inactive.
  - The four device controllers are loaded and configured, left inactive.
  - Nothing is ever activated by this file. The operator arms a device with
    `ros2 control set_hardware_component_state <component> active`, then
    activates its controller.

Independently of this file, both hardware plugins refuse an activation that
comes sooner than min_inactive_sec_before_activate after configure
(config/rh56f1/real_bringup_safety.yaml), so the description cannot enable
motors on its own even when started without this launch file.

Separate from the fake-only openarm.rh56f1_bimanual.launch.py.
"""

import os
from pathlib import Path
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from device_guard import hold_device_lock, require_no_controller_manager  # noqa: E402
from rh56f1_description import HAND_CONFIGURATIONS  # noqa: E402
from rh56f1_real_description import (  # noqa: E402
    build_runtime_variant,
    controller_params,
    device_plan,
    render_real_control_description,
)


def _default_canonical_urdf() -> str:
    relative = Path("urdf/generated/rl/openarm_rh56f1_bi_rl.urdf")
    candidates = []
    if os.environ.get("KUKU_LAB_ROOT"):
        candidates.append(Path(os.environ["KUKU_LAB_ROOT"]) / relative)
    candidates.extend(
        [Path("/workspace/kuku_lab") / relative, Path.home() / "kuku_lab" / relative]
    )
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / relative)
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return ""


def _bool(context: LaunchContext, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value in ("true", "1", "yes"):
        return True
    if value in ("false", "0", "no"):
        return False
    raise RuntimeError(f"launch argument {name}:={value!r} is not true/false")


def _text(context: LaunchContext, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _setup(context: LaunchContext):
    if _text(context, "i_understand_this_moves_real_hardware").lower() != "true":
        raise RuntimeError(
            "openarm.rh56f1_bimanual_real.launch.py refuses to start: "
            "i_understand_this_moves_real_hardware must be 'true' (it has no default). "
            "Read docs/rh56f1-real-bringup.md first."
        )
    canonical_urdf = Path(_text(context, "canonical_urdf"))
    manifest_text = _text(context, "manifest")
    manifest = Path(manifest_text) if manifest_text else None
    hand_configuration = _text(context, "hand_configuration")
    options = {
        "hand_configuration": hand_configuration,
        "enable_right_arm": _bool(context, "enable_right_arm"),
        "enable_left_arm": _bool(context, "enable_left_arm"),
        "right_hand_transport": _text(context, "right_hand_transport"),
        "left_hand_transport": _text(context, "left_hand_transport"),
    }
    _, contract, _ = build_runtime_variant(canonical_urdf, hand_configuration, manifest)
    plan = device_plan(
        contract, allow_mock_hands=_bool(context, "allow_mock_hands_in_real_launch"), **options
    )
    # Single ownership with the split bringup (device_guard): the same
    # controller manager name, CAN interface or hand is never owned twice.
    require_no_controller_manager(_text(context, "namespace").strip("/"))
    for side in ("right", "left"):
        if options[f"enable_{side}_arm"]:
            hold_device_lock(f"can_{_text(context, f'{side}_can_interface')}")
    for component in plan["components"]:
        if component.startswith("rh56f1_"):
            hold_device_lock(component)

    description_share = Path(get_package_share_directory("openarm_description"))
    description = render_real_control_description(
        source_urdf=canonical_urdf,
        manifest=manifest,
        wrapper_xacro=description_share / "urdf/robot/openarm_rh56f1_bimanual_real.urdf.xacro",
        right_can_interface=_text(context, "right_can_interface"),
        left_can_interface=_text(context, "left_can_interface"),
        can_fd=_bool(context, "can_fd"),
        right_hand_port=_text(context, "right_hand_port"),
        left_hand_port=_text(context, "left_hand_port"),
        right_hand_baudrate=int(_text(context, "right_hand_baudrate")),
        left_hand_baudrate=int(_text(context, "left_hand_baudrate")),
        right_hand_device_id=int(_text(context, "right_hand_device_id")),
        left_hand_device_id=int(_text(context, "left_hand_device_id")),
        **options,
    )

    static = yaml.safe_load(
        (
            Path(get_package_share_directory("openarm_bringup"))
            / "config/controllers/openarm_rh56f1_real_controllers.yaml"
        ).read_text()
    )
    with tempfile.NamedTemporaryFile(
        mode="w", prefix="openarm_rh56f1_real_controllers_", suffix=".yaml", delete=False
    ) as handle:
        yaml.safe_dump(controller_params(static, plan), handle, sort_keys=False)
        controllers_file = handle.name

    namespace = _text(context, "namespace").strip("/") or None
    manager = f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    robot_description = {"robot_description": description}
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            output="both",
            parameters=[robot_description, controllers_file],
        ),
    ]
    # One spawner per controller: spawner stops at its first failure, and one
    # device must not keep the others from loading. The arm broadcaster is
    # activated (read-only); hand broadcasters are loaded inactive because their
    # component starts unconfigured; device controllers are left inactive.
    for name, spec in plan["broadcasters"].items():
        mode = [] if spec["active_at_launch"] else ["--inactive"]
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[name, *mode, "--controller-manager", manager],
            )
        )
    for name in plan["controllers"]:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[name, "--inactive", "--controller-manager", manager],
            )
        )
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            "i_understand_this_moves_real_hardware",
            description="Must be 'true'. No default: this brings up real OpenArm/RH56F1 hardware.",
        ),
        DeclareLaunchArgument(
            "canonical_urdf",
            default_value=_default_canonical_urdf(),
            description="Absolute path to canonical openarm_rh56f1_bi_rl.urdf.",
        ),
        DeclareLaunchArgument(
            "manifest",
            default_value="",
            description="Canonical manifest; default: <canonical_urdf stem>_manifest.yaml.",
        ),
        DeclareLaunchArgument(
            "hand_configuration",
            default_value="both",
            choices=list(HAND_CONFIGURATIONS),
            description="RH56F1 selection: arm_only, left, right, or both.",
        ),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument("can_fd", default_value="true"),
        DeclareLaunchArgument(
            "enable_right_arm",
            default_value="true",
            description="false brings up the rest without the right OpenArm.",
        ),
        DeclareLaunchArgument(
            "enable_left_arm",
            default_value="true",
            description="false brings up the rest without the left OpenArm.",
        ),
        DeclareLaunchArgument(
            "right_hand_transport",
            default_value="",
            description="Required when the right hand is enabled: rs485 (not implemented yet; "
            "refuses to connect) or mock (needs allow_mock_hands_in_real_launch:=true).",
        ),
        DeclareLaunchArgument(
            "left_hand_transport",
            default_value="",
            description="Required when the left hand is enabled; see right_hand_transport.",
        ),
        DeclareLaunchArgument(
            "allow_mock_hands_in_real_launch",
            default_value="false",
            description="true permits a mock hand here; its simulated state is published on "
            "/rh56f1_mock_joint_state_broadcaster/joint_states, never /joint_states.",
        ),
        DeclareLaunchArgument("right_hand_port", default_value=""),
        DeclareLaunchArgument("left_hand_port", default_value=""),
        DeclareLaunchArgument("right_hand_baudrate", default_value="0"),
        DeclareLaunchArgument("left_hand_baudrate", default_value="0"),
        DeclareLaunchArgument("right_hand_device_id", default_value="0"),
        DeclareLaunchArgument("left_hand_device_id", default_value="0"),
        DeclareLaunchArgument("namespace", default_value=""),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup)])
