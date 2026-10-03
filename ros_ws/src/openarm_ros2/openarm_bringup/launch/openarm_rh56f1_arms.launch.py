# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""OpenArm arms only, with the RH56F1 hands' geometry, under /controller_manager.

The split bringup's arm process, the counterpart of openarm.bimanual.launch.py:
one controller manager for both arms (14 joints), no hand hardware and no hand
resource. The description keeps both mounted hands, so the palm frames
(``r_hl_palm_sensor``, ``l_hl_palm_sensor``) and the hand meshes are where the
canonical URDF puts them. Each hand runs from rh56f1_hand.launch.py, in its own
process, and this launch neither starts nor waits for one.

The arms' joint_state_broadcaster publishes on ``/openarm/joint_states``. With
``start_model:=true`` (default) this launch also includes
openarm_rh56f1_model.launch.py: the one robot_state_publisher and the merger
that builds ``/joint_states``. Pass ``start_model:=false`` when that runs on its
own.

runtime:=fake  mock_components/GenericSystem, trajectory controllers active
               (right/left_joint_trajectory_controller, canonical joint names)
runtime:=real  OpenArmHW with the integrated real bringup's safety file and
               startup policy: components inactive, controllers
               (rh56f1_<side>_arm_controller) loaded inactive, nothing moves at
               launch. Needs i_understand_this_moves_real_hardware:=true.
"""

import os
from pathlib import Path
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from device_guard import hold_device_lock, require_no_controller_manager  # noqa: E402
from rh56f1_split_description import (  # noqa: E402
    ARM_NAMESPACE,
    ARM_STATE_TOPIC,
    RUNTIMES,
    controller_manager,
    fake_arm_description,
    real_arm_description,
)

def _default_canonical_urdf() -> str:
    relative = Path("urdf/generated/rl/openarm_rh56f1_bi_rl.urdf")
    candidates = []
    if os.environ.get("KUKU_LAB_ROOT"):
        candidates.append(Path(os.environ["KUKU_LAB_ROOT"]) / relative)
    candidates += [Path("/workspace/kuku_lab") / relative, Path.home() / "kuku_lab" / relative]
    candidates += [parent / relative for parent in Path(__file__).resolve().parents]
    return next((str(c) for c in candidates if c.is_file()), "")


def _text(context: LaunchContext, name: str) -> str:
    return context.perform_substitution(LaunchConfiguration(name))


def _bool(context: LaunchContext, name: str) -> bool:
    value = _text(context, name).strip().lower()
    if value not in ("true", "false"):
        raise ValueError(f"{name} must be true or false, got {value!r}")
    return value == "true"


def _setup(context: LaunchContext):
    runtime = _text(context, "runtime")
    if runtime not in RUNTIMES:
        raise ValueError(f"runtime must be one of {RUNTIMES}")
    canonical_urdf = Path(_text(context, "canonical_urdf"))
    manifest_text = _text(context, "manifest")
    manifest = Path(manifest_text) if manifest_text else None
    description_share = Path(get_package_share_directory("openarm_description"))
    bringup_share = Path(get_package_share_directory("openarm_bringup"))
    manager = controller_manager(ARM_NAMESPACE)

    if runtime == "real":
        if _text(context, "i_understand_this_moves_real_hardware").lower() != "true":
            raise RuntimeError(
                "runtime:=real brings up the real OpenArm arms: pass "
                "i_understand_this_moves_real_hardware:=true after reading "
                "docs/openarm-rh56f1-split-bringup.md and docs/rh56f1-real-bringup.md")
    if _bool(context, "check_ownership"):
        require_no_controller_manager(ARM_NAMESPACE)
    remappings = [("/joint_states", ARM_STATE_TOPIC),
                  ("/dynamic_joint_states", "/openarm/dynamic_joint_states")]

    if runtime == "fake":
        description = fake_arm_description(
            canonical_urdf, description_share / "urdf/robot/openarm_rh56f1_bimanual.urdf.xacro")
        controllers_file = str(bringup_share / "config/controllers/openarm_rh56f1_fake_controllers.yaml")
        spawners = [
            ["joint_state_broadcaster"],
            ["left_joint_trajectory_controller", "right_joint_trajectory_controller"],
        ]
    else:
        enable = {"right": _bool(context, "enable_right_arm"),
                  "left": _bool(context, "enable_left_arm")}
        can = {"right": _text(context, "right_can_interface"),
               "left": _text(context, "left_can_interface")}
        # One process per CAN interface, whichever bringup asks for it.
        for side in ("right", "left"):
            if enable[side]:
                hold_device_lock(f"can_{can[side]}")
        from rh56f1_real_description import controller_params

        description, plan = real_arm_description(
            canonical_urdf, manifest,
            description_share / "urdf/robot/openarm_rh56f1_bimanual_real.urdf.xacro",
            right_can_interface=can["right"], left_can_interface=can["left"],
            can_fd=_bool(context, "can_fd"), enable_right_arm=enable["right"],
            enable_left_arm=enable["left"])
        static = yaml.safe_load(
            (bringup_share / "config/controllers/openarm_rh56f1_real_controllers.yaml").read_text())
        with tempfile.NamedTemporaryFile(
            mode="w", prefix="openarm_rh56f1_arms_controllers_", suffix=".yaml", delete=False
        ) as handle:
            yaml.safe_dump(controller_params(static, plan), handle, sort_keys=False)
            controllers_file = handle.name
        # Broadcasters active (read only); arm controllers loaded inactive, as in
        # the integrated real bringup: activation is an explicit operator step.
        spawners = [[name] if spec["active_at_launch"] else [name, "--inactive"]
                    for name, spec in plan["broadcasters"].items()]
        spawners += [[name, "--inactive"] for name in plan["controllers"]]

    robot_description = {"robot_description": description}
    nodes = []
    if runtime == "real":
        nodes.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda *_: Path(controllers_file).unlink(missing_ok=True))))
    nodes += [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[robot_description, controllers_file],
            remappings=remappings,
        ),
    ]
    # One spawner per line of the list: a failing controller must not stop the others.
    for arguments in spawners:
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*arguments, "--controller-manager", manager],
        ))
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("runtime", default_value="fake", choices=list(RUNTIMES),
                              description="fake: GenericSystem. real: OpenArmHW (CAN)."),
        DeclareLaunchArgument("canonical_urdf", default_value=_default_canonical_urdf(),
                              description="Absolute path to canonical openarm_rh56f1_bi_rl.urdf."),
        DeclareLaunchArgument("manifest", default_value="",
                              description="Canonical manifest; default: <canonical_urdf stem>_manifest.yaml."),
        DeclareLaunchArgument("start_model", default_value="true", choices=["true", "false"],
                              description="Also start the integrated model (robot_state_publisher + merger)."),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"],
                              description="Start RViz with the integrated model (needs start_model)."),
        DeclareLaunchArgument("hand_display_placeholder", default_value="true",
                              choices=["true", "false"],
                              description="Draw a hand with no fresh state at the open pose (marked, display only)."),
        DeclareLaunchArgument("check_ownership", default_value="true", choices=["true", "false"],
                              description="Refuse to start when /controller_manager already runs."),
        DeclareLaunchArgument("i_understand_this_moves_real_hardware", default_value="",
                              description="Must be 'true' for runtime:=real. No default."),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument("can_fd", default_value="true"),
        DeclareLaunchArgument("enable_right_arm", default_value="true"),
        DeclareLaunchArgument("enable_left_arm", default_value="true"),
    ]
    model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(__file__).resolve().parent / "openarm_rh56f1_model.launch.py")),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in ("runtime", "canonical_urdf", "manifest", "use_rviz",
                         "hand_display_placeholder")
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_model")),
    )
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup), model])
