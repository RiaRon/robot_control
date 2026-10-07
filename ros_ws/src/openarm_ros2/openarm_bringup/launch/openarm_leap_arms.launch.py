# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""OpenArm arms only, with the LEAP hands' geometry, under /controller_manager (fake).

The split bringup's arm process, like openarm_rh56f1_arms.launch.py: one controller
manager for both arms (14 joints), no hand resource. The description keeps both
mounted LEAP hands, so palms and fingertips are where the canonical URDF puts them.
Each hand runs from leap_hand.launch.py; this launch neither starts nor waits for one.

The arms' joint_state_broadcaster publishes on ``/openarm/joint_states``. With
``start_model:=true`` (default) this also includes openarm_leap_model.launch.py
(the one robot_state_publisher and the merger that builds ``/joint_states``).

runtime:=fake  mock_components/GenericSystem, right/left_joint_trajectory_controller
               active, then held once at their measured position
               (fake_trajectory_controller_prime.py) so they accept topic commands.
There is no real runtime here yet.
"""

from pathlib import Path
import sys
import tempfile

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from device_guard import require_no_controller_manager  # noqa: E402
from leap_split_description import (  # noqa: E402
    ARM_CONTROLLERS,
    ARM_DESCRIPTION_TOPIC,
    ARM_NAMESPACE,
    ARM_STATE_TOPIC,
    controller_manager,
    default_canonical_urdf,
    default_manifest_for,
    description_nodes,
    fake_arm_controllers,
    fake_arm_description,
    prime_fake_trajectory_controllers,
)


def _setup(context: LaunchContext):
    text = lambda name: context.perform_substitution(LaunchConfiguration(name))
    if text("runtime") != "fake":
        raise RuntimeError("openarm_leap_arms.launch.py has only runtime:=fake for now")
    if text("check_ownership").lower() == "true":
        require_no_controller_manager(ARM_NAMESPACE)
    canonical_urdf = Path(text("canonical_urdf"))
    manifest = Path(text("manifest")) if text("manifest") else default_manifest_for(canonical_urdf)
    manager = controller_manager(ARM_NAMESPACE)
    description = fake_arm_description(canonical_urdf, manifest)
    with tempfile.NamedTemporaryFile(
        mode="w", prefix="openarm_leap_arms_controllers_", suffix=".yaml", delete=False
    ) as handle:
        yaml.safe_dump(fake_arm_controllers(manifest, description), handle, sort_keys=False)
        controllers_file = handle.name

    nodes = [RegisterEventHandler(OnShutdown(
        on_shutdown=lambda *_: Path(controllers_file).unlink(missing_ok=True)))]
    nodes += description_nodes(description, ARM_NAMESPACE, ARM_DESCRIPTION_TOPIC, {
        "parameters": [controllers_file],
        "remappings": [("/joint_states", ARM_STATE_TOPIC),
                       ("/dynamic_joint_states", "/openarm/dynamic_joint_states")],
    })
    # One spawner per line: a failing controller must not stop the others.
    arms = [ARM_CONTROLLERS["left"], ARM_CONTROLLERS["right"]]
    for arguments in (["joint_state_broadcaster"], arms):
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*arguments, "--controller-manager", manager],
        ))
    nodes.append(prime_fake_trajectory_controllers(nodes[-1], manager, arms))
    return nodes


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("runtime", default_value="fake", choices=["fake"]),
        DeclareLaunchArgument("canonical_urdf", default_value=default_canonical_urdf(__file__),
                              description="Canonical openarm_leap_bi_rl.urdf (urdf repo)."),
        DeclareLaunchArgument("manifest", default_value="",
                              description="Default: <canonical_urdf stem>_manifest.yaml."),
        DeclareLaunchArgument("start_model", default_value="true", choices=["true", "false"],
                              description="Also start robot_state_publisher + merger."),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("hand_display_placeholder", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("check_ownership", default_value="true", choices=["true", "false"]),
    ]
    model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(__file__).resolve().parent / "openarm_leap_model.launch.py")),
        launch_arguments={name: LaunchConfiguration(name) for name in
                          ("canonical_urdf", "manifest", "use_rviz", "hand_display_placeholder")}.items(),
        condition=IfCondition(LaunchConfiguration("start_model")),
    )
    return LaunchDescription(arguments + [OpaqueFunction(function=_setup), model])
