# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""One LEAP Hand under its own controller manager, /leap_<side>/controller_manager (fake).

Like rh56f1_hand.launch.py: its own namespace, controller manager,
joint_state_broadcaster (``/leap_<side>/joint_states``) and
``<side>_hand_trajectory_controller`` over all 16 joints, independent of the arm
process and of the other hand. No robot_state_publisher: the integrated model's one
publisher (openarm_leap_model.launch.py) draws the hand from these joint states.

    ros2 launch openarm_bringup leap_hand.launch.py side:=right

runtime:=fake only: mock_components/GenericSystem. The real hand (Dynamixel, via
ros2_control) is not connected yet.
"""

from pathlib import Path
import sys
import tempfile

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from device_guard import hold_device_lock, require_no_controller_manager  # noqa: E402
from leap_split_description import (  # noqa: E402
    FAKE_HAND_CONTROLLERS,
    HAND_NAMESPACE,
    SIDES,
    controller_manager,
    default_canonical_urdf,
    default_manifest_for,
    description_nodes,
    fake_hand_controllers,
    fake_hand_description,
    prime_fake_trajectory_controllers,
)


def _setup(context: LaunchContext):
    text = lambda name: context.perform_substitution(LaunchConfiguration(name))
    side = text("side")
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}")
    if text("runtime") != "fake":
        raise RuntimeError("leap_hand.launch.py has no real hand backend yet; only runtime:=fake")
    namespace = HAND_NAMESPACE[side]
    manager = controller_manager(namespace)
    if text("check_ownership").lower() == "true":
        require_no_controller_manager(namespace)
    # The same hand must not be owned twice (fake here, real elsewhere later).
    hold_device_lock(f"leap_{side}_hand")

    canonical_urdf = Path(text("canonical_urdf"))
    manifest = Path(text("manifest")) if text("manifest") else default_manifest_for(canonical_urdf)
    description = fake_hand_description(side, canonical_urdf, manifest)
    with tempfile.NamedTemporaryFile(
        mode="w", prefix=f"leap_{side}_hand_controllers_", suffix=".yaml", delete=False
    ) as handle:
        yaml.safe_dump(fake_hand_controllers(side, manifest, description), handle, sort_keys=False)
        controllers_file = handle.name

    nodes = [RegisterEventHandler(OnShutdown(
        on_shutdown=lambda *_: Path(controllers_file).unlink(missing_ok=True)))]
    nodes += description_nodes(description, namespace, f"/{namespace}/robot_description",
                               {"parameters": [controllers_file]})
    for name in ("joint_state_broadcaster", FAKE_HAND_CONTROLLERS[side]):
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", manager],
        ))
    nodes.append(prime_fake_trajectory_controllers(nodes[-1], manager, [FAKE_HAND_CONTROLLERS[side]]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side", choices=list(SIDES)),
        DeclareLaunchArgument("runtime", default_value="fake", choices=["fake"]),
        DeclareLaunchArgument("canonical_urdf", default_value=default_canonical_urdf(__file__)),
        DeclareLaunchArgument("manifest", default_value=""),
        DeclareLaunchArgument("check_ownership", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_setup),
    ])
