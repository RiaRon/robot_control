# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""One RH56F1 hand under its own controller manager, /rh56f1_<side>/controller_manager.

The split bringup's hand process, after dg5f_right_driver.launch.py: its own
namespace, controller manager, joint_state_broadcaster
(``/rh56f1_<side>/joint_states``) and trajectory controller, independent of the
arm process and of the other hand. It owns exactly six actuators; passive/mimic
joints are not resources. Unlike the DG-5F launch it starts no
robot_state_publisher: the integrated model's one publisher
(openarm_rh56f1_model.launch.py) draws the hand from these joint states, so no
link is published twice.

The controller manager receives its own description as a parameter (the
integrated geometry plus only this hand's ros2_control block), so it needs no
other process to start.

runtime:=fake  mock_components/GenericSystem, canonical joint names,
               <side>_hand_trajectory_controller active, then held once at
               its measured position (fake_trajectory_controller_prime.py) so
               it accepts topic commands.
runtime:=real  refused: there is no hand backend in this bringup yet. See
               docs/openarm-rh56f1-split-bringup.md for what a backend must
               provide to be connected here.
"""

import os
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
from rh56f1_description import FAKE_HAND_CONTROLLERS, default_manifest_for  # noqa: E402
from rh56f1_split_description import (  # noqa: E402
    HAND_NAMESPACE,
    SIDES,
    controller_manager,
    fake_hand_controllers,
    fake_hand_description,
    prime_fake_trajectory_controllers,
)


def _default_canonical_urdf() -> str:
    relative = Path("urdf/generated/rl/openarm_rh56f1_bi_rl.urdf")
    candidates = []
    if os.environ.get("KUKU_LAB_ROOT"):
        candidates.append(Path(os.environ["KUKU_LAB_ROOT"]) / relative)
    candidates += [Path("/workspace/kuku_lab") / relative, Path.home() / "kuku_lab" / relative]
    candidates += [parent / relative for parent in Path(__file__).resolve().parents]
    return next((str(c) for c in candidates if c.is_file()), "")


def _setup(context: LaunchContext):
    text = lambda name: context.perform_substitution(LaunchConfiguration(name))
    side = text("side")
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}")
    if text("runtime") != "fake":
        raise RuntimeError(
            "rh56f1_hand.launch.py has no real hand backend yet; only runtime:=fake "
            "exists. The contract a backend must meet is in "
            "docs/openarm-rh56f1-split-bringup.md.")
    namespace = HAND_NAMESPACE[side]
    manager = controller_manager(namespace)
    if text("check_ownership").lower() == "true":
        require_no_controller_manager(namespace)
    # The same hand must not be owned twice (fake here, real elsewhere).
    hold_device_lock(f"rh56f1_{side}_hand")

    canonical_urdf = Path(text("canonical_urdf"))
    manifest = Path(text("manifest")) if text("manifest") else default_manifest_for(canonical_urdf)
    description = fake_hand_description(side, canonical_urdf, manifest)
    params = fake_hand_controllers(side, manifest, description)
    with tempfile.NamedTemporaryFile(
        mode="w", prefix=f"rh56f1_{side}_hand_controllers_", suffix=".yaml", delete=False
    ) as handle:
        yaml.safe_dump(params, handle, sort_keys=False)
        controllers_file = handle.name

    nodes = [RegisterEventHandler(OnShutdown(
        on_shutdown=lambda *_: Path(controllers_file).unlink(missing_ok=True)))]
    nodes += [Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="both",
        parameters=[{"robot_description": description}, controllers_file],
    )]
    for name in ("joint_state_broadcaster", FAKE_HAND_CONTROLLERS[side]):
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", manager],
        ))
    # Topic commands (the glove adapter) are dropped by a fresh
    # joint_trajectory_controller 2.47.0 until it has finished one action goal.
    nodes.append(prime_fake_trajectory_controllers(nodes[-1], manager, [FAKE_HAND_CONTROLLERS[side]]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side", choices=list(SIDES)),
        DeclareLaunchArgument("runtime", default_value="fake", choices=["fake"],
                              description="Only fake exists; there is no real hand backend here."),
        DeclareLaunchArgument("canonical_urdf", default_value=_default_canonical_urdf()),
        DeclareLaunchArgument("manifest", default_value=""),
        DeclareLaunchArgument("check_ownership", default_value="true", choices=["true", "false"]),
        OpaqueFunction(function=_setup),
    ])
