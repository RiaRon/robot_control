# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Fake-only bimanual OpenArm + selectable RH56F1 bringup."""

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rh56f1_description import (  # noqa: E402
    HAND_CONFIGURATIONS,
    RH56F1_STATE_POLICIES,
    render_control_description,
)


def _default_canonical_urdf() -> str:
    relative = Path("urdf/generated/rl/openarm_rh56f1_bi_rl.urdf")
    candidates = []
    if os.environ.get("KUKU_LAB_ROOT"):
        candidates.append(Path(os.environ["KUKU_LAB_ROOT"]) / relative)
    candidates.extend(
        [
            Path("/workspace/kuku_lab") / relative,
            Path("/home/cbj4/kuku_lab") / relative,
        ]
    )
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / relative)
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return ""


def _namespace(context: LaunchContext, namespace) -> str | None:
    value = context.perform_substitution(namespace).strip("/")
    return value or None


def _controller_manager(context: LaunchContext, namespace) -> str:
    value = _namespace(context, namespace)
    return f"/{value}/controller_manager" if value else "/controller_manager"


def _robot_nodes(
    context: LaunchContext,
    canonical_urdf,
    hand_configuration,
    rh56f1_state_policy,
    use_fake_hardware,
    controllers_file,
    namespace,
):
    fake = context.perform_substitution(use_fake_hardware).lower() in (
        "1",
        "true",
        "yes",
    )
    description_package = get_package_share_directory("openarm_description")
    wrapper = Path(description_package) / (
        "urdf/robot/openarm_rh56f1_bimanual.urdf.xacro"
    )
    description = render_control_description(
        source_urdf=Path(context.perform_substitution(canonical_urdf)),
        wrapper_xacro=wrapper,
        hand_configuration=context.perform_substitution(hand_configuration),
        state_policy=context.perform_substitution(rh56f1_state_policy),
        use_fake_hardware=fake,
    )
    node_namespace = _namespace(context, namespace)
    robot_description = {"robot_description": description}
    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=node_namespace,
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=node_namespace,
            output="both",
            parameters=[
                robot_description,
                context.perform_substitution(controllers_file),
            ],
        ),
    ]


def _joint_state_broadcaster(context: LaunchContext, namespace):
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=_namespace(context, namespace),
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                _controller_manager(context, namespace),
            ],
        )
    ]


def _arm_controllers(context: LaunchContext, namespace):
    return [
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=_namespace(context, namespace),
            arguments=[
                "left_joint_trajectory_controller",
                "right_joint_trajectory_controller",
                "--controller-manager",
                _controller_manager(context, namespace),
            ],
        )
    ]


def generate_launch_description():
    canonical_urdf = LaunchConfiguration("canonical_urdf")
    hand_configuration = LaunchConfiguration("hand_configuration")
    rh56f1_state_policy = LaunchConfiguration("rh56f1_state_policy")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    namespace = LaunchConfiguration("namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("openarm_bringup"),
            "config",
            "controllers",
            "openarm_rh56f1_fake_controllers.yaml",
        ]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "canonical_urdf",
            default_value=_default_canonical_urdf(),
            description="Absolute path to canonical openarm_rh56f1_bi_rl.urdf.",
        ),
        DeclareLaunchArgument(
            "hand_configuration",
            default_value="both",
            choices=list(HAND_CONFIGURATIONS),
            description="RH56F1 selection: arm_only, left, right, or both.",
        ),
        DeclareLaunchArgument(
            "rh56f1_state_policy",
            default_value="parked",
            choices=list(RH56F1_STATE_POLICIES),
            description=(
                "inactive exports no hand interfaces; parked exports six "
                "fake-only zero-state actuators per selected hand."
            ),
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            choices=["true"],
            description="Must remain true; no RH56F1 real backend exists here.",
        ),
        DeclareLaunchArgument(
            "namespace", default_value="", description="Optional ROS namespace."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz (disabled for headless fake validation).",
        ),
    ]

    robot_nodes = OpaqueFunction(
        function=_robot_nodes,
        args=[
            canonical_urdf,
            hand_configuration,
            rh56f1_state_policy,
            use_fake_hardware,
            controllers_file,
            namespace,
        ],
    )
    joint_state_broadcaster = OpaqueFunction(
        function=_joint_state_broadcaster, args=[namespace]
    )
    arm_controllers = OpaqueFunction(
        function=_arm_controllers, args=[namespace]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("openarm_description"), "rviz", "bimanual.rviz"]
            ),
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes,
            rviz,
            TimerAction(period=1.0, actions=[joint_state_broadcaster]),
            TimerAction(period=1.0, actions=[arm_controllers]),
        ]
    )
