# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""The integrated OpenArm + LEAP model: one robot_state_publisher, one merger.

No hardware, no controller manager, no command; the only TF publisher of the model.
It reads each device's own joint states through the merger
(openarm_rh56f1_joint_state_merger.py, hand-agnostic):

    /openarm/joint_states      arms (14)        ─┐
    /leap_right/joint_states   right hand (16)  ─┼─▶ /joint_states (measured)
    /leap_left/joint_states    left hand (16)   ─┘   /openarm_leap/display_joint_states
                                                      └▶ robot_state_publisher ─▶ /tf, /tf_static

While a hand has no fresh state its fingers are drawn at 0 from a marked display
placeholder (``hand_display_placeholder:=true``) or not at all (``false``).
"""

import json
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from leap_split_description import (  # noqa: E402
    DISPLAY_STATE_TOPIC,
    MERGED_STATE_TOPIC,
    SOURCE_STATUS_TOPIC,
    default_canonical_urdf,
    default_manifest_for,
    model_description,
    state_sources,
)


def _setup(context: LaunchContext):
    text = lambda name: context.perform_substitution(LaunchConfiguration(name))
    canonical_urdf = Path(text("canonical_urdf"))
    manifest = Path(text("manifest")) if text("manifest") else default_manifest_for(canonical_urdf)
    merger_arguments = [
        "--sources", json.dumps(state_sources(manifest)),
        "--merged-topic", MERGED_STATE_TOPIC,
        "--display-topic", DISPLAY_STATE_TOPIC,
        "--status-topic", SOURCE_STATUS_TOPIC,
        "--stale-after-sec", text("stale_after_sec"),
    ]
    if text("hand_display_placeholder").lower() != "true":
        merger_arguments.append("--no-display-placeholder")
    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": model_description(canonical_urdf)}],
            remappings=[("joint_states", DISPLAY_STATE_TOPIC)],
        ),
        Node(
            package="openarm_bringup",
            executable="openarm_rh56f1_joint_state_merger.py",
            name="openarm_leap_joint_state_merger",
            output="screen",
            arguments=merger_arguments,
        ),
    ]
    if text("use_rviz").lower() == "true":
        # The RH56F1 split config is hand-agnostic: fixed frame body_root, /robot_description.
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", str(Path(get_package_share_directory("openarm_bringup"))
                                 / "config/rviz/openarm_rh56f1_split.rviz")],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("canonical_urdf", default_value=default_canonical_urdf(__file__)),
        DeclareLaunchArgument("manifest", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("hand_display_placeholder", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("stale_after_sec", default_value="0.5"),
        OpaqueFunction(function=_setup),
    ])
