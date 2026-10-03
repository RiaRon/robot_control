# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""The integrated OpenArm + RH56F1 model: one robot_state_publisher, one merger.

No hardware, no controller manager, no command. This is the only process that
publishes TF for the integrated model, so no link is published by two
robot_state_publishers. It reads each device's own joint states through the
merger (openarm_rh56f1_joint_state_merger.py):

    /openarm/joint_states         arms (14)        ─┐
    /rh56f1_right/joint_states    right hand (6)   ─┼─▶ /joint_states (measured)
    /rh56f1_left/joint_states     left hand (6)    ─┘   /openarm_rh56f1/display_joint_states
                                                         └▶ robot_state_publisher ─▶ /tf, /tf_static

The arm-to-palm-sensor transforms need only the arm joints. Fingertip
transforms are only as good as the hand state: while a hand has none, its
fingers are drawn at the open pose from a marked display placeholder
(``hand_display_placeholder:=true``) or not at all (``false``).
"""

import json
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rh56f1_description import default_manifest_for  # noqa: E402
from rh56f1_split_description import (  # noqa: E402
    DISPLAY_STATE_TOPIC,
    MERGED_STATE_TOPIC,
    RUNTIMES,
    SOURCE_STATUS_TOPIC,
    model_description,
    state_sources,
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
    runtime = text("runtime")
    if runtime not in RUNTIMES:
        raise ValueError(f"runtime must be one of {RUNTIMES}")
    canonical_urdf = Path(text("canonical_urdf"))
    manifest = Path(text("manifest")) if text("manifest") else default_manifest_for(canonical_urdf)
    sources = state_sources(runtime, manifest)
    merger_arguments = [
        "--sources", json.dumps(sources),
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
            parameters=[{"robot_description": model_description(runtime, canonical_urdf, manifest)}],
            remappings=[("joint_states", DISPLAY_STATE_TOPIC)],
        ),
        Node(
            package="openarm_bringup",
            executable="openarm_rh56f1_joint_state_merger.py",
            name="openarm_rh56f1_joint_state_merger",
            output="screen",
            arguments=merger_arguments,
        ),
    ]
    if text("use_rviz").lower() == "true":
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
        DeclareLaunchArgument("runtime", default_value="fake", choices=list(RUNTIMES),
                              description="Joint naming: fake = canonical, real = manifest source names."),
        DeclareLaunchArgument("canonical_urdf", default_value=_default_canonical_urdf()),
        DeclareLaunchArgument("manifest", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("hand_display_placeholder", default_value="true",
                              choices=["true", "false"]),
        DeclareLaunchArgument("stale_after_sec", default_value="0.5",
                              description="A device older than this is stale in the merger."),
        OpaqueFunction(function=_setup),
    ])
