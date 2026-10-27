# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""RH56F1 right hand: rh56f1_hand.launch.py with side:=right.

/rh56f1_right/controller_manager, /rh56f1_right/joint_states. Every other
argument of rh56f1_hand.launch.py passes through from the command line.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("runtime", default_value="fake", choices=["fake"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(Path(__file__).resolve().parent / "rh56f1_hand.launch.py")),
            launch_arguments={"side": "right",
                              "runtime": LaunchConfiguration("runtime")}.items(),
        ),
    ])
