# Copyright 2026 KUKU Robot Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Nova2 glove input for one RH56F1 hand: retarget node + fake-hand adapter.

    /senseglove/glove<serial>/<rh|lh>/joint_states   (glove driver, or the synthetic glove)
      -> retarget node (unmodified, from the vendored inspire_hand-main snapshot
         or a newer checkout given with retarget_source:=)
      -> /inspire_<side>/retarget/joint_states       (hand targets)
      -> rh56f1_glove_teleop.ros_adapter
      -> /rh56f1_<side>/<side>_hand_trajectory_controller/joint_trajectory (only with execute:=true)

It starts neither the hand's controller manager (rh56f1_<side>_hand.launch.py)
nor the arms, and nothing here stops them. Only the retarget node of the
producer package is run: no hand bridge, serial port, haptics or logger.

execute:=false (default) maps and reports on /rh56f1_<side>/glove_adapter/status
without publishing a command. execute:=true commands the fake hand only after
the adapter has checked that the hand manager runs only GenericSystem.
"""

import os
from pathlib import Path

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

VENDORED = Path("robot_control/third_party/inspire_hand_senseglove_teleop/senseglove_teleop")


def _kuku_lab() -> Path | None:
    candidates = []
    if os.environ.get("KUKU_LAB_ROOT"):
        candidates.append(Path(os.environ["KUKU_LAB_ROOT"]))
    candidates += [Path("/workspace/kuku_lab"), Path.home() / "kuku_lab"]
    candidates += list(Path(__file__).resolve().parents)
    return next((c for c in candidates if (c / "robot_control/src").is_dir()), None)


def _setup(context: LaunchContext):
    text = lambda name: context.perform_substitution(LaunchConfiguration(name)).strip()
    side = text("side")
    if side not in ("right", "left"):
        raise ValueError("side must be right or left")
    serial = text("glove_serial")
    if not serial:
        raise ValueError("glove_serial is required: the glove's serial number in its topic name")
    root = _kuku_lab()
    retarget_source = Path(text("retarget_source") or (root / VENDORED if root else ""))
    if not (retarget_source / "senseglove_teleop" / "retarget_node.py").is_file():
        raise FileNotFoundError(f"no senseglove_teleop/retarget_node.py under {retarget_source}")
    retarget_params = Path(text("retarget_params") or retarget_source / "config/mapping.yaml")
    robot_control_src = Path(text("robot_control_src") or (root / "robot_control/src" if root else ""))
    if not (robot_control_src / "rh56f1_glove_teleop").is_dir():
        raise FileNotFoundError(f"no rh56f1_glove_teleop under {robot_control_src}")
    hand = "rh" if side == "right" else "lh"
    path = os.environ.get("PYTHONPATH", "")

    retarget = ExecuteProcess(
        name=f"senseglove_inspire_retarget_{side}",
        cmd=["python3", "-m", "senseglove_teleop.retarget_node", "--ros-args",
             "-r", f"__node:=senseglove_inspire_retarget_{side}",
             "--params-file", str(retarget_params),
             "-p", f"side:={side}",
             "-p", f"input_topic:=/senseglove/glove{serial}/{hand}/joint_states"],
        additional_env={"PYTHONPATH": f"{retarget_source}:{path}"},
        output="screen",
    )
    adapter_cmd = ["python3", "-m", "rh56f1_glove_teleop.ros_adapter", "--side", side]
    if text("thumb_pitch"):
        adapter_cmd += ["--thumb-pitch", text("thumb_pitch")]
    if text("thumb_yaw"):
        adapter_cmd += ["--thumb-yaw", text("thumb_yaw")]
    if text("execute").lower() == "true":
        adapter_cmd.append("--execute")
    adapter = ExecuteProcess(
        name=f"rh56f1_{side}_glove_adapter",
        cmd=adapter_cmd,
        additional_env={"PYTHONPATH": f"{robot_control_src}:{path}"},
        output="screen",
    )
    return [retarget, adapter]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("side", choices=["right", "left"]),
        DeclareLaunchArgument("glove_serial", default_value="",
                              description="Glove serial number in /senseglove/glove<serial>/... (required)."),
        DeclareLaunchArgument("execute", default_value="false", choices=["true", "false"],
                              description="Command the fake hand controller (default: dry run)."),
        DeclareLaunchArgument("thumb_pitch", default_value="", choices=["", "hold", "follow"]),
        DeclareLaunchArgument("thumb_yaw", default_value="", choices=["", "hold", "follow"]),
        DeclareLaunchArgument("retarget_source", default_value="",
                              description="Directory holding the senseglove_teleop Python package "
                                          "(default: the vendored snapshot)."),
        DeclareLaunchArgument("retarget_params", default_value="",
                              description="Retarget calibration yaml (default: <retarget_source>/config/mapping.yaml)."),
        DeclareLaunchArgument("robot_control_src", default_value=""),
        OpaqueFunction(function=_setup),
    ])
