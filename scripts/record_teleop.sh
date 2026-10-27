#!/usr/bin/env bash
# Record an OpenArm + LEAP teleop session (fake or real) to bags/<time>_<name> (MCAP).
#
#   scripts/record_teleop.sh [name] [extra ros2 bag record args...]     Ctrl-C stops
#
# Recorded: measured and merged joint states (arms, both LEAP hands, neck), Quest
# inputs (/quest/*), teleop status and targets (/quest_teleop/*), wrist-camera hand
# targets (/leap_hand_tracking/*), every arm/hand trajectory command, TF and the
# model. Camera images are not ROS topics here (quest_view reads the head camera
# itself), so they are not in the bag.
set -euo pipefail
cd "$(dirname "$0")/.."
name="${1:-teleop}"; shift || true
out="bags/$(date +%Y%m%d_%H%M%S)_${name}"
mkdir -p bags
topics='^/(joint_states|openarm/joint_states|openarm_leap/(head/joint_states|display_joint_states|joint_state_sources)'
topics+='|leap_(right|left)/joint_states|quest/.*|quest_teleop/.*|leap_hand_tracking/.*'
topics+='|(right|left)_joint_trajectory_controller/joint_trajectory'
topics+='|leap_(right|left)/(right|left)_hand_trajectory_controller/joint_trajectory'
topics+='|tf|tf_static|robot_description)$'
echo "recording -> $out (Ctrl-C to stop)"
exec ros2 bag record -s mcap -o "$out" -e "$topics" "$@"
