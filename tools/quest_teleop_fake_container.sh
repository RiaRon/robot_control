#!/usr/bin/env bash
# A Humble shell for Quest teleoperation on FAKE hardware only.
#
# No device is passed in (no CAN, serial or USB), all capabilities are dropped,
# the root filesystem and the kuku_lab mount are read-only, and the image's
# /root/ros2_ws overlay (which carries the real OpenArmHW plugin) is not
# sourced. The only thing opened to the outside is UDP port 5006, so a Quest on
# the same network can send controller packets to this PC. This script cannot
# reach a real robot.
#
#   tools/quest_teleop_fake_container.sh              # first terminal: build, then a shell
#   docker exec -it quest-teleop-fake bash --rcfile /tmp/quest_env.sh   # more terminals
#   tools/quest_teleop_fake_container.sh <command>    # run one command and exit
set -euo pipefail

KUKU_LAB="${KUKU_LAB:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
IMAGE="${IMAGE:-thchzh/ros2:openarm-humble}"
NAME="${QUEST_TELEOP_CONTAINER:-quest-teleop-fake}"
PORT="${QUEST_UDP_PORT:-5006}"
gui_args=()
if [ "${QUEST_TELEOP_GUI:-0}" = "1" ]; then
  : "${DISPLAY:?Run from a desktop terminal with DISPLAY set}"
  command -v xauth >/dev/null
  QUEST_RVIZ_AUTH=$(mktemp /tmp/quest-rviz-xauth.XXXXXX)
  trap 'rm -f "$QUEST_RVIZ_AUTH"' EXIT
  xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$QUEST_RVIZ_AUTH" nmerge -
  if [ ! -s "$QUEST_RVIZ_AUTH" ]; then
    echo "No display authentication entry was found" >&2
    exit 1
  fi
  gui_args=(
    -e DISPLAY="$DISPLAY"
    -e XAUTHORITY=/tmp/quest-rviz.xauth
    -e QT_QPA_PLATFORM=xcb
    -e QT_X11_NO_MITSHM=1
    -e LIBGL_ALWAYS_SOFTWARE=1
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro
    -v "$QUEST_RVIZ_AUTH":/tmp/quest-rviz.xauth:ro
  )
fi
tty=()
if [ -t 0 ] && [ -t 1 ]; then tty=(-it); fi

docker run --rm --init "${tty[@]}" "${gui_args[@]}" --name "$NAME" --user "$(id -u):$(id -g)" \
  --cap-drop ALL --security-opt no-new-privileges --read-only \
  -p "$PORT:5006/udp" \
  --tmpfs /tmp:rw,exec,size=2g,mode=1777 --tmpfs /humble_overlay:rw,exec,size=2g,mode=1777 \
  -v "$KUKU_LAB":/workspace/kuku_lab:ro \
  -e ROS_DOMAIN_ID="${QUEST_TELEOP_ROS_DOMAIN_ID:-228}" -e ROS_LOCALHOST_ONLY=1 \
  -e HOME=/tmp/home -e ROS_HOME=/tmp/ros -e PYTHONDONTWRITEBYTECODE=1 \
  -e KUKU_LAB_ROOT=/workspace/kuku_lab \
  --entrypoint bash "$IMAGE" -c '
set -eo pipefail
mkdir -p "$HOME"
source /opt/ros/humble/setup.bash
cd /tmp
colcon --log-base /tmp/colcon_log build \
  --base-paths /workspace/kuku_lab/robot_control/ros_ws/src/openarm_description \
               /workspace/kuku_lab/robot_control/ros_ws/src/openarm_ros2/openarm_bringup \
  --packages-select openarm_description openarm_bringup \
  --build-base /humble_overlay/build --install-base /humble_overlay/install \
  2>&1 | tail -1
cat > /tmp/quest_env.sh <<RC
source /opt/ros/humble/setup.bash
source /humble_overlay/install/setup.bash
export PYTHONPATH="/workspace/kuku_lab/robot_control/src:\$PYTHONPATH"
cd /workspace/kuku_lab/robot_control
PS1="(quest-teleop-fake) \w \$ "
RC
source /tmp/quest_env.sh
if [ "$#" -gt 0 ]; then exec "$@"; fi
echo "fake hardware only: no CAN/serial/USB device is in this container"
exec bash --rcfile /tmp/quest_env.sh
' quest-teleop-fake "$@"
