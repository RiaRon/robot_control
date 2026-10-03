#!/usr/bin/env bash
# Split OpenArm + RH56F1 bringup (arms, right hand, left hand: three controller
# managers) on fake hardware, in an isolated Humble container, followed by the
# Quest teleop regression on the split arms.
#
# Same isolation as run_humble_fake_motion_regression.sh: non-root user, no
# network, no devices, no capabilities, read-only root, read-only kuku_lab
# mount. Only openarm_description and openarm_bringup are built; the image's
# /root/ros2_ws overlay (with the real OpenArmHW plugin) is not sourced.
#
# Usage: tests/run_humble_split_bringup.sh [split|quest|real_double|all]   (default all)
set -euo pipefail

KUKU_LAB="${KUKU_LAB:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
IMAGE="${IMAGE:-thchzh/ros2:openarm-humble}"
DOMAIN="${SPLIT_BRINGUP_ROS_DOMAIN_ID:-226}"
LOG_DIR="${LOG_DIR:-$(mktemp -d -t split-bringup-XXXXXX)}"
WHAT="${1:-all}"
NAME="split-bringup-$$"
mkdir -p "$LOG_DIR"
echo "logs: $LOG_DIR"

docker run --rm --init --name "$NAME" --user "$(id -u):$(id -g)" \
  --network none --cap-drop ALL --security-opt no-new-privileges --read-only \
  --tmpfs /tmp:rw,exec,size=2g,mode=1777 --tmpfs /humble_overlay:rw,exec,size=2g,mode=1777 \
  -v "$KUKU_LAB":/workspace/kuku_lab:ro -v "$LOG_DIR":/probe_logs \
  -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=1 \
  -e HOME=/tmp/home -e ROS_HOME=/tmp/ros -e PYTHONDONTWRITEBYTECODE=1 \
  -e KUKU_LAB_ROOT=/workspace/kuku_lab -e PROBE_LOG_DIR=/probe_logs \
  -e WHAT="$WHAT" \
  --entrypoint bash "$IMAGE" -c '
set -eo pipefail
mkdir -p "$HOME"
source /opt/ros/humble/setup.bash
echo "== isolation"
echo "ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
if env | grep -qi jazzy; then echo "Jazzy path in environment"; exit 1; fi
echo "net: $(ls /sys/class/net | tr "\n" " ")"
devices=$(ls /dev | grep -E "^(tty(USB|ACM|S)|can|serial|bus)" || true)
if [ -n "$devices" ]; then echo "unexpected devices: $devices"; exit 1; fi
echo "serial/CAN/USB devices: none"

echo "== build"
cd /tmp
colcon --log-base /tmp/colcon_log build \
  --base-paths /workspace/kuku_lab/robot_control/ros_ws/src/openarm_description \
               /workspace/kuku_lab/robot_control/ros_ws/src/openarm_ros2/openarm_bringup \
  --packages-select openarm_description openarm_bringup \
  --build-base /humble_overlay/build --install-base /humble_overlay/install \
  2>&1 | tail -1
source /humble_overlay/install/setup.bash
export PYTHONPATH="/workspace/kuku_lab/robot_control/src:$PYTHONPATH"
cd /workspace/kuku_lab/robot_control

echo "== static tests"
python3 -m pytest -q -p no:cacheprovider tests/test_openarm_rh56f1_split_bringup.py \
  tests/test_quest_teleop.py tests/test_openarm_rh56f1_control_description.py \
  tests/test_openarm_rh56f1_real_control_description.py 2>&1 | tail -2

status=0
if [ "$WHAT" = all ] || [ "$WHAT" = split ]; then
  echo "== split bringup probe"
  python3 tests/humble_split_bringup_probe.py || status=1
fi
if [ "$WHAT" = all ] || [ "$WHAT" = quest ]; then
  echo "== Quest teleop probe on the split arms"
  python3 tests/humble_quest_teleop_probe.py || status=1
fi
if [ "$WHAT" = all ] || [ "$WHAT" = real_double ]; then
  echo "== real split arm description with arm test doubles"
  PROBE_LOG_DIR=/probe_logs/real_double python3 tests/humble_quest_teleop_real_double_probe.py || status=1
fi
exit "$status"
' 2>&1 | tee "$LOG_DIR/run.log"
status=${PIPESTATUS[0]}
if docker ps -a --format "{{.Names}}" | grep -qx "$NAME"; then
  echo "container $NAME still present"; exit 1
fi
echo "container removed; exit status $status"
exit "$status"
