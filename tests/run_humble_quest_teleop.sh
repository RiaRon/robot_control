#!/usr/bin/env bash
# Quest -> OpenArm arm teleoperation on fake hardware, in an isolated Humble
# container. Input is synthetic: UDP packets in the Quest app's format, sent on
# the container's loopback interface.
#
# Same isolation as run_humble_fake_motion_regression.sh: non-root user, no
# network, no devices, no capabilities, read-only root, read-only kuku_lab
# mount. Only openarm_description and openarm_bringup are built, and the
# image's /root/ros2_ws overlay (which carries the real OpenArmHW plugin) is
# not sourced. Commands reach mock_components/GenericSystem only.
#
# Usage: tests/run_humble_quest_teleop.sh [scenario]
#   all (default)   every fake-bringup scenario, then real_double
#   follow_right | relative_right | left | guards   one fake-bringup scenario
#   real_double     the real description with the arm plugins swapped for
#                   GenericSystem: source joint names and the real controller
set -euo pipefail

KUKU_LAB="${KUKU_LAB:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
IMAGE="${IMAGE:-thchzh/ros2:openarm-humble}"
DOMAIN="${QUEST_TELEOP_ROS_DOMAIN_ID:-230}"
LOG_DIR="${LOG_DIR:-$(mktemp -d -t quest-teleop-XXXXXX)}"
SCENARIO="${1:-all}"
NAME="quest-teleop-$$"
mkdir -p "$LOG_DIR"
echo "logs: $LOG_DIR"

docker run --rm --init --name "$NAME" --user "$(id -u):$(id -g)" \
  --network none --cap-drop ALL --security-opt no-new-privileges --read-only \
  --tmpfs /tmp:rw,exec,size=2g,mode=1777 --tmpfs /humble_overlay:rw,exec,size=2g,mode=1777 \
  -v "$KUKU_LAB":/workspace/kuku_lab:ro -v "$LOG_DIR":/probe_logs \
  -e ROS_DOMAIN_ID="$DOMAIN" -e ROS_LOCALHOST_ONLY=1 \
  -e HOME=/tmp/home -e ROS_HOME=/tmp/ros -e PYTHONDONTWRITEBYTECODE=1 \
  -e KUKU_LAB_ROOT=/workspace/kuku_lab -e PROBE_LOG_DIR=/probe_logs \
  -e SCENARIO="$SCENARIO" \
  --entrypoint bash "$IMAGE" -c '
set -eo pipefail
mkdir -p "$HOME"
source /opt/ros/humble/setup.bash
echo "== isolation"
echo "ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
python3 -c "import sys, numpy; print(\"python\", sys.version.split()[0], \"numpy\", numpy.__version__)"
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
  2>&1 | tail -3
source /humble_overlay/install/setup.bash
echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
export PYTHONPATH="/workspace/kuku_lab/robot_control/src:$PYTHONPATH"

echo "== unit tests"
cd /workspace/kuku_lab/robot_control
python3 -m pytest -q -p no:cacheprovider tests/test_quest_teleop.py 2>&1 | tail -2

echo "== receive-only monitor against synthetic packets"
python3 -m openarm_quest_teleop.monitor --seconds 2.2 --period 1.0 > /tmp/monitor.txt &
monitor=$!
sleep 0.5
python3 -m openarm_quest_teleop.synth --scenario still --seconds 1.5
wait "$monitor"
tail -9 /tmp/monitor.txt

status=0
if [ "$SCENARIO" != "real_double" ]; then
  echo "== runtime probe (fake bringup)"
  python3 tests/humble_quest_teleop_probe.py --scenario "$SCENARIO" || status=1
fi
if [ "$SCENARIO" = "all" ] || [ "$SCENARIO" = "real_double" ]; then
  # The real description, with the arm plugins swapped for GenericSystem. The
  # hand plugin (mock transport) is built so the description loads; it is
  # never configured. openarm_hardware and openarm_can are not built.
  echo "== build rh56f1_hardware (mock transport only is used)"
  cd /tmp
  colcon --log-base /tmp/colcon_log build \
    --base-paths /workspace/kuku_lab/robot_control/ros_ws/src/rh56f1_ros2/rh56f1_hardware \
    --packages-select rh56f1_hardware --cmake-args -DBUILD_TESTING=OFF \
    --build-base /humble_overlay/build --install-base /humble_overlay/install \
    2>&1 | tail -2
  source /humble_overlay/install/setup.bash
  export PYTHONPATH="/workspace/kuku_lab/robot_control/src:$PYTHONPATH"
  cd /workspace/kuku_lab/robot_control
  echo "== runtime probe (real description, arm test doubles)"
  PROBE_LOG_DIR=/probe_logs/real_double python3 tests/humble_quest_teleop_real_double_probe.py || status=1
fi
exit "$status"
' 2>&1 | tee "$LOG_DIR/run.log"
status=${PIPESTATUS[0]}
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "container $NAME still present"; exit 1
fi
echo "container removed; exit status $status"
exit "$status"
