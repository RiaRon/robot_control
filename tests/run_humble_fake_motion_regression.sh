#!/usr/bin/env bash
# Fake-only OpenArm + RH56F1 motion regression in an isolated Humble container.
#
# Non-root user, no network, no devices, no capabilities, read-only root and a
# read-only kuku_lab mount. Only openarm_description and openarm_bringup are built, into
# a tmpfs overlay; the image's /root/ros2_ws overlay (which carries the real
# OpenArmHW plugin) is not sourced, so no real hardware plugin is even on the
# plugin path. Commands reach mock_components/GenericSystem only.
#
# Usage: tests/run_humble_fake_motion_regression.sh [probe scenario, default all]
set -euo pipefail

KUKU_LAB="${KUKU_LAB:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
IMAGE="${IMAGE:-thchzh/ros2:openarm-humble}"
DOMAIN="${FAKE_MOTION_ROS_DOMAIN_ID:-231}"
LOG_DIR="${LOG_DIR:-$(mktemp -d -t rh56f1-fake-motion-XXXXXX)}"
SCENARIO="${1:-all}"
NAME="rh56f1-fake-motion-$$"
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

echo "== static tests"
cd /workspace/kuku_lab/robot_control
PYTHONPATH="src:$PYTHONPATH" python3 -m pytest -q -p no:cacheprovider \
  tests/test_openarm_rh56f1_control_description.py 2>&1 | tail -2

echo "== check_urdf"
python3 - <<PY
import subprocess, sys
from pathlib import Path
sys.path.insert(0, "/humble_overlay/install/openarm_bringup/share/openarm_bringup/launch")
from rh56f1_description import HAND_CONFIGURATIONS, RH56F1_STATE_POLICIES, render_control_description, ros2_control_joint_names
share = Path("/humble_overlay/install/openarm_description/share/openarm_description")
for policy in RH56F1_STATE_POLICIES:
    for configuration in HAND_CONFIGURATIONS:
        text = render_control_description(
            source_urdf=Path("/workspace/kuku_lab/urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"),
            wrapper_xacro=share / "urdf/robot/openarm_rh56f1_bimanual.urdf.xacro",
            hand_configuration=configuration, state_policy=policy, use_fake_hardware=True)
        path = Path(f"/tmp/{policy}_{configuration}.urdf"); path.write_text(text)
        run = subprocess.run(["check_urdf", str(path)], capture_output=True, text=True)
        ok = run.returncode == 0 and "Successfully Parsed" in run.stdout
        verdict = "OK" if ok else "FAIL"
        print(f"{policy:17s} {configuration:9s} check_urdf={verdict} resources={len(ros2_control_joint_names(text))}")
        if not ok: sys.exit(1)
PY

echo "== launch arguments"
ros2 launch openarm_bringup openarm.rh56f1_bimanual.launch.py --show-args | grep -A3 -E "rh56f1_state_policy|manifest" | head -12

echo "== runtime probe"
python3 /workspace/kuku_lab/robot_control/tests/humble_fake_hand_motion_probe.py --scenario "$SCENARIO"
' 2>&1 | tee "$LOG_DIR/run.log"
status=${PIPESTATUS[0]}
if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "container $NAME still present"; exit 1
fi
echo "container removed; exit status $status"
exit "$status"
