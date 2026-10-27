#!/usr/bin/env python3
"""Jazzy runtime probe: synthetic headset page -> quest_view -> pink IK teleop and neck -> fake OpenArm + LEAP.

    openarm_leap_arms.launch.py                     fake arms + model (Jazzy)
    openarm_quest_view.server --test-pattern        the headset page's server (page :18787, poses TCP :16543)
    openarm_quest_teleop.ros_bridge                 quest_view TCP -> /quest/* topics (incl. /quest/hmd/pose)
    openarm_quest_teleop.ros_teleop  --execute      quest_teleop_leap.yaml: palm r_hl_palm, pink IK,
                                                    first moves the arm to rh56f1_aglt_home
    openarm_head.ros_head  (fake neck)              /quest/hmd/pose -> pan/tilt -> /openarm_leap/head/joint_states
    openarm_quest_teleop.synth --set-front          plays the page: robot front, then
                       --scenario axes              grip, 5 cm forward/back, left/back, up/back, release
                       --scenario head              headset 20 deg left and back, 10 deg up and back

Run from robot_control with ROS 2 Jazzy and ros_ws/install sourced, the pink venv
present, on an otherwise unused ROS_DOMAIN_ID:

    ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
        .venv/bin/python tests/jazzy_leap_quest_teleop_probe.py

Only mock_components/GenericSystem is commanded. Output: CHECK lines, then
LEAP_QUEST_TELEOP_PROBE=PASS or FAIL.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.time import Time

from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "src/openarm_quest_teleop/config/leap_hand/quest_teleop_leap.yaml"
VENV_PYTHON = ROOT / ".venv/bin/python"
PAGE_PORT, POSE_PORT = "18787", "16543"  # not the real 8787 / 65432
QUEST_VIEW_PORTS = ["--port", PAGE_PORT, "--tcp-port", POSE_PORT, "--sync-port", "14200",
                    "--frame-port", "17126", "--head-status-port", "17121"]
#: rh56f1_aglt_home, right arm (poses/openarm_leap.yaml = sim2real rh56f1_aglt.yaml)
HOME = np.array([-1.2127, 0.2026, 0.6538, 1.7608, 0.3791, 0.5785, 0.6646])
FAILED: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"CHECK {name} {'PASS' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
    if not ok:
        FAILED.append(name)
    return ok


class Process:
    def __init__(self, label: str, command: list[str]):
        self.label = label
        self.log_path = Path(tempfile.gettempdir()) / f"leap_quest_probe_{label}.log"
        env = {**os.environ, "PYTHONPATH": f"{ROOT / 'src'}:{os.environ.get('PYTHONPATH', '')}"}
        self.process = subprocess.Popen(command, stdout=open(self.log_path, "w"),
                                        stderr=subprocess.STDOUT, start_new_session=True,
                                        env=env, cwd=ROOT)

    def text(self) -> str:
        return self.log_path.read_text(errors="replace")

    def stop(self) -> bool:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
        try:
            self.process.wait(timeout=20)
            return True
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            return False


def main() -> int:
    if not VENV_PYTHON.is_file():
        print(f"pink venv missing: {VENV_PYTHON} (docs/leap_hand/quest-teleop-leap-pink.md)")
        return 1
    os.environ["OPENARM_RH56F1_LOCK_DIR"] = tempfile.mkdtemp(prefix="leap_quest_locks_")
    py = str(VENV_PYTHON)
    processes = [Process("arms", ["ros2", "launch", "openarm_bringup", "openarm_leap_arms.launch.py"])]
    rclpy.init()
    node = rclpy.create_node("leap_quest_teleop_probe")
    status: dict = {}
    history: list[dict] = []

    def on_status(message):
        status.update(json.loads(message.data))
        history.append(dict(status, t=time.monotonic()))

    node.create_subscription(String, "/quest_teleop/right/status", on_status, 10)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    def palm(frame="r_hl_palm"):
        try:
            t = buffer.lookup_transform("body_root", frame, Time(), RclpyDuration(seconds=0.2))
            return np.array([t.transform.translation.x, t.transform.translation.y,
                             t.transform.translation.z])
        except Exception:
            return None

    def active():
        client = node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=1.0):
            return set()
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        return {c.name for c in future.result().controller if c.state == "active"} \
            if future.result() else set()

    try:
        check("arm_controllers_active", spin_until(
            lambda: {"right_joint_trajectory_controller", "left_joint_trajectory_controller"}
            <= active(), 60.0))
        # The primer holds each arm once (left, then right); a goal sent before the
        # right arm's hold would be preempted by it.
        check("right_arm_primed", spin_until(
            lambda: "right_joint_trajectory_controller: held at its measured position"
            in processes[0].text(), 30.0))
        joints = {}
        node.create_subscription(JointState, "/openarm/joint_states",
                                 lambda m: joints.update(zip(m.name, m.position)), 10)
        spin_until(lambda: "r_aj_1" in joints, 10.0)
        right = [f"r_aj_{i}" for i in range(1, 8)]
        check("arm_starts_at_zero", np.allclose([joints[n] for n in right], 0.0, atol=1e-3))

        processes.append(Process("quest_view", [py, "-m", "openarm_quest_view.server", "--test-pattern",
                                                *QUEST_VIEW_PORTS]))
        processes.append(Process("bridge", [py, "-m", "openarm_quest_teleop.ros_bridge",
                                            "--config", str(CONFIG), "--port", POSE_PORT]))
        processes.append(Process("head", [py, "-m", "openarm_head.ros_head", "--config", str(CONFIG),
                                          "--no-log", "--udp-target", "127.0.0.1:17121"]))
        processes.append(Process("teleop", [py, "-m", "openarm_quest_teleop.ros_teleop",
                                            "--config", str(CONFIG), "--arm", "right",
                                            "--runtime", "fake", "--execute"]))
        check("teleop_started_executing_with_pink_on_the_leap_palm", spin_until(
            lambda: "EXECUTING" in processes[-1].text(), 40.0),
            next((l for l in processes[-1].text().splitlines() if "palm frame" in l), "")[-160:])
        moving_at = time.monotonic() if spin_until(
            lambda: "moving to start pose rh56f1_aglt_home" in processes[-1].text(), 20.0) else None
        arrived = spin_until(lambda: "at start pose rh56f1_aglt_home" in processes[-1].text(), 30.0)
        took = time.monotonic() - moving_at if moving_at else float("nan")
        spin_until(lambda: False, 0.3)
        q_home = np.array([joints[n] for n in right])
        check("teleop_moved_the_arm_to_the_start_pose_first", moving_at is not None and arrived
              and np.allclose(q_home, HOME, atol=0.02) and took >= 1.7608 / 0.3 - 0.5,
              f"took {took:.1f} s, largest error {np.max(np.abs(q_home - HOME)):.4f} rad")
        log = processes[-1].text()
        check("no_following_before_arrival",
              "state engaged" not in log[:log.index("at start pose")] if arrived else False)
        spin_until(lambda: palm() is not None and palm("l_hl_palm") is not None, 10.0)
        start, left_start = palm(), palm("l_hl_palm")

        history.clear()
        synth = subprocess.Popen([py, "-m", "openarm_quest_teleop.synth", "--config", str(CONFIG),
                                  "--port", PAGE_PORT, "--scenario", "axes", "--set-front"],
                                 env={**os.environ, "PYTHONPATH": str(ROOT / "src")}, cwd=ROOT,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        samples = []  # (palm displacement from start) while the synth runs
        while synth.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
            p = palm()
            if p is not None:
                samples.append(p - start)
        spin_until(lambda: False, 1.5)
        displacement = np.array(samples)
        engaged = [h for h in history if h.get("state") == "engaged" and h.get("following")]
        check("engaged_once_and_streamed", status.get("engagements") == 1
              and status.get("commands_sent", 0) > 500,
              f"engagements {status.get('engagements')}, commands {status.get('commands_sent')}")
        ik_pos = max((h["ik_position_error_m"] or 0.0) for h in engaged) if engaged else float("nan")
        ik_rot = max((h["ik_rotation_error_rad"] or 0.0) for h in engaged) if engaged else float("nan")
        check("pink_ik_errors_within_tolerance", engaged and ik_pos <= 1e-3 and ik_rot <= 1e-2,
              f"max {ik_pos * 1000:.3f} mm, {ik_rot:.4f} rad over {len(engaged)} status samples")
        check("no_ik_failure_logged", "IK failed" not in processes[-1].text())
        peaks = displacement.max(axis=0) if len(displacement) else np.zeros(3)
        check("palm_follows_5cm_forward_left_up",
              len(displacement) > 0 and np.allclose(peaks, [0.05, 0.05, 0.05], atol=0.004),
              f"peak displacement x/y/z = {np.round(peaks * 1000, 1)} mm")
        lag = max((np.linalg.norm(np.array(h["target_xyz"]) - np.array(h["palm_xyz"]))
                   for h in engaged if h.get("target_xyz") and h.get("palm_xyz")), default=float("nan"))
        check("palm_tracks_the_target", lag < 0.01, f"max target-palm distance {lag * 1000:.1f} mm")
        end = palm()
        check("palm_returns_to_start", end is not None and np.linalg.norm(end - start) < 0.002,
              f"{np.linalg.norm(end - start) * 1000:.2f} mm" if end is not None else "no tf")
        check("released_to_idle", status.get("state") == "idle", str(status.get("state")))
        left = palm("l_hl_palm")
        check("left_arm_untouched", left is not None and np.linalg.norm(left - left_start) < 1e-6)

        # Neck: the headset turns 20 deg left and 10 deg up; the fake neck follows and the model's
        # joints show it (head_j_pan positive = right, head_j_tilt positive = down).
        head = []
        node.create_subscription(JointState, "/openarm_leap/head/joint_states",
                                 lambda m: head.append(dict(zip(m.name, m.position))), 10)
        check("neck_anchored_on_the_front", spin_until(
            lambda: "Head anchored" in processes[3].text(), 10.0),
            next((l for l in processes[3].text().splitlines() if "anchored" in l), "")[-120:])
        head.clear()
        synth = subprocess.Popen([py, "-m", "openarm_quest_teleop.synth", "--config", str(CONFIG),
                                  "--port", PAGE_PORT, "--scenario", "head"],
                                 env={**os.environ, "PYTHONPATH": str(ROOT / "src")}, cwd=ROOT,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        while synth.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.02)
        spin_until(lambda: False, 1.0)
        pan = np.array([h["head_j_pan"] for h in head]) if head else np.zeros(1)
        tilt = np.array([h["head_j_tilt"] for h in head]) if head else np.zeros(1)
        check("neck_pans_left_with_the_headset", pan.min() < -0.25 and abs(pan[-1]) < 0.03,
              f"head_j_pan min {pan.min():+.3f} rad, end {pan[-1]:+.3f} ({len(head)} samples)")
        check("neck_tilts_up_with_the_headset", tilt.min() < -0.12 and abs(tilt[-1]) < 0.03,
              f"head_j_tilt min {tilt.min():+.3f} rad, end {tilt[-1]:+.3f}")
        check("neck_stays_in_its_window", np.all(np.abs(pan) <= np.radians(20) + 1e-3)
              and np.all(np.abs(tilt) <= np.radians(15) + 1e-3))
    except Exception as error:  # noqa: BLE001
        check("completed", False, repr(error))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            check(f"{process.label}_stopped_by_sigint", process.stop())
    print(f"LEAP_QUEST_TELEOP_PROBE={'FAIL' if FAILED else 'PASS'}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
