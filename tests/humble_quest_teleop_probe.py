#!/usr/bin/env python3
"""Humble runtime probe: synthetic Quest packets -> fake OpenArm arm teleop.

Run only inside the isolated Humble container (no network, no devices) after
building openarm_description and openarm_bringup into an overlay, with
robot_control/src on PYTHONPATH. Each scenario starts the split fake bringup
(openarm_rh56f1_arms.launch.py with the integrated model, and each hand under
its own controller manager), bends both fake arms with the existing trajectory
action, then starts the real bridge and teleop processes and feeds them UDP
packets on the loopback interface, in the Quest app's format. The teleop reads
the arm joints from /openarm/joint_states and the arm manager's own
robot_description parameter (runtime ``fake``).

  follow_right    input path, enable, six directions, orientation hold,
                  release / re-enable, input loss, malformed packets, IK
                  failure, and that nothing but the right arm is commanded
  relative_right  relative-rotation orientation mode
  left            the same code driving the left arm
  guards          dry run, stale joint state, enabling at the straight arm
                  (not blocked since min_enable_singular_value is 0.0), and the
                  real-hardware confirmation (description text only; no real
                  plugin is ever loaded)

Before any goal is sent the probe requires that the only hardware component is
mock_components/GenericSystem. Output: CHECK lines, then
QUEST_TELEOP_PROBE=PASS or FAIL.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, str(Path(__file__).resolve().parent))
import humble_fake_hand_motion_probe as base  # noqa: E402
import humble_split_bringup_probe as split_probe  # noqa: E402

from openarm_quest_teleop.config import DEFAULT_CONFIG, load_config  # noqa: E402
from openarm_quest_teleop.packet import ControllerPose, ros_to_unity_pose  # noqa: E402
from openarm_quest_teleop.synth import QuestSender, axis_rotation  # noqa: E402

check = base.check
LOG_ROOT = base.LOG_ROOT
PORT = 5006
ARM = {"right": base.GROUPS["right_arm"], "left": base.GROUPS["left_arm"]}
HANDS = base.GROUPS["right_hand"] + base.GROUPS["left_hand"]
PALM = {"right": "r_hl_palm_sensor", "left": "l_hl_palm_sensor"}
COMMAND_TOPIC = {side: f"/{base.CONTROLLER[f'{side}_arm']}/joint_trajectory"
                 for side in ("right", "left")}
#: A bent, non-singular fake start pose (all zeros is the arm hanging straight
#: down, a singular pose a local IK step barely leaves). Fake-only values.
BENT = {"right": [0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0],
        "left": [-0.3, -0.15, 0.0, 1.2, 0.0, 0.0, 0.0]}

POSITION_TOL = 2e-3   # palm displacement vs commanded controller displacement
ROTATION_TOL = 2e-2
STILL = 1e-6          # "did not move"


class Process:
    """One of our own nodes, in its own process group, logging to a file."""

    def __init__(self, name: str, module: str, *args: str):
        self.name = name
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_ROOT / f"{name}.log"
        self.log = open(self.log_path, "w")
        self.process = subprocess.Popen(
            [sys.executable, "-u", "-m", module, *args],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def wait(self, timeout: float) -> int | None:
        try:
            return self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def stop(self) -> bool:
        """SIGINT; True if it exited without escalation."""
        clean = True
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                clean = False
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()
        return clean

    def text(self) -> str:
        self.log.flush()
        return self.log_path.read_text()


class TeleopProbe(base.Probe):
    def __init__(self, name):
        super().__init__(name)
        self.status = {}          # arm -> latest teleop status dict
        self.bridge_status = None
        self.quest_pose = {}
        self.quest_joy = {}
        self.commands = {"right": [], "left": []}
        self.joint_watch = None   # (baseline, names, worst)
        self.joint_q = {}
        for arm in ("right", "left"):
            self.create_subscription(
                String, f"/quest_teleop/{arm}/status",
                lambda m, arm=arm: self.status.__setitem__(arm, json.loads(m.data)), 10)
            self.create_subscription(
                PoseStamped, f"/quest/{arm}/pose",
                lambda m, arm=arm: self.quest_pose.__setitem__(arm, m), 10)
            self.create_subscription(
                Joy, f"/quest/{arm}/joy",
                lambda m, arm=arm: self.quest_joy.__setitem__(arm, m), 10)
            self.create_subscription(
                JointTrajectory, COMMAND_TOPIC[arm],
                lambda m, arm=arm: self.commands[arm].append(m), 100)
        self.bridge_rate_max = 0.0
        self.create_subscription(String, "/quest/status", self._bridge_status, 10)

    def controllers(self, device):
        """Controllers of one split-bringup manager, or None if it does not answer."""
        try:
            reply = self.call(ListControllers,
                              f"{split_probe.MANAGER[device]}/list_controllers", timeout=5.0)
        except AssertionError:
            return None
        return None if reply is None else {c.name: c for c in reply.controller}

    def _bridge_status(self, message):
        self.bridge_status = json.loads(message.data)
        self.bridge_rate_max = max(self.bridge_rate_max, self.bridge_status["rate_hz"])

    def _state(self, message):
        # /joint_states carries one device per message in the split bringup:
        # keep the latest value of every joint, by name.
        self.latest = message
        self.joint_q.update(zip(message.name, message.position))
        if self.joint_watch is not None:
            baseline, names, worst = self.joint_watch
            positions = dict(zip(message.name, message.position))
            for name in names:
                if name in positions:
                    worst[name] = max(worst.get(name, 0.0),
                                      abs(positions[name] - baseline[name]))

    def state(self):
        self.spin_until(lambda: set(base.ORDER) <= set(self.joint_q), label="all 26 joints")
        return dict(self.joint_q)

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, action) -> None:
        """Run a sender action in a thread while this node keeps spinning."""
        thread = threading.Thread(target=action)
        thread.start()
        while thread.is_alive():
            rclpy.spin_once(self, timeout_sec=0.02)
        thread.join()

    def wait_status(self, arm, predicate, timeout=5.0, label="status"):
        try:
            self.spin_until(lambda: arm in self.status and predicate(self.status[arm]),
                            timeout=timeout, label=label)
            return True
        except AssertionError:
            return False

    def palm(self, arm):
        """Palm pose from the joint states by offline FK, checked against TF."""
        return self.kinematics.world(PALM[arm], self.state())

    def palm_tf_matches_fk(self, arm) -> tuple[bool, float]:
        worst = [0.0]

        def matched():
            error = base._pose_error(self.palm(arm), self.tf("body_root", PALM[arm]))
            worst[0] = max(error)
            return worst[0] < base.FK_TOL

        try:
            self.spin_until(matched, timeout=5.0, label="TF == FK")
            return True, worst[0]
        except AssertionError:
            return False, worst[0]


class KeepAliveSender(QuestSender):
    """QuestSender that keeps streaming between scripted actions, like a headset.

    A real Quest sends continuously; the scripted actions (hold, move) send
    their own packets, and between them this thread repeats the current state
    at the same rate, so a slow probe step is not mistaken for input loss.
    ``silence`` and ``quiet`` stop it, for the tests that need no valid input.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._lock = threading.RLock()
        self._last = 0.0
        self._quiet_until = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._keepalive, daemon=True)
        self._thread.start()

    def send(self):
        with self._lock:
            super().send()
            self._last = time.monotonic()

    def quiet(self, seconds):
        self._quiet_until = time.monotonic() + seconds

    def silence(self, seconds):
        self.quiet(seconds)
        time.sleep(seconds)
        with self._lock:
            self.headset_time += seconds

    def _keepalive(self):
        while self._running:
            now = time.monotonic()
            if now >= self._quiet_until and now - self._last >= self.period:
                self.send()
            time.sleep(self.period / 3)

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)
        super().close()


def _moved(before, after):
    """(translation vector, rotation angle) from one 4x4 pose to another."""
    relative = before[:3, :3].T @ after[:3, :3]
    cosine = max(-1.0, min(1.0, (np.trace(relative) - 1.0) / 2.0))
    return after[:3, 3] - before[:3, 3], math.acos(cosine)


def arm_manager_description(node):
    """The arm controller manager's own description (hardware blocks included)."""
    from rcl_interfaces.srv import GetParameters

    reply = node.call(GetParameters, "/controller_manager/get_parameters",
                      GetParameters.Request(names=["robot_description"]))
    return reply.values[0].string_value


def start_stack(tag):
    """Split fake bringup, both arms bent, ready for teleop. Returns (launches, node)."""
    launches = [
        split_probe.Launch(f"{tag}.arms", "openarm_rh56f1_arms.launch.py", "runtime:=fake"),
        split_probe.Launch(f"{tag}.right_hand", "rh56f1_right_hand.launch.py"),
        split_probe.Launch(f"{tag}.left_hand", "rh56f1_left_hand.launch.py"),
    ]
    node = TeleopProbe(f"quest_teleop_probe_{tag}")
    try:
        return launches, _prepare(tag, node)
    except Exception:
        split_probe.finish(tag, node, launches)
        raise


def _prepare(tag, node):
    controllers = {"joint_state_broadcaster", base.CONTROLLER["left_arm"],
                   base.CONTROLLER["right_arm"]}
    split_probe.wait_controllers(node, "arms", controllers)
    for side in ("right", "left"):
        split_probe.wait_controllers(
            node, side, {"joint_state_broadcaster", f"{side}_hand_trajectory_controller"})
    node.spin_until(lambda: node.description is not None, label="/robot_description")
    node.kinematics = base.Kinematics(node.description)
    classes = []
    for device in ("arms", "right", "left"):
        reply = node.call(ListHardwareComponents,
                          f"{split_probe.MANAGER[device]}/list_hardware_components")
        classes += [c.class_type for c in reply.component]
    check(f"{tag}.only_generic_system", classes == ["mock_components/GenericSystem"] * 3,
          str(classes))
    if classes != ["mock_components/GenericSystem"] * 3:
        raise AssertionError("a non-GenericSystem hardware component is present")
    codes, _, _ = node.send({"right_arm": BENT["right"], "left_arm": BENT["left"]}, seconds=2.0)
    expected = {name: 0.0 for name in base.ORDER}
    expected.update(zip(ARM["right"], BENT["right"]))
    expected.update(zip(ARM["left"], BENT["left"]))
    node.spin_until(
        lambda: all(abs(node.state()[n] - expected[n]) < 1e-6 for n in base.ORDER),
        timeout=10.0, label="bent start pose")
    node.spin_for(0.5)
    node.commands = {"right": [], "left": []}
    check(f"{tag}.fake_start_pose_bent", all(c == 0 for c in codes.values()),
          f"right {BENT['right']}")
    return node


def start_teleop(node, tag, arm, *extra, execute=True, expect_running=True):
    args = ["--arm", arm, "--runtime", "fake", *extra]
    if execute:
        args.append("--execute")
    process = Process(f"{tag}.teleop_{arm}", "openarm_quest_teleop.ros_teleop", *args)
    if expect_running:
        ok = node.wait_status(arm, lambda s: s["executing"] is execute, timeout=20.0)
        check(f"{tag}.teleop_{arm}_running", ok and process.process.poll() is None,
              f"executing={execute}")
    return process


def finish(tag, launches, node, processes):
    for process in processes:
        clean = process.stop()
        check(f"{tag}.{process.name.split('.')[-1]}_stopped_by_sigint", clean)
    split_probe.finish(tag, node, launches)
    ours = subprocess.run(["ps", "-eo", "args="], capture_output=True, text=True).stdout
    left = [line for line in ours.splitlines() if "openarm_quest_teleop.ros_" in line]
    check(f"{tag}.no_teleop_process_left", not left, str(left))


# ---------------------------------------------------------------- scenarios
def scenario_follow_right():
    tag = "follow_right"
    launches, node = start_stack(tag)
    processes = []
    sender = KeepAliveSender("127.0.0.1", PORT)
    arm = "right"
    try:
        processes.append(Process(f"{tag}.bridge", "openarm_quest_teleop.ros_bridge"))
        teleop = start_teleop(node, tag, arm)
        processes.append(teleop)
        others = [n for n in base.ORDER if n not in ARM[arm]]
        node.joint_watch = (dict(node.state()), others, {})

        # --- input path: UDP -> PoseStamped + Joy ---------------------------
        node.drive(lambda: sender.hold(2.5))
        node.spin_until(lambda: arm in node.quest_pose and arm in node.quest_joy
                        and node.bridge_status is not None, label="quest topics")
        sender.quiet(1.0)   # stop the stream, so the last pose and Joy are one packet's
        node.spin_for(0.3)  # let that packet's pose and Joy both arrive
        pose, joy = node.quest_pose[arm], node.quest_joy[arm]
        got = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        check(f"{tag}.pose_topic_in_quest_world",
              pose.header.frame_id == "quest_world"
              and np.allclose(got, sender.position[arm], atol=1e-6),
              f"/quest/right/pose {np.round(got, 3).tolist()}")
        check(f"{tag}.joy_topic_layout",
              len(joy.axes) == 4 and list(joy.buttons) == [0, 0, 1]
              and joy.header.stamp == pose.header.stamp,
              f"axes {list(joy.axes)} buttons {list(joy.buttons)}")
        age = (time.time_ns() - (pose.header.stamp.sec * 10**9 + pose.header.stamp.nanosec)) * 1e-9
        check(f"{tag}.stamp_is_pc_receive_time", 0.0 <= age < 2.0, f"age {age:.3f} s")
        check(f"{tag}.receive_rate_reported", node.bridge_rate_max > 60.0,
              f"{node.bridge_rate_max} Hz (sent at 90), "
              f"{node.bridge_status['packets_published']} packets")
        check(f"{tag}.idle_sends_nothing",
              node.status[arm]["state"] == "idle" and not node.commands[arm],
              f"hardware {node.status[arm]['hardware']}")

        # --- enable: the target is where the palm already is ----------------
        before_q = dict(node.state())
        before_palm = node.palm(arm)
        sender.grip[arm] = 1.0
        node.drive(lambda: sender.hold(1.0))
        ok = node.wait_status(arm, lambda s: s["state"] == "engaged" and s["following"])
        status = node.status[arm]
        check(f"{tag}.enable_engages", ok and status["engagements"] == 1, str(status["state"]))
        gap = np.linalg.norm(np.array(status["target_xyz"]) - before_palm[:3, 3])
        check(f"{tag}.enable_target_is_current_palm", gap < STILL, f"|target - palm| {gap:.1e} m")
        drift = max(abs(node.state()[n] - before_q[n]) for n in base.ORDER)
        check(f"{tag}.enable_moves_nothing", drift < 1e-9 and len(node.commands[arm]) > 10,
              f"max joint change {drift:.1e} rad over {len(node.commands[arm])} commands")

        # --- six directions, orientation held -------------------------------
        for label, delta in (("forward", (0.05, 0, 0)), ("back", (-0.05, 0, 0)),
                             ("left", (0, 0.05, 0)), ("right", (0, -0.05, 0)),
                             ("up", (0, 0, 0.05)), ("down", (0, 0, -0.05))):
            start = node.palm(arm)
            node.drive(lambda: (sender.move(arm, delta, seconds=1.0), sender.hold(0.8)))
            moved, turned = _moved(start, node.palm(arm))
            error = np.linalg.norm(moved - np.array(delta))
            check(f"{tag}.move_{label}", error < POSITION_TOL and turned < ROTATION_TOL,
                  f"palm moved {np.round(moved, 4).tolist()} m, turned {turned:.4f} rad")
        ok, worst = node.palm_tf_matches_fk(arm)
        check(f"{tag}.palm_tf_matches_joint_state_fk", ok, f"max {worst:.1e}")

        # --- orientation hold: turning the controller does not turn the palm -
        start = node.palm(arm)
        node.drive(lambda: (sender.move(arm, seconds=1.0, axis=(0, 0, 1), angle=0.3),
                            sender.hold(0.5)))
        moved, turned = _moved(start, node.palm(arm))
        check(f"{tag}.orientation_hold", np.linalg.norm(moved) < POSITION_TOL
              and turned < ROTATION_TOL, f"turned {turned:.4f} rad for a 0.3 rad controller turn")

        # --- release holds; re-enable starts from the current pose ----------
        sender.grip[arm] = 0.0
        node.drive(lambda: sender.hold(0.5))
        check(f"{tag}.release_goes_idle", node.status[arm]["state"] == "idle")
        held, sent = node.palm(arm), len(node.commands[arm])
        node.drive(lambda: (sender.move(arm, (0.0, 0.08, 0.0), seconds=0.8), sender.hold(0.3)))
        moved, _ = _moved(held, node.palm(arm))
        check(f"{tag}.released_motion_is_ignored",
              np.linalg.norm(moved) < STILL and len(node.commands[arm]) == sent,
              f"palm moved {np.linalg.norm(moved):.1e} m, {len(node.commands[arm]) - sent} commands")
        sender.grip[arm] = 1.0
        node.drive(lambda: sender.hold(0.8))
        moved, _ = _moved(held, node.palm(arm))
        check(f"{tag}.reenable_does_not_jump",
              node.status[arm]["state"] == "engaged" and node.status[arm]["engagements"] == 2
              and np.linalg.norm(moved) < STILL, f"palm moved {np.linalg.norm(moved):.1e} m")
        node.drive(lambda: (sender.move(arm, (0, 0, 0.03), seconds=0.8), sender.hold(0.8)))
        moved, _ = _moved(held, node.palm(arm))
        check(f"{tag}.reenable_follows_from_new_start",
              np.linalg.norm(moved - np.array((0, 0, 0.03))) < POSITION_TOL,
              f"palm moved {np.round(moved, 4).tolist()} m")

        # --- input loss: locks, and does not resume by itself ----------------
        held, sent = node.palm(arm), len(node.commands[arm])
        node.drive(lambda: sender.silence(0.8))
        ok = node.wait_status(arm, lambda s: s["state"] == "locked")
        check(f"{tag}.input_loss_locks", ok and "stale" in node.status[arm]["reason"],
              str(node.status[arm]["reason"]))
        sent = len(node.commands[arm])
        node.drive(lambda: (sender.move(arm, (0.04, 0, 0), seconds=0.6), sender.hold(0.6)))
        moved, _ = _moved(held, node.palm(arm))
        check(f"{tag}.no_automatic_resume",
              node.status[arm]["state"] == "locked" and len(node.commands[arm]) == sent
              and np.linalg.norm(moved) < STILL
              and "release and press" in node.status[arm]["reason"],
              f"{len(node.commands[arm]) - sent} commands, palm moved {np.linalg.norm(moved):.1e} m")
        sender.grip[arm] = 0.0
        node.drive(lambda: sender.hold(0.4))
        sender.grip[arm] = 1.0
        node.drive(lambda: sender.hold(0.6))
        check(f"{tag}.reenable_after_loss",
              node.status[arm]["state"] == "engaged" and node.status[arm]["engagements"] == 3)

        # --- malformed packets are dropped; only they arriving is a loss ----
        held = node.palm(arm)
        before_status = dict(node.bridge_status)
        bad_quaternion = sender.packet()
        bad_quaternion["rc"] = {**bad_quaternion["rc"], "qw": 3.0, "x": 5.0}

        def mixed():
            for _ in range(45):
                sender.send_raw(b"\x00\xffnot json")
                sender.send_raw(b'{"t": NaN, "rc": {"x": Infinity}}')
                sender.send()
                time.sleep(sender.period)
            sender.hold(1.3)  # long enough for the bridge's next status

        node.drive(mixed)
        moved, _ = _moved(held, node.palm(arm))
        dropped = (node.bridge_status["datagrams_malformed"]
                   - before_status["datagrams_malformed"])
        check(f"{tag}.malformed_datagrams_dropped_valid_ones_still_followed",
              dropped == 90 and node.status[arm]["state"] == "engaged"
              and np.linalg.norm(moved) < STILL,
              f"{dropped} dropped, state {node.status[arm]['state']}, "
              f"palm moved {np.linalg.norm(moved):.1e} m")

        def only_bad():
            sender.quiet(70 * sender.period + 0.1)
            for index in range(70):
                bad = sender.packet()
                bad["t"] = sender.headset_time + index * sender.period
                bad["rc"] = {**bad["rc"], "x": float(index), "qw": 2.0}
                sender.send_raw(json.dumps(bad).encode())
                time.sleep(sender.period)
            sender.headset_time += 1.0

        node.drive(only_bad)
        ok = node.wait_status(arm, lambda s: s["state"] == "locked")
        node.drive(lambda: sender.hold(1.3))
        moved, _ = _moved(held, node.palm(arm))
        refused = node.bridge_status["packets_refused"] - before_status["packets_refused"]
        check(f"{tag}.only_invalid_packets_is_input_loss",
              ok and node.status[arm]["state"] == "locked" and refused > 30
              and "quaternion" in str(node.bridge_status["last_refusal"])
              and np.linalg.norm(moved) < STILL,
              f"{refused} packets refused ({node.bridge_status['last_refusal']}); "
              f"teleop: {node.status[arm]['reason']}")

        # --- IK failure: an unreachable target blocks commands, then recovers
        sender.grip[arm] = 0.0
        node.drive(lambda: sender.hold(0.4))
        sender.grip[arm] = 1.0
        node.drive(lambda: sender.hold(0.6))
        node.drive(lambda: (sender.move(arm, (0.39, 0, 0), seconds=4.0), sender.hold(0.5)))
        ok = node.wait_status(arm, lambda s: s["reason"] and s["reason"].startswith("IK failed"))
        sent, stuck = len(node.commands[arm]), node.palm(arm)
        node.drive(lambda: sender.hold(0.6))
        moved, _ = _moved(stuck, node.palm(arm))
        check(f"{tag}.unreachable_target_blocks_commands",
              ok and node.status[arm]["state"] == "engaged"
              and len(node.commands[arm]) == sent and np.linalg.norm(moved) < STILL,
              str(node.status[arm]["reason"]))
        node.drive(lambda: (sender.move(arm, (-0.39, 0, 0), seconds=4.0), sender.hold(1.5)))
        check(f"{tag}.recovers_when_reachable_again",
              node.status[arm]["following"] and node.status[arm]["reason"] is None,
              str(node.status[arm]["reason"]))

        # --- nothing but the right arm was ever commanded -------------------
        worst = max(node.joint_watch[2].values())
        check(f"{tag}.left_arm_and_hands_never_moved", worst == 0.0,
              f"{len(others)} joints (left arm 7 + hands 12), max excursion {worst:.1e} rad")
        names = {tuple(m.joint_names) for m in node.commands[arm]}
        check(f"{tag}.commands_name_only_the_right_arm", names == {tuple(ARM[arm])},
              f"{len(node.commands[arm])} messages on {COMMAND_TOPIC[arm]}")
        check(f"{tag}.left_arm_topic_untouched",
              not node.commands["left"] and node.count_publishers(COMMAND_TOPIC["left"]) == 0,
              f"{len(node.commands['left'])} messages, "
              f"{node.count_publishers(COMMAND_TOPIC['left'])} publishers")
        listed = node.call(ListControllers, "/controller_manager/list_controllers").controller
        check(f"{tag}.arm_manager_has_no_hand_controller",
              sorted(c.name for c in listed) == sorted(
                  ["joint_state_broadcaster", base.CONTROLLER["left_arm"],
                   base.CONTROLLER["right_arm"]]),
              str(sorted(c.name for c in listed)))
        hand_topics = [f"/rh56f1_{side}/{side}_hand_trajectory_controller/joint_trajectory"
                       for side in ("right", "left")]
        check(f"{tag}.hand_managers_never_commanded",
              all(node.count_publishers(t) == 0 for t in hand_topics),
              str({t: node.count_publishers(t) for t in hand_topics}))
        hands = max(abs(node.state()[n]) for n in HANDS)
        check(f"{tag}.hands_still_at_start", hands == 0.0, f"12 actuators, max |q| {hands}")
        log = teleop.text()
        check(f"{tag}.teleop_uses_split_endpoints",
              "description from /controller_manager parameter robot_description" in log
              and "joint states from /openarm/joint_states" in log)
    finally:
        sender.close()
        finish(tag, launches, node, processes)


def scenario_relative_right():
    tag = "relative_right"
    launches, node = start_stack(tag)
    processes = []
    sender = KeepAliveSender("127.0.0.1", PORT)
    arm = "right"
    try:
        processes.append(Process(f"{tag}.bridge", "openarm_quest_teleop.ros_bridge"))
        processes.append(start_teleop(node, tag, arm, "--orientation-mode", "relative"))
        # The controller starts at an arbitrary attitude, unlike the palm's.
        sender.rotation[arm] = axis_rotation((1, 2, 3), 0.7)
        node.drive(lambda: sender.hold(1.0))
        sender.grip[arm] = 1.0
        node.drive(lambda: sender.hold(0.8))
        check(f"{tag}.engaged", node.status[arm]["state"] == "engaged")
        for label, axis, angle in (("yaw_left_about_base_z", (0, 0, 1), 0.2),
                                   ("roll_about_base_x", (1, 0, 0), 0.15)):
            start = node.palm(arm)
            node.drive(lambda: (sender.move(arm, seconds=1.5, axis=axis, angle=angle),
                                sender.hold(1.0)))
            palm = node.palm(arm)
            expected = axis_rotation(axis, angle) @ start[:3, :3]
            _, error = _moved(np.vstack([np.c_[expected, [0, 0, 0]], [0, 0, 0, 1]]), palm)
            _, turned = _moved(start, palm)
            drift = np.linalg.norm(palm[:3, 3] - start[:3, 3])
            check(f"{tag}.{label}", error < ROTATION_TOL and drift < POSITION_TOL,
                  f"palm turned {turned:.4f} rad (asked {angle}), axis error {error:.4f} rad, "
                  f"position drift {drift * 1000:.2f} mm")
        start = node.palm(arm)
        node.drive(lambda: (sender.move(arm, (0.03, 0, 0.03), seconds=1.0, axis=(0, 0, 1),
                                        angle=-0.1), sender.hold(1.0)))
        palm = node.palm(arm)
        expected = axis_rotation((0, 0, 1), -0.1) @ start[:3, :3]
        _, error = _moved(np.vstack([np.c_[expected, [0, 0, 0]], [0, 0, 0, 1]]), palm)
        offset = np.linalg.norm(palm[:3, 3] - start[:3, 3] - np.array((0.03, 0, 0.03)))
        check(f"{tag}.position_and_rotation_together",
              error < ROTATION_TOL and offset < POSITION_TOL,
              f"position error {offset * 1000:.2f} mm, rotation error {error:.4f} rad")
        ok, worst = node.palm_tf_matches_fk(arm)
        check(f"{tag}.palm_tf_matches_joint_state_fk", ok, f"max {worst:.1e}")
    finally:
        sender.close()
        finish(tag, launches, node, processes)


def scenario_left():
    tag = "left"
    launches, node = start_stack(tag)
    processes = []
    sender = KeepAliveSender("127.0.0.1", PORT)
    arm = "left"
    try:
        processes.append(Process(f"{tag}.bridge", "openarm_quest_teleop.ros_bridge"))
        processes.append(start_teleop(node, tag, arm))
        others = [n for n in base.ORDER if n not in ARM[arm]]
        node.joint_watch = (dict(node.state()), others, {})
        node.drive(lambda: sender.hold(1.0))
        # The right controller's grip must not enable the left arm.
        sender.grip["right"] = 1.0
        node.drive(lambda: sender.hold(0.6))
        check(f"{tag}.other_controllers_grip_is_ignored",
              node.status[arm]["state"] == "idle" and not node.commands[arm])
        sender.grip["right"] = 0.0
        sender.grip[arm] = 1.0
        node.drive(lambda: sender.hold(0.8))
        check(f"{tag}.engaged", node.status[arm]["state"] == "engaged")
        for label, delta in (("forward", (0.04, 0, 0)), ("left", (0, 0.04, 0)),
                             ("up", (0, 0, 0.04))):
            start = node.palm(arm)
            node.drive(lambda: (sender.move(arm, delta, seconds=1.0), sender.hold(0.8)))
            moved, turned = _moved(start, node.palm(arm))
            check(f"{tag}.move_{label}",
                  np.linalg.norm(moved - np.array(delta)) < POSITION_TOL
                  and turned < ROTATION_TOL,
                  f"palm moved {np.round(moved, 4).tolist()} m")
        worst = max(node.joint_watch[2].values())
        check(f"{tag}.right_arm_and_hands_never_moved", worst == 0.0,
              f"{len(others)} joints, max excursion {worst:.1e} rad")
        check(f"{tag}.right_arm_topic_untouched",
              not node.commands["right"] and node.count_publishers(COMMAND_TOPIC["right"]) == 0)
        check(f"{tag}.commands_name_only_the_left_arm",
              {tuple(m.joint_names) for m in node.commands[arm]} == {tuple(ARM[arm])},
              f"{len(node.commands[arm])} messages")
    finally:
        sender.close()
        finish(tag, launches, node, processes)


def scenario_guards():
    tag = "guards"
    launches, node = start_stack(tag)
    processes = []
    sender = KeepAliveSender("127.0.0.1", PORT)
    arm = "right"
    try:
        processes.append(Process(f"{tag}.bridge", "openarm_quest_teleop.ros_bridge"))

        # --- dry run: the whole pipeline runs, nothing reaches the robot -----
        dry = start_teleop(node, f"{tag}.dry", arm, execute=False)
        node.drive(lambda: sender.hold(0.8))
        sender.grip[arm] = 1.0
        held = node.palm(arm)
        node.drive(lambda: (sender.hold(0.5), sender.move(arm, (0, 0, 0.04), seconds=0.8),
                            sender.hold(0.5)))
        status = node.status[arm]
        moved, _ = _moved(held, node.palm(arm))
        check(f"{tag}.dry_run_computes_but_publishes_nothing",
              status["state"] == "engaged" and status["executing"] is False
              and status["commands_sent"] == 0 and not node.commands[arm]
              and node.count_publishers(COMMAND_TOPIC[arm]) == 0
              and np.linalg.norm(moved) < STILL,
              f"state {status['state']}, target {np.round(status['target_xyz'], 3).tolist()}, "
              f"{node.count_publishers(COMMAND_TOPIC[arm])} command publishers")
        check(f"{tag}.dry_teleop_stopped_by_sigint", dry.stop())
        sender.grip[arm] = 0.0
        node.status.clear()

        # --- stale joint state: no joint topic, so nothing is ever sent ------
        stale = start_teleop(node, f"{tag}.stale", arm, "--joint-states-topic",
                             "/quest_probe/no_such_joint_states")
        node.drive(lambda: sender.hold(0.6))
        sender.grip[arm] = 1.0
        node.drive(lambda: (sender.hold(0.5), sender.move(arm, (0, 0, 0.04), seconds=0.8)))
        status = node.status[arm]
        check(f"{tag}.no_joint_state_blocks_enable",
              status["state"] == "idle" and status["reason"] == "no joint state"
              and status["commands_sent"] == 0 and not node.commands[arm],
              str(status["reason"]))
        check(f"{tag}.stale_teleop_stopped_by_sigint", stale.stop())
        sender.grip[arm] = 0.0
        node.status.clear()

        # --- the straight arm: enabling is not blocked (user decision,
        # min_enable_singular_value 0.0), and it starts without a jump ------
        codes, _, _ = node.send({"right_arm": [0.0] * 7}, seconds=2.0)
        node.spin_until(lambda: all(abs(node.state()[n]) < 1e-6 for n in ARM[arm]),
                        timeout=10.0, label="straight arm")
        node.spin_for(0.5)
        node.commands[arm].clear()
        straight = start_teleop(node, f"{tag}.straight", arm)
        node.drive(lambda: sender.hold(0.6))
        sender.grip[arm] = 1.0
        node.drive(lambda: sender.hold(0.6))
        status = node.status[arm]
        first = [list(m.points[0].positions) for m in node.commands[arm][:20]]
        drift = max(abs(node.state()[n]) for n in ARM[arm])
        check(f"{tag}.straight_arm_enable_not_blocked",
              status["state"] == "engaged" and "singular" not in str(status["reason"])
              and first and all(p == [0.0] * 7 for p in first) and drift < 1e-9,
              f"state {status['state']}, {len(node.commands[arm])} commands, first all zero")
        check(f"{tag}.straight_teleop_stopped_by_sigint", straight.stop())
        sender.grip[arm] = 0.0
        node.commands = {"right": [], "left": []}

        # --- real hardware: refused without its own confirmation ------------
        # Only the description *text* names a real plugin; nothing loads it.
        real_text = arm_manager_description(node).replace(
            "mock_components/GenericSystem", "openarm_hardware/OpenArmHW")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        topic = "/quest_probe/real_looking_description"
        publisher = node.create_publisher(String, topic, latched)
        publisher.publish(String(data=real_text))
        config = load_config(DEFAULT_CONFIG)
        config["teleop"]["robot_description_topic"] = topic
        # No manager answers here, so the teleop falls back to the topic above.
        for runtime in config["teleop"]["runtimes"].values():
            runtime["controller_manager"] = "/quest_probe_no_manager"
        config_path = LOG_ROOT / f"{tag}.real_looking.yaml"
        config_path.write_text(yaml.safe_dump(config))

        def refused(label, *flags):
            process = Process(f"{tag}.{label}", "openarm_quest_teleop.ros_teleop",
                              "--config", str(config_path), "--arm", arm, *flags)
            deadline = time.monotonic() + 40
            while process.process.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            code = process.process.poll()
            text = process.text()
            process.stop()
            return code, text

        code, text = refused("real_execute_only", "--runtime", "real", "--execute")
        check(f"{tag}.real_hardware_execute_alone_refused",
              code == 2 and "this is real hardware" in text and "--confirm-real-hardware" in text,
              f"exit {code}")
        code, text = refused("real_as_fake", "--runtime", "fake", "--execute",
                             "--confirm-real-hardware")
        check(f"{tag}.real_hardware_as_fake_runtime_refused",
              code == 2 and "use --runtime real" in text, f"exit {code}")
        code, text = refused("real_confirmed_no_controller", "--runtime", "real", "--execute",
                             "--confirm-real-hardware")
        check(f"{tag}.confirmed_real_still_needs_its_active_controller",
              code == 2 and "rh56f1_right_arm_controller is not loaded" in text, f"exit {code}")
        check(f"{tag}.guards_sent_no_command", not node.commands["right"]
              and not node.commands["left"])
    finally:
        sender.close()
        finish(tag, launches, node, processes)


SCENARIOS = {
    "follow_right": scenario_follow_right,
    "relative_right": scenario_relative_right,
    "left": scenario_left,
    "guards": scenario_guards,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", choices=[*SCENARIOS, "all"], default="all")
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        print("refusing to run: set an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
        return 2
    leftovers = base._ros_processes()
    if leftovers:
        print(f"refusing to run: ROS processes already running: {leftovers}")
        return 2
    assert json.loads(json.dumps(ros_to_unity_pose(ControllerPose((0, 0, 0), (0, 0, 0, 1)))))
    rclpy.init()
    try:
        for name in (SCENARIOS if args.scenario == "all" else [args.scenario]):
            print(f"=== scenario {name}", flush=True)
            try:
                SCENARIOS[name]()
            except Exception as error:  # a scenario error must not skip the others
                check(f"{name}.completed", False, repr(error))
    finally:
        rclpy.shutdown()
    graph = subprocess.run([sys.executable, "-c", base._GRAPH_SNIPPET], capture_output=True,
                           text=True, timeout=60)
    check("final.no_ros_node_left_in_domain",
          graph.returncode == 0 and graph.stdout.strip() == "[]",
          graph.stdout.strip() or graph.stderr.strip()[-200:])
    print(f"CHECKS_FAILED={len(base.FAILURES)}")
    print(f"QUEST_TELEOP_PROBE={'PASS' if not base.FAILURES else 'FAIL'}", flush=True)
    return 0 if not base.FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
