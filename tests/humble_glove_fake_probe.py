#!/usr/bin/env python3
"""Humble runtime probe: synthetic Nova2 glove -> retarget -> adapter -> fake RH56F1 hand.

Runs on the split bringup (arms under /controller_manager, each hand under its
own manager, one integrated model) inside the isolated Humble container. The
glove is synthetic: this probe publishes Nova2-named JointState on
/senseglove/gloveSYNTH/<rh|lh>/joint_states. The retarget node is the vendored
inspire_hand-main one, unmodified; rh56f1_glove_input.launch.py runs it with the
adapter.

  dry_run   right hand + glove input without execute: nothing commanded, no
            serial port opened, a manager that does not answer is refused
  combined  arms + both hands + both gloves + Quest on the right arm:
            glove-driven fingers, hand-only and arm-only motion, Quest and
            glove together, no target on /joint_states, stale and invalid
            input, single command owner, killed adapter / hand manager and
            restarts

Only mock_components/GenericSystem is ever commanded. Output: CHECK lines, then
GLOVE_FAKE_PROBE=PASS or FAIL.
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
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import humble_fake_hand_motion_probe as base  # noqa: E402
import humble_split_bringup_probe as split_probe  # noqa: E402
from humble_quest_teleop_probe import KeepAliveSender, Process  # noqa: E402

from rh56f1_glove_teleop import synth_glove  # noqa: E402
from rh56f1_glove_teleop.adapter import build_contract, load_config  # noqa: E402
from robot_control.profile import load_builtin_profile  # noqa: E402

check = base.check
SERIAL = "SYNTH"
SIDES = ("right", "left")
PREFIX = {"right": "r", "left": "l"}
HAND = split_probe.HAND
ARM = split_probe.ARM
PALM = split_probe.PALM
FINGERS = split_probe.FINGERS
SOURCE_NAMES = ("pinky_proximal_joint", "ring_proximal_joint", "middle_proximal_joint",
                "index_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint")
HAND_COMMAND = {s: f"/rh56f1_{s}/{s}_hand_trajectory_controller/joint_trajectory" for s in SIDES}
ARM_COMMAND = {s: f"/{s}_joint_trajectory_controller/joint_trajectory" for s in SIDES}
BENT = {"right_arm": [0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0],
        "left_arm": [-0.3, -0.15, 0.0, 1.2, 0.0, 0.0, 0.0]}
SAME = 1e-6
MOVED = 1e-4

_profile = load_builtin_profile("openarm_rh56f1")
_order = yaml.safe_load(Path(_profile.manifest_path).read_text())["control_joint_order"]
CONTRACT = {s: build_contract(load_config(), _profile, _order, s) for s in SIDES}
CALIBRATION = synth_glove.load_calibration(None)


def expected_fingers(side, closure):
    """Canonical targets for a glove closure, four fingers (thumbs are held)."""
    c = CONTRACT[side]
    return {c.prefix + a: closure * c.upper[c.index(a)]
            for a in ("index_1", "middle_1", "ring_1", "pinky_1")}


class GloveProbe(split_probe.Probe):
    def __init__(self, name):
        super().__init__(name)
        # Checks read each device's latest state: keep only the newest sample
        # of the device topics, and a short queue on the integrated topics
        # (only their names are checked).
        for subscription in list(self.subscriptions):
            if subscription.topic_name in self.received:
                self.destroy_subscription(subscription)
        for topic in self.received:
            depth = 1 if topic in split_probe.SOURCE_TOPIC.values() else 20
            self.create_subscription(JointState, topic,
                                     lambda m, t=topic: self._joint_state(t, m), depth)
        self.closure = {s: None for s in SIDES}       # None = glove silent
        self.nan_glove = {s: False for s in SIDES}
        self.adapter = {}
        self.retarget_count = {s: 0 for s in SIDES}
        self.commands = {f"{s}_hand": [] for s in SIDES}
        self.commands.update({f"{s}_arm": [] for s in SIDES})
        self.glove = {s: self.create_publisher(JointState, synth_glove.glove_topic(SERIAL, s), 10)
                      for s in SIDES}
        self.retarget_pub = {s: self.create_publisher(JointState,
                                                      f"/inspire_{s}/retarget/joint_states", 10)
                             for s in SIDES}
        for side in SIDES:
            self.create_subscription(
                String, f"/rh56f1_{side}/glove_adapter/status",
                lambda m, s=side: self.adapter.__setitem__(s, json.loads(m.data)), 10)
            self.create_subscription(
                JointState, f"/inspire_{side}/retarget/joint_states",
                lambda m, s=side: self.retarget_count.__setitem__(s, self.retarget_count[s] + 1),
                50)
            self.create_subscription(JointTrajectory, HAND_COMMAND[side],
                                     lambda m, s=side: self.commands[f"{s}_hand"].append(m), 100)
            self.create_subscription(JointTrajectory, ARM_COMMAND[side],
                                     lambda m, s=side: self.commands[f"{s}_arm"].append(m), 100)
        self.create_timer(1.0 / 60.0, self._feed)
        # One persistent executor, so the glove feed timer keeps running at its
        # rate during every wait of this probe.
        from rclpy.executors import SingleThreadedExecutor

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._executor.spin_once(timeout_sec=0.02)

    def spin_until(self, predicate, timeout=20.0, label="condition"):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise AssertionError(f"timeout waiting for {label}")
            self._executor.spin_once(timeout_sec=0.05)
        return True

    def destroy_node(self):
        self._executor.remove_node(self)
        self._executor.shutdown()
        return super().destroy_node()

    def _feed(self):
        for side in SIDES:
            closure = self.closure[side]
            if closure is None:
                continue
            closures = {k: closure for k in (*synth_glove.FINGERS, "thumb_pitch", "thumb_yaw")}
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = synth_glove.joint_names(side)
            values = synth_glove.positions(side, closures, CALIBRATION)
            if self.nan_glove[side]:
                values = [float("nan")] * len(values)
            message.position = values
            self.glove[side].publish(message)

    def drive(self, action):
        """Run a Quest sender action in a thread while this node keeps spinning."""
        import threading

        thread = threading.Thread(target=action)
        thread.start()
        while thread.is_alive():
            self._executor.spin_once(timeout_sec=0.02)
        thread.join()

    def enable(self, side):
        reply = self.call(Trigger, f"/rh56f1_{side}/glove_adapter/enable", Trigger.Request())
        return reply is not None and reply.success

    def hand_at(self, side, closure, timeout=8.0):
        wanted = expected_fingers(side, closure)
        try:
            self.spin_until(lambda: all(abs(self.measured().get(n, math.inf) - v) < 1e-4
                                        for n, v in wanted.items()), timeout, f"{side} hand")
            return True
        except AssertionError:
            return False

    def wait_adapter(self, side, predicate, timeout=10.0):
        try:
            self.spin_until(lambda: side in self.adapter and predicate(self.adapter[side]),
                            timeout, f"{side} adapter")
            return True
        except AssertionError:
            return False


def glove_launch(label, side, execute=True):
    return split_probe.Launch(label, "rh56f1_glove_input.launch.py", f"side:={side}",
                              f"glove_serial:={SERIAL}", f"execute:={'true' if execute else 'false'}")


def open_device_fds(pattern):
    """tty/serial/USB/CAN device files held open by processes matching *pattern*."""
    found = []
    out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True).stdout
    for line in out.splitlines():
        pid, _, args = line.strip().partition(" ")
        if pattern not in args.split()[:4]:
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                target = os.readlink(f"/proc/{pid}/fd/{fd}")
                if target.startswith("/dev/") and not target.startswith(("/dev/null", "/dev/pts",
                                                                         "/dev/urandom", "/dev/shm")):
                    found.append((pid, target))
        except OSError:
            pass
    return found


MODULES = {"senseglove_teleop.retarget_node", "rh56f1_glove_teleop.ros_adapter",
           "openarm_quest_teleop.ros_bridge", "openarm_quest_teleop.ros_teleop"}


def leftovers():
    """Our python -m processes, judged by their module argument only."""
    out = subprocess.run(["ps", "-eo", "args="], capture_output=True, text=True).stdout
    ours = [line for line in out.splitlines() if MODULES & set(line.split()[:4])]
    return ours + split_probe.leftovers()


def finish(tag, node, launches, processes=()):
    for process in processes:
        check(f"{tag}.{process.name.split('.')[-1]}_stopped_by_sigint", process.stop())
    split_probe.finish(tag, node, launches)
    deadline = time.monotonic() + 15
    while leftovers() and time.monotonic() < deadline:
        time.sleep(0.5)
    check(f"{tag}.no_glove_or_teleop_process_left", not leftovers(), str(leftovers()))


# ---------------------------------------------------------------- scenarios
def scenario_dry_run():
    tag = "dry_run"
    node = GloveProbe(f"glove_probe_{tag}")
    launches = [split_probe.Launch(f"{tag}.right_hand", "rh56f1_right_hand.launch.py"),
                glove_launch(f"{tag}.right_glove", "right", execute=False)]
    try:
        split_probe.wait_controllers(node, "right", {"joint_state_broadcaster",
                                                     "right_hand_trajectory_controller"})
        node.closure["right"] = 0.6
        ok = node.wait_adapter("right", lambda s: s["target"] is not None, timeout=20.0)
        node.spin_for(1.5)
        status = node.adapter.get("right", {})
        q = node.measured()
        check(f"{tag}.targets_mapped_but_not_sent",
              ok and status["executing"] is False and status["commands"] == 0
              and node.count_publishers(HAND_COMMAND["right"]) == 0
              and not node.commands["right_hand"]
              and all(q[n] == 0.0 for n in HAND["right"]),
              f"state {status.get('state')}, target "
              f"{[round(v, 3) for v in status.get('target') or []]}, 0 command publishers")
        check(f"{tag}.retarget_output_is_the_old_contract", node.retarget_count["right"] > 30,
              f"{node.retarget_count['right']} target messages")
        fds = (open_device_fds("senseglove_teleop.retarget_node")
               + open_device_fds("rh56f1_glove_teleop.ros_adapter"))
        bridge = [l for l in subprocess.run(["ps", "-eo", "args="], capture_output=True,
                                             text=True).stdout.splitlines() if "hand_bridge" in l]
        check(f"{tag}.no_serial_port_and_no_hand_bridge", not fds and not bridge, f"{fds} {bridge}")
        # --execute against a manager that does not answer (no left hand runs).
        refused = Process(f"{tag}.adapter_left_no_manager", "rh56f1_glove_teleop.ros_adapter",
                          "--side", "left", "--execute")
        deadline = time.monotonic() + 60
        while refused.process.poll() is None and time.monotonic() < deadline:
            node.spin_for(0.05)
        code, text = refused.process.poll(), refused.text()
        refused.stop()
        check(f"{tag}.unanswering_manager_refused", code == 2 and "does not answer" in text,
              f"exit {code}")
    except Exception as error:  # noqa: BLE001
        check(f"{tag}.completed", False, repr(error))
    finally:
        node.closure = {s: None for s in SIDES}
        finish(tag, node, launches)


def scenario_combined():
    tag = "combined"
    node = GloveProbe(f"glove_probe_{tag}")
    arms = split_probe.Launch(f"{tag}.arms", "openarm_rh56f1_arms.launch.py", "runtime:=fake")
    hands = {s: split_probe.Launch(f"{tag}.{s}_hand", f"rh56f1_{s}_hand.launch.py") for s in SIDES}
    launches = [arms, *hands.values()]
    processes = []
    sender = None
    try:
        split_probe.wait_controllers(node, "arms", {"joint_state_broadcaster",
                                                    "right_joint_trajectory_controller",
                                                    "left_joint_trajectory_controller"})
        for side in SIDES:
            split_probe.wait_controllers(node, side, {"joint_state_broadcaster",
                                                      f"{side}_hand_trajectory_controller"})
        node.send(dict(BENT), seconds=2.0)
        node.reached(dict(BENT))
        node.kinematics = base.Kinematics(split_probe.base_model_description(node))
        gloves = {s: glove_launch(f"{tag}.{s}_glove", s) for s in SIDES}
        launches += list(gloves.values())
        for side in SIDES:
            check(f"{tag}.{side}_adapter_executing",
                  node.wait_adapter(side, lambda s: s["executing"] is True, timeout=40.0))

        # --- gloves drive their own hand only ------------------------------
        arm_before = {n: node.measured()[n] for n in ARM}
        palm_before = {s: node.tf("body_root", PALM[s]) for s in SIDES}
        tips_before = split_probe.palm_relative_tips(node, "right")
        node.closure = {"right": 0.5, "left": 0.2}
        ok = node.hand_at("right", 0.5) and node.hand_at("left", 0.2)
        q = node.measured()
        check(f"{tag}.gloves_drive_fingers_independently",
              ok and all(q[f"{PREFIX[s]}_hj_{t}"] == 0.0 for s in SIDES
                         for t in ("thumb_1", "thumb_2")),
              "right 50 %, left 20 % closure; thumbs held at their measured 0.0; measured "
              + str({s: [round(q[n], 4) for n in CONTRACT[s].canonical] for s in SIDES})
              + " adapters " + str({s: (node.adapter[s]["state"], node.adapter[s]["reason"])
                                    for s in SIDES})
              + " command subscribers " + str({s: node.count_subscribers(HAND_COMMAND[s])
                                               for s in SIDES}))
        node.spin_for(0.3)
        tips = split_probe.palm_relative_tips(node, "right")
        check(f"{tag}.hand_motion_leaves_arm_and_palm",
              all(node.measured()[n] == v for n, v in arm_before.items())
              and max(base._pose_distance(palm_before[s], node.tf("body_root", PALM[s]))
                      for s in SIDES) < SAME
              and min(base._pose_distance(tips_before[f], tips[f]) for f in FINGERS
                      if f != "thumb") > MOVED)
        split_probe.tf_matches_fk(node, f"{tag}.gloves", SIDES, node.measured)
        index = node.measured()["r_hj_index_1"]
        mimic = next(j for j in node.kinematics.mimics("r") if j["name"] == "r_hj_index_2")
        check(f"{tag}.mimic_follows_glove_driven_master",
              abs(node.kinematics.value("r_hj_index_2", node.measured()) - 1.1169 * index) < 1e-9
              and mimic["mimic"][1] == 1.1169, f"index_1 {index:.4f}")

        # --- arm only: palm moves, fingers stay relative to it -------------
        tips_before = split_probe.palm_relative_tips(node, "left")
        palm = node.tf("body_root", PALM["left"])
        node.send({"left_arm": [-0.2, -0.25, 0.05, 1.0, 0.05, 0.0, 0.0]})
        node.reached({"left_arm": [-0.2, -0.25, 0.05, 1.0, 0.05, 0.0, 0.0]})
        node.spin_for(0.3)
        tips = split_probe.palm_relative_tips(node, "left")
        check(f"{tag}.arm_motion_carries_glove_held_hand",
              base._pose_distance(palm, node.tf("body_root", PALM["left"])) > MOVED
              and max(base._pose_distance(tips_before[f], tips[f]) for f in FINGERS) < SAME
              and node.hand_at("left", 0.2, timeout=1.0))

        # --- Quest on the right arm while the right glove closes -----------
        node.commands = {k: [] for k in node.commands}
        processes.append(Process(f"{tag}.quest_bridge", "openarm_quest_teleop.ros_bridge"))
        teleop = Process(f"{tag}.quest_teleop", "openarm_quest_teleop.ros_teleop", "--arm",
                         "right", "--runtime", "fake", "--execute")
        processes.append(teleop)
        sender = KeepAliveSender("127.0.0.1", 5006)
        node.spin_for(4.0)
        sender.grip["right"] = 1.0
        node.spin_for(1.0)
        palm = node.tf("body_root", PALM["right"])
        node.closure["right"] = 0.9
        node.drive(lambda: (sender.move("right", (0.04, 0, 0), seconds=1.5), sender.hold(1.0)))
        moved = node.tf("body_root", PALM["right"])[:3, 3] - palm[:3, 3]
        check(f"{tag}.quest_arm_and_glove_hand_together",
              np.linalg.norm(moved - np.array((0.04, 0, 0))) < 2e-3
              and node.hand_at("right", 0.9, timeout=3.0),
              f"palm moved {np.round(moved, 4).tolist()} m; right fingers at 90 % closure")
        arm_names = {tuple(m.joint_names) for m in node.commands["right_arm"]}
        hand_names = {tuple(m.joint_names) for m in node.commands["right_hand"]}
        check(f"{tag}.separate_command_paths",
              arm_names == {tuple(ARM[:7])} and hand_names == {CONTRACT["right"].canonical}
              and node.count_publishers(ARM_COMMAND["right"]) == 1
              and node.count_publishers(HAND_COMMAND["right"]) == 1
              and not node.commands["left_arm"],
              f"{len(node.commands['right_arm'])} arm, {len(node.commands['right_hand'])} hand "
              "messages, one publisher each")
        sender.grip["right"] = 0.0
        node.drive(lambda: sender.hold(0.4))

        # --- no target is ever published as joint state --------------------
        start = time.monotonic()
        node.spin_for(1.0)
        names = {n for m in node.since("/joint_states", start) for n in m.name}
        display = {n for m in node.since("/openarm_rh56f1/display_joint_states", start)
                   for n in m.name}
        check(f"{tag}.targets_never_on_joint_states",
              not (names | display) & set(SOURCE_NAMES)
              and node.count_publishers("/joint_states") == 1
              and split_probe.tf_publishers(node) == (1, 1),
              f"{len(names)} names on /joint_states, publishers "
              f"{node.count_publishers('/joint_states')}, TF {split_probe.tf_publishers(node)}")

        # --- stale glove: lock, no automatic resume, explicit enable -------
        node.closure["right"] = None
        locked = node.wait_adapter("right", lambda s: s["state"] == "locked", timeout=3.0)
        node.closure["right"] = 0.3
        node.spin_for(1.5)
        still = node.adapter["right"]["state"] == "locked" and node.hand_at("right", 0.9, 0.5)
        left_ok = node.hand_at("left", 0.2, 0.5)
        resumed = node.enable("right") and node.hand_at("right", 0.3)
        check(f"{tag}.stale_glove_locks_until_enable", locked and still and left_ok and resumed,
              str(node.adapter["right"]["reason"]))

        # --- invalid targets are rejected; only invalid is a loss ----------
        rejected = node.adapter["right"]["targets_rejected"]
        bad = [
            (list(SOURCE_NAMES), [float("nan")] * 6),
            (list(SOURCE_NAMES[:5]) + ["ring_proximal_joint"], [0.1] * 6),
            (list(SOURCE_NAMES), [0.1] * 5),
            (list(SOURCE_NAMES[:4]), [0.1] * 4),
        ]
        for _ in range(20):
            for names_, positions in bad:
                message = JointState(name=names_, position=positions)
                node.retarget_pub["right"].publish(message)
            node.spin_for(0.05)
        node.spin_for(0.5)
        check(f"{tag}.invalid_targets_rejected_valid_ones_followed",
              node.adapter["right"]["targets_rejected"] - rejected >= 80
              and node.adapter["right"]["state"] == "following" and node.hand_at("right", 0.3, 1.0),
              f"{node.adapter['right']['targets_rejected'] - rejected} rejected: "
              f"{node.adapter['right']['last_rejection']}")
        count = node.retarget_count["right"]
        node.nan_glove["right"] = True
        locked = node.wait_adapter("right", lambda s: s["state"] == "locked", timeout=3.0)
        dropped = node.retarget_count["right"] - count
        node.nan_glove["right"] = False
        node.spin_for(0.5)
        check(f"{tag}.nan_glove_dropped_by_retarget_and_locks",
              locked and dropped < 5 and node.enable("right") and node.hand_at("right", 0.3),
              f"{dropped} targets from NaN glove input")

        # --- one command owner per hand -------------------------------------
        twin = Process(f"{tag}.adapter_right_twin", "rh56f1_glove_teleop.ros_adapter",
                       "--side", "right", "--execute")
        deadline = time.monotonic() + 60
        while twin.process.poll() is None and time.monotonic() < deadline:
            node.spin_for(0.05)
        code, text = twin.process.poll(), twin.text()
        twin.stop()
        check(f"{tag}.second_command_owner_refused",
              code == 2 and "already owned" in text, f"exit {code}")

        # --- adapter killed: arm and other hand carry on; restart -----------
        gloves["right"].stop(signal.SIGKILL)
        node.closure = {"right": 0.7, "left": 0.6}
        arm_ok = node.send({"right_arm": BENT["right_arm"]}) == {"right_arm": 0}
        check(f"{tag}.right_adapter_killed_others_continue",
              arm_ok and node.reached({"right_arm": BENT["right_arm"]})
              and node.hand_at("left", 0.6) and node.hand_at("right", 0.3, 0.5))
        gloves["right"] = glove_launch(f"{tag}.right_glove_restart", "right")
        launches.append(gloves["right"])
        check(f"{tag}.right_adapter_restarted", node.hand_at("right", 0.7, timeout=40.0))

        # --- right hand manager killed: arm and left carry on; restart -------
        hands["right"].stop(signal.SIGKILL)
        locked = node.wait_adapter("right", lambda s: s["state"] == "locked", timeout=5.0)
        node.closure["left"] = 0.1
        check(f"{tag}.right_hand_manager_killed_others_continue",
              locked and node.hand_at("left", 0.1)
              and node.send({"left_arm": BENT["left_arm"]}) == {"left_arm": 0},
              str(node.adapter["right"]["reason"]))
        hands["right"] = split_probe.Launch(f"{tag}.right_hand_restart", "rh56f1_right_hand.launch.py")
        launches.append(hands["right"])
        split_probe.wait_controllers(node, "right", {"joint_state_broadcaster",
                                                     "right_hand_trajectory_controller"})
        node.spin_for(1.0)
        still_locked = node.adapter["right"]["state"] == "locked"
        check(f"{tag}.restarted_hand_needs_enable",
              still_locked and node.enable("right") and node.hand_at("right", 0.7))
    except Exception as error:  # noqa: BLE001
        check(f"{tag}.completed", False, repr(error))
    finally:
        node.closure = {s: None for s in SIDES}
        if sender is not None:
            sender.close()
        finish(tag, node, launches, processes)


SCENARIOS = {"dry_run": scenario_dry_run, "combined": scenario_combined}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", choices=[*SCENARIOS, "all"], default="all")
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        print("refusing to run: set an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
        return 2
    if leftovers():
        print(f"refusing to run: processes already running: {leftovers()}")
        return 2
    rclpy.init()
    try:
        for name in (SCENARIOS if args.scenario == "all" else [args.scenario]):
            print(f"=== scenario {name}", flush=True)
            SCENARIOS[name]()
    finally:
        rclpy.shutdown()
    graph = subprocess.run([sys.executable, "-c", base._GRAPH_SNIPPET], capture_output=True,
                           text=True, timeout=60)
    check("final.no_ros_node_left_in_domain",
          graph.returncode == 0 and graph.stdout.strip() == "[]",
          graph.stdout.strip() or graph.stderr.strip()[-200:])
    print(f"CHECKS_FAILED={len(base.FAILURES)}")
    print(f"GLOVE_FAKE_PROBE={'PASS' if not base.FAILURES else 'FAIL'}", flush=True)
    return 0 if not base.FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
