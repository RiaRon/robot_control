#!/usr/bin/env python3
"""Humble runtime probe: the split OpenArm + RH56F1 bringup on fake hardware.

Three controller managers, started and stopped independently:

    openarm_rh56f1_arms.launch.py      /controller_manager                 14 arm joints
    rh56f1_right_hand.launch.py        /rh56f1_right/controller_manager     6
    rh56f1_left_hand.launch.py         /rh56f1_left/controller_manager      6
    openarm_rh56f1_model.launch.py     one robot_state_publisher + the joint-state merger

Scenarios:

  arm_only      arms (with the model) and no hand: resources, state, TF, motion
  hands_alone   each hand with no arm and no model: configure, activate, command
  all           all three managers and the model: ownership, /joint_states, TF,
                arm / hand / simultaneous motion; then a hand killed, the arm
                killed, both restarted

Only mock_components/GenericSystem is ever commanded. Output: CHECK lines, then
SPLIT_BRINGUP_PROBE=PASS or FAIL.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    ListHardwareInterfaces,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parent))
import humble_fake_hand_motion_probe as base  # noqa: E402

import json  # noqa: E402

check = base.check
LOG_ROOT = Path(os.environ.get("PROBE_LOG_DIR", "/tmp/split_bringup")) / "split"
CANONICAL = base.CANONICAL
ORDER = base.ORDER
GROUPS = base.GROUPS
ARM = GROUPS["right_arm"] + GROUPS["left_arm"]
HAND = {"right": GROUPS["right_hand"], "left": GROUPS["left_hand"]}
MIMIC_JOINTS = [f"{p}_hj_{f}" for p in "rl" for f in ("thumb_3", "thumb_4", "index_2",
                                                      "middle_2", "ring_2", "pinky_2")]
MANAGER = {"arms": "/controller_manager", "right": "/rh56f1_right/controller_manager",
           "left": "/rh56f1_left/controller_manager"}
ACTION = {
    "right_arm": "/right_joint_trajectory_controller/follow_joint_trajectory",
    "left_arm": "/left_joint_trajectory_controller/follow_joint_trajectory",
    "right_hand": "/rh56f1_right/right_hand_trajectory_controller/follow_joint_trajectory",
    "left_hand": "/rh56f1_left/left_hand_trajectory_controller/follow_joint_trajectory",
}
SOURCE_TOPIC = {"openarm": "/openarm/joint_states", "rh56f1_right": "/rh56f1_right/joint_states",
                "rh56f1_left": "/rh56f1_left/joint_states"}
PALM = {"right": "r_hl_palm_sensor", "left": "l_hl_palm_sensor"}
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
# Fake-only targets inside the URDF limits; not physical poses.
TARGET = {
    "right_arm": [0.30, 0.15, 0.05, 1.20, -0.05, 0.03, 0.06],
    "left_arm": [-0.30, -0.15, 0.05, 1.20, -0.05, 0.03, 0.06],
    "right_hand": [0.40, 0.20, 0.50, 0.45, 0.55, 0.60],
    "left_hand": [0.35, 0.25, 0.45, 0.40, 0.50, 0.55],
}
TARGET2 = {
    "right_arm": [0.20, 0.25, -0.05, 1.00, 0.05, -0.03, -0.06],
    "left_arm": [-0.20, -0.25, -0.05, 1.00, 0.05, -0.03, -0.06],
    "right_hand": [0.20, 0.10, 0.30, 0.25, 0.35, 0.15],
    "left_hand": [0.15, 0.05, 0.20, 0.30, 0.25, 0.35],
}
SAME = 1e-6
MOVED = 1e-4


class Launch:
    """One `ros2 launch openarm_bringup <file>` in its own process group."""

    def __init__(self, label: str, launch_file: str, *arguments: str):
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        self.label = label
        self.log = open(LOG_ROOT / f"{label}.log", "w")
        self.process = subprocess.Popen(
            ["ros2", "launch", "openarm_bringup", launch_file,
             f"canonical_urdf:={CANONICAL}", *arguments],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def alive(self) -> bool:
        return self.process.poll() is None

    def stop(self, sig=signal.SIGINT) -> bool:
        """Stop the whole group; True if it went on *sig* alone."""
        clean = True
        if self.process.poll() is None:
            os.killpg(self.process.pid, sig)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                clean = False
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        if sig == signal.SIGKILL:
            # The group leader is gone; make sure no child outlived it.
            try:
                os.killpg(self.process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.log.close()
        return clean

    def text(self) -> str:
        return (LOG_ROOT / f"{self.label}.log").read_text()


class Probe(Node):
    def __init__(self, name):
        super().__init__(name)
        base.OWN_NODES.add(name)
        self.latest = {}          # topic -> latest JointState
        self.received = {}        # topic -> list of (arrival, msg)
        self.status = None
        self.kinematics = None
        self.actions = {}
        for topic in ["/joint_states", "/openarm_rh56f1/display_joint_states",
                      *SOURCE_TOPIC.values()]:
            self.received[topic] = []
            self.create_subscription(JointState, topic,
                                     lambda m, t=topic: self._joint_state(t, m), 200)
        self.create_subscription(
            String, "/openarm_rh56f1/joint_state_sources",
            lambda m: setattr(self, "status", json.loads(m.data)), 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def destroy_node(self):
        for client in self.actions.values():
            client.destroy()
        self.actions.clear()
        return super().destroy_node()

    def _joint_state(self, topic, message):
        self.latest[topic] = message
        self.received[topic].append((time.monotonic(), message))
        if len(self.received[topic]) > 5000:
            del self.received[topic][:1000]

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def spin_until(self, predicate, timeout=20.0, label="condition"):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise AssertionError(f"timeout waiting for {label}")
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def call(self, service_type, name, request=None, timeout=10.0):
        client = self.create_client(service_type, name)
        try:
            if not self.spin_until(lambda: client.service_is_ready(), timeout, name):
                return None
            future = client.call_async(request or service_type.Request())
            self.spin_until(future.done, timeout, f"reply from {name}")
            return future.result()
        except AssertionError:
            return None
        finally:
            self.destroy_client(client)

    def measured(self) -> dict:
        """Latest position of every joint, from each device's own topic."""
        q = {}
        for topic in SOURCE_TOPIC.values():
            message = self.latest.get(topic)
            if message is not None:
                q.update(zip(message.name, message.position))
        return q

    def since(self, topic, start):
        return [m for t, m in self.received[topic] if t >= start]

    def tf(self, target, source):
        self.spin_until(lambda: self.buffer.can_transform(
            target, source, Time(), timeout=RclpyDuration(seconds=0.0)),
            label=f"TF {target} <- {source}")
        t = self.buffer.lookup_transform(target, source, Time()).transform
        r = t.rotation
        return base._homogeneous(base._quaternion_matrix(r.x, r.y, r.z, r.w),
                                 (t.translation.x, t.translation.y, t.translation.z))

    def controllers(self, device):
        reply = self.call(ListControllers, f"{MANAGER[device]}/list_controllers")
        return None if reply is None else {c.name: c for c in reply.controller}

    def send(self, commands, seconds=1.5):
        handles = {}
        for group, positions in commands.items():
            if group not in self.actions:
                self.actions[group] = ActionClient(self, FollowJointTrajectory, ACTION[group])
            client = self.actions[group]
            self.spin_until(client.server_is_ready, label=f"{group} action")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(GROUPS[group])
            point = JointTrajectoryPoint(positions=[float(v) for v in positions])
            point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [point]
            handles[group] = client.send_goal_async(goal)
        results = {}
        for group, future in handles.items():
            self.spin_until(future.done, label=f"{group} accept")
            assert future.result().accepted, f"{group} goal rejected"
            results[group] = future.result().get_result_async()
        self.spin_until(lambda: all(f.done() for f in results.values()), 15.0, "results")
        return {g: f.result().result.error_code for g, f in results.items()}

    def reached(self, commands, timeout=10.0):
        def done():
            q = self.measured()
            return all(abs(q.get(j, math.inf) - v) < 1e-4
                       for g, values in commands.items() for j, v in zip(GROUPS[g], values))
        try:
            self.spin_until(done, timeout, "targets")
            return True
        except AssertionError:
            return False


def wait_controllers(node, device, names, timeout=40.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        listed = node.controllers(device)
        if listed is not None and set(listed) == set(names) and all(
                c.state == "active" for c in listed.values()):
            return listed
        node.spin_for(0.3)
    listed = node.controllers(device)
    raise AssertionError(f"{device} controllers: "
                         f"{None if listed is None else {n: c.state for n, c in listed.items()}}")


def check_manager(node, tag, device, expected_joints, controllers):
    listed = wait_controllers(node, device, controllers)
    components = node.call(ListHardwareComponents,
                           f"{MANAGER[device]}/list_hardware_components").component
    check(f"{tag}.{device}.only_generic_system",
          [c.class_type for c in components] == ["mock_components/GenericSystem"],
          str([(c.name, c.class_type) for c in components]))
    interfaces = node.call(ListHardwareInterfaces, f"{MANAGER[device]}/list_hardware_interfaces")
    commands = sorted(i.name for i in interfaces.command_interfaces)
    states = sorted(i.name for i in interfaces.state_interfaces)
    wanted = sorted(f"{j}/position" for j in expected_joints)
    check(f"{tag}.{device}.resources_exactly_its_joints",
          commands == wanted and states == wanted,
          f"{len(commands)} command, {len(states)} state interfaces")
    check(f"{tag}.{device}.no_mimic_resource",
          not any(i.split("/")[0] in MIMIC_JOINTS for i in commands + states))
    claimed = [i for c in listed.values() for i in c.claimed_interfaces]
    check(f"{tag}.{device}.every_command_claimed_once",
          sorted(claimed) == wanted, f"{len(claimed)} claimed")
    return claimed


def tf_publishers(node):
    return node.count_publishers("/tf"), node.count_publishers("/tf_static")


def palm_relative_tips(node, side):
    return {f: node.tf(PALM[side], f"{side[0]}_hl_{f}_tip") for f in FINGERS}


def tf_matches_fk(node, tag, sides, q_display):
    links = [PALM[s] for s in sides] + [f"{s[0]}_hl_{f}_tip" for s in sides for f in FINGERS]
    links += [j["child"] for s in sides for j in node.kinematics.mimics(s[0])]
    worst = [0.0]

    def converged():
        worst[0] = 0.0
        q = q_display()
        for link in links:
            error = base._pose_error(node.kinematics.world(link, q), node.tf("body_root", link))
            worst[0] = max(worst[0], *error)
        return worst[0] < base.FK_TOL

    try:
        node.spin_until(converged, 10.0, "TF == FK")
        ok = True
    except AssertionError:
        ok = False
    check(f"{tag}.tf_matches_fk", ok, f"{len(links)} frames, max {worst[0]:.1e}")


def leftovers():
    out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True).stdout
    merger = [line.strip() for line in out.splitlines() if "joint_state_merger" in line]
    return merger + base._ros_processes()


def finish(tag, node, launches):
    for launch in launches:
        if launch.alive():
            check(f"{tag}.{launch.label}_stopped_by_sigint", launch.stop())
        elif not launch.log.closed:
            launch.log.close()
    node.destroy_node()
    deadline = time.monotonic() + 15
    remaining = leftovers()
    while remaining and time.monotonic() < deadline:
        time.sleep(0.5)
        remaining = leftovers()
    check(f"{tag}.no_process_left", not remaining, str(remaining))


# ---------------------------------------------------------------- scenarios
def scenario_arm_only():
    tag = "arm_only"
    node = Probe(f"split_probe_{tag}")
    launches = [Launch(f"{tag}.arms", "openarm_rh56f1_arms.launch.py", "runtime:=fake")]
    try:
        check_manager(node, tag, "arms", ARM, ["joint_state_broadcaster",
                                               "right_joint_trajectory_controller",
                                               "left_joint_trajectory_controller"])
        names = [(n, ns) for n, ns in node.get_node_names_and_namespaces()]
        check(f"{tag}.no_hand_manager_running",
              not any(ns.startswith("/rh56f1_") for _, ns in names), str(sorted(names)))
        node.spin_until(lambda: node.status is not None and "/openarm/joint_states" in node.latest,
                        label="arm state and merger status")
        start = time.monotonic()
        node.spin_for(1.5)
        arm_msgs = node.since("/openarm/joint_states", start)
        merged = node.since("/joint_states", start)
        merged_names = {n for m in merged for n in m.name}
        check(f"{tag}.arm_state_on_its_own_topic",
              len(arm_msgs) > 50 and set(arm_msgs[-1].name) == set(ARM),
              f"{len(arm_msgs)} messages, {len(arm_msgs[-1].name)} joints")
        check(f"{tag}.joint_states_has_only_the_14_measured_arm_joints",
              merged_names == set(ARM) and {m.header.frame_id for m in merged} == {"openarm"},
              f"{len(merged_names)} names in {len(merged)} messages")
        check(f"{tag}.merger_reports_hands_absent",
              node.status["openarm"]["state"] == "fresh"
              and node.status["rh56f1_right"]["state"] == "never"
              and node.status["rh56f1_left"]["state"] == "never"
              and node.status["rh56f1_right"]["display_placeholder"],
              json.dumps({k: v["state"] for k, v in node.status.items()}))
        display = node.since("/openarm_rh56f1/display_joint_states", start)
        placeholder = [m for m in display if m.header.frame_id == "display_placeholder"]
        check(f"{tag}.hands_drawn_from_marked_placeholder",
              {n for m in placeholder for n in m.name} == set(HAND["right"] + HAND["left"])
              and all(v == 0.0 for m in placeholder for v in m.position),
              f"{len(placeholder)} placeholder messages")
        check(f"{tag}.one_tf_publisher", tf_publishers(node) == (1, 1), str(tf_publishers(node)))
        node.kinematics = base.Kinematics(base_model_description(node))

        def display_q():
            q = {j: 0.0 for j in ORDER}
            q.update(node.measured())
            return q

        tf_matches_fk(node, f"{tag}.start", ("right", "left"), display_q)
        before = {s: (node.tf("body_root", PALM[s]), palm_relative_tips(node, s))
                  for s in ("right", "left")}
        codes = node.send({"right_arm": TARGET["right_arm"]})
        ok = node.reached({"right_arm": TARGET["right_arm"]})
        node.spin_for(0.3)
        palm = node.tf("body_root", PALM["right"])
        tips = palm_relative_tips(node, "right")
        check(f"{tag}.arm_moves_palm_and_mounted_hand_follow",
              codes == {"right_arm": 0} and ok
              and base._pose_distance(before["right"][0], palm) > MOVED
              and max(base._pose_distance(before["right"][1][f], tips[f]) for f in FINGERS) < SAME,
              f"palm moved {base._pose_distance(before['right'][0], palm):.3f}")
        check(f"{tag}.left_side_unchanged",
              base._pose_distance(before["left"][0], node.tf("body_root", PALM["left"])) < SAME)
        tf_matches_fk(node, f"{tag}.after_motion", ("right", "left"), display_q)
    except Exception as error:  # noqa: BLE001
        check(f"{tag}.completed", False, repr(error))
    finally:
        finish(tag, node, launches)


def base_model_description(node):
    """The integrated model robot_state_publisher draws (latched topic)."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    holder = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subscription = node.create_subscription(
        String, "/robot_description", lambda m: holder.setdefault("urdf", m.data), qos)
    node.spin_until(lambda: "urdf" in holder, label="/robot_description")
    node.destroy_subscription(subscription)
    return holder["urdf"]


def scenario_hands_alone():
    tag = "hands_alone"
    node = Probe(f"split_probe_{tag}")
    launches = [Launch(f"{tag}.right", "rh56f1_right_hand.launch.py"),
                Launch(f"{tag}.left", "rh56f1_left_hand.launch.py")]
    try:
        for side in ("right", "left"):
            check_manager(node, tag, side, HAND[side],
                          ["joint_state_broadcaster", f"{side}_hand_trajectory_controller"])
        names = node.get_node_names_and_namespaces()
        check(f"{tag}.no_arm_manager_and_no_model",
              ("controller_manager", "/") not in names
              and not any(n == "robot_state_publisher" for n, _ in names), str(sorted(names)))
        for side in ("right", "left"):
            group = f"{side}_hand"
            codes = node.send({group: TARGET[group]})
            check(f"{tag}.{side}_hand_fake_command_without_arm",
                  codes == {group: 0} and node.reached({group: TARGET[group]}),
                  f"{SOURCE_TOPIC['rh56f1_' + side]}")
        # One hand going away leaves the other working.
        check(f"{tag}.right_hand_stopped", launches[0].stop())
        codes = node.send({"left_hand": TARGET2["left_hand"]})
        check(f"{tag}.left_hand_still_commands_after_right_stops",
              codes == {"left_hand": 0} and node.reached({"left_hand": TARGET2["left_hand"]}))
    except Exception as error:  # noqa: BLE001
        check(f"{tag}.completed", False, repr(error))
    finally:
        finish(tag, node, launches)


def scenario_all():
    tag = "all"
    node = Probe(f"split_probe_{tag}")
    model = Launch(f"{tag}.model", "openarm_rh56f1_model.launch.py", "runtime:=fake")
    arms = Launch(f"{tag}.arms", "openarm_rh56f1_arms.launch.py", "runtime:=fake",
                  "start_model:=false")
    right = Launch(f"{tag}.right", "rh56f1_right_hand.launch.py")
    left = Launch(f"{tag}.left", "rh56f1_left_hand.launch.py")
    launches = [model, arms, right, left]
    try:
        claims = check_manager(node, tag, "arms", ARM, ["joint_state_broadcaster",
                                                        "right_joint_trajectory_controller",
                                                        "left_joint_trajectory_controller"])
        for side in ("right", "left"):
            claims += check_manager(node, tag, side, HAND[side],
                                    ["joint_state_broadcaster",
                                     f"{side}_hand_trajectory_controller"])
        check(f"{tag}.three_managers_26_resources_no_overlap",
              len(claims) == len(set(claims)) == 26
              and sorted(c.split("/")[0] for c in claims) == sorted(ORDER), f"{len(claims)}")
        node.spin_until(lambda: node.status is not None and all(
            v["state"] == "fresh" for v in node.status.values()), label="all sources fresh")
        start = time.monotonic()
        node.spin_for(1.0)
        merged = node.since("/joint_states", start)
        latest = {}
        for message in merged:
            latest[message.header.frame_id] = message
        union = [n for m in latest.values() for n in m.name]
        check(f"{tag}.joint_states_26_each_once_by_owner",
              sorted(union) == sorted(ORDER) and set(latest) == set(SOURCE_TOPIC),
              str({k: len(v.name) for k, v in latest.items()}))
        display = node.since("/openarm_rh56f1/display_joint_states", start)
        check(f"{tag}.no_placeholder_while_hands_fresh",
              not any(m.header.frame_id == "display_placeholder" for m in display))
        check(f"{tag}.one_tf_publisher", tf_publishers(node) == (1, 1), str(tf_publishers(node)))
        node.kinematics = base.Kinematics(base_model_description(node))
        tf_matches_fk(node, f"{tag}.start", ("right", "left"), node.measured)
        model_root = node.kinematics
        for side in ("right", "left"):
            fk = model_root.world(PALM[side], {j: 0.0 for j in ORDER})
            stage2 = (-0.001350455, -0.137557902, 0.090184013) if side == "right" else (
                0.001350499, 0.137557797, 0.090184109)
            check(f"{tag}.{side}_palm_frame_matches_stage2_fk",
                  np.allclose(fk[:3, 3], stage2, atol=1e-6), str(np.round(fk[:3, 3], 6)))

        # Arm alone: hand joint values and palm-relative fingers stay.
        before_tips = palm_relative_tips(node, "right")
        before_palm = node.tf("body_root", PALM["right"])
        hands_before = {j: node.measured()[j] for j in HAND["right"] + HAND["left"]}
        node.send({"right_arm": TARGET["right_arm"]})
        node.reached({"right_arm": TARGET["right_arm"]})
        node.spin_for(0.3)
        tips = palm_relative_tips(node, "right")
        check(f"{tag}.arm_motion_carries_hand",
              base._pose_distance(before_palm, node.tf("body_root", PALM["right"])) > MOVED
              and max(base._pose_distance(before_tips[f], tips[f]) for f in FINGERS) < SAME
              and all(node.measured()[j] == v for j, v in hands_before.items()))

        # Hand alone: arm joints and palm stay, fingertips and mimic links move.
        arm_before = {j: node.measured()[j] for j in ARM}
        palm_before = node.tf("body_root", PALM["right"])
        mimic_before = {j["name"]: node.tf(j["parent"], j["child"])
                        for j in model_root.mimics("r")}
        node.send({"right_hand": TARGET["right_hand"]})
        node.reached({"right_hand": TARGET["right_hand"]})
        node.spin_for(0.3)
        tips_after = palm_relative_tips(node, "right")
        mimic_after = {j["name"]: node.tf(j["parent"], j["child"]) for j in model_root.mimics("r")}
        check(f"{tag}.hand_motion_moves_only_fingers",
              all(node.measured()[j] == v for j, v in arm_before.items())
              and base._pose_distance(palm_before, node.tf("body_root", PALM["right"])) < SAME
              and min(base._pose_distance(tips[f], tips_after[f]) for f in FINGERS) > MOVED
              and min(base._pose_distance(mimic_before[k], mimic_after[k]) for k in mimic_before)
              > MOVED)
        tf_matches_fk(node, f"{tag}.after_hand", ("right", "left"), node.measured)

        # All four groups at once, across three managers: no crosstalk.
        codes = node.send(dict(TARGET2))
        ok = node.reached(TARGET2)
        q = node.measured()
        error = max(abs(q[j] - v) for g, values in TARGET2.items()
                    for j, v in zip(GROUPS[g], values))
        check(f"{tag}.four_groups_three_managers_no_crosstalk",
              set(codes.values()) == {0} and ok and error < 1e-4, f"max error {error:.1e}")
        tf_matches_fk(node, f"{tag}.after_all", ("right", "left"), node.measured)

        # --- the right hand fails: arm and left hand carry on --------------
        right_hand_q = {j: node.measured()[j] for j in HAND["right"]}
        right.stop(signal.SIGKILL)
        killed = time.monotonic()
        node.spin_until(lambda: node.status["rh56f1_right"]["state"] == "stale",
                        timeout=5.0, label="right hand stale")
        codes = node.send({"right_arm": TARGET["right_arm"], "left_hand": TARGET["left_hand"]})
        ok = node.reached({"right_arm": TARGET["right_arm"], "left_hand": TARGET["left_hand"]})
        arm_rate = len(node.since("/openarm/joint_states", killed)) / (time.monotonic() - killed)
        check(f"{tag}.right_hand_killed_arm_and_left_hand_continue",
              set(codes.values()) == {0} and ok and arm_rate > 50,
              f"arm state {arm_rate:.0f} Hz after the kill")
        after_kill = time.monotonic()
        node.spin_for(1.0)
        restamped = [m for m in node.since("/joint_states", after_kill)
                     if m.header.frame_id == "rh56f1_right"]
        placeholder = [m for m in node.since("/openarm_rh56f1/display_joint_states", after_kill)
                       if m.header.frame_id == "display_placeholder"]
        check(f"{tag}.dead_hand_not_republished_as_measured",
              not restamped and node.status["rh56f1_right"]["state"] == "stale",
              f"{len(restamped)} right-hand messages after the kill")
        open_tips = palm_relative_tips(node, "right")
        expected = {f: node.kinematics.world(f"r_hl_{f}_tip", {j: 0.0 for j in ORDER})
                    for f in FINGERS}
        palm0 = node.kinematics.world(PALM["right"], {j: 0.0 for j in ORDER})
        relative_open = {f: np.linalg.inv(palm0) @ expected[f] for f in FINGERS}
        check(f"{tag}.dead_hand_shown_as_marked_placeholder_not_frozen",
              placeholder and {n for m in placeholder for n in m.name} == set(HAND["right"])
              and max(base._pose_distance(open_tips[f], relative_open[f]) for f in FINGERS) < 1e-5
              and any(abs(v) > 0.1 for v in right_hand_q.values()),
              f"{len(placeholder)} placeholder messages; fingers drawn open, not at the last "
              f"measured pose")

        # --- restart the right hand ---------------------------------------
        right = Launch(f"{tag}.right_restart", "rh56f1_right_hand.launch.py")
        launches.append(right)
        check_manager(node, f"{tag}.restart", "right", HAND["right"],
                      ["joint_state_broadcaster", "right_hand_trajectory_controller"])
        node.spin_until(lambda: node.status["rh56f1_right"]["state"] == "fresh",
                        label="right hand fresh again")
        mark = time.monotonic()
        node.spin_for(0.6)
        check(f"{tag}.placeholder_stops_when_hand_returns",
              not [m for m in node.since("/openarm_rh56f1/display_joint_states", mark)
                   if m.header.frame_id == "display_placeholder" and "r_hj_thumb_1" in m.name])

        # --- the arm fails: both hands carry on ---------------------------
        arms.stop(signal.SIGKILL)
        node.spin_until(lambda: node.status["openarm"]["state"] == "stale", timeout=5.0,
                        label="arms stale")
        codes = node.send({"right_hand": TARGET2["right_hand"], "left_hand": TARGET2["left_hand"]})
        ok = node.reached({"right_hand": TARGET2["right_hand"], "left_hand": TARGET2["left_hand"]})
        check(f"{tag}.arm_killed_hands_continue", set(codes.values()) == {0} and ok)

        # --- restart the arm ----------------------------------------------
        arms = Launch(f"{tag}.arms_restart", "openarm_rh56f1_arms.launch.py", "runtime:=fake",
                      "start_model:=false")
        launches.append(arms)
        check_manager(node, f"{tag}.restart", "arms", ARM,
                      ["joint_state_broadcaster", "right_joint_trajectory_controller",
                       "left_joint_trajectory_controller"])
        codes = node.send({"left_arm": TARGET["left_arm"]})
        check(f"{tag}.arm_restarted_and_commands",
              codes == {"left_arm": 0} and node.reached({"left_arm": TARGET["left_arm"]}))
        check(f"{tag}.still_one_tf_publisher", tf_publishers(node) == (1, 1),
              str(tf_publishers(node)))
        # Two managers for the same namespace are refused.
        twin = Launch(f"{tag}.arms_twin", "openarm_rh56f1_arms.launch.py", "runtime:=fake",
                      "start_model:=false")
        twin.process.wait(timeout=90)
        check(f"{tag}.second_arm_manager_refused",
              twin.process.returncode != 0 and "already running" in twin.text(),
              f"exit {twin.process.returncode}")
        twin.log.close()
    except Exception as error:  # noqa: BLE001
        check(f"{tag}.completed", False, repr(error))
    finally:
        finish(tag, node, launches)


SCENARIOS = {"arm_only": scenario_arm_only, "hands_alone": scenario_hands_alone,
             "all": scenario_all}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", choices=[*SCENARIOS, "every"], default="every")
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        print("refusing to run: set an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
        return 2
    if leftovers():
        print(f"refusing to run: ROS processes already running: {leftovers()}")
        return 2
    rclpy.init()
    try:
        for name in (SCENARIOS if args.scenario == "every" else [args.scenario]):
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
    print(f"SPLIT_BRINGUP_PROBE={'PASS' if not base.FAILURES else 'FAIL'}", flush=True)
    return 0 if not base.FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
