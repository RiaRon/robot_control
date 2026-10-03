#!/usr/bin/env python3
"""Humble runtime probe: fake arm + RH56F1 hand motion, individually and together.

Run only inside the isolated Humble container (no network, no devices) after
building openarm_description and openarm_bringup into an overlay. For each
scenario the probe starts ``openarm.rh56f1_bimanual.launch.py`` itself, checks
it, and shuts it down:

  parked_both        existing parked policy: no hand controller, hands stay at
                     zero, and the existing humble_fake_runtime_probe.py passes
  commandable_right  fake_commandable + right hand: right arm only, right hand
                     only, right arm + hand together, arm again at a bent hand
  commandable_both   fake_commandable + both hands: each of the four groups
                     alone, then all four together

Before any goal is sent the probe requires that the only hardware component is
mock_components/GenericSystem. Joint targets are small fake-only test values
inside the canonical URDF limits; they are not physical poses and say nothing
about the RH56F1 raw <-> radian conversion. Output: CHECK lines, then
FAKE_MOTION_PROBE=PASS or FAIL.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration
import yaml

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    ListHardwareInterfaces,
)
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(
    0, str(Path(get_package_share_directory("openarm_bringup")) / "launch")
)
from rh56f1_description import FAKE_HAND_CONTROLLERS  # noqa: E402

KUKU = Path(os.environ.get("KUKU_LAB_ROOT", "/workspace/kuku_lab"))
CANONICAL = KUKU / "urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"
MANIFEST = KUKU / "urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml"
LOG_ROOT = Path(os.environ.get("PROBE_LOG_DIR", "/tmp/rh56f1_fake_motion"))
PARKED_PROBE = Path(__file__).resolve().with_name("humble_fake_runtime_probe.py")

ORDER = yaml.safe_load(MANIFEST.read_text())["control_joint_order"]
PREFIX = {"right": "r", "left": "l"}
GROUPS = {
    f"{side}_{kind}": [n for n in ORDER if n.startswith(f"{p}_{kind[0]}j_")]
    for side, p in PREFIX.items()
    for kind in ("arm", "hand")
}
CONTROLLER = {
    "right_arm": "right_joint_trajectory_controller",
    "left_arm": "left_joint_trajectory_controller",
    "right_hand": FAKE_HAND_CONTROLLERS["right"],
    "left_hand": FAKE_HAND_CONTROLLERS["left"],
}
FINGERS = ("thumb", "index", "middle", "ring", "pinky")

# Fake-only test targets (rad), distinct per joint so a misrouted command shows.
TARGETS = {
    "right_arm": [[0.05, 0.10, 0.04, 0.12, -0.05, 0.03, 0.06],
                  [0.08, 0.15, -0.04, 0.20, 0.05, -0.03, -0.06]],
    "left_arm": [[-0.05, -0.10, 0.04, 0.12, -0.05, 0.03, 0.06],
                 [-0.08, -0.15, -0.04, 0.20, 0.05, -0.03, -0.06]],
    "right_hand": [[0.20, 0.10, 0.30, 0.25, 0.35, 0.15],
                   [0.40, 0.20, 0.50, 0.45, 0.55, 0.60]],
    "left_hand": [[0.15, 0.05, 0.20, 0.30, 0.25, 0.35],
                  [0.35, 0.25, 0.45, 0.40, 0.50, 0.55]],
}

STATE_TOL = 1e-4      # commanded joint reached (rad)
HOLD_TOL = 1e-9       # joint not commanded in a step (rad)
POSE_SAME = 1e-6      # TF unchanged (m / rad)
POSE_MOVED = 1e-4     # TF changed (m / rad)
FK_TOL = 1e-5         # runtime TF vs offline FK of the joint states

FAILURES: list[str] = []
OWN_NODES: set[str] = set()  # this process's nodes may linger in its own graph cache


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"CHECK {name}: {'PASS' if ok else 'FAIL'}{' ' + detail if detail else ''}",
          flush=True)
    if not ok:
        FAILURES.append(name)
    return ok


# ---------------------------------------------------------------- kinematics
def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = map(float, (math.cos(r), math.sin(r), math.cos(p),
                                        math.sin(p), math.cos(y), math.sin(y)))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, angle):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _homogeneous(rotation, translation):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


class Kinematics:
    """Offline FK of the runtime robot_description (URDF mimic rules included)."""

    def __init__(self, urdf: str):
        root = ET.fromstring(urdf)
        self.joints = {}
        self.by_child = {}
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            axis = joint.find("axis")
            mimic = joint.find("mimic")
            limit = joint.find("limit")
            entry = {
                "name": joint.get("name"),
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "origin": _homogeneous(
                    _rpy(*_floats(origin.get("rpy") if origin is not None else "", (0, 0, 0))),
                    _floats(origin.get("xyz") if origin is not None else "", (0, 0, 0))),
                "axis": _floats(axis.get("xyz") if axis is not None else "", (1, 0, 0)),
                "mimic": None if mimic is None else (
                    mimic.get("joint"), float(mimic.get("multiplier", 1.0)),
                    float(mimic.get("offset", 0.0))),
                "limit": None if limit is None or limit.get("lower") is None else (
                    float(limit.get("lower")), float(limit.get("upper"))),
            }
            self.joints[entry["name"]] = entry
            self.by_child[entry["child"]] = entry
        self.ros2_control = [j.get("name") for j in root.findall("ros2_control/joint")]
        self.plugins = [p.text for p in root.findall("ros2_control/hardware/plugin")]

    def value(self, name, q):
        joint = self.joints[name]
        if joint["mimic"]:
            master, multiplier, offset = joint["mimic"]
            return multiplier * self.value(master, q) + offset
        return q.get(name, 0.0)

    def local(self, joint, value):
        if joint["type"] in ("revolute", "continuous"):
            motion = _homogeneous(_axis_angle(joint["axis"], value), (0, 0, 0))
        elif joint["type"] == "prismatic":
            motion = _homogeneous(np.eye(3), np.asarray(joint["axis"]) * value)
        else:
            motion = np.eye(4)
        return joint["origin"] @ motion

    def world(self, link, q):
        transform = np.eye(4)
        while link in self.by_child:
            joint = self.by_child[link]
            transform = self.local(joint, self.value(joint["name"], q)) @ transform
            link = joint["parent"]
        return transform

    def mimics(self, prefix):
        return [j for j in self.joints.values()
                if j["mimic"] and j["name"].startswith(f"{prefix}_hj_")]


def _quaternion_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _pose_error(a, b):
    """(translation error m, rotation error rad) between two 4x4 transforms."""
    relative = a[:3, :3].T @ b[:3, :3]
    cosine = max(-1.0, min(1.0, (np.trace(relative) - 1.0) / 2.0))
    return float(np.linalg.norm(a[:3, 3] - b[:3, 3])), math.acos(cosine)


def _pose_distance(a, b):
    return max(_pose_error(a, b))


def _rotation_about(axis, rotation):
    """Angle of ``rotation`` about unit ``axis`` (rotation must be about axis)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    skew = (rotation - rotation.T) / 2.0
    sine = float(np.dot(a, [skew[2, 1], skew[0, 2], skew[1, 0]]))
    cosine = (np.trace(rotation) - 1.0) / 2.0
    return math.atan2(sine, cosine)


# ---------------------------------------------------------------- ROS client
class Probe(Node):
    def __init__(self, name):
        super().__init__(name)
        OWN_NODES.add(name)
        self.latest = None
        self.watch = None
        self.bad_messages = []
        self.expected_names = None
        self.create_subscription(JointState, "/joint_states", self._state, 50)
        self.description = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description",
                                 lambda m: setattr(self, "description", m.data), latched)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.actions = {}
        self.kinematics = None

    def destroy_node(self):
        # Action clients keep the rcl node (and its graph entry) alive otherwise.
        for action in self.actions.values():
            action.destroy()
        self.actions.clear()
        return super().destroy_node()

    def _state(self, message):
        self.latest = message
        if self.expected_names is not None:
            if (len(message.name) != len(set(message.name))
                    or set(message.name) != self.expected_names):
                self.bad_messages.append(list(message.name))
        if self.watch is not None:
            baseline, names, worst = self.watch
            positions = dict(zip(message.name, message.position))
            for name in names:
                worst[name] = max(worst.get(name, 0.0), abs(positions[name] - baseline[name]))

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
            self.spin_until(client.service_is_ready, timeout, f"service {name}")
            future = client.call_async(request or service_type.Request())
            self.spin_until(future.done, timeout, f"reply from {name}")
            return future.result()
        finally:
            self.destroy_client(client)

    def state(self):
        self.spin_until(lambda: self.latest is not None, label="/joint_states")
        return dict(zip(self.latest.name, self.latest.position))

    def tf(self, target, source):
        self.spin_until(lambda: self.buffer.can_transform(
            target, source, Time(), timeout=RclpyDuration(seconds=0.0)),
            label=f"TF {target} <- {source}")
        t = self.buffer.lookup_transform(target, source, Time()).transform
        r = t.rotation
        return _homogeneous(_quaternion_matrix(r.x, r.y, r.z, r.w),
                            (t.translation.x, t.translation.y, t.translation.z))

    def send(self, commands, seconds=1.5):
        """Send every goal before waiting for any result; returns timing."""
        handles, sent_at = {}, {}
        for group, positions in commands.items():
            name = CONTROLLER[group]
            if name not in self.actions:
                self.actions[name] = ActionClient(
                    self, FollowJointTrajectory, f"/{name}/follow_joint_trajectory")
            action = self.actions[name]
            self.spin_until(action.server_is_ready, label=f"{name} action server")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(GROUPS[group])
            point = JointTrajectoryPoint()
            point.positions = [float(v) for v in positions]
            point.time_from_start = Duration(sec=int(seconds),
                                             nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [point]
            handles[group] = action.send_goal_async(goal)
            sent_at[group] = time.monotonic()
        accepted_at, results, done_at = {}, {}, {}
        for group, future in handles.items():
            self.spin_until(future.done, label=f"{group} goal acceptance")
            handle = future.result()
            if not handle.accepted:
                raise AssertionError(f"{group} goal rejected")
            accepted_at[group] = time.monotonic()
            results[group] = handle.get_result_async()
        self.spin_until(lambda: self._stamp(results, done_at), timeout=15.0,
                        label="goal results")
        codes = {g: f.result().result.error_code for g, f in results.items()}
        return codes, accepted_at, done_at

    @staticmethod
    def _stamp(results, done_at):
        now = time.monotonic()
        for group, future in results.items():
            if future.done() and done_at.get(group) is None:
                done_at[group] = now
        return all(future.done() for future in results.values())


# ---------------------------------------------------------------- launch control
class Launch:
    def __init__(self, scenario, configuration, policy):
        LOG_ROOT.mkdir(parents=True, exist_ok=True)
        self.log_path = LOG_ROOT / f"{scenario}.launch.log"
        self.log = open(self.log_path, "w")
        self.process = subprocess.Popen(
            ["ros2", "launch", "openarm_bringup", "openarm.rh56f1_bimanual.launch.py",
             f"canonical_urdf:={CANONICAL}", f"hand_configuration:={configuration}",
             f"rh56f1_state_policy:={policy}", "use_fake_hardware:=true",
             "use_rviz:=false"],
            stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop(self):
        """SIGINT the whole launch group; report whether escalation was needed."""
        escalated = None
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                escalated = "SIGTERM"
                os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    escalated = "SIGKILL"
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait()
        try:
            os.killpg(self.process.pid, 0)
            group_alive = True
        except ProcessLookupError:
            group_alive = False
        self.log.close()
        return escalated, group_alive


def _ros_processes():
    """ROS processes judged by their executable (first tokens), not by any text."""
    out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True, text=True).stdout
    names = {"ros2_control_node", "robot_state_publisher", "spawner", "rviz2",
             "humble_fake_runtime_probe.py"}
    found = []
    for line in out.splitlines():
        pid, _, args = line.strip().partition(" ")
        tokens = [os.path.basename(t) for t in args.split()[:3]]
        is_launch = "ros2" in tokens[:2] and "launch" in tokens[1:3]
        if int(pid) != os.getpid() and (is_launch or names & set(tokens[:2])):
            found.append(line.strip())
    return found


# ---------------------------------------------------------------- scenario pieces
def wait_ready(node, controllers, expected_joints):
    node.spin_until(lambda: node.description is not None, label="/robot_description")
    node.expected_names = set(expected_joints)

    def active():
        reply = node.call(ListControllers, "/controller_manager/list_controllers")
        states = {c.name: c.state for c in reply.controller}
        return set(states) == set(controllers) and all(s == "active" for s in states.values())

    deadline = time.monotonic() + 40
    while time.monotonic() < deadline:
        try:
            if active():
                return
        except AssertionError:
            pass
        time.sleep(0.3)
    reply = node.call(ListControllers, "/controller_manager/list_controllers")
    raise AssertionError(f"controllers not active: {[(c.name, c.state) for c in reply.controller]}")


def check_fake_only(node, tag, kinematics):
    """Interlock: nothing but GenericSystem may be loaded before a goal is sent."""
    components = node.call(ListHardwareComponents,
                           "/controller_manager/list_hardware_components").component
    classes = sorted({c.class_type for c in components})
    ok = (kinematics.plugins == ["mock_components/GenericSystem"]
          and classes == ["mock_components/GenericSystem"])
    check(f"{tag}.only_generic_system", ok,
          f"description={kinematics.plugins} runtime={[(c.name, c.class_type) for c in components]}")
    if not ok:
        raise AssertionError("a non-GenericSystem hardware component is present; refusing to command")


def check_interfaces(node, tag, kinematics, expected_joints, controllers, hands_claimed):
    mimic_names = {j["name"] for j in kinematics.joints.values() if j["mimic"]}
    check(f"{tag}.ros2_control_resources_manifest_order",
          kinematics.ros2_control == [n for n in ORDER if n in expected_joints],
          f"{len(kinematics.ros2_control)} resources")
    reply = node.call(ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces")
    commands = {i.name: i for i in reply.command_interfaces}
    states = {i.name: i for i in reply.state_interfaces}
    wanted = {f"{j}/position" for j in expected_joints}
    check(f"{tag}.command_interfaces_exact", set(commands) == wanted,
          f"{len(commands)} command interfaces, all position")
    check(f"{tag}.state_interfaces_exact", set(states) == wanted, f"{len(states)} state interfaces")
    check(f"{tag}.no_mimic_hardware_axis",
          not any(i.split("/")[0] in mimic_names for i in list(commands) + list(states)),
          f"{len(mimic_names)} mimic joints in URDF, 0 exported")
    hand = [n for n in commands if "_hj_" in n]
    check(f"{tag}.hand_command_interfaces_claimed={hands_claimed}",
          all(commands[n].is_available and commands[n].is_claimed is hands_claimed for n in hand),
          f"{len(hand)} hand command interfaces")

    listed = node.call(ListControllers, "/controller_manager/list_controllers").controller
    by_name = {c.name: c for c in listed}
    check(f"{tag}.controller_names", set(by_name) == set(controllers), str(sorted(by_name)))
    claims = [i for c in listed for i in c.claimed_interfaces]
    check(f"{tag}.no_interface_claimed_twice", len(claims) == len(set(claims)),
          f"{len(claims)} claimed")
    check(f"{tag}.broadcaster_claims_nothing",
          list(by_name["joint_state_broadcaster"].claimed_interfaces) == [])
    for group, name in CONTROLLER.items():
        if name not in by_name:
            continue
        controller = by_name[name]
        joints = node.call(GetParameters, f"/{name}/get_parameters",
                           GetParameters.Request(names=["joints"])).values[0].string_array_value
        check(f"{tag}.{name}.joint_order", list(joints) == GROUPS[group], " ".join(joints))
        check(f"{tag}.{name}.claimed",
              sorted(controller.claimed_interfaces) == sorted(f"{j}/position" for j in GROUPS[group])
              and controller.type == "joint_trajectory_controller/JointTrajectoryController",
              f"{len(controller.claimed_interfaces)} x position")


def check_joint_states(node, tag, expected_joints, kinematics):
    node.state()
    publishers = node.count_publishers("/joint_states")
    names = list(node.latest.name)
    mimic_names = {j["name"] for j in kinematics.joints.values() if j["mimic"]}
    check(f"{tag}.joint_states_single_publisher", publishers == 1, f"publishers={publishers}")
    check(f"{tag}.joint_states_each_once",
          len(names) == len(set(names)) == len(expected_joints) and set(names) == set(expected_joints),
          f"{len(names)} names")
    check(f"{tag}.joint_states_no_mimic", not set(names) & mimic_names)


def frames(prefix):
    return [f"{prefix}_hl_palm_sensor"] + [f"{prefix}_hl_{f}_tip" for f in FINGERS]


def snapshot(node, sides):
    shot = {}
    for side in sides:
        p = PREFIX[side]
        palm = f"{p}_hl_palm_sensor"
        for frame in frames(p):
            shot[("world", frame)] = node.tf("body_root", frame)
            if frame != palm:
                shot[("palm", frame)] = node.tf(palm, frame)
        for joint in node.kinematics.mimics(p):
            shot[("mimic", joint["name"])] = node.tf(joint["parent"], joint["child"])
    return shot


def wait_tf_matches_fk(node, tag, q, sides):
    links = [f for side in sides for f in frames(PREFIX[side])]
    links += [j["child"] for side in sides for j in node.kinematics.mimics(PREFIX[side])]
    worst = {}

    def converged():
        worst.clear()
        for link in links:
            if not node.buffer.can_transform("body_root", link, Time()):
                return False
            error = _pose_error(node.kinematics.world(link, q), node.tf("body_root", link))
            worst[link] = error
        return all(max(e) < FK_TOL for e in worst.values())

    try:
        node.spin_until(converged, timeout=10.0, label="TF == FK")
        ok = True
    except AssertionError:
        ok = False
    t, r = (max(e[0] for e in worst.values()), max(e[1] for e in worst.values())) if worst else (-1, -1)
    check(f"{tag}.tf_matches_fk", ok, f"{len(links)} frames, max {t:.1e} m / {r:.1e} rad")


def check_mimics(node, tag, q, sides):
    worst, count = 0.0, 0
    for side in sides:
        for joint in node.kinematics.mimics(PREFIX[side]):
            measured = node.tf(joint["parent"], joint["child"])
            motion = np.linalg.inv(joint["origin"]) @ measured
            angle = _rotation_about(joint["axis"], motion[:3, :3])
            expected = node.kinematics.value(joint["name"], q)
            worst = max(worst, abs(angle - expected))
            count += 1
    check(f"{tag}.mimic_follows_master", worst < FK_TOL,
          f"{count} mimic joints, max |measured - (multiplier*master+offset)| {worst:.1e} rad")


def step(node, tag, expected, commands, sides, before):
    """Command ``commands`` (group -> positions) and verify every contract."""
    for group, positions in commands.items():
        for joint, value in zip(GROUPS[group], positions):
            low, high = node.kinematics.joints[joint]["limit"]
            if not low <= value <= high:
                raise AssertionError(f"{joint} target {value} outside URDF [{low}, {high}]")
    commanded = {j for g in commands for j in GROUPS[g]}
    held = [j for j in expected if j not in commanded]
    node.watch = (dict(node.state()), held, {})
    codes, accepted, done = node.send(commands)
    target = dict(expected)
    for group, positions in commands.items():
        target.update(zip(GROUPS[group], positions))

    def reached():
        s = node.state()
        return all(abs(s[j] - target[j]) < STATE_TOL for j in commanded)

    node.spin_until(reached, timeout=10.0, label=f"{tag} state")
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.05)
    worst_held = max(node.watch[2].values(), default=0.0)
    node.watch = None
    state = node.state()
    check(f"{tag}.goals_succeeded", all(c == 0 for c in codes.values()), str(codes))
    if len(commands) > 1:
        check(f"{tag}.goals_overlapped_in_time", max(accepted.values()) < min(done.values()),
              f"last accept {max(accepted.values()) - min(accepted.values()):.3f}s after first; "
              f"first finish {min(done.values()) - max(accepted.values()):.3f}s after that")
    error = max(abs(state[j] - target[j]) for j in commanded)
    check(f"{tag}.commanded_joints_reached", error < STATE_TOL,
          f"{len(commanded)} joints, max error {error:.1e} rad")
    check(f"{tag}.uncommanded_joints_never_moved", worst_held < HOLD_TOL,
          f"{len(held)} joints, max excursion during motion {worst_held:.1e} rad")
    check(f"{tag}.joint_states_names_stable", not node.bad_messages,
          f"{len(node.bad_messages)} malformed messages")
    wait_tf_matches_fk(node, tag, state, sides)
    check_mimics(node, tag, state, sides)
    after = snapshot(node, sides)

    moved_groups = set(commands)
    for side in sides:
        p = PREFIX[side]
        palm = f"{p}_hl_palm_sensor"
        tips = [f for f in frames(p) if f != palm]
        arm_moved = f"{side}_arm" in moved_groups
        hand_moved = f"{side}_hand" in moved_groups
        world_palm = _pose_distance(before[("world", palm)], after[("world", palm)])
        world_tips = min(_pose_distance(before[("world", f)], after[("world", f)]) for f in tips)
        relative = [_pose_distance(before[("palm", f)], after[("palm", f)]) for f in tips]
        mimic = [_pose_distance(before[k], after[k]) for k in before
                 if k[0] == "mimic" and k[1].startswith(f"{p}_hj_")]
        if arm_moved:
            check(f"{tag}.{side}.palm_world_moved", world_palm > POSE_MOVED, f"{world_palm:.4f}")
            check(f"{tag}.{side}.fingertips_world_moved", world_tips > POSE_MOVED,
                  f"min {world_tips:.4f}")
        else:
            check(f"{tag}.{side}.palm_world_unchanged", world_palm < POSE_SAME, f"{world_palm:.1e}")
        if hand_moved:
            check(f"{tag}.{side}.fingertips_rel_palm_moved", min(relative) > POSE_MOVED,
                  f"min {min(relative):.4f} over {len(relative)} tips")
            check(f"{tag}.{side}.mimic_links_moved", min(mimic) > POSE_MOVED,
                  f"min {min(mimic):.4f} over {len(mimic)}")
        else:
            check(f"{tag}.{side}.fingertips_rel_palm_unchanged", max(relative) < POSE_SAME,
                  f"max {max(relative):.1e}")
            check(f"{tag}.{side}.mimic_links_unchanged", max(mimic) < POSE_SAME,
                  f"max {max(mimic):.1e}")
        if not arm_moved and not hand_moved:
            check(f"{tag}.{side}.fingertips_world_unchanged",
                  max(_pose_distance(before[("world", f)], after[("world", f)]) for f in tips)
                  < POSE_SAME)
    return target, after


def teardown(launch, tag):
    escalated, group_alive = launch.stop()
    check(f"{tag}.launch_stopped_by_sigint", escalated is None and not group_alive,
          f"escalated={escalated}")
    remaining = []
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        remaining = _ros_processes()
        if not remaining:
            break
        time.sleep(0.5)
    check(f"{tag}.no_ros_process_left", not remaining, str(remaining))
    generated = glob.glob(os.path.join(tempfile.gettempdir(), "openarm_rh56f1_fake_hands_*"))
    check(f"{tag}.generated_hand_params_removed", not generated, str(generated))
    watcher = Node(f"fake_motion_graph_{tag}")
    OWN_NODES.add(watcher.get_name())
    try:
        deadline = time.monotonic() + 15
        nodes = []
        while time.monotonic() < deadline:
            rclpy.spin_once(watcher, timeout_sec=0.2)
            nodes = [n for n in watcher.get_node_names() if n not in OWN_NODES]
            if not nodes:
                break
        check(f"{tag}.no_ros_node_left", not nodes, str(nodes))
    finally:
        watcher.destroy_node()


# ---------------------------------------------------------------- scenarios
def scenario_parked_both():
    tag = "parked_both"
    launch = Launch(tag, "both", "parked")
    node = Probe("fake_motion_probe_parked")
    try:
        controllers = {"joint_state_broadcaster", CONTROLLER["left_arm"], CONTROLLER["right_arm"]}
        wait_ready(node, controllers, ORDER)
        node.kinematics = Kinematics(node.description)
        check_fake_only(node, tag, node.kinematics)
        check_interfaces(node, tag, node.kinematics, ORDER, controllers, hands_claimed=False)
        hands = GROUPS["right_hand"] + GROUPS["left_hand"]
        end = time.monotonic() + 2.0
        worst = 0.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            worst = max(worst, max(abs(node.state()[j]) for j in hands))
        check(f"{tag}.hands_zero_parked", worst == 0.0, f"12 actuators, max |q| {worst}")
        check_joint_states(node, tag, ORDER, node.kinematics)
        # The existing Stage 3 probe: arms move, parked hands follow rigidly.
        run = subprocess.run([sys.executable, str(PARKED_PROBE)], capture_output=True,
                             text=True, timeout=120)
        (LOG_ROOT / f"{tag}.existing_probe.log").write_text(run.stdout + run.stderr)
        check(f"{tag}.existing_humble_fake_runtime_probe",
              run.returncode == 0 and "RUNTIME_PROBE=PASS" in run.stdout,
              " ".join(l for l in run.stdout.split() if "=" in l))
        listed = node.call(ListControllers, "/controller_manager/list_controllers").controller
        check(f"{tag}.still_no_hand_controller",
              not any(c.name in FAKE_HAND_CONTROLLERS.values() for c in listed)
              and len(listed) == 3, str(sorted(c.name for c in listed)))
        state = node.state()
        check(f"{tag}.hands_still_zero_after_arm_motion",
              all(state[j] == 0.0 for j in hands))
    finally:
        node.destroy_node()
        teardown(launch, tag)


def scenario_commandable_right():
    tag = "commandable_right"
    launch = Launch(tag, "right", "fake_commandable")
    node = Probe("fake_motion_probe_right")
    joints = [n for n in ORDER if not n.startswith("l_hj_")]
    try:
        controllers = {"joint_state_broadcaster", CONTROLLER["left_arm"],
                       CONTROLLER["right_arm"], CONTROLLER["right_hand"]}
        wait_ready(node, controllers, joints)
        node.kinematics = Kinematics(node.description)
        check_fake_only(node, tag, node.kinematics)
        check_interfaces(node, tag, node.kinematics, joints, controllers, hands_claimed=True)
        check_joint_states(node, tag, joints, node.kinematics)
        expected = {j: 0.0 for j in joints}
        check(f"{tag}.start_all_zero", all(abs(node.state()[j]) < 1e-9 for j in joints))
        sides = ("right",)
        wait_tf_matches_fk(node, f"{tag}.start", node.state(), sides)
        before = snapshot(node, sides)
        left_wrist = node.tf("body_root", "l_al_7")

        expected, before = step(node, f"{tag}.1_right_arm_only", expected,
                                {"right_arm": TARGETS["right_arm"][0]}, sides, before)
        expected, before = step(node, f"{tag}.2_right_hand_only", expected,
                                {"right_hand": TARGETS["right_hand"][0]}, sides, before)
        expected, before = step(node, f"{tag}.3_right_arm_and_hand", expected,
                                {"right_arm": TARGETS["right_arm"][1],
                                 "right_hand": TARGETS["right_hand"][1]}, sides, before)
        expected, before = step(node, f"{tag}.4_right_arm_only_bent_hand", expected,
                                {"right_arm": TARGETS["right_arm"][0]}, sides, before)
        state = node.state()
        check(f"{tag}.left_arm_never_moved",
              all(state[j] == 0.0 for j in GROUPS["left_arm"])
              and _pose_distance(left_wrist, node.tf("body_root", "l_al_7")) < POSE_SAME)
        check_joint_states(node, f"{tag}.end", joints, node.kinematics)
    finally:
        node.destroy_node()
        teardown(launch, tag)


def scenario_commandable_both():
    tag = "commandable_both"
    launch = Launch(tag, "both", "fake_commandable")
    node = Probe("fake_motion_probe_both")
    try:
        controllers = {"joint_state_broadcaster", *CONTROLLER.values()}
        wait_ready(node, controllers, ORDER)
        node.kinematics = Kinematics(node.description)
        check_fake_only(node, tag, node.kinematics)
        check_interfaces(node, tag, node.kinematics, ORDER, controllers, hands_claimed=True)
        check_joint_states(node, tag, ORDER, node.kinematics)
        sides = ("right", "left")
        expected = {j: 0.0 for j in ORDER}
        wait_tf_matches_fk(node, f"{tag}.start", node.state(), sides)
        before = snapshot(node, sides)
        for index, group in enumerate(("right_arm", "right_hand", "left_arm", "left_hand"), 1):
            expected, before = step(node, f"{tag}.{index}_{group}_only", expected,
                                    {group: TARGETS[group][0]}, sides, before)
        expected, before = step(node, f"{tag}.5_all_four_groups", expected,
                                {g: TARGETS[g][1] for g in GROUPS}, sides, before)
        check_joint_states(node, f"{tag}.end", ORDER, node.kinematics)
    finally:
        node.destroy_node()
        teardown(launch, tag)


_GRAPH_SNIPPET = """
import time, rclpy
rclpy.init()
node = rclpy.create_node("fake_motion_final_graph_check")
end = time.monotonic() + 3.0
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
print(sorted(n for n in node.get_node_names() if n != node.get_name()))
node.destroy_node()
rclpy.shutdown()
"""


SCENARIOS = {
    "parked_both": scenario_parked_both,
    "commandable_right": scenario_commandable_right,
    "commandable_both": scenario_commandable_both,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", choices=[*SCENARIOS, "all"], default="all")
    args = parser.parse_args()
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        print("refusing to run: set an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
        return 2
    leftovers = _ros_processes()
    if leftovers:
        print(f"refusing to run: ROS processes already running: {leftovers}")
        return 2
    rclpy.init()
    try:
        for name in (SCENARIOS if args.scenario == "all" else [args.scenario]):
            print(f"=== scenario {name}", flush=True)
            try:
                SCENARIOS[name]()
            except Exception as error:  # a scenario error must not skip teardown of others
                check(f"{name}.completed", False, repr(error))
    finally:
        rclpy.shutdown()
    # A fresh participant sees the whole domain, this probe included.
    graph = subprocess.run([sys.executable, "-c", _GRAPH_SNIPPET], capture_output=True,
                           text=True, timeout=60)
    check("final.no_ros_node_left_in_domain", graph.returncode == 0 and graph.stdout.strip() == "[]",
          graph.stdout.strip() or graph.stderr.strip()[-200:])
    check("final.no_ros_process_left", not _ros_processes(), str(_ros_processes()))
    print(f"CHECKS_FAILED={len(FAILURES)}")
    print(f"FAKE_MOTION_PROBE={'PASS' if not FAILURES else 'FAIL'}", flush=True)
    return 0 if not FAILURES else 1


if __name__ == "__main__":
    sys.exit(main())
