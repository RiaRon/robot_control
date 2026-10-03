#!/usr/bin/env python3
"""Humble runtime probe for the REAL OpenArm + RH56F1 description, with test doubles.

Run only inside the isolated Humble container (no network, no devices), after
building openarm_description, openarm_bringup, rh56f1_hardware and
openarm_hardware into an overlay. It never touches hardware:

  - Arms: OpenArmHW needs SocketCAN, so each arm component's plugin is swapped
    for mock_components/GenericSystem with the same joints and interfaces
    (rh56f1_real_description arm_test_double). OpenArmHW's own startup gate,
    fresh-state check and stale latch are covered by its gtest and review.
  - Hands: the real rh56f1_hardware/Rh56f1HW plugin with transport=mock
    (standing in for a device) or rs485 (the refusing stub).

Commands go only to these doubles. Each scenario prints CHECK lines and the
probe ends with RUNTIME_PROBE=PASS or FAIL.
"""

from __future__ import annotations

import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import DynamicJointState
from controller_manager_msgs.srv import (ListControllers, ListHardwareComponents,
                                         SetHardwareComponentState, SwitchController)
from lifecycle_msgs.msg import State
from sensor_msgs.msg import JointState
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

LAUNCH_DIR = Path(get_package_share_directory("openarm_bringup")) / "launch"
sys.path.insert(0, str(LAUNCH_DIR))
import rh56f1_real_description as real  # noqa: E402

KUKU = Path(os.environ.get("KUKU_LAB_ROOT", "/workspace/kuku_lab"))
CANONICAL = KUKU / "urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"
MANIFEST = KUKU / "urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml"
WRAPPER = (Path(get_package_share_directory("openarm_description"))
           / "urdf/robot/openarm_rh56f1_bimanual_real.urdf.xacro")
STATIC_CONTROLLERS = (Path(get_package_share_directory("openarm_bringup"))
                      / "config/controllers/openarm_rh56f1_real_controllers.yaml")
LOG_ROOT = Path(os.environ.get("PROBE_LOG_DIR", "/tmp/rh56f1_probe"))
INITIAL = [0.30, 0.20, 0.30, 0.30, 0.30, 0.30]
FAULT_WRITE_BITS = {"comm_stale": 1, "transport": 2, "device": 4, "non_finite": 8}

RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    RESULTS.append((name, bool(ok), detail))
    print(f"CHECK {name}={'PASS' if ok else 'FAIL'} {detail}", flush=True)
    return ok


def render(hand_configuration, transports, enable_arms=(True, True), extra=None):
    return real.render_real_control_description(
        source_urdf=CANONICAL, manifest=MANIFEST, wrapper_xacro=WRAPPER,
        hand_configuration=hand_configuration, right_can_interface="vcan_unused0",
        left_can_interface="vcan_unused1", can_fd=True,
        enable_right_arm=enable_arms[0], enable_left_arm=enable_arms[1],
        right_hand_transport=transports["right"], left_hand_transport=transports["left"],
        right_hand_port="/dev/null", left_hand_port="/dev/null",
        right_hand_baudrate=115200, left_hand_baudrate=115200,
        right_hand_device_id=1, left_hand_device_id=2,
        arm_test_double=True, extra_hardware_params=extra or {})


class Graph:
    """robot_state_publisher + ros2_control_node + one spawner per controller."""

    def __init__(self, name, description, plan, *, initial_state=True):
        self.dir = LOG_ROOT / name
        self.dir.mkdir(parents=True, exist_ok=True)
        static = yaml.safe_load(STATIC_CONTROLLERS.read_text())
        params = real.controller_params(static, plan)
        if not initial_state:
            params["controller_manager"]["ros__parameters"].pop(
                "hardware_components_initial_state", None)
        params["controller_manager"]["ros__parameters"]["robot_description"] = description
        (self.dir / "cm.yaml").write_text(yaml.safe_dump(params, sort_keys=False))
        (self.dir / "rsp.yaml").write_text(yaml.safe_dump(
            {"robot_state_publisher": {"ros__parameters": {"robot_description": description}}}))
        (self.dir / "robot_description.urdf").write_text(description)
        self.plan = plan
        self.processes = []

    def _run(self, label, *args):
        log = open(self.dir / f"{label}.log", "w")
        process = subprocess.Popen(["ros2", "run", *args], stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.processes.append((label, process))
        return process

    def start(self, node):
        self._run("rsp", "robot_state_publisher", "robot_state_publisher", "--ros-args",
                  "--params-file", str(self.dir / "rsp.yaml"))
        self._run("cm", "controller_manager", "ros2_control_node", "--ros-args",
                  "--params-file", str(self.dir / "cm.yaml"))
        node.wait_for_service(node.list_hw, "list_hardware_components")
        spawners = []
        for name, spec in self.plan["broadcasters"].items():
            mode = [] if spec["active_at_launch"] else ["--inactive"]
            spawners.append(self._run(f"spawn_{name}", "controller_manager", "spawner", name,
                                      *mode, "-c", "/controller_manager"))
        for name in self.plan["controllers"]:
            spawners.append(self._run(f"spawn_{name}", "controller_manager", "spawner", name,
                                      "--inactive", "-c", "/controller_manager"))
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and any(p.poll() is None for p in spawners):
            rclpy.spin_once(node, timeout_sec=0.1)
        return {label: p.poll() for label, p in self.processes if label.startswith("spawn_")}

    def stop(self):
        for _, process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for _, process in self.processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)


class Probe(Node):
    def __init__(self):
        super().__init__("rh56f1_runtime_probe")
        self.list_hw = self.create_client(ListHardwareComponents,
                                          "/controller_manager/list_hardware_components")
        self.set_hw = self.create_client(SetHardwareComponentState,
                                         "/controller_manager/set_hardware_component_state")
        self.list_ctrl = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.joint_messages: list[JointState] = []
        self.local_messages: dict[str, list[JointState]] = {}
        self.status: dict[str, dict[str, dict[str, float]]] = {}
        self.create_subscription(JointState, "/joint_states", self.joint_messages.append, 100)
        for side in ("right", "left"):
            topic = f"/rh56f1_{side}_hand_state_broadcaster/joint_states"
            self.local_messages[side] = []
            self.create_subscription(JointState, topic, self.local_messages[side].append, 50)
            self.create_subscription(
                DynamicJointState, f"/rh56f1_{side}_hand_status_broadcaster/dynamic_joint_states",
                lambda msg, side=side: self._status(side, msg), 50)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _status(self, side, msg):
        self.status[side] = {
            joint: dict(zip(values.interface_names, values.values))
            for joint, values in zip(msg.joint_names, msg.interface_values)}

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result()

    def wait_for_service(self, client, label, timeout=30.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{label} service did not appear")

    def connect_hand(self, side):
        """The operator's explicit configure step, then its read-only broadcasters."""
        ok = self.set_component(f"rh56f1_{side}_hand", "inactive")
        ok = ok and self.switch_controllers(activate=[
            f"rh56f1_{side}_hand_state_broadcaster", f"rh56f1_{side}_hand_status_broadcaster"])
        return ok

    def hardware_states(self):
        response = self.call(self.list_hw, ListHardwareComponents.Request())
        return {c.name: c.state.label for c in response.component} if response else {}

    def controller_states(self):
        response = self.call(self.list_ctrl, ListControllers.Request())
        return {c.name: c.state for c in response.controller} if response else {}

    def set_component(self, name, label):
        request = SetHardwareComponentState.Request()
        request.name = name
        request.target_state = State(
            id={"active": State.PRIMARY_STATE_ACTIVE, "inactive": State.PRIMARY_STATE_INACTIVE}[label],
            label=label)
        response = self.call(self.set_hw, request)
        self.spin_for(0.3)
        return bool(response and response.ok)

    def switch_controllers(self, activate=(), deactivate=()):
        request = SwitchController.Request()
        request.activate_controllers = list(activate)
        request.deactivate_controllers = list(deactivate)
        request.strictness = SwitchController.Request.STRICT
        response = self.call(self.switch, request)
        return bool(response and response.ok)

    def latest_position(self, joint):
        for msg in reversed(self.joint_messages):
            if joint in msg.name:
                return msg.position[msg.name.index(joint)]
        return math.nan

    def send_goal(self, controller, joints, positions, seconds):
        client = ActionClient(self, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=10.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        point = JointTrajectoryPoint(positions=[float(p) for p in positions])
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [point]
        future = client.send_goal_async(goal)
        end = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return bool(future.result() and future.result().accepted)

    def transform(self, parent, child):
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                return t.transform
            except tf2_ros.TransformException:
                rclpy.spin_once(self, timeout_sec=0.05)
        return None

    def status_value(self, side, joint, interface):
        return self.status.get(side, {}).get(joint, {}).get(interface, math.nan)


def distance(a, b):
    return math.dist((a.translation.x, a.translation.y, a.translation.z),
                     (b.translation.x, b.translation.y, b.translation.z))


def names(contract, side):
    return [contract["hands"][side][k]["name"] for k in real.HAND_ACTUATORS]


# ---------------------------------------------------------------------------- scenarios
def scenario_main(node):
    _, contract, _ = real.build_runtime_variant(CANONICAL, "both", MANIFEST)
    right, left = names(contract, "right"), names(contract, "left")
    initial = ",".join(str(v) for v in INITIAL)
    description = render("both", {"right": "mock", "left": "mock"}, extra={
        "rh56f1_right_hand": {"mock_initial_positions": initial},
        # Left: a dwell no operator reaches, to see activation refused.
        "rh56f1_left_hand": {"mock_initial_positions": initial,
                             "min_inactive_sec_before_activate": "600"}})
    # Right hand stands in for a real device; left is an explicitly allowed mock.
    plan = real.device_plan(contract, hand_configuration="both", enable_right_arm=True,
                            enable_left_arm=True, right_hand_transport="rs485",
                            left_hand_transport="mock", allow_mock_hands=True)
    urdf_joints = {j.get("name") for j in ET.fromstring(description).findall("joint")}
    graph = Graph("main", description, plan)
    try:
        spawn = graph.start(node)
        check("main.spawners_ok", all(code == 0 for code in spawn.values()), str(spawn))
        node.spin_for(1.0)
        hw = node.hardware_states()
        check("main.arms_start_inactive_hands_unconfigured",
              hw == {"openarm_rh56f1_right_arm": "inactive", "openarm_rh56f1_left_arm": "inactive",
                     "rh56f1_right_hand": "unconfigured", "rh56f1_left_hand": "unconfigured"},
              str(hw))
        ctrl = node.controller_states()
        check("main.device_controllers_inactive",
              all(ctrl.get(c) == "inactive" for c in plan["controllers"]), str(ctrl))
        check("main.arm_broadcaster_active_hand_broadcasters_loaded_inactive",
              ctrl.get("joint_state_broadcaster") == "active"
              and all(ctrl.get(b) == "inactive" for b in plan["broadcasters"] if b.startswith("rh56f1_")),
              str(ctrl))
        check("main.explicit_hand_configure", node.connect_hand("right") and node.connect_hand("left"),
              str(node.hardware_states()))
        node.spin_for(3.0)  # longer than the 2.0 s activation dwell after configure
        node.joint_messages.clear()
        node.spin_for(1.0)

        seen, duplicates = set(), False
        for msg in node.joint_messages[-200:]:
            duplicates |= len(msg.name) != len(set(msg.name))
            seen |= set(msg.name)
        check("main.joint_states_exactly_real_devices", seen == set(plan["real_joints"]),
              f"{len(seen)} names; missing={sorted(set(plan['real_joints']) - seen)} "
              f"extra={sorted(seen - set(plan['real_joints']))}")
        check("main.joint_states_no_duplicates", not duplicates)
        check("main.joint_states_names_in_runtime_urdf", seen <= urdf_joints,
              str(sorted(seen - urdf_joints)))
        mock_seen = {n for m in node.local_messages["left"] for n in m.name}
        check("main.mock_hand_only_on_its_local_topic",
              mock_seen == set(left) and seen.isdisjoint(left), str(sorted(mock_seen)))
        check("main.mock_flag_visible",
              node.status_value("left", left[0], "is_mock") == 1.0)
        writes_before = {s: node.status_value(s, (right if s == "right" else left)[0], "writes")
                         for s in ("right", "left")}
        check("main.no_device_write_before_activation",
              all(v == 0.0 for v in writes_before.values()), str(writes_before))

        refused = not node.set_component("rh56f1_left_hand", "active")
        check("main.activation_refused_before_dwell",
              refused and node.hardware_states().get("rh56f1_left_hand") == "inactive",
              str(node.hardware_states().get("rh56f1_left_hand")))

        check("main.explicit_hand_activation", node.set_component("rh56f1_right_hand", "active")
              and node.hardware_states().get("rh56f1_right_hand") == "active")
        node.spin_for(1.0)
        writes = node.status_value("right", right[0], "writes")
        held = [node.latest_position(j) for j in right]
        check("main.activation_holds_measured_pose",
              writes > 0 and all(abs(a - b) < 1e-9 for a, b in zip(held, INITIAL)),
              f"writes={writes} positions={held}")
        check("main.right_hand_no_fault", node.status_value("right", right[0], "fault") == 0.0)

        check("main.explicit_arm_activation",
              node.set_component("openarm_rh56f1_right_arm", "active"))
        check("main.controllers_activate", node.switch_controllers(
            activate=["rh56f1_right_arm_controller", "rh56f1_right_hand_controller"]))

        palm_before = node.transform("body_root", "r_hl_palm_sensor")
        joint2 = "openarm_right_joint2"
        target = node.latest_position(joint2) + 0.1
        node.send_goal("rh56f1_right_arm_controller", [joint2], [target], 0.5)
        node.spin_for(1.5)
        palm_after = node.transform("body_root", "r_hl_palm_sensor")
        check("main.arm_motion_moves_palm_sensor_tf",
              palm_before is not None and palm_after is not None
              and distance(palm_before, palm_after) > 1e-3,
              f"moved {distance(palm_before, palm_after) if palm_before and palm_after else 'n/a'} m")
        left_palm = node.transform("body_root", "l_hl_palm_sensor")
        check("main.left_palm_sensor_tf_present", left_palm is not None)

        index = right[2]
        mimic_child = next(j.find("child").get("link") for j in ET.fromstring(description).findall("joint")
                           if j.find("mimic") is not None and j.find("mimic").get("joint") == index)
        tip_before = node.transform("r_hl_palm_sensor", "r_hl_index_tip")
        mimic_before = node.transform("r_hl_palm_sensor", mimic_child)
        samples = []
        node.send_goal("rh56f1_right_hand_controller", [index], [INITIAL[2] + 0.10], 0.2)
        end = time.monotonic() + 1.6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
            samples.append((time.monotonic(), node.latest_position(index)))
        # Each device write moves at most 0.02 rad, writes are >= 0.1 s apart:
        # every observed jump <= 0.02 and 0.10 rad takes >= 4 write periods.
        jumps = [abs(b[1] - a[1]) for a, b in zip(samples, samples[1:]) if b[1] != a[1]]
        start = next((t for t, p in samples if abs(p - INITIAL[2]) > 1e-9), None)
        done = next((t for t, p in samples if abs(p - (INITIAL[2] + 0.10)) < 1e-9), None)
        final = node.latest_position(index)
        check("main.hand_step_limited", jumps and max(jumps) <= 0.02 + 1e-9,
              f"largest single step {max(jumps) if jumps else 'n/a'} rad over {len(jumps)} steps")
        check("main.hand_velocity_limited", start is not None and done is not None
              and done - start >= 0.4 - 0.05,
              f"0.10 rad took {None if start is None or done is None else round(done - start, 3)} s "
              f"(>= 0.4 s expected at 0.02 rad / 0.1 s)")
        check("main.hand_reaches_target", abs(final - (INITIAL[2] + 0.10)) < 1e-6, f"final={final}")
        tip_after = node.transform("r_hl_palm_sensor", "r_hl_index_tip")
        mimic_after = node.transform("r_hl_palm_sensor", mimic_child)
        check("main.hand_motion_moves_fingertip_tf", distance(tip_before, tip_after) > 1e-3)
        check("main.mimic_link_follows", distance(mimic_before, mimic_after) > 1e-4,
              f"{mimic_child}")

        thumb2 = right[1]
        upper = contract["hands"]["right"]["thumb_2"]["upper"]
        node.send_goal("rh56f1_right_hand_controller", [thumb2], [0.60], 0.5)
        clamped_seen = False
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            clamped_seen |= node.status_value("right", thumb2, "command_clamped") == 1.0
        final = node.latest_position(thumb2)
        check("main.clamp_reported_on_status", clamped_seen)
        check("main.clamped_command_stops_at_limit", abs(final - upper) < 1e-9, f"{final} vs {upper}")
        log = (graph.dir / "cm.log").read_text()
        check("main.clamp_logged", "clamped to joint limits" in log and thumb2 in log)

        # Deactivate, then explicitly re-arm: the command is reseeded from state.
        node.switch_controllers(deactivate=["rh56f1_right_hand_controller"])
        before = [node.latest_position(j) for j in right]
        check("main.hand_deactivate", node.set_component("rh56f1_right_hand", "inactive"))
        writes_inactive = node.status_value("right", right[0], "writes")
        node.spin_for(0.5)
        check("main.no_writes_while_inactive",
              node.status_value("right", right[0], "writes") == writes_inactive)
        check("main.reactivation", node.set_component("rh56f1_right_hand", "active"))
        node.spin_for(1.0)
        after = [node.latest_position(j) for j in right]
        check("main.reactivation_reseeds_without_motion",
              all(abs(a - b) < 1e-9 for a, b in zip(before, after))
              and node.status_value("right", right[0], "writes") > writes_inactive,
              f"before={before} after={after}")
        check("main.untouched_arm_stays_inactive",
              node.hardware_states().get("openarm_rh56f1_left_arm") == "inactive")
    finally:
        graph.stop()


def scenario_faults(node):
    _, contract, _ = real.build_runtime_variant(CANONICAL, "both", MANIFEST)
    right, left = names(contract, "right"), names(contract, "left")
    initial = ",".join(str(v) for v in INITIAL)
    description = render("both", {"right": "mock", "left": "mock"}, enable_arms=(False, False), extra={
        # ~50 Hz polling: reads start failing ~8 s after configure.
        "rh56f1_right_hand": {"mock_initial_positions": initial, "mock_fail_reads_after": "400"},
        "rh56f1_left_hand": {"mock_initial_positions": initial, "mock_device_error_after_reads": "300"}})
    # ~50 Hz polling from configure: right reads fail after ~8 s, left errs after ~6 s.
    plan = real.device_plan(contract, hand_configuration="both", enable_right_arm=False,
                            enable_left_arm=False, right_hand_transport="rs485",
                            left_hand_transport="rs485", allow_mock_hands=False)
    graph = Graph("faults", description, plan)
    try:
        graph.start(node)
        check("faults.hands_configure", node.connect_hand("right") and node.connect_hand("left"))
        node.spin_for(2.5)
        check("faults.both_hands_arm", node.set_component("rh56f1_right_hand", "active")
              and node.set_component("rh56f1_left_hand", "active"))
        node.spin_for(9.0)
        for side, joints, bit in (("right", right, FAULT_WRITE_BITS["comm_stale"]),
                                  ("left", left, FAULT_WRITE_BITS["device"])):
            fault = node.status_value(side, joints[0], "fault")
            writes1 = node.status_value(side, joints[0], "writes")
            node.spin_for(1.0)
            writes2 = node.status_value(side, joints[0], "writes")
            check(f"faults.{side}_latched", not math.isnan(fault) and int(fault) & bit, f"fault={fault}")
            check(f"faults.{side}_writes_stopped", writes1 == writes2 and writes1 > 0,
                  f"{writes1} -> {writes2}")
        hw = node.hardware_states()
        log = (graph.dir / "cm.log").read_text()
        check("faults.errors_surfaced", "FAULT latched" in log, f"component states after fault: {hw}")
        rearm = node.set_component("rh56f1_right_hand", "inactive") and node.set_component(
            "rh56f1_right_hand", "active")
        check("faults.rearm_refused_while_comm_lost",
              not rearm and node.hardware_states().get("rh56f1_right_hand") != "active",
              str(node.hardware_states()))
    finally:
        graph.stop()


def scenario_bypass(node):
    """No hardware_components_initial_state: controller_manager auto-activates.

    The hand plugins refuse the immediate auto-activation; Humble's
    controller_manager then aborts the whole node. Either way nothing is armed.
    """
    _, contract, _ = real.build_runtime_variant(CANONICAL, "both", MANIFEST)
    description = render("both", {"right": "mock", "left": "mock"}, enable_arms=(False, False))
    plan = real.device_plan(contract, hand_configuration="both", enable_right_arm=False,
                            enable_left_arm=False, right_hand_transport="rs485",
                            left_hand_transport="rs485", allow_mock_hands=False)
    graph = Graph("bypass", description, plan, initial_state=False)
    try:
        graph._run("cm", "controller_manager", "ros2_control_node", "--ros-args",
                   "--params-file", str(graph.dir / "cm.yaml"))
        cm = graph.processes[-1][1]
        try:
            cm.wait(timeout=20)
        except subprocess.TimeoutExpired:
            pass
        log = (graph.dir / "cm.log").read_text()
        check("bypass.auto_activation_refused_by_plugin",
              "auto-activation at startup is refused" in log)
        check("bypass.nothing_armed", "ARMED" not in log and "commands enabled" not in log)
        check("bypass.node_did_not_run_with_active_hardware", cm.poll() not in (None, 0),
              f"exit={cm.poll()}")
    finally:
        graph.stop()


def scenario_rs485(node):
    _, contract, _ = real.build_runtime_variant(CANONICAL, "both", MANIFEST)
    description = render("both", {"right": "rs485", "left": "mock"})
    plan = real.device_plan(contract, hand_configuration="both", enable_right_arm=True,
                            enable_left_arm=True, right_hand_transport="rs485",
                            left_hand_transport="mock", allow_mock_hands=True)
    graph = Graph("rs485", description, plan)
    try:
        spawn = graph.start(node)
        node.spin_for(1.0)
        refused = not node.set_component("rh56f1_right_hand", "inactive")
        hw = node.hardware_states()
        check("rs485.configure_refused_node_survives", refused and hw.get("rh56f1_right_hand") in
              ("unconfigured", "finalized"), str(hw))
        check("rs485.arms_unaffected", hw.get("openarm_rh56f1_right_arm") == "inactive"
              and hw.get("openarm_rh56f1_left_arm") == "inactive", str(hw))
        check("rs485.other_hand_still_connects", node.connect_hand("left")
              and node.hardware_states().get("rh56f1_left_hand") == "inactive")
        ctrl = node.controller_states()
        check("rs485.arm_state_still_published", ctrl.get("joint_state_broadcaster") == "active"
              and any(m.name for m in node.joint_messages[-20:]), str(spawn))
        check("rs485.refusal_logged", "not_implemented" in (graph.dir / "cm.log").read_text())
    finally:
        graph.stop()


def main():
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Probe()
    try:
        for scenario in (scenario_main, scenario_faults, scenario_bypass, scenario_rs485):
            print(f"=== {scenario.__name__}", flush=True)
            try:
                scenario(node)
            except Exception as error:  # noqa: BLE001
                check(f"{scenario.__name__}.completed", False, repr(error))
            node.spin_for(2.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    failed = [name for name, ok, _ in RESULTS if not ok]
    print(f"CHECKS {len(RESULTS) - len(failed)}/{len(RESULTS)} passed; failed={failed}")
    print(f"RUNTIME_PROBE={'PASS' if not failed else 'FAIL'}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
