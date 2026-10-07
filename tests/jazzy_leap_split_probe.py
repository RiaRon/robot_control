#!/usr/bin/env python3
"""Jazzy runtime probe: the split OpenArm + LEAP bringup on fake hardware.

    openarm_leap_arms.launch.py         /controller_manager              14 arm joints (+ model)
    leap_hand.launch.py side:=right     /leap_right/controller_manager   16
    leap_hand.launch.py side:=left      /leap_left/controller_manager    16

Checks resources per manager, /joint_states, TF, a hand and an arm command, and a clean
shutdown. Only mock_components/GenericSystem is commanded. Run from robot_control with
ROS 2 Jazzy and ros_ws/install sourced, on an otherwise unused ROS_DOMAIN_ID:

    ROS_DOMAIN_ID=173 python3 tests/jazzy_leap_split_probe.py

Output: CHECK lines, then LEAP_SPLIT_PROBE=PASS or FAIL.
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

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, str(Path(__file__).resolve().parents[1]
                       / "ros_ws/src/openarm_ros2/openarm_bringup/launch"))
import leap_split_description as leap  # noqa: E402

CANONICAL = Path(leap.default_canonical_urdf(__file__))
OWNED = leap.device_joints(leap.default_manifest_for(CANONICAL))
MANAGER = {"arms": "/controller_manager", "right": "/leap_right/controller_manager",
           "left": "/leap_left/controller_manager"}
CONTROLLERS = {"arms": list(leap.ARM_CONTROLLERS.values()),
               "right": [leap.FAKE_HAND_CONTROLLERS["right"]],
               "left": [leap.FAKE_HAND_CONTROLLERS["left"]]}
FAILED: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    print(f"CHECK {name} {'PASS' if ok else 'FAIL'} {detail}".rstrip(), flush=True)
    if not ok:
        FAILED.append(name)
    return ok


class Launch:
    def __init__(self, label: str, *arguments: str):
        self.label = label
        self.log = open(Path(tempfile.gettempdir()) / f"leap_probe_{label}.log", "w")
        self.process = subprocess.Popen(["ros2", "launch", "openarm_bringup", *arguments],
                                        stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def stop(self) -> bool:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
        try:
            self.process.wait(timeout=20)
            return True
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            return False


class Probe:
    def __init__(self):
        self.node = rclpy.create_node("leap_split_probe")
        self.joints: dict[str, float] = {}
        self.sources: dict = {}
        self.node.create_subscription(JointState, "/joint_states", self._joint_state,
                                      qos_profile_sensor_data)
        self.node.create_subscription(String, leap.SOURCE_STATUS_TOPIC,
                                      lambda m: setattr(self, "sources", json.loads(m.data)), 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self.node)

    def _joint_state(self, message):
        self.joints.update(zip(message.name, message.position))

    def spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def call(self, service_type, name: str, timeout: float = 10.0):
        client = self.node.create_client(service_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(service_type.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result()

    def active(self, device: str) -> set[str]:
        result = self.call(ListControllers, f"{MANAGER[device]}/list_controllers", timeout=2.0)
        return {c.name for c in result.controller} if result and all(
            c.state == "active" for c in result.controller) else set()

    def commands(self, device: str) -> list[str]:
        result = self.call(ListHardwareInterfaces, f"{MANAGER[device]}/list_hardware_interfaces")
        return [i.name for i in result.command_interfaces] if result else []

    def tip(self, frame: str):
        try:
            t = self.buffer.lookup_transform("body_root", frame, Time(),
                                             RclpyDuration(seconds=0.5)).transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def send(self, action: str, joints: list[str], positions: list[float]) -> bool:
        client = ActionClient(self.node, FollowJointTrajectory, action)
        if not client.wait_for_server(timeout_sec=10.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=positions, time_from_start=Duration(sec=1))]
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=15.0)
        return result.result() is not None and result.result().result.error_code == 0


def distance(a, b) -> float:
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


def main() -> int:
    if not CANONICAL.is_file():
        print(f"canonical LEAP URDF not found: {CANONICAL}")
        return 1
    lock_dir = tempfile.mkdtemp(prefix="leap_probe_locks_")
    os.environ["OPENARM_RH56F1_LOCK_DIR"] = lock_dir
    launches = [Launch("arms", "openarm_leap_arms.launch.py"),
                Launch("right", "leap_hand.launch.py", "side:=right"),
                Launch("left", "leap_hand.launch.py", "side:=left")]
    rclpy.init()
    probe = Probe()
    try:
        for device in ("arms", "right", "left"):
            ok = probe.spin_until(lambda d=device: set(CONTROLLERS[d]) | {"joint_state_broadcaster"}
                                  <= probe.active(d), timeout=60.0)
            check(f"{device}.controllers_active", ok, str(sorted(probe.active(device))))
        commands = {d: probe.commands(d) for d in ("arms", "right", "left")}
        for device, names in commands.items():
            expected = [f"{j}/position" for j in OWNED[device]]
            check(f"{device}.resources_exactly_its_joints", sorted(names) == sorted(expected),
                  f"{len(names)} command interfaces")
        everything = sum(commands.values(), [])
        check("managers_never_share_a_resource", len(everything) == len(set(everything)) == 46)

        all_joints = set(sum(OWNED.values(), []))
        check("joint_states_has_all_46",
              probe.spin_until(lambda: all_joints <= set(probe.joints), timeout=10.0),
              f"{len(set(probe.joints) & all_joints)}/46")
        devices = ("openarm", "leap_right", "leap_left")
        check("merger_reports_three_fresh_devices", probe.spin_until(
            lambda: all(probe.sources.get(d, {}).get("state") == "fresh" for d in devices),
            timeout=10.0), str(sorted(probe.sources)))
        check("head_drawn_from_marked_placeholder", probe.spin_until(
            lambda: probe.sources.get("head", {}).get("display_placeholder") is True, timeout=5.0))
        check("tf_reaches_head_camera", probe.spin_until(
            lambda: probe.tip("head_camera") is not None, timeout=5.0))

        rest_tip = None
        probe.spin_until(lambda: probe.tip("r_hl_index_tip") is not None, timeout=10.0)
        rest_tip = probe.tip("r_hl_index_tip")
        rest_palm = probe.tip("r_hl_palm")
        check("tf_body_root_to_fingertip", rest_tip is not None and rest_palm is not None)

        hand = OWNED["right"]
        target = [1.0 if j == "r_hj_index_1" else 0.0 for j in hand]
        check("right_hand_command_succeeds", probe.send(
            "/leap_right/right_hand_trajectory_controller/follow_joint_trajectory", hand, target))
        check("right_index_reached", probe.spin_until(
            lambda: abs(probe.joints.get("r_hj_index_1", 0.0) - 1.0) < 1e-3, timeout=5.0),
            f"{probe.joints.get('r_hj_index_1')}")
        probe.spin_until(lambda: False, timeout=0.5)
        bent = probe.tip("r_hl_index_tip")
        palm = probe.tip("r_hl_palm")
        check("fingertip_moves_palm_stays",
              bent is not None and distance(bent, rest_tip) > 0.03 and distance(palm, rest_palm) < 1e-4,
              f"tip {distance(bent, rest_tip):.3f} m" if bent else "no tf")

        arm = OWNED["arms"][:7]
        check("right_arm_command_succeeds", probe.send(
            "/right_joint_trajectory_controller/follow_joint_trajectory", arm,
            [0.5 if j == "r_aj_4" else 0.0 for j in arm]))
        probe.spin_until(lambda: abs(probe.joints.get("r_aj_4", 0.0) - 0.5) < 1e-3, timeout=5.0)
        probe.spin_until(lambda: False, timeout=0.5)
        moved = probe.tip("r_hl_palm")
        check("arm_motion_carries_the_hand",
              moved is not None and distance(moved, rest_palm) > 0.05,
              f"palm {distance(moved, rest_palm):.3f} m" if moved else "no tf")
        check("left_hand_untouched", all(abs(probe.joints.get(j, 0.0)) < 1e-6 for j in OWNED["left"]))
    except Exception as error:  # noqa: BLE001
        check("completed", False, repr(error))
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()
        for launch in launches:
            check(f"{launch.label}.stopped_by_sigint", launch.stop())
    print(f"LEAP_SPLIT_PROBE={'FAIL' if FAILED else 'PASS'}")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
