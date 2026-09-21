#!/usr/bin/env python3
"""Runtime probe for the isolated Humble GenericSystem integration.

Run only against ``openarm.rh56f1_bimanual.launch.py`` with fake hardware.
The probe sends two small arm-only trajectories; it never commands hand joints.
"""

from __future__ import annotations

import math
import sys
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from rclpy.action import ActionClient
import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINTS = {
    "left": [f"l_aj_{index}" for index in range(1, 8)],
    "right": [f"r_aj_{index}" for index in range(1, 8)],
}
HAND_ACTUATORS = [
    f"{side}_hj_{name}"
    for side in ("r", "l")
    for name in ("thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "pinky_1")
]
EXPECTED_JOINTS = set(ARM_JOINTS["left"] + ARM_JOINTS["right"] + HAND_ACTUATORS)


def _vector(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return (
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )


def _distance(left, right):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


class Probe(Node):
    def __init__(self):
        super().__init__("openarm_rh56f1_fake_runtime_probe")
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._state, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.actions = {
            side: ActionClient(
                self,
                FollowJointTrajectory,
                f"/{side}_joint_trajectory_controller/follow_joint_trajectory",
            )
            for side in ("left", "right")
        }

    def _state(self, message):
        self.latest_joint_state = message

    def spin_until(self, predicate, timeout=15.0, label="condition"):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise AssertionError(f"timeout waiting for {label}")

    def transform(self, target, source):
        self.spin_until(
            lambda: self.buffer.can_transform(
                target, source, Time(), timeout=RclpyDuration(seconds=0.0)
            ),
            label=f"TF {target} <- {source}",
        )
        return _vector(self.buffer.lookup_transform(target, source, Time()))

    def assert_controllers(self):
        self.spin_until(
            self.controllers.service_is_ready, label="list_controllers service"
        )
        future = self.controllers.call_async(ListControllers.Request())
        self.spin_until(future.done, label="controller list")
        controllers = {controller.name: controller for controller in future.result().controller}
        expected = {
            "joint_state_broadcaster",
            "left_joint_trajectory_controller",
            "right_joint_trajectory_controller",
        }
        assert set(controllers) == expected, sorted(controllers)
        assert all(controller.state == "active" for controller in controllers.values())
        claimed = {
            interface
            for controller in controllers.values()
            for interface in controller.claimed_interfaces
        }
        assert not any(name in interface for name in HAND_ACTUATORS for interface in claimed)
        for side in ("left", "right"):
            for joint in ARM_JOINTS[side]:
                assert f"{joint}/position" in claimed

    def state_map(self):
        self.spin_until(
            lambda: self.latest_joint_state is not None,
            label="/joint_states",
        )
        message = self.latest_joint_state
        assert len(message.name) == len(set(message.name)), "duplicate joint names"
        assert set(message.name) == EXPECTED_JOINTS, sorted(message.name)
        return dict(zip(message.name, message.position))

    def send_arm_goal(self, side, joint_two_position):
        action = self.actions[side]
        self.spin_until(action.server_is_ready, label=f"{side} trajectory action")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS[side]
        point = JointTrajectoryPoint()
        point.positions = [0.0, joint_two_position, 0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1)
        goal.trajectory.points = [point]
        sent = action.send_goal_async(goal)
        self.spin_until(sent.done, label=f"{side} goal acceptance")
        handle = sent.result()
        assert handle.accepted
        result = handle.get_result_async()
        self.spin_until(result.done, timeout=10.0, label=f"{side} goal result")
        assert result.result().result.error_code == 0


def main():
    rclpy.init()
    node = Probe()
    try:
        node.assert_controllers()
        topics = {name for name, _ in node.get_topic_names_and_types()}
        assert {"/joint_states", "/tf", "/tf_static"} <= topics

        before_state = node.state_map()
        assert all(abs(before_state[name]) < 1e-9 for name in HAND_ACTUATORS)

        before_world = {
            side: {
                "palm": node.transform("body_root", f"{prefix}_hl_palm_sensor"),
                "tip": node.transform("body_root", f"{prefix}_hl_index_tip"),
                "relative": node.transform(
                    f"{prefix}_hl_palm_sensor", f"{prefix}_hl_index_tip"
                ),
                "passive": node.transform(
                    f"{prefix}_hl_palm_sensor", f"{prefix}_hl_thumb_4"
                ),
            }
            for side, prefix in (("left", "l"), ("right", "r"))
        }

        node.send_arm_goal("left", 0.03)
        node.send_arm_goal("right", 0.03)
        node.spin_until(
            lambda: node.latest_joint_state is not None
            and abs(
                dict(
                    zip(
                        node.latest_joint_state.name,
                        node.latest_joint_state.position,
                    )
                ).get("r_aj_2", 0.0)
                - 0.03
            )
            < 1e-4,
            label="updated fake arm state",
        )
        after_state = node.state_map()
        assert abs(after_state["l_aj_2"] - 0.03) < 1e-4
        assert abs(after_state["r_aj_2"] - 0.03) < 1e-4
        assert all(abs(after_state[name]) < 1e-9 for name in HAND_ACTUATORS)

        for side, prefix in (("left", "l"), ("right", "r")):
            after_palm = node.transform("body_root", f"{prefix}_hl_palm_sensor")
            after_tip = node.transform("body_root", f"{prefix}_hl_index_tip")
            after_relative = node.transform(
                f"{prefix}_hl_palm_sensor", f"{prefix}_hl_index_tip"
            )
            after_passive = node.transform(
                f"{prefix}_hl_palm_sensor", f"{prefix}_hl_thumb_4"
            )
            assert _distance(before_world[side]["palm"], after_palm) > 1e-4
            assert _distance(before_world[side]["tip"], after_tip) > 1e-4
            assert _distance(before_world[side]["relative"], after_relative) < 1e-6
            assert _distance(before_world[side]["passive"], after_passive) < 1e-6

        print("RUNTIME_PROBE=PASS")
        print(f"JOINT_STATE_COUNT={len(after_state)}")
        print("HAND_ACTUATORS_PARKED=12")
        print("ACTIVE_CONTROLLERS=3")
        print("PALM_AND_FINGERTIP_TF_FOLLOW_ARM=PASS")
        print("PARKED_RELATIVE_FINGER_TF=PASS")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print(f"RUNTIME_PROBE=FAIL: {error}", file=sys.stderr)
        raise
