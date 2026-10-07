#!/usr/bin/env python3
"""Make freshly spawned fake trajectory controllers accept topic commands.

joint_trajectory_controller 2.47.0 (the Humble binary this bringup runs on)
declares ``std::atomic<bool> rt_has_pending_goal_;`` without an initializer
and builds as C++17, so the flag starts indeterminate. Only a finished action
goal sets it to false. While it reads true, ``update()`` drops every
trajectory that arrives on ``~/joint_trajectory`` without logging anything.
The controller still reports active, still answers services and keeps
publishing its state, but a topic client (Quest teleop, the glove adapter,
``ros2 topic pub``) never moves it. Whether a given controller instance is
affected changes from one start to the next. Upstream later initialises the
flag to false.

This one-shot program runs right after a fake manager's trajectory
controllers are spawned. For each one it sends a FollowJointTrajectory goal
whose single point is the controller's own measured position, and waits for
SUCCEEDED, which clears the flag. The goal is zero motion: it holds the joints
where they are.

It refuses unless every hardware component of the manager is
``mock_components/GenericSystem``. It is not used with real hardware.

    fake_trajectory_controller_prime.py --controller-manager /rh56f1_right/controller_manager \\
        right_hand_trajectory_controller
"""

from __future__ import annotations

import argparse
import sys
import time

FAKE_PLUGIN = "mock_components/GenericSystem"
HOLD_SEC = 0.05


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("controllers", nargs="+")
    parser.add_argument("--controller-manager", required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    args, _ = parser.parse_known_args(argv)

    import rclpy
    from rclpy.action import ActionClient
    from action_msgs.msg import GoalStatus
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTrajectoryControllerState
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from trajectory_msgs.msg import JointTrajectoryPoint

    manager = args.controller_manager.rstrip("/")
    namespace = manager.rsplit("/", 1)[0]
    rclpy.init()
    node = rclpy.create_node("fake_trajectory_controller_prime",
                             namespace=namespace or "/")
    log = node.get_logger()
    deadline = time.monotonic() + args.timeout

    def spin_until(predicate):
        while not predicate():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(node, timeout_sec=0.05)
        return True

    def call(service_type, name):
        client = node.create_client(service_type, name)
        try:
            if not spin_until(client.service_is_ready):
                return None
            future = client.call_async(service_type.Request())
            return future.result() if spin_until(future.done) else None
        finally:
            node.destroy_client(client)

    def fail(reason: str) -> int:
        log.error(reason)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    components = call(ListHardwareComponents, f"{manager}/list_hardware_components")
    if components is None:
        return fail(f"{manager} does not answer")
    # Jazzy moved the plugin to plugin_name (class_type is deprecated and empty); Humble has class_type only.
    plugins = {getattr(c, "plugin_name", "") or c.class_type for c in components.component}
    if plugins != {FAKE_PLUGIN}:
        return fail(f"refusing: {manager} runs {sorted(plugins)}, not only {FAKE_PLUGIN}")

    for controller in args.controllers:
        def active():
            listed = call(ListControllers, f"{manager}/list_controllers")
            return listed is not None and any(
                c.name == controller and c.state == "active" for c in listed.controller)
        if not spin_until(active):
            return fail(f"{controller} is not active on {manager}")

        state = {}
        subscription = node.create_subscription(
            JointTrajectoryControllerState, f"{namespace}/{controller}/controller_state",
            lambda m: state.__setitem__("msg", m), 10)
        if not spin_until(lambda: "msg" in state):
            return fail(f"no state from {controller}")
        node.destroy_subscription(subscription)
        measured = state["msg"]
        positions = list(measured.feedback.positions)
        if len(positions) != len(measured.joint_names):
            return fail(f"{controller} state has no measured position for every joint")

        client = ActionClient(node, FollowJointTrajectory,
                              f"{namespace}/{controller}/follow_joint_trajectory")
        if not spin_until(client.server_is_ready):
            return fail(f"{controller} action server does not answer")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(measured.joint_names)
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=positions, time_from_start=Duration(nanosec=int(HOLD_SEC * 1e9)))]
        sent = client.send_goal_async(goal)
        if not spin_until(sent.done) or not sent.result().accepted:
            return fail(f"{controller} did not accept the hold goal")
        result = sent.result().get_result_async()
        if not spin_until(result.done) or result.result().status != GoalStatus.STATUS_SUCCEEDED:
            return fail(f"{controller} hold goal did not succeed")
        client.destroy()
        log.info(f"{controller}: held at its measured position; topic commands are accepted")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
