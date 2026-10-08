"""rclpy node: Quest pose + Joy + joint states -> one arm's trajectory controller.

    <quest pose/joy topics>  ->  relative palm target  ->  IK (7 arm joints)
    /joint_states            ->  command gate          ->  /<controller>/joint_trajectory

One arm per process. Only that arm's trajectory controller is ever published
to: no hand topic, and not the other arm. The command is a
``trajectory_msgs/JointTrajectory`` with one point, on the controller's topic
interface, the same way ``robotctl pose follow`` streams.

Nothing is published without ``--execute``. If the running description's arm
is driven by anything other than ``mock_components/GenericSystem``, the node
also requires ``--confirm-real-hardware``; without it, it refuses to start in
execute mode. Without ``--execute`` it runs the whole pipeline and reports
what it would do on ``<prefix>/<arm>/status``.

Before following, the arm is brought to its start pose when one is named
(``teleop.start_pose.name`` in the config, ``--start-pose`` on the command line,
``--no-start-pose`` to skip): one slow joint-space FollowJointTrajectory goal from
the measured joints, then a check that the arm is there. Only then does the
teleop loop start, so the enable input cannot engage before the arm has arrived.
A dry run reports the move and does not make it.

Following starts when the enable input is pressed and stops when it is
released; see ``teleop.py`` for the rules.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from robot_control.ros_adapter import CONTROLLER_STREAM_TOPIC
from .config import (
    RUNTIMES,
    ConfigError,
    bind_arm,
    build_teleop,
    controller_for,
    execution_refusal,
    runtime_endpoint,
    start_motion_duration,
    start_pose_for,
    start_pose_settings,
    load_config,
    load_profile_for,
)
from .packet import ControllerPose, matrix_pose, pose_matrix
from .relative import ORIENTATION_MODES
from .teleop import ENGAGED, ControllerSample, JointSample

STARTUP_TIMEOUT_SEC = 15.0
DESCRIPTION_SERVICE_TIMEOUT_SEC = 5.0
STATUS_PERIOD_SEC = 0.1


def _matrix(message) -> np.ndarray:
    position, orientation = message.pose.position, message.pose.orientation
    return pose_matrix(ControllerPose(
        (position.x, position.y, position.z),
        (orientation.x, orientation.y, orientation.z, orientation.w),
    ))


def _stamp_ns(message) -> int:
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Quest controller -> OpenArm arm teleoperation (one arm)")
    parser.add_argument("--config", help="quest_teleop.yaml (default: the packaged one)")
    parser.add_argument("--arm", default="right", help="arm from the config (right | left)")
    parser.add_argument("--runtime", choices=RUNTIMES, default="fake",
                        help="which bringup's controller names to use")
    parser.add_argument("--controller", help="override the arm trajectory controller name")
    parser.add_argument("--orientation-mode", choices=ORIENTATION_MODES)
    parser.add_argument("--joint-states-topic")
    parser.add_argument("--execute", action="store_true",
                        help="publish joint commands (default: dry run)")
    parser.add_argument("--confirm-real-hardware", action="store_true",
                        help="required with --execute when the arm is not fake hardware")
    parser.add_argument("--start-pose",
                        help="pose (poses/<profile>.yaml) to move the arm to before following; "
                             "default: teleop.start_pose.name in the config")
    parser.add_argument("--no-start-pose", action="store_true",
                        help="follow from wherever the arm is")
    parser.add_argument("--poses", help="pose store (default: poses/<profile>.yaml)")
    args = parser.parse_args(argv)

    config = load_config(args.config)
    teleop_config = config["teleop"]
    profile = load_profile_for(config)

    import rclpy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers
    from rclpy.action import ActionClient
    from geometry_msgs.msg import PoseStamped
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    rclpy.init()
    node = rclpy.create_node(f"quest_teleop_{args.arm}")
    log = node.get_logger()

    def spin_until(predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while not predicate():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(node, timeout_sec=0.05)
        return True

    def call(service_type, name, request, timeout_sec=STARTUP_TIMEOUT_SEC):
        client = node.create_client(service_type, name)
        try:
            if not spin_until(client.service_is_ready, timeout_sec):
                return None
            future = client.call_async(request)
            if not spin_until(future.done, STARTUP_TIMEOUT_SEC):
                return None
            return future.result()
        finally:
            node.destroy_client(client)

    def fail(message: str) -> int:
        log.error(message)
        node.destroy_node()
        rclpy.shutdown()
        return 2

    try:
        endpoint = runtime_endpoint(config, args.runtime)
    except ConfigError as error:
        return fail(str(error))
    manager = endpoint["controller_manager"].rstrip("/")

    # ------------------------------------------------ the running description
    # The arm controller manager's own robot_description parameter: the
    # description it loaded, hardware blocks included. In the split bringup
    # /robot_description is the integrated model for TF and has no
    # ros2_control block, so it is only a fallback.
    description: dict[str, str] = {}
    reply = call(GetParameters, f"{manager}/get_parameters",
                 GetParameters.Request(names=["robot_description"]),
                 timeout_sec=DESCRIPTION_SERVICE_TIMEOUT_SEC)
    if reply is not None and reply.values and reply.values[0].string_value:
        description["urdf"] = reply.values[0].string_value
        description["source"] = f"{manager} parameter robot_description"
    else:
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(
            String, teleop_config["robot_description_topic"],
            lambda message: description.setdefault("urdf", message.data), latched)
        if not spin_until(lambda: "urdf" in description, STARTUP_TIMEOUT_SEC):
            return fail(f"no robot description from {manager} or on "
                        f"{teleop_config['robot_description_topic']}")
        description["source"] = teleop_config["robot_description_topic"]
    try:
        binding = bind_arm(description["urdf"], profile, config, args.arm)
        core = build_teleop(description["urdf"], profile, config, binding,
                            orientation_mode=args.orientation_mode)
        controller = args.controller or controller_for(config, profile, binding, args.runtime)
    except (ConfigError, ValueError) as error:
        return fail(str(error))

    log.info(
        f"arm {binding.arm}: group {binding.group}, joints {binding.naming}-named, "
        f"palm frame {binding.control_frame} in {binding.base_frame}, "
        f"hardware {list(binding.plugins)}, controller {controller}, "
        f"description from {description['source']}, "
        f"joint states from {args.joint_states_topic or endpoint['joint_states_topic']}")
    refusal = execution_refusal(
        binding, args.runtime, args.execute, args.confirm_real_hardware)
    if refusal is not None:
        return fail(refusal)

    # ------------------------------------------------ the arm's own controller
    listed = call(ListControllers, f"{manager}/list_controllers",
                  ListControllers.Request())
    states = {} if listed is None else {c.name: c.state for c in listed.controller}
    problem = None
    if controller not in states:
        problem = f"controller {controller} is not loaded (loaded: {sorted(states)})"
    elif states[controller] != "active":
        problem = f"controller {controller} is {states[controller]}, not active"
    else:
        reply = call(GetParameters, f"/{controller}/get_parameters",
                     GetParameters.Request(names=["joints"]))
        joints = [] if reply is None else list(reply.values[0].string_array_value)
        if joints != list(binding.runtime_names):
            problem = (f"controller {controller} drives {joints}, not this arm's "
                       f"{list(binding.runtime_names)}")
    if problem is not None:
        if args.execute:
            return fail(problem)
        log.warning(f"{problem}; continuing because this is a dry run")

    # ------------------------------------------------------------------ inputs
    quest = config["quest"]["topics"]
    side = teleop_config["arms"][args.arm]["quest_side"]
    latest = {"pose": None, "joy": None, "sample": None, "reference": None, "joints": None}

    def pair_input(now: float) -> None:
        """Form a sample once the pose and Joy of one packet have both arrived."""
        joy = latest["joy"]
        if joy is None or len(joy.axes) < 4 or len(joy.buttons) < 3:
            return
        pose = None
        if joy.buttons[2]:
            message = latest["pose"]
            if message is None or _stamp_ns(message) != _stamp_ns(joy):
                return
            pose = _matrix(message)
        reference = latest["reference"]
        latest["sample"] = ControllerSample(
            arrival_sec=now,
            pose=pose,
            trigger=float(joy.axes[0]),
            grip=float(joy.axes[1]),
            buttons=(bool(joy.buttons[0]), bool(joy.buttons[1])),
            reference=None if reference is None
            or now - reference[0] > float(teleop_config["input_timeout_sec"])
            else reference[1],
        )
        latest["joy"] = None

    def on_pose(message):
        latest["pose"] = message
        pair_input(time.monotonic())

    def on_joy(message):
        latest["joy"] = message
        pair_input(time.monotonic())

    def on_reference(message):
        latest["reference"] = (time.monotonic(), _matrix(message))

    def on_joints(message):
        q = binding.to_canonical(dict(zip(message.name, message.position)))
        if q is not None:
            latest["joints"] = JointSample(time.monotonic(), q)

    node.create_subscription(PoseStamped, quest[side]["pose"], on_pose, 10)
    node.create_subscription(Joy, quest[side]["joy"], on_joy, 10)
    node.create_subscription(PoseStamped, quest["reference_pose"], on_reference, 10)
    node.create_subscription(
        JointState, args.joint_states_topic or endpoint["joint_states_topic"],
        on_joints, qos_profile_sensor_data)

    # ----------------------------------------------------------------- outputs
    prefix = f"{teleop_config['status_topic_prefix']}/{binding.arm}"
    status_publisher = node.create_publisher(String, f"{prefix}/status", 10)
    target_publisher = node.create_publisher(PoseStamped, f"{prefix}/target_pose", 10)
    command_topic = f"/{controller}/{CONTROLLER_STREAM_TOPIC}"
    command_publisher = None
    if args.execute:
        # Created only in execute mode, so a dry run has no way to command.
        command_publisher = node.create_publisher(JointTrajectory, command_topic, 10)
        if not spin_until(lambda: command_publisher.get_subscription_count() > 0,
                          STARTUP_TIMEOUT_SEC):
            return fail(f"nothing is subscribed to {command_topic}")
    horizon = float(teleop_config["stream_horizon_sec"])
    counters = {"commands": 0, "engagements": 0, "last": None, "last_status": 0.0,
                "last_logged": None}
    log.info(
        ("EXECUTING: commands go to " + command_topic) if args.execute
        else "DRY RUN: nothing is published to the robot; pass --execute to follow")

    def tick():
        now = time.monotonic()
        result = core.step(now, latest["sample"], latest["joints"])
        if result.just_engaged:
            counters["engagements"] += 1
        if result.command is not None and command_publisher is not None:
            trajectory = JointTrajectory()
            trajectory.joint_names = list(binding.runtime_names)
            point = JointTrajectoryPoint()
            point.positions = binding.to_runtime(result.command)
            point.time_from_start = Duration(
                sec=int(horizon), nanosec=int((horizon % 1.0) * 1e9))
            trajectory.points.append(point)
            command_publisher.publish(trajectory)
            counters["commands"] += 1
        if result.target is not None:
            pose = matrix_pose(result.target)
            message = PoseStamped()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = binding.base_frame
            position, orientation = message.pose.position, message.pose.orientation
            position.x, position.y, position.z = pose.position
            orientation.x, orientation.y, orientation.z, orientation.w = pose.orientation
            target_publisher.publish(message)

        # Logged when the state or the kind of reason changes; the numbers
        # inside a reason (an IK error, say) change every cycle.
        kind = None if result.reason is None else result.reason.split(":")[0]
        key = (result.state, kind, result.limited)
        if key != counters["last_logged"]:
            counters["last_logged"] = key
            log.info(f"state {result.state}"
                     + (f" | {result.reason}" if result.reason else "")
                     + (f" | bounded by {result.limited}" if result.limited else ""))
        if now - counters["last_status"] >= STATUS_PERIOD_SEC or result.just_engaged:
            counters["last_status"] = now
            joints = latest["joints"]
            palm = None if joints is None else core.chain.pose(joints.q)
            status = {
                "arm": binding.arm,
                "state": result.state,
                "following": result.state == ENGAGED and result.command is not None,
                "reason": result.reason,
                "limited": result.limited,
                "executing": args.execute,
                "controller": controller,
                "hardware": list(binding.plugins),
                "input_age_sec": result.input_age_sec,
                "joint_age_sec": result.joint_age_sec,
                "ik_position_error_m": result.position_error_m,
                "ik_rotation_error_rad": result.rotation_error_rad,
                "commands_sent": counters["commands"],
                "engagements": counters["engagements"],
                "target_xyz": None if result.target is None
                else [float(v) for v in result.target[:3, 3]],
                "palm_xyz": None if palm is None else [float(v) for v in palm[:3, 3]],
            }
            status_publisher.publish(String(data=json.dumps(status)))

    # --------------------------------------------------------- the start pose
    start = start_pose_settings(config)
    start_name = None if args.no_start_pose else (args.start_pose or start["name"])
    if start_name:
        try:
            goal_q = start_pose_for(profile, binding, start_name, args.poses)
        except ConfigError as error:
            return fail(str(error))
        if not spin_until(lambda: latest["joints"] is not None, STARTUP_TIMEOUT_SEC):
            return fail("no arm joint state to start the move to the start pose from")
        duration = start_motion_duration(latest["joints"].q, goal_q, start)
        if not args.execute:
            log.info(f"DRY RUN: would move to start pose {start_name} over {duration:.1f} s; "
                     "following from the current pose instead")
        else:
            log.info(f"moving to start pose {start_name} over {duration:.1f} s "
                     f"({[round(float(v), 4) for v in goal_q]})")
            client = ActionClient(node, FollowJointTrajectory,
                                  f"/{controller}/follow_joint_trajectory")
            if not client.wait_for_server(timeout_sec=STARTUP_TIMEOUT_SEC):
                return fail(f"no follow_joint_trajectory action on {controller}")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(binding.runtime_names)
            point = JointTrajectoryPoint()
            point.positions = binding.to_runtime(goal_q)
            point.time_from_start = Duration(sec=int(duration),
                                             nanosec=int((duration % 1.0) * 1e9))
            goal.trajectory.points.append(point)
            sent = client.send_goal_async(goal)
            if not spin_until(sent.done, STARTUP_TIMEOUT_SEC) or not sent.result().accepted:
                return fail(f"{controller} did not accept the move to {start_name}")
            done = sent.result().get_result_async()
            if not spin_until(done.done, duration + STARTUP_TIMEOUT_SEC):
                return fail(f"the move to {start_name} did not finish")
            if done.result().result.error_code != 0:
                return fail(f"the move to {start_name} failed: "
                            f"{done.result().result.error_string}")
            arrived = latest["joints"].q if latest["joints"] else None
            if not spin_until(lambda: latest["joints"] is not None and float(np.max(np.abs(
                    latest["joints"].q - goal_q))) <= float(start["tolerance_rad"]), 2.0):
                error = None if arrived is None else float(np.max(np.abs(arrived - goal_q)))
                return fail(f"the arm is not at {start_name} after the move "
                            f"(largest error {error} rad)")
            log.info(f"at start pose {start_name}; press the enable input to follow")

    node.create_timer(1.0 / profile.endpoint().command_rate_hz, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Nothing to send on the way out: the trajectory controller holds the
        # last commanded joint position.
        log.info(f"stopped after {counters['commands']} commands")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
