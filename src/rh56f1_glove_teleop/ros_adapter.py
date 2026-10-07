"""rclpy node: glove hand targets -> one fake RH56F1 hand's trajectory controller.

    /inspire_<side>/retarget/joint_states   (targets, from the retarget node)
    /rh56f1_<side>/joint_states             (measured, the hand manager's broadcaster)
      -> adapter (by-name mapping, clutch, CommandGate)
      -> /rh56f1_<side>/<side>_hand_trajectory_controller/joint_trajectory

Without ``--execute`` nothing is published to the controller (dry run): the
targets and the mapped commands only appear on the status topic. With it, the
node first requires that the hand's controller manager runs only
mock_components/GenericSystem, exports exactly the six position commands, and
has the trajectory controller active over those joints in manifest order;
anything else, or a manager that does not answer, is refused. One command
owner per hand: a second adapter for the same hand is refused (a flock on
``rh56f1_<side>_hand_command``, separate from the hand manager's own
``rh56f1_<side>_hand`` lock, and no other publisher on the command topic).

Status: ``/rh56f1_<side>/glove_adapter/status`` (std_msgs/String, JSON).
Resume after a lock: ``ros2 service call /rh56f1_<side>/glove_adapter/enable
std_srvs/srv/Trigger``. The mapped target is never published as joint state.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

from .adapter import (
    FAKE_MAPPING_NOTE,
    FOLLOWING,
    GloveHandAdapter,
    JointSample,
    TargetSample,
    build_contract,
    fake_hand_refusal,
    fill,
    load_config,
)

STARTUP_TIMEOUT_SEC = 30.0  # service discovery can be slow while many nodes start
STATUS_PERIOD_SEC = 0.2


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="glove hand targets -> one fake RH56F1 hand")
    parser.add_argument("--side", choices=("right", "left"), required=True)
    parser.add_argument("--config", help="glove_adapter.yaml (default: the packaged one)")
    parser.add_argument("--target-topic", help="override the producer's target topic")
    parser.add_argument("--thumb-pitch", choices=("hold", "follow"))
    parser.add_argument("--thumb-yaw", choices=("hold", "follow"))
    parser.add_argument("--execute", action="store_true",
                        help="publish commands to the fake hand controller (default: dry run)")
    args, _ = parser.parse_known_args(argv)

    import rclpy
    from builtin_interfaces.msg import Duration
    from controller_manager_msgs.srv import (
        ListControllers, ListHardwareComponents, ListHardwareInterfaces)
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    import yaml

    from robot_control.profile import load_builtin_profile

    config = load_config(args.config)
    side = args.side
    profile = load_builtin_profile("openarm_rh56f1")
    order = yaml.safe_load(Path(profile.manifest_path).read_text())["control_joint_order"]
    contract = build_contract(config, profile, order, side, args.thumb_pitch, args.thumb_yaw)
    timing = config["timing"]
    period = 1.0 / float(timing["command_rate_hz"])
    core = GloveHandAdapter(contract, period, float(timing["target_timeout_sec"]),
                            float(timing["joint_state_timeout_sec"]))

    hand = config["hand"]
    manager = fill(hand["controller_manager"], side=side).rstrip("/")
    controller = fill(hand["controller"], side=side)
    namespace = manager.rsplit("/", 1)[0]
    command_topic = f"{namespace}/{controller}/joint_trajectory"
    target_topic = args.target_topic or fill(config["source"]["topic"], side=side)
    joint_topic = fill(hand["joint_states_topic"], side=side)

    rclpy.init()
    node = rclpy.create_node("glove_adapter", namespace=namespace)
    log = node.get_logger()

    def spin_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(node, timeout_sec=0.05)
        return True

    def call(service_type, name, request):
        client = node.create_client(service_type, name)
        try:
            if not spin_until(client.service_is_ready, STARTUP_TIMEOUT_SEC):
                return None
            future = client.call_async(request)
            return future.result() if spin_until(future.done, STARTUP_TIMEOUT_SEC) else None
        finally:
            node.destroy_client(client)

    def refuse(reason: str) -> int:
        log.error(f"refusing --execute: {reason}")
        node.destroy_node()
        rclpy.shutdown()
        return 2

    publisher = None
    if args.execute:
        # Fake hardware only, checked on the manager that will receive the commands.
        components = call(ListHardwareComponents, f"{manager}/list_hardware_components",
                          ListHardwareComponents.Request())
        interfaces = call(ListHardwareInterfaces, f"{manager}/list_hardware_interfaces",
                          ListHardwareInterfaces.Request()) if components else None
        listed = call(ListControllers, f"{manager}/list_controllers",
                      ListControllers.Request()) if components else None
        reply = call(GetParameters, f"{namespace}/{controller}/get_parameters",
                     GetParameters.Request(names=["joints"])) if listed else None
        refusal = fake_hand_refusal(
            contract, manager, controller, hand["fake_plugin"],
            None if components is None else [c.class_type for c in components.component],
            None if interfaces is None else [i.name for i in interfaces.command_interfaces],
            None if listed is None else {c.name: c.state for c in listed.controller}.get(controller),
            None if reply is None or not reply.values else list(reply.values[0].string_array_value))
        if refusal is not None:
            return refuse(refusal)
        # One command owner per hand, without touching the manager's own lock.
        from ament_index_python.packages import get_package_share_directory

        sys.path.insert(0, str(Path(get_package_share_directory("openarm_bringup")) / "launch"))
        from device_guard import OwnershipError, hold_device_lock

        try:
            hold_device_lock(f"rh56f1_{side}_hand_command")
        except OwnershipError as error:
            return refuse(str(error))
        spin_until(lambda: False, 1.0)  # let discovery see other publishers
        if node.count_publishers(command_topic) > 0:
            return refuse(f"{command_topic} already has a publisher")
        publisher = node.create_publisher(JointTrajectory, command_topic, 10)
        # As the Quest teleop does: stream only once the controller is matched.
        if not spin_until(lambda: publisher.get_subscription_count() > 0, STARTUP_TIMEOUT_SEC):
            return refuse(f"nothing is subscribed to {command_topic}")

    latest = {"joints": None}

    def on_target(message):
        core.offer_target(TargetSample(time.monotonic(), tuple(message.name),
                                       tuple(message.position)))

    def on_joints(message):
        latest["joints"] = JointSample(time.monotonic(), dict(zip(message.name, message.position)))

    node.create_subscription(JointState, target_topic, on_target, 10)
    node.create_subscription(JointState, joint_topic, on_joints, qos_profile_sensor_data)
    status_publisher = node.create_publisher(String, "~/status", 10)

    def on_enable(_request, response):
        response.message = core.enable(time.monotonic())
        response.success = True
        log.info(f"enable: {response.message}")
        return response

    node.create_service(Trigger, "~/enable", on_enable)
    horizon = float(timing["stream_horizon_sec"])
    if not 0.0 <= horizon < period:
        log.error(f"stream_horizon_sec {horizon} must be in [0, {period}) (the command period)")
        node.destroy_node()
        rclpy.shutdown()
        return 2
    memo = {"last_status": 0.0, "last_key": None, "last": None}
    log.info(
        f"{target_topic} -> {command_topic if args.execute else '(dry run, nothing published)'}; "
        f"hand state {joint_topic}; thumb pitch {contract.thumb_pitch}, yaw {contract.thumb_yaw}; "
        f"{FAKE_MAPPING_NOTE}")

    def tick():
        now = time.monotonic()
        result = core.step(now, latest["joints"])
        memo["last"] = result
        if result.command is not None and publisher is not None:
            message = JointTrajectory()
            message.joint_names = list(contract.canonical)
            point = JointTrajectoryPoint(positions=[float(v) for v in result.command])
            point.time_from_start = Duration(sec=int(horizon), nanosec=int((horizon % 1) * 1e9))
            message.points.append(point)
            publisher.publish(message)
        key = (result.state, None if result.reason is None else result.reason.split(":")[0])
        if key != memo["last_key"]:
            memo["last_key"] = key
            log.info(f"state {result.state}" + (f" | {result.reason}" if result.reason else ""))
        if now - memo["last_status"] >= STATUS_PERIOD_SEC:
            memo["last_status"] = now
            counters = core.counters
            status = {
                "side": side, "state": result.state, "reason": result.reason,
                "following": result.state == FOLLOWING and result.command is not None,
                "executing": args.execute, "command_topic": command_topic,
                "target_topic": target_topic, "joint_states_topic": joint_topic,
                "joints": list(contract.canonical),
                "target": None if result.target is None else [float(v) for v in result.target],
                "command": None if result.command is None else [float(v) for v in result.command],
                "limited": result.limited,
                "target_age_sec": result.target_age_sec, "joint_age_sec": result.joint_age_sec,
                "thumb": {"pitch": contract.thumb_pitch, "yaw": contract.thumb_yaw},
                "targets_accepted": counters.accepted, "targets_rejected": counters.rejected,
                "last_rejection": counters.last_rejection,
                "targets_clamped": counters.clamped,
                "last_clamped": list(counters.last_clamped),
                "commands": counters.commands if args.execute else 0,
                "command_subscribers": None if publisher is None
                else publisher.get_subscription_count(),
                "mapping": FAKE_MAPPING_NOTE,
            }
            status_publisher.publish(String(data=json.dumps(status)))

    node.create_timer(period, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        log.info(f"stopped after {core.counters.commands if args.execute else 0} commands")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
