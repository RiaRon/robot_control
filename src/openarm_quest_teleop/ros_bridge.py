"""rclpy node: Quest input -> PoseStamped and Joy topics.

The input is ``quest.source`` in the config: ``quest_view`` (the headset page's
poses from quest_view, TCP 65432) or ``udp`` (the dora-openarm-vr app's packets).

Publishes, per controller (topic names and frame from the config):

    <pose topic>  geometry_msgs/PoseStamped  pose in ``quest_world``; only for
                  a packet whose pose is valid
    <joy topic>   sensor_msgs/Joy            axes    [trigger, grip, stick_x, stick_y]
                                             buttons [primary, secondary, pose_valid]
                                             (primary/secondary: A/B right, X/Y left)

plus the packet's reference pose (quest_view: the robot's front), the headset
pose (quest_view, ``topics.hmd_pose``) and a JSON status string once a second. Both
messages of one packet carry the same ``header.stamp``: this PC's clock when
the datagram was read. A packet is published once. When the Quest stops
sending, the topics go quiet rather than repeat the last pose, which is what
lets the teleop see the loss.

This node never commands the robot.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from .config import load_config
from .packet import SIDES, PacketError
from .quest_view_receiver import open_source
from .smoothing import OneEuroPoseSmoother


#: How far the headset clock must fall back to be read as an app restart
#: rather than a late datagram.
HEADSET_RESTART_SEC = 1.0


def _stamp(message, recv_ns: int) -> None:
    message.header.stamp.sec = recv_ns // 1_000_000_000
    message.header.stamp.nanosec = recv_ns % 1_000_000_000


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", help="teleop config (default: config/leap_hand/quest_teleop_leap.yaml; RH56F1: config/rh56f1/quest_teleop.yaml)")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    args = parser.parse_args(argv)
    config = load_config(args.config)["quest"]

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("quest_bridge")
    topics = config["topics"]
    pose_publishers = {
        side: node.create_publisher(PoseStamped, topics[side]["pose"], 10) for side in SIDES
    }
    joy_publishers = {
        side: node.create_publisher(Joy, topics[side]["joy"], 10) for side in SIDES
    }
    reference_publisher = node.create_publisher(PoseStamped, topics["reference_pose"], 10)
    hmd_publisher = (node.create_publisher(PoseStamped, topics["hmd_pose"], 10)
                     if "hmd_pose" in topics else None)
    status_publisher = node.create_publisher(String, topics["status"], 10)

    smoothing = config.get("smoothing", {})
    smoothers = None
    if smoothing.get("enabled"):
        options = {key: float(value) for key, value in smoothing.items() if key != "enabled"}
        smoothers = {side: OneEuroPoseSmoother(**options) for side in SIDES}

    receiver, parse, source = open_source(config, args.host, args.port)
    node.get_logger().info(f"reading Quest input from {source}")
    frame_id = config["frame_id"]
    state = {"sequence": 0, "refused": 0, "last_error": None, "last_headset_time": None,
             "published": 0, "window_start": time.monotonic(), "window_count": 0}

    def pose_message(pose, recv_ns):
        message = PoseStamped()
        _stamp(message, recv_ns)
        message.header.frame_id = frame_id
        position, orientation = message.pose.position, message.pose.orientation
        position.x, position.y, position.z = pose.position
        orientation.x, orientation.y, orientation.z, orientation.w = pose.orientation
        return message

    def publish_new_packet():
        packet = receiver.latest()
        if packet is None or packet.sequence == state["sequence"]:
            return
        state["sequence"] = packet.sequence
        try:
            frame = parse(packet.message)
            # A repeated or out-of-order datagram is not a new measurement. A
            # clock that fell back by more than the window is the app having
            # restarted, which starts a new sequence.
            previous = state["last_headset_time"]
            if frame.headset_time is not None and previous is not None:
                if previous - HEADSET_RESTART_SEC < frame.headset_time <= previous:
                    raise PacketError(
                        f"headset time did not advance ({frame.headset_time} <= {previous})")
        except PacketError as error:
            state["refused"] += 1
            state["last_error"] = str(error)
            return
        state["last_headset_time"] = frame.headset_time
        state["published"] += 1
        state["window_count"] += 1
        for side in SIDES:
            controller = frame.controllers[side]
            pose = controller.pose
            if smoothers is not None:
                now = packet.recv_ns * 1e-9
                if pose is None:
                    smoothers[side].suspend(now)
                else:
                    smoothed = smoothers[side].smooth(
                        now, np.array([*pose.position, *pose.orientation]))
                    quaternion = smoothed[3:] / np.linalg.norm(smoothed[3:])
                    pose = type(pose)(tuple(map(float, smoothed[:3])),
                                      tuple(map(float, quaternion)))
            joy = Joy()
            _stamp(joy, packet.recv_ns)
            joy.header.frame_id = frame_id
            joy.axes = [controller.trigger, controller.grip, *controller.stick]
            joy.buttons = [int(controller.buttons[0]), int(controller.buttons[1]),
                           int(pose is not None)]
            if pose is not None:
                pose_publishers[side].publish(pose_message(pose, packet.recv_ns))
            joy_publishers[side].publish(joy)
        if frame.reference is not None:
            reference_publisher.publish(pose_message(frame.reference, packet.recv_ns))
        if frame.hmd is not None and hmd_publisher is not None:
            hmd_publisher.publish(pose_message(frame.hmd, packet.recv_ns))

    def publish_status():
        accepted, malformed = receiver.counts()
        latest = receiver.latest()
        elapsed = time.monotonic() - state["window_start"]
        status = {
            "source": source,
            "bind_error": receiver.bind_error(),
            "datagrams_accepted": accepted,
            "datagrams_malformed": malformed,
            "packets_refused": state["refused"],
            "last_refusal": state["last_error"],
            "packets_published": state["published"],
            "rate_hz": round(state["window_count"] / elapsed, 1) if elapsed > 0 else 0.0,
            "last_packet_age_sec": None if latest is None
            else round((time.time_ns() - latest.recv_ns) * 1e-9, 3),
        }
        state["window_start"], state["window_count"] = time.monotonic(), 0
        status_publisher.publish(String(data=json.dumps(status)))

    node.create_timer(1.0 / float(config["publish_rate_hz"]), publish_new_packet)
    node.create_timer(1.0, publish_status)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
