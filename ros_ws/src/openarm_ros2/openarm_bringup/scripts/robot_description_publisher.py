#!/usr/bin/env python3
"""Publish the ``robot_description`` parameter once, latched, on the ``robot_description`` topic.

Jazzy's controller_manager (4.x) reads its description only from that topic. In the split
bringups each controller manager needs its own description (the whole model plus only the
ros2_control block it owns), so each gets one of these next to it; remap the topic to
where that manager listens. It publishes no TF and reads nothing.
"""

import sys

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main() -> int:
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("robot_description_publisher")
    description = node.declare_parameter("robot_description", "").value
    if not description:
        node.get_logger().error("robot_description parameter is empty")
        return 1
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(String, "robot_description", qos)
    publisher.publish(String(data=description))
    node.get_logger().info(f"latched robot_description on {publisher.topic_name}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
