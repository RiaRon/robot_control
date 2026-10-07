"""ROS 2 node: senseglove_ros joint_states -> Inspire retarget JointState.

Nova2Dex의 ``nova2_inspire_retarget/retarget_node.py``와 같은 역할이지만,
입력이 Windows UDP 브리지(ManusGlove 메시지)가 아니라 senseglove_ros가
블루투스로 직접 발행하는 ``sensor_msgs/JointState``다. 출력 토픽/관절
이름은 Nova2Dex와 동일하게 맞춰 호환성을 유지한다.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from senseglove_teleop.mapping import DEFAULT_CONFIG, JOINT_NAMES, direct_map, source_joint_names


class SenseGloveInspireRetargetNode(Node):
    def __init__(self):
        super().__init__("senseglove_inspire_retarget")
        self.declare_parameter("side", "left")  # "left" 또는 "right"
        self.declare_parameter("glove_serial", "00795")
        self.declare_parameter("input_topic", "")  # 비우면 side/serial로 자동 구성
        self.declare_parameter("output_topic", "")  # 비우면 side로 자동 구성
        self.declare_parameter("warning_period_sec", 2.0)
        for key, value in DEFAULT_CONFIG.items():
            self.declare_parameter(key, value)

        side = str(self.get_parameter("side").value).lower()
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.side_prefix = "l_" if side == "left" else "r_"
        glove_serial = str(self.get_parameter("glove_serial").value)
        side_short = "lh" if side == "left" else "rh"

        input_topic = str(self.get_parameter("input_topic").value)
        if not input_topic:
            input_topic = f"/senseglove/glove{glove_serial}/{side_short}/joint_states"
        output_topic = str(self.get_parameter("output_topic").value)
        if not output_topic:
            output_topic = f"/inspire_{side}/retarget/joint_states"

        self.mapping_config = {key: float(self.get_parameter(key).value) for key in DEFAULT_CONFIG}
        self.warning_period = max(0.1, float(self.get_parameter("warning_period_sec").value))
        self._last_warning = 0.0
        self._expected_joints = set(source_joint_names(self.side_prefix))

        self._publisher = self.create_publisher(JointState, output_topic, 10)
        self._subscription = self.create_subscription(JointState, input_topic, self._on_glove, 10)

        self.get_logger().info(
            f"senseglove_inspire_retarget: {input_topic} -> {output_topic} "
            f"(side={side}, glove_serial={glove_serial}, joint order={list(JOINT_NAMES)})"
        )

    def _warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning >= self.warning_period:
            self._last_warning = now
            self.get_logger().warning(message)

    def _on_glove(self, msg: JointState) -> None:
        positions = {name: pos for name, pos in zip(msg.name, msg.position)}
        if not self._expected_joints.issubset(positions.keys()):
            missing = self._expected_joints - positions.keys()
            self._warn_throttled(f"glove frame missing joints: {sorted(missing)}")
            return

        try:
            targets = direct_map(positions, self.side_prefix, self.mapping_config)
        except (KeyError, TypeError, ValueError) as exc:
            self._warn_throttled(f"skipping invalid glove frame: {exc}")
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(JOINT_NAMES)
        out.position = list(targets)
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SenseGloveInspireRetargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
