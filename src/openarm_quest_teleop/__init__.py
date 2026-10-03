"""Meta Quest controller -> OpenArm arm teleoperation.

Input -> target -> IK -> command, arm only:

    udp_receiver  UDP JSON packets from the Quest app (derived from upstream)
    packet        validation and the one Unity -> ROS frame conversion
    relative      palm target relative to the enable instant
    ik            pose IK on robot_control.kinematics.Chain
    teleop        enable clutch, freshness checks, command gate
    ros_bridge    rclpy node: packets -> PoseStamped + Joy
    ros_teleop    rclpy node: PoseStamped + Joy + joint states -> trajectory
    monitor       receive-only diagnostic and recorder (no ROS)
    synth         synthetic and replayed packets (no ROS)

Nothing here commands a hand, and nothing is published without ``--execute``.
"""
