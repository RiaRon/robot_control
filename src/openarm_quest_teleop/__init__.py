"""Meta Quest controller -> OpenArm arm teleoperation.

Input -> target -> IK -> command, arm only:

    quest_view_receiver  quest_view's TCP pose stream (the headset page); quest.source picks the input
    udp_receiver  UDP JSON packets from the Quest app (derived from upstream)
    packet        validation and the one Unity -> ROS frame conversion
    relative      palm target relative to the enable instant
    ik, pink_ik   IK settings / result, and the pink IK
    teleop        enable clutch, freshness checks, command gate
    ros_bridge    rclpy node: packets -> PoseStamped + Joy
    ros_teleop    rclpy node: PoseStamped + Joy + joint states -> trajectory
    monitor       receive-only diagnostic and recorder (no ROS)
    synth         synthetic and replayed packets (no ROS)

The headset page itself is ``openarm_quest_view``; the neck that follows the
headset is ``openarm_head``. Nothing here commands a hand, and nothing is
published without ``--execute``.
"""
