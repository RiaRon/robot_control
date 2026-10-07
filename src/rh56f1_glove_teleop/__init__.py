"""Nova2 glove -> RH56F1 fake hand, through the retarget node of a hand producer.

    adapter      contract, by-name mapping, clutch/stale rules, CommandGate (no ROS)
    ros_adapter  rclpy node: retarget targets + hand joint states -> trajectory
    synth_glove  synthetic Nova2 JointState publisher, for testing without a glove

The adapter commands one hand's fake trajectory controller only, never an arm
and never a real device.
"""
