"""Wrist camera -> LEAP Hand: MediaPipe landmarks retargeted to the 16 LEAP joints.

    retarget   landmarks -> per-joint human angles -> capped LEAP targets, smoothing
    ros_node   camera (or --test) -> retarget -> the hand's trajectory controller
"""
