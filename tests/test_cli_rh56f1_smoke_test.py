"""`rh56f1 smoke-test`: a single-joint move relative to the current measured
pose, for the OPENARM_RH56F1_INTEGRATION_STATUS.md Stage 4 first-motion
procedure. Dry run is the default and must never touch ROS; --execute must
read the current pose, move only the named joint, and refuse outside the
tool's own small delta/velocity ceiling or the profile's joint limits.
"""

import sys

import numpy as np
import pytest

from robot_control.cli import (
    RH56F1_MAX_ARM_DELTA_RAD,
    RH56F1_MAX_HAND_DELTA_RAD,
    RH56F1_MAX_VELOCITY_RAD_PER_SEC,
    RH56F1_MIN_VELOCITY_RAD_PER_SEC,
    main,
)


@pytest.fixture
def no_ros(monkeypatch):
    monkeypatch.setitem(sys.modules, "rclpy", None)


class RecordingHand:
    """Records every trajectory; its state follows the last waypoint."""

    def __init__(self, state):
        self.joints = np.asarray(state, dtype=float)
        self.trajectories = []

    def __enter__(self):
        return self

    def __exit__(self, *_exception):
        return None

    def read_state(self, timeout_sec=None):
        return self.joints.copy()

    def send_trajectory(self, points, period_sec):
        self.trajectories.append([np.asarray(p, dtype=float).copy() for p in points])
        self.joints = np.asarray(points[-1], dtype=float).copy()
        self.period_sec = period_sec


@pytest.fixture
def right_hand(monkeypatch):
    from robot_control import ros_adapter

    stub = RecordingHand(np.zeros(6))
    monkeypatch.setattr(ros_adapter, "RosAdapter", lambda *a, **k: stub)
    return stub


def _args(execute=False, **overrides):
    base = dict(side="right", device="hand", joint="r_hj_thumb_1", delta=0.02, velocity=0.02)
    base.update(overrides)
    argv = ["rh56f1", "smoke-test"]
    for key, value in base.items():
        argv += [f"--{key}", str(value)]
    if execute:
        argv.append("--execute")
    return argv


def test_dry_run_never_imports_ros(no_ros, capsys):
    assert main(_args()) == 0
    output = capsys.readouterr().out
    assert "DRY RUN" in output
    assert "rh56f1_right_hand" in output
    assert "r_hj_thumb_1" in output


def test_dry_run_prints_the_relative_delta_not_an_absolute_target(no_ros, capsys):
    assert main(_args(delta=0.02, velocity=0.01)) == 0
    output = capsys.readouterr().out
    assert "+0.0200" in output
    assert "2.00 s" in output  # 0.02 / 0.01


def test_execute_moves_only_the_named_joint(right_hand, capsys):
    right_hand.joints = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    assert main(_args(execute=True, joint="r_hj_index_1", delta=0.02, velocity=0.02)) == 0

    (trajectory,) = right_hand.trajectories
    final = trajectory[-1]
    index_of_index1 = 2  # thumb1, thumb2, index1, middle1, ring1, pinky1
    assert final[index_of_index1] == pytest.approx(0.12)
    for i, value in enumerate(final):
        if i != index_of_index1:
            assert value == pytest.approx(0.1)


def test_execute_reads_current_pose_as_the_move_origin(right_hand, capsys):
    right_hand.joints = np.array([0.3, 0.0, 0.0, 0.0, 0.0, 0.0])

    assert main(_args(execute=True, joint="r_hj_thumb_1", delta=0.01, velocity=0.02)) == 0

    (trajectory,) = right_hand.trajectories
    assert trajectory[-1][0] == pytest.approx(0.31)
    output = capsys.readouterr().out
    assert "current=+0.3000" in output


@pytest.mark.parametrize("delta", [0.0, RH56F1_MAX_HAND_DELTA_RAD + 0.001, -0.5])
def test_hand_delta_outside_ceiling_is_refused(no_ros, delta, capsys):
    rc = main(_args(delta=delta))
    assert rc == 3
    assert "refused" in capsys.readouterr().out


def test_arm_has_a_larger_but_still_bounded_delta_ceiling(no_ros, capsys):
    ok = main(_args(side="left", device="arm", joint="l_aj_2",
                    delta=RH56F1_MAX_ARM_DELTA_RAD, velocity=0.02))
    assert ok == 0

    too_far = main(_args(side="left", device="arm", joint="l_aj_2",
                         delta=RH56F1_MAX_ARM_DELTA_RAD + 0.001, velocity=0.02))
    assert too_far == 3


@pytest.mark.parametrize(
    "velocity",
    [RH56F1_MIN_VELOCITY_RAD_PER_SEC / 2, RH56F1_MAX_VELOCITY_RAD_PER_SEC * 2],
)
def test_velocity_outside_ceiling_is_refused(no_ros, velocity, capsys):
    rc = main(_args(velocity=velocity))
    assert rc == 3
    assert "refused" in capsys.readouterr().out


def test_unknown_joint_for_the_group_is_an_error_not_a_refusal(no_ros, capsys):
    rc = main(_args(joint="l_hj_thumb_1"))  # a left-hand joint under --side right
    assert rc == 2
    assert "error" in capsys.readouterr().out


def test_execute_refuses_a_target_outside_the_joint_limit(right_hand, capsys):
    # r_hj_thumb_1's canonical upper bound is 2.0943951023931953 rad.
    right_hand.joints = np.array([2.08, 0.0, 0.0, 0.0, 0.0, 0.0])

    rc = main(_args(execute=True, joint="r_hj_thumb_1", delta=0.02, velocity=0.02))

    assert rc == 3
    assert not right_hand.trajectories, "an out-of-limit target must never be sent"
    assert "outside" in capsys.readouterr().out


def test_execute_side_selects_the_correct_group(monkeypatch, capsys):
    from robot_control import ros_adapter

    seen_groups = []

    def factory(profile, group_name, execute):
        seen_groups.append(group_name)
        return RecordingHand(np.zeros(6))

    monkeypatch.setattr(ros_adapter, "RosAdapter", factory)

    assert main(_args(execute=True, side="left", device="hand", joint="l_hj_pinky_1",
                      delta=0.01, velocity=0.01)) == 0
    assert seen_groups == ["rh56f1_left_hand"]
