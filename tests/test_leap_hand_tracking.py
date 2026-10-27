"""Wrist camera -> LEAP retargeting (openarm_leap_tracking.retarget), no camera, no ROS."""

import math
from pathlib import Path

import numpy as np
import pytest

from openarm_leap_tracking import retarget as rt

URDF = Path(__file__).resolve().parents[2] / "urdf/generated/rl/openarm_leap_bi_rl.urdf"


@pytest.mark.parametrize("side", rt.SIDES)
def test_flat_hand_reads_zero_and_curl_reads_back(side):
    flat = rt.human_angles(rt.synthetic_hand(side), side)
    for f in rt.FINGERS:
        assert flat[f"{f}_1"] == pytest.approx(0, abs=1e-6)
        assert flat[f"{f}_3"] == pytest.approx(0, abs=1e-6) and flat[f"{f}_4"] == pytest.approx(0, abs=1e-6)
    bent = rt.human_angles(rt.synthetic_hand(side, curl=0.5), side)
    for f in rt.FINGERS:
        assert [bent[f"{f}_{k}"] for k in (1, 3, 4)] == pytest.approx([0.5, 0.5, 0.5], abs=1e-6)


@pytest.mark.parametrize("side", rt.SIDES)
def test_spread_is_toward_the_thumb_and_thumb_rotation_toward_the_palm(side):
    spread = rt.human_angles(rt.synthetic_hand(side, spread=0.2), side)
    assert spread["index_2"] == pytest.approx(0.2, abs=1e-6)
    assert spread["ring_2"] == pytest.approx(-0.2, abs=1e-6)
    assert spread["middle_2"] == pytest.approx(0, abs=1e-6)
    thumb = rt.human_angles(rt.synthetic_hand(side, thumb=0.7), side)
    assert thumb["thumb_1"] == pytest.approx(0.7, abs=1e-6)


def test_robot_targets_respect_the_caps_and_the_urdf():
    maps = rt.joint_maps(rt.load_config(), rt.urdf_limits(URDF, "right") if URDF.is_file() else None)
    fist = rt.to_robot(rt.human_angles(rt.synthetic_hand(curl=1.45, spread=0.5, thumb=1.5), "right"), maps)
    assert fist["index_1"] == pytest.approx(math.radians(82.85))
    assert fist["index_3"] == pytest.approx(math.radians(60.0))
    assert fist["index_4"] == pytest.approx(math.radians(79.23))
    assert fist["index_2"] == pytest.approx(math.radians(10.0))
    assert fist["middle_2"] == 0.0  # held by the override
    assert all(maps[s].lower <= v <= maps[s].upper for s, v in fist.items())
    if URDF.is_file():
        limits = rt.urdf_limits(URDF, "right")
        assert set(limits) == set(rt.JOINTS)
        assert all(limits[s][0] <= maps[s].lower and maps[s].upper <= limits[s][1] for s in rt.JOINTS)


def test_smoother_starts_where_told_and_limits_each_step():
    smooth = rt.Smoother(alpha=1.0, max_step=0.1)
    smooth.reset({"index_1": 0.0})
    assert smooth({"index_1": 1.0})["index_1"] == pytest.approx(0.1)
    assert smooth({"index_1": 1.0})["index_1"] == pytest.approx(0.2)
    half = rt.Smoother(alpha=0.5, max_step=10.0)
    half.reset({"index_1": 0.0})
    assert half({"index_1": 1.0})["index_1"] == pytest.approx(0.5)


def test_degenerate_landmarks_are_refused():
    with pytest.raises(ValueError):
        rt.human_angles(np.zeros((21, 3)), "right")
