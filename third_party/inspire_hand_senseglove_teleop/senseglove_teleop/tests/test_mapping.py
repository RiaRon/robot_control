"""mapping.py 순수 함수 단위 테스트 (ROS/하드웨어 불필요)."""

import math

import pytest

from senseglove_teleop.mapping import (
    DEFAULT_CONFIG,
    JOINT_LIMITS_RAD,
    JOINT_NAMES,
    direct_map,
    source_joint_names,
)


def _glove_positions(side_prefix: str, **overrides) -> dict:
    """모든 소스 관절을 0.0으로 채운 뒤 overrides로 덮어쓴다."""
    positions = {name: 0.0 for name in source_joint_names(side_prefix)}
    positions.update({f"{side_prefix}{k}": v for k, v in overrides.items()})
    return positions


def test_joint_names_and_limits_match_length():
    assert len(JOINT_NAMES) == len(JOINT_LIMITS_RAD) == 6


def test_fully_open_maps_to_zero():
    # "폄" 기준(raw 입력)은 관절마다 다르다: 손가락/엄지굽힘은 zero_rad=0.0이지만
    # 엄지회전(thumb_brake)은 zero_rad=-0.174533(URDF 하한)이다. 모든 관절을
    # 0.0으로 채우면 이 가정이 깨진다는 걸 이 테스트가 처음에 잡아냈다.
    positions = _glove_positions(
        "l_",
        pinky_pip=DEFAULT_CONFIG["pinky_zero_rad"],
        ring_pip=DEFAULT_CONFIG["ring_zero_rad"],
        middle_pip=DEFAULT_CONFIG["middle_zero_rad"],
        index_pip=DEFAULT_CONFIG["index_zero_rad"],
        thumb_pip=DEFAULT_CONFIG["thumb_pitch_zero_rad"],
        thumb_brake=DEFAULT_CONFIG["thumb_yaw_zero_rad"],
    )
    result = direct_map(positions, "l_", DEFAULT_CONFIG)
    assert result == pytest.approx((0.0,) * 6, abs=1e-9)


def test_fully_closed_maps_to_inspire_limit():
    positions = _glove_positions(
        "l_",
        pinky_pip=1.745329,
        ring_pip=1.745329,
        middle_pip=1.745329,
        index_pip=1.745329,
        thumb_pip=0.872665,
        thumb_brake=1.047198,
    )
    result = direct_map(positions, "l_", DEFAULT_CONFIG)
    for value, limit in zip(result, JOINT_LIMITS_RAD):
        assert value == pytest.approx(limit, abs=1e-6)


def test_half_closed_index_maps_to_half_inspire_limit():
    positions = _glove_positions("l_", index_pip=1.745329 / 2)
    result = direct_map(positions, "l_", DEFAULT_CONFIG)
    index_out = result[JOINT_NAMES.index("index_proximal_joint")]
    assert index_out == pytest.approx(JOINT_LIMITS_RAD[3] / 2, abs=1e-3)


def test_out_of_range_input_is_clamped_not_extrapolated():
    positions = _glove_positions("l_", index_pip=999.0)
    result = direct_map(positions, "l_", DEFAULT_CONFIG)
    index_out = result[JOINT_NAMES.index("index_proximal_joint")]
    assert index_out == pytest.approx(JOINT_LIMITS_RAD[3], abs=1e-6)

    positions_neg = _glove_positions("l_", index_pip=-999.0)
    result_neg = direct_map(positions_neg, "l_", DEFAULT_CONFIG)
    assert result_neg[JOINT_NAMES.index("index_proximal_joint")] == pytest.approx(0.0, abs=1e-6)


def test_missing_joint_raises_value_error():
    positions = _glove_positions("l_")
    del positions["l_index_pip"]
    with pytest.raises(ValueError):
        direct_map(positions, "l_", DEFAULT_CONFIG)


def test_nan_input_raises_value_error():
    positions = _glove_positions("l_", index_pip=math.nan)
    with pytest.raises(ValueError):
        direct_map(positions, "l_", DEFAULT_CONFIG)


def test_right_side_prefix_uses_r_joints():
    positions = _glove_positions("r_", index_pip=1.745329)
    result = direct_map(positions, "r_", DEFAULT_CONFIG)
    index_out = result[JOINT_NAMES.index("index_proximal_joint")]
    assert index_out == pytest.approx(JOINT_LIMITS_RAD[3], abs=1e-6)


def test_source_joint_names_count():
    assert len(source_joint_names("l_")) == 6  # 4손가락 pip + thumb_pip + thumb_brake
