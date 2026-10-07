"""Pure mapping logic: senseglove_ros joint_states -> Inspire RH56F1 joint targets.

이 모듈은 ROS와 시리얼 하드웨어에 의존하지 않는 순수 함수만 담는다(Nova2Dex의
``nova2_inspire_retarget/mapping.py`` 구조를 그대로 따름). 입력은 Windows UDP
브리지(ManusGlove 메시지)가 아니라, senseglove_ros가 블루투스로 직접 받아
ROS 2 토픽(``/senseglove/glove<serial>/<lh|rh>/joint_states``)으로 내는
관절 각도(라디안)다.

출력 관절 이름/각도 스케일은 Nova2Dex의 ``nova2_inspire_retarget``과 동일하게
맞췄다(AnyDexRetarget의 Inspire 어댑터, G1 서비스와 같은 규약). 0 = 완전히 폄,
최댓값 = 완전히 쥠.

주의: 엄지(thumb_pitch, thumb_yaw)는 계산해 함께 발행하지만, 하드웨어
브리지(hand_bridge_node.py)가 실제로 쓰는 건 굽힘(thumb_pitch)뿐이다.
기본값(drive_thumb=False)에서는 굽힘도 고정 각도를 보내고, drive_thumb=True로
켜면 굽힘만 이 값을 쓰며 항상 [1100,1350]으로 클램프한다. 엄지 매핑 방향
(어느 쪽이 폄/쥠인지)은 2026-09-22 기준 실기로 검증 중이다. thumb_yaw는
drive_thumb 값과 무관하게 hand_bridge_node에서 항상 무시되고 고정값(950)이
전송된다(2026-09-28, 조작 편의를 위한 사용자 결정 — 이 모듈은 계산/발행만
계속하고 있어 필요하면 다시 켤 수 있다).
"""

from math import isfinite

# Nova2Dex의 nova2_inspire_retarget과 동일한 순서/이름/라디안 한계.
# (AnyDexRetarget Inspire 어댑터, G1 서비스 규약과 호환)
JOINT_NAMES = (
    "pinky_proximal_joint",
    "ring_proximal_joint",
    "middle_proximal_joint",
    "index_proximal_joint",
    "thumb_proximal_pitch_joint",
    "thumb_proximal_yaw_joint",
)
JOINT_LIMITS_RAD = (1.47, 1.47, 1.47, 1.47, 0.6, 1.308)

FINGERS = ("pinky", "ring", "middle", "index")

# senseglove_ros 쪽 관절 이름(측 접두사 + 이 이름). URDF(nova2_left/right.description.xacro)의
# revolute joint 이름과 그대로 일치해야 한다.
FINGER_SOURCE_JOINT = "pip"  # index/middle/ring/pinky의 폐색(closure) 신호로 pip를 사용
THUMB_PITCH_SOURCE_JOINT = "thumb_pip"
THUMB_YAW_SOURCE_JOINT = "thumb_brake"

# senseglove_ros URDF(nova2_*.description.xacro)에서 그대로 가져온 기본 범위(라디안).
# 좌우 동일. 세션마다 손 크기/착용 상태가 달라 열림값이 정확히 0이 아닐 수 있으므로,
# 실사용 전에 config/mapping.yaml의 zero/range를 실측값으로 보정하는 것을 권장한다.
DEFAULT_CONFIG = {
    "pinky_zero_rad": 0.0, "pinky_range_rad": 1.745329,
    "ring_zero_rad": 0.0, "ring_range_rad": 1.745329,
    "middle_zero_rad": 0.0, "middle_range_rad": 1.745329,
    "index_zero_rad": 0.0, "index_range_rad": 1.745329,
    "thumb_pitch_zero_rad": 0.0, "thumb_pitch_range_rad": 0.872665,
    "thumb_yaw_zero_rad": -0.174533, "thumb_yaw_range_rad": 1.221731,
}


def _unit(value: float, zero: float, span: float) -> float:
    """value를 [zero, zero+span] 구간에서 [0, 1]로 클램프해 정규화한다."""
    if not all(isfinite(number) for number in (value, zero, span)):
        raise ValueError("mapping inputs must be finite")
    if abs(span) < 1e-6:
        raise ValueError("mapping range must not be zero")
    return min(1.0, max(0.0, (value - zero) / span))


def source_joint_names(side_prefix: str) -> tuple[str, ...]:
    """이 매핑이 구독해야 하는 senseglove_ros 관절 이름 전체(순서 무관)."""
    names = [f"{side_prefix}{finger}_{FINGER_SOURCE_JOINT}" for finger in FINGERS]
    names.append(f"{side_prefix}{THUMB_PITCH_SOURCE_JOINT}")
    names.append(f"{side_prefix}{THUMB_YAW_SOURCE_JOINT}")
    return tuple(names)


def direct_map(
    glove_positions: dict[str, float], side_prefix: str, config: dict[str, float] = DEFAULT_CONFIG
) -> tuple[float, ...]:
    """senseglove_ros JointState 위치(라디안 dict)를 Inspire 관절 라디안으로 변환한다.

    Args:
        glove_positions: {"l_index_pip": 0.19, ...} 형태. JointState.name과
            position을 짝지은 dict (호출자가 만든다).
        side_prefix: "l_" 또는 "r_".
        config: DEFAULT_CONFIG와 같은 키를 가진 보정값 dict.

    Returns:
        JOINT_NAMES 순서에 대응하는 라디안 튜플 (pinky, ring, middle, index,
        thumb_pitch, thumb_yaw). 각 값은 [0, JOINT_LIMITS_RAD[i]] 안에 있다.
    """
    values: list[float] = []
    for finger, limit in zip(FINGERS, JOINT_LIMITS_RAD[:4]):
        key = f"{side_prefix}{finger}_{FINGER_SOURCE_JOINT}"
        if key not in glove_positions:
            raise ValueError(f"missing glove joint: {key}")
        raw = float(glove_positions[key])
        normalized = _unit(raw, config[f"{finger}_zero_rad"], config[f"{finger}_range_rad"])
        values.append(normalized * limit)

    pitch_key = f"{side_prefix}{THUMB_PITCH_SOURCE_JOINT}"
    if pitch_key not in glove_positions:
        raise ValueError(f"missing glove joint: {pitch_key}")
    thumb_pitch = (
        _unit(
            float(glove_positions[pitch_key]),
            config["thumb_pitch_zero_rad"],
            config["thumb_pitch_range_rad"],
        )
        * JOINT_LIMITS_RAD[4]
    )

    yaw_key = f"{side_prefix}{THUMB_YAW_SOURCE_JOINT}"
    if yaw_key not in glove_positions:
        raise ValueError(f"missing glove joint: {yaw_key}")
    thumb_yaw = (
        _unit(
            float(glove_positions[yaw_key]),
            config["thumb_yaw_zero_rad"],
            config["thumb_yaw_range_rad"],
        )
        * JOINT_LIMITS_RAD[5]
    )

    return tuple(values) + (thumb_pitch, thumb_yaw)
