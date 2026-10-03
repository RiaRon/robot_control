"""Quest packet validation and the one Unity -> ROS frame conversion.

Packet field names, validity codes and the handedness flip follow
dora-openarm-vr (``quest_receiver.py`` at 072ce98, Apache-2.0); see
``third_party/README.md``. The maths is restated in numpy because upstream
needs scipy and Python 3.11.

Frames
------
The app sends Unity world poses: left-handed, x right, y up, z forward,
metres, quaternion ``qx qy qz qw``. Upstream converts in two steps, a
handedness flip (``p = [x, y, -z]``, ``q = [-qx, -qy, qz, qw]``) and the fixed
rotation ``_FRAME_ROT``. Their product is applied here exactly once:

    quest_world (ROS convention): x forward, y left, z up
    p_ros = [ z_unity, -x_unity,  y_unity]
    q_ros = [-qz_unity,  qx_unity, -qy_unity, qw_unity]   (x, y, z, w)

The rotation is a change of basis (``C R C^T``), so the controller's own axes
are expressed in the ROS convention too. Upstream instead right-multiplies a
gripper-specific ``R_FIX``; a relative rotation, which is all the teleop uses,
is the same either way. Nothing downstream converts again.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np

#: Pose validity codes sent by the app (upstream ``quest_receiver.py``).
VALID_OK = 0
VALID_STALE = 1
VALID_INVALID = 2
VALID_NAMES = {VALID_OK: "OK", VALID_STALE: "STALE", VALID_INVALID: "INVALID"}

#: How far a quaternion's norm may be from one before the packet is refused.
#: Float32 round-off over JSON is ~1e-6; a norm of 0.9 is a broken packet, not
#: something to normalize away.
QUATERNION_NORM_TOLERANCE = 1e-2

SIDES = ("left", "right")
_POSE_KEY = {"left": "lc", "right": "rc"}
_VALID_KEY = {"left": "vl", "right": "vr"}
_TRIGGER_KEY = {"left": "lt", "right": "rt"}
_GRIP_KEY = {"left": "lg", "right": "rg"}
_STICK_KEYS = {"left": ("lsx", "lsy"), "right": ("rsx", "rsy")}
#: Quest face buttons: A/B are on the right controller, X/Y on the left.
_BUTTON_KEYS = {"left": ("x", "y"), "right": ("a", "b")}


class PacketError(ValueError):
    """The packet cannot be used; the message says which field and why."""


@dataclass(frozen=True)
class ControllerPose:
    """A pose in ``quest_world`` (ROS convention). Quaternion is x, y, z, w."""

    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


@dataclass(frozen=True)
class ControllerInput:
    """One controller's part of a packet.

    ``pose`` is None unless the packet marked it OK. A STALE pose is the
    headset repeating its last good one, so it is not a new measurement and is
    not offered as a pose either.
    """

    side: str
    pose: ControllerPose | None
    validity: int
    trigger: float
    grip: float
    stick: tuple[float, float]
    buttons: tuple[bool, bool]

    @property
    def pose_valid(self) -> bool:
        return self.pose is not None


@dataclass(frozen=True)
class QuestFrame:
    """A validated packet. ``headset_time`` is the headset's clock, seconds."""

    headset_time: float | None
    validity: int
    controllers: dict[str, ControllerInput]
    reference: ControllerPose | None


def _number(message: Mapping[str, Any], key: str, default: float | None = None) -> float:
    if key not in message:
        if default is None:
            raise PacketError(f"missing field {key!r}")
        return default
    value = message[key]
    # bool is an int in Python; a pose coordinate sent as true/false is broken.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PacketError(f"field {key!r} is not a number: {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise PacketError(f"field {key!r} is not finite")
    return value


def _validity(message: Mapping[str, Any], key: str) -> int:
    if key not in message:
        return VALID_OK
    value = message[key]
    if isinstance(value, bool) or value not in VALID_NAMES:
        raise PacketError(f"field {key!r} is not a validity code: {value!r}")
    return int(value)


def _button(message: Mapping[str, Any], key: str) -> bool:
    value = message.get(key, False)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    raise PacketError(f"field {key!r} is not a button state: {value!r}")


def unity_to_ros_pose(raw: Mapping[str, Any], label: str = "pose") -> ControllerPose:
    """Validate one Unity pose object and convert it to ``quest_world``."""
    if not isinstance(raw, Mapping):
        raise PacketError(f"{label} is not an object")
    x, y, z = (_number(raw, key) for key in ("x", "y", "z"))
    qx, qy, qz, qw = (_number(raw, key) for key in ("qx", "qy", "qz", "qw"))
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise PacketError(f"{label} quaternion norm is {norm:.4f}, not 1")
    return ControllerPose(
        position=(z, -x, y),
        orientation=(-qz / norm, qx / norm, -qy / norm, qw / norm),
    )


def ros_to_unity_pose(pose: ControllerPose) -> dict[str, float]:
    """Inverse of :func:`unity_to_ros_pose`, for synthetic input and tests."""
    px, py, pz = pose.position
    qx, qy, qz, qw = pose.orientation
    return {"x": -py, "y": pz, "z": px, "qx": qy, "qy": -qz, "qz": -qx, "qw": qw}


def parse_packet(message: Mapping[str, Any]) -> QuestFrame:
    """Validate a decoded packet. Raises :class:`PacketError` to refuse it.

    Buttons, triggers and grips are read whatever the pose validity is, as
    upstream does, but any field that is present and broken refuses the whole
    packet: a half-trusted packet is not something to steer an arm with.
    """
    if not isinstance(message, Mapping):
        raise PacketError("packet is not a JSON object")
    overall = _validity(message, "v")
    headset_time = _number(message, "t") if "t" in message else None

    controllers: dict[str, ControllerInput] = {}
    for side in SIDES:
        # Use each controller's own validity when available.
        # Older packets without a side flag fall back to overall validity.
        validity = (
            _validity(message, _VALID_KEY[side])
            if _VALID_KEY[side] in message else overall
        )
        raw = message.get(_POSE_KEY[side])
        pose = None
        if raw is None:
            validity = VALID_INVALID
        elif validity == VALID_OK:
            pose = unity_to_ros_pose(raw, _POSE_KEY[side])
        stick_x, stick_y = _STICK_KEYS[side]
        primary, secondary = _BUTTON_KEYS[side]
        controllers[side] = ControllerInput(
            side=side,
            pose=pose,
            validity=validity,
            trigger=_number(message, _TRIGGER_KEY[side], 0.0),
            grip=_number(message, _GRIP_KEY[side], 0.0),
            stick=(_number(message, stick_x, 0.0), _number(message, stick_y, 0.0)),
            buttons=(_button(message, primary), _button(message, secondary)),
        )

    reference = None
    if message.get("rf") is not None and overall == VALID_OK:
        reference = unity_to_ros_pose(message["rf"], "rf")
    return QuestFrame(headset_time, overall, controllers, reference)


# ---------------------------------------------------------------- rotations
def quaternion_matrix(orientation) -> np.ndarray:
    """3x3 rotation of a unit quaternion given as x, y, z, w."""
    x, y, z, w = (float(value) for value in orientation)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Unit quaternion (x, y, z, w) of a rotation matrix, with w >= 0."""
    m = np.asarray(rotation, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4)
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = (s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = ((m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s)
    norm = math.sqrt(sum(value * value for value in q))
    sign = -1.0 if q[3] < 0.0 else 1.0
    return tuple(sign * value / norm for value in q)


def pose_matrix(pose: ControllerPose) -> np.ndarray:
    """4x4 homogeneous transform of a pose."""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_matrix(pose.orientation)
    matrix[:3, 3] = pose.position
    return matrix


def matrix_pose(matrix: np.ndarray) -> ControllerPose:
    return ControllerPose(
        position=tuple(float(value) for value in matrix[:3, 3]),
        orientation=matrix_quaternion(matrix[:3, :3]),
    )
