"""Relative palm target: follow the controller's motion since enable.

No spatial calibration between the Quest and the robot is assumed. At enable
the controller pose and the robot's palm pose are both captured, and from then
on

    p_target = p_robot_start + scale * M (p_ctrl - p_ctrl_start)
    R_target = (M R_ctrl R_ctrl_start^T M^T) R_robot_start      (relative mode)
    R_target = R_robot_start                                    (hold mode)

``M = axis_mapping @ heading^T`` takes a displacement in ``quest_world`` to the
robot base frame. The rotation is conjugated by M rather than multiplied by
it: the controller's turn since enable is a rotation about an axis in
``quest_world``, and re-expressing that axis in the base frame is what makes
"turn left" turn the palm left whatever the palm's own orientation is. A
constant offset between the controller's grip frame and the palm frame cancels
in ``R_ctrl R_ctrl_start^T``, which is why none is needed.
"""

from __future__ import annotations

import math

import numpy as np

ORIENTATION_MODES = ("hold", "relative")
HEADING_MODES = ("world", "reference")

#: A reference whose forward axis is closer to vertical than this has no usable
#: heading (a headset hanging face-down from the neck, for instance).
MIN_HEADING_HORIZONTAL = 0.5


class MappingError(ValueError):
    """The mapping cannot be built or engaged; the message says why."""


def yaw_matrix(angle: float) -> np.ndarray:
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def heading_of(rotation: np.ndarray) -> float:
    """Yaw of a frame's forward (x) axis about ``quest_world`` z (up)."""
    forward = np.asarray(rotation, dtype=float)[:, 0]
    horizontal = math.hypot(forward[0], forward[1])
    if horizontal < MIN_HEADING_HORIZONTAL:
        raise MappingError(
            "the reference pose points too close to vertical to give a heading "
            f"(horizontal component {horizontal:.2f}); use heading mode 'world'"
        )
    return math.atan2(forward[1], forward[0])


class RelativeTargetMapper:
    """Maps controller poses to palm targets, relative to an enable instant."""

    def __init__(
        self,
        *,
        axis_mapping=None,
        position_scale: float = 1.0,
        orientation_mode: str = "hold",
        heading_mode: str = "world",
        yaw_offset_rad: float = 0.0,
        max_offset_m: float | None = None,
    ):
        mapping = np.eye(3) if axis_mapping is None else np.asarray(axis_mapping, dtype=float)
        if mapping.shape != (3, 3) or not np.isfinite(mapping).all():
            raise MappingError("axis_mapping must be a finite 3x3 matrix")
        # A signed permutation or any other orthonormal matrix; anything else
        # would shear the motion and is not a frame change.
        if not np.allclose(mapping @ mapping.T, np.eye(3), atol=1e-6):
            raise MappingError("axis_mapping must be orthonormal (rows are unit axes)")
        if not math.isfinite(position_scale) or position_scale <= 0.0:
            raise MappingError("position_scale must be positive and finite")
        if orientation_mode not in ORIENTATION_MODES:
            raise MappingError(f"orientation_mode must be one of {ORIENTATION_MODES}")
        if heading_mode not in HEADING_MODES:
            raise MappingError(f"heading mode must be one of {HEADING_MODES}")
        if max_offset_m is not None and not max_offset_m > 0.0:
            raise MappingError("max_offset_m must be positive")
        self.axis_mapping = mapping
        self.position_scale = float(position_scale)
        self.orientation_mode = orientation_mode
        self.heading_mode = heading_mode
        self.yaw_offset_rad = float(yaw_offset_rad)
        self.max_offset_m = max_offset_m
        self._controller_start: np.ndarray | None = None
        self._robot_start: np.ndarray | None = None
        self._map: np.ndarray | None = None

    @property
    def engaged(self) -> bool:
        return self._controller_start is not None

    def engage(
        self,
        controller: np.ndarray,
        robot: np.ndarray,
        reference: np.ndarray | None = None,
    ) -> None:
        """Capture both start poses (4x4). Raises MappingError to refuse."""
        controller = np.asarray(controller, dtype=float)
        robot = np.asarray(robot, dtype=float)
        if not (np.isfinite(controller).all() and np.isfinite(robot).all()):
            raise MappingError("start poses must be finite")
        heading = self.yaw_offset_rad
        if self.heading_mode == "reference":
            if reference is None:
                raise MappingError(
                    "heading mode 'reference' needs the packet's rf pose, and "
                    "none has arrived"
                )
            heading += heading_of(np.asarray(reference, dtype=float)[:3, :3])
        self._map = self.axis_mapping @ yaw_matrix(heading).T
        self._controller_start = controller.copy()
        self._robot_start = robot.copy()

    def release(self) -> None:
        self._controller_start = None
        self._robot_start = None
        self._map = None

    def target(self, controller: np.ndarray) -> tuple[np.ndarray, bool]:
        """Return the palm target (4x4, base frame) and whether it was clamped."""
        if self._controller_start is None:
            raise MappingError("not engaged")
        controller = np.asarray(controller, dtype=float)
        offset = self.position_scale * (
            self._map @ (controller[:3, 3] - self._controller_start[:3, 3])
        )
        clamped = False
        if self.max_offset_m is not None:
            distance = float(np.linalg.norm(offset))
            if distance > self.max_offset_m:
                offset *= self.max_offset_m / distance
                clamped = True
        target = self._robot_start.copy()
        target[:3, 3] = self._robot_start[:3, 3] + offset
        if self.orientation_mode == "relative":
            turn = controller[:3, :3] @ self._controller_start[:3, :3].T
            target[:3, :3] = self._map @ turn @ self._map.T @ self._robot_start[:3, :3]
        return target, clamped
