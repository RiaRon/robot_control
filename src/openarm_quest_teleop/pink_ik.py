"""Pose IK for one arm with pink: Pinocchio, QP-based differential IK.

The same contract as ``ik.solve_pose`` (an ``IkResult``; joint limits held inside
the iteration; a target that is not met within the budget, or a solution that
ends far from its seed, is refused), so ``ArmTeleop`` follows with either. Only
the solver differs: each iteration is one pink QP over a palm frame task and a
light posture task toward the seed, under the configuration and velocity limits,
solved with daqp. This is the method of OpenArm's own ``openarm_control`` (mink,
a MuJoCo port of pink), here on the URDF the bringup actually runs.

The model is the running description reduced to this arm's seven joints: every
other joint (the other arm, the hand, the head) is locked at zero, which does not
move the palm frame. Limits are the profile's, written into the reduced model.

pink and pinocchio are not in the ROS distribution: install them into a venv
that also sees the system ROS packages (docs/quest-teleop-leap-pink.md).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .ik import IkResult, IkSettings


@dataclass(frozen=True)
class PinkSettings:
    #: Integration step of one QP iteration. With the task gain at 1 one
    #: iteration closes the linearized error, so this only scales the velocity
    #: limit per iteration (velocity * dt).
    dt: float = 0.05
    position_cost: float = 1.0
    orientation_cost: float = 1.0
    #: Pull toward the seed (the previous solution): a small regularizer that
    #: keeps the redundant arm from drifting while the palm task is met.
    posture_cost: float = 1e-3
    lm_damping: float = 1e-6
    solver: str = "daqp"


class PinkIk:
    """``solve(target, seed, lower, upper, settings) -> IkResult`` for one arm."""

    def __init__(
        self,
        urdf: str,
        runtime_names: tuple[str, ...] | list[str],
        frame: str,
        *,
        sign: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        velocity: np.ndarray,
        settings: PinkSettings = PinkSettings(),
    ):
        import pinocchio as pin
        from pink.limits import ConfigurationLimit, VelocityLimit
        from pink.tasks import FrameTask, PostureTask

        self._pin = pin
        full = pin.buildModelFromXML(urdf)
        ids = [full.getJointId(name) for name in runtime_names]
        if any(i >= full.njoints for i in ids):
            missing = [n for n, i in zip(runtime_names, ids) if i >= full.njoints]
            raise ValueError(f"the description has no joints {missing}")
        locked = [j for j in range(1, full.njoints) if j not in set(ids)]
        model = pin.buildReducedModel(full, locked, pin.neutral(full))
        if not model.existFrame(frame):
            raise ValueError(f"the description has no frame {frame!r}")
        self._index = np.array([model.idx_qs[model.getJointId(n)] for n in runtime_names])
        self._sign = np.asarray(sign, dtype=float)
        # Canonical limits -> runtime (model) limits; a sign of -1 swaps the bounds.
        lo, hi = np.asarray(lower, float) * self._sign, np.asarray(upper, float) * self._sign
        model.lowerPositionLimit[self._index] = np.minimum(lo, hi)
        model.upperPositionLimit[self._index] = np.maximum(lo, hi)
        model.velocityLimit[self._index] = np.asarray(velocity, dtype=float)
        self._model, self._data = model, model.createData()
        self._frame_id = model.getFrameId(frame)
        self.settings = settings
        self._task = FrameTask(frame, position_cost=settings.position_cost,
                               orientation_cost=settings.orientation_cost,
                               lm_damping=settings.lm_damping)
        self._posture = PostureTask(cost=settings.posture_cost)
        self._limits = [ConfigurationLimit(model), VelocityLimit(model)]

    # ------------------------------------------------------------ helpers
    def _model_q(self, q_canonical: np.ndarray) -> np.ndarray:
        q = self._pin.neutral(self._model)
        q[self._index] = np.asarray(q_canonical, dtype=float) * self._sign
        return q

    def pose(self, q_canonical: np.ndarray) -> np.ndarray:
        """Palm pose (4x4) in the description's root frame."""
        self._pin.framesForwardKinematics(self._model, self._data, self._model_q(q_canonical))
        return self._data.oMf[self._frame_id].homogeneous.copy()

    def _errors(self, q_canonical, target) -> tuple[float, float]:
        current = self.pose(q_canonical)
        position = float(np.linalg.norm(target[:3, 3] - current[:3, 3]))
        cosine = (np.trace(current[:3, :3].T @ target[:3, :3]) - 1.0) / 2.0
        return position, float(np.arccos(np.clip(cosine, -1.0, 1.0)))

    # -------------------------------------------------------------- solve
    def solve(self, target, seed, lower, upper, ik: IkSettings = IkSettings()) -> IkResult:
        import pink

        pin = self._pin
        target = np.asarray(target, dtype=float)
        seed = np.asarray(seed, dtype=float)
        if target.shape != (4, 4) or not np.isfinite(target).all():
            return IkResult(False, seed, float("nan"), float("nan"), 0, "target is not finite")
        if seed.shape != (len(self._index),) or not np.isfinite(seed).all():
            return IkResult(False, seed, float("nan"), float("nan"), 0, "seed is not finite")

        start = np.clip(seed, lower, upper)
        configuration = pink.Configuration(self._model, self._data, self._model_q(start))
        self._task.set_target(pin.SE3(target[:3, :3].copy(), target[:3, 3].copy()))
        self._posture.set_target(self._model_q(start))
        q = start
        position_error = rotation_error = float("inf")
        iterations = 0
        for iterations in range(ik.max_iterations + 1):
            q = np.clip(configuration.q[self._index] * self._sign, lower, upper)
            position_error, rotation_error = self._errors(q, target)
            if (position_error <= ik.position_tolerance_m
                    and rotation_error <= ik.rotation_tolerance_rad):
                break
            if iterations == ik.max_iterations:
                return IkResult(
                    False, q, position_error, rotation_error, iterations,
                    f"target not reachable: {position_error * 1000:.1f} mm, "
                    f"{rotation_error:.3f} rad left after {iterations} iterations",
                )
            velocity = pink.solve_ik(
                configuration, [self._task, self._posture], self.settings.dt,
                solver=self.settings.solver, limits=self._limits, safety_break=False)
            configuration.integrate_inplace(velocity, self.settings.dt)

        if not np.isfinite(q).all():
            return IkResult(False, seed, position_error, rotation_error, iterations,
                            "solution is not finite")
        distance = float(np.max(np.abs(q - seed)))
        if distance > ik.max_seed_distance_rad:
            return IkResult(
                False, q, position_error, rotation_error, iterations,
                f"solution is {distance:.3f} rad from the previous one "
                f"(limit {ik.max_seed_distance_rad:.3f}); refusing a jump",
            )
        return IkResult(True, q, position_error, rotation_error, iterations)
