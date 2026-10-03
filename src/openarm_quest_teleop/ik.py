"""Pose IK for one arm, iterated on the existing damped-least-squares step.

``robot_control.kinematics.Chain`` already supplies FK, the Jacobian and one
damped step (``delta_q``); the marker servo takes exactly one such step per
cycle and lets the next cycle continue. Teleoperation needs to know whether a
target is reachable *before* commanding toward it, so the same step is
iterated here until the palm is at the target or the budget runs out.

Only the chain's own joints change: the chain is built from one arm's seven
joints, so neither the hand nor the other arm can be used to reach a target.
The seed is the previous solution, which keeps consecutive solutions on one
branch; a solution that still ends far from its seed is refused rather than
followed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from robot_control.kinematics import Chain, DEFAULT_DAMPING, twist_between


@dataclass(frozen=True)
class IkSettings:
    max_iterations: int = 30
    position_tolerance_m: float = 1e-3
    rotation_tolerance_rad: float = 1e-2
    damping: float = DEFAULT_DAMPING
    #: Largest joint change one iteration may make, so a far target is walked
    #: toward instead of linearized across.
    max_iteration_step_rad: float = 0.1
    #: Largest distance a solution may end from its seed. Consecutive teleop
    #: targets are millimetres apart; a solve that needs more than this has
    #: jumped branches or is chasing a target that leapt.
    max_seed_distance_rad: float = 0.5


@dataclass(frozen=True)
class IkResult:
    ok: bool
    q: np.ndarray
    position_error_m: float
    rotation_error_rad: float
    iterations: int
    reason: str | None = None


def solve_pose(
    chain: Chain,
    target: np.ndarray,
    seed: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    settings: IkSettings = IkSettings(),
) -> IkResult:
    """Solve for the joint values that put the chain's tip at *target*.

    Joint limits are enforced inside the iteration, so a returned solution is
    always within them; a target that can only be met outside them comes back
    as not reachable, with the remaining error.
    """
    target = np.asarray(target, dtype=float)
    seed = np.asarray(seed, dtype=float)
    if target.shape != (4, 4) or not np.isfinite(target).all():
        return IkResult(False, seed, float("nan"), float("nan"), 0, "target is not finite")
    if seed.shape != (len(chain),) or not np.isfinite(seed).all():
        return IkResult(False, seed, float("nan"), float("nan"), 0, "seed is not finite")

    q = np.clip(seed, lower, upper)
    position_error = rotation_error = float("inf")
    iterations = 0
    for iterations in range(settings.max_iterations + 1):
        twist = twist_between(chain.pose(q), target)
        position_error = float(np.linalg.norm(twist[:3]))
        rotation_error = float(np.linalg.norm(twist[3:]))
        if (
            position_error <= settings.position_tolerance_m
            and rotation_error <= settings.rotation_tolerance_rad
        ):
            break
        if iterations == settings.max_iterations:
            return IkResult(
                False, q, position_error, rotation_error, iterations,
                f"target not reachable: {position_error * 1000:.1f} mm, "
                f"{rotation_error:.3f} rad left after {iterations} iterations",
            )
        step = chain.delta_q(q, twist, settings.damping)
        largest = float(np.max(np.abs(step)))
        if largest > settings.max_iteration_step_rad:
            step *= settings.max_iteration_step_rad / largest
        q = np.clip(q + step, lower, upper)

    if not np.isfinite(q).all():
        return IkResult(False, seed, position_error, rotation_error, iterations,
                        "solution is not finite")
    distance = float(np.max(np.abs(q - seed)))
    if distance > settings.max_seed_distance_rad:
        return IkResult(
            False, q, position_error, rotation_error, iterations,
            f"solution is {distance:.3f} rad from the previous one "
            f"(limit {settings.max_seed_distance_rad:.3f}); refusing a jump",
        )
    return IkResult(True, q, position_error, rotation_error, iterations)
