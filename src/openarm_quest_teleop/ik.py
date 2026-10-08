"""IK settings and result shared by the teleop core and its solver (pink_ik).

The teleop core needs to know whether a target is reachable *before* commanding
toward it, so the solver iterates until the palm is at the target or the budget
runs out, holds the joint limits inside the iteration, and refuses a solution
that ends far from its seed (the previous solution): consecutive teleop targets
are millimetres apart, so a large change means a jumped branch or a leaping
target. Only the arm's own seven joints change.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Damping of the former damped-least-squares step, kept as the IkSettings
#: default so existing configs still load; pink uses its own lm_damping.
DEFAULT_DAMPING = 0.05


@dataclass(frozen=True)
class IkSettings:
    max_iterations: int = 30
    position_tolerance_m: float = 1e-3
    rotation_tolerance_rad: float = 1e-2
    damping: float = DEFAULT_DAMPING  # unused by pink; accepted for old configs
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
