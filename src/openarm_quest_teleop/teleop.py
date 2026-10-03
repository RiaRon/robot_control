"""Arm teleoperation core: enable clutch, relative target, IK, command gate.

Pure Python and numpy, with no ROS import, so every rule below is tested
without a graph. The ROS node only feeds it samples and publishes what it
returns.

States
------
``idle``     not following; the arm holds whatever it was last commanded.
``engaged``  the enable input is held and every check passes; one joint
             command is produced per step.
``locked``   following was cut by a fault (input lost or invalid, joint state
             stale, input jumped). It does not resume by itself: the enable
             input must be released and pressed again, which re-captures both
             start poses at the then-current pose.

Each step produces at most one command, and only while engaged. Releasing the
enable input stops the stream at once; the trajectory controller then holds
the last commanded joint position.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import math

import numpy as np

from robot_control.kinematics import Chain
from robot_control.safety import CommandGate, SafetyError
from .ik import IkSettings, solve_pose
from .relative import MappingError, RelativeTargetMapper

IDLE = "idle"
ENGAGED = "engaged"
LOCKED = "locked"

ENABLE_SOURCES = ("grip", "trigger", "button_primary", "button_secondary")


@dataclass(frozen=True)
class ControllerSample:
    """The newest controller input, and when this process received it.

    ``arrival_sec`` is the consumer's monotonic clock at arrival. It is the
    only time used to judge freshness; the headset's clock and the joint-state
    stamps are never compared with it.
    """

    arrival_sec: float
    pose: np.ndarray | None  # 4x4 in quest_world, or None when not valid
    grip: float = 0.0
    trigger: float = 0.0
    buttons: tuple[bool, bool] = (False, False)
    reference: np.ndarray | None = None

    def enable_value(self, source: str) -> float:
        if source == "grip":
            return self.grip
        if source == "trigger":
            return self.trigger
        if source == "button_primary":
            return 1.0 if self.buttons[0] else 0.0
        if source == "button_secondary":
            return 1.0 if self.buttons[1] else 0.0
        raise ValueError(f"enable source must be one of {ENABLE_SOURCES}")


@dataclass(frozen=True)
class JointSample:
    """The newest measured arm joints (canonical order), and their arrival."""

    arrival_sec: float
    q: np.ndarray


@dataclass(frozen=True)
class TeleopSettings:
    enable_source: str = "grip"
    press_threshold: float = 0.7
    release_threshold: float = 0.3
    input_timeout_sec: float = 0.2
    joint_state_timeout_sec: float = 0.2
    #: A controller pose that moves further than this between two consecutive
    #: samples is a tracking glitch, not a hand.
    max_input_jump_m: float = 0.1
    max_input_jump_rad: float = 0.5
    #: Enabling is refused while the arm's Jacobian is this close to losing
    #: rank. The OpenArm hanging straight down (all joints zero, elbow on its
    #: lower limit) is such a pose: a local IK step cannot leave it, so
    #: following from there would only report IK failures. Zero disables it.
    min_enable_singular_value: float = 0.0
    ik: IkSettings = field(default_factory=IkSettings)

    def __post_init__(self):
        if self.enable_source not in ENABLE_SOURCES:
            raise ValueError(f"enable source must be one of {ENABLE_SOURCES}")
        if not 0.0 <= self.release_threshold < self.press_threshold <= 1.0:
            raise ValueError(
                "thresholds must satisfy 0 <= release < press <= 1 "
                f"(got release {self.release_threshold}, press {self.press_threshold})"
            )
        for name in ("input_timeout_sec", "joint_state_timeout_sec",
                     "max_input_jump_m", "max_input_jump_rad"):
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class StepResult:
    state: str
    #: Joint command in canonical order, or None when nothing may be sent.
    command: np.ndarray | None = None
    #: Why nothing was sent, or why following was cut. None when following.
    reason: str | None = None
    #: What bounded the command (velocity / lead / position limit, workspace).
    limited: str | None = None
    target: np.ndarray | None = None
    just_engaged: bool = False
    position_error_m: float | None = None
    rotation_error_rad: float | None = None
    input_age_sec: float | None = None
    joint_age_sec: float | None = None


def _rotation_angle(a: np.ndarray, b: np.ndarray) -> float:
    cosine = (np.trace(a.T @ b) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, float(cosine))))


class ArmTeleop:
    """One arm's teleoperation. ``step`` is called once per command period."""

    def __init__(
        self,
        chain: Chain,
        mapper: RelativeTargetMapper,
        *,
        lower: np.ndarray,
        upper: np.ndarray,
        velocity: np.ndarray,
        command_period_sec: float,
        max_lead: np.ndarray | None = None,
        names: list[str] | None = None,
        settings: TeleopSettings = TeleopSettings(),
    ):
        self.chain = chain
        self.mapper = mapper
        self.settings = settings
        self._limits = dict(
            lower=np.asarray(lower, dtype=float),
            upper=np.asarray(upper, dtype=float),
            velocity=np.asarray(velocity, dtype=float),
            command_period_sec=float(command_period_sec),
            max_lead=None if max_lead is None else np.asarray(max_lead, dtype=float),
            names=names,
        )
        self.state = IDLE
        self._gate: CommandGate | None = None
        self._solution: np.ndarray | None = None
        self._previous_pose: np.ndarray | None = None
        # Enabling needs a press *edge*: the input must have been seen
        # released since start-up and since the last time following ended.
        self._armed = False
        self._lock_reason: str | None = None
        # Why the press now being held did not start following; shown until
        # the input is released, so a status read later still says why.
        self._refusal: str | None = None

    # -------------------------------------------------------------- helpers
    def _stop(self, state: str, reason: str | None = None) -> None:
        self.state = state
        self.mapper.release()
        self._gate = None
        self._solution = None
        self._previous_pose = None
        self._armed = False
        self._lock_reason = reason if state == LOCKED else None

    def _fault(self, reason: str, pressed: bool = False, **ages) -> StepResult:
        """Cut following if it was running; report why nothing is sent.

        A press made while something is wrong is spent: once the fault clears
        the input must be released and pressed again, so following never
        starts at a moment the operator did not choose.
        """
        if self.state == ENGAGED:
            self._stop(LOCKED, reason)
        elif pressed:
            self._armed = False
            self._refusal = f"enable was pressed while: {reason}"
        return StepResult(self.state, reason=reason, **ages)

    # ----------------------------------------------------------------- step
    def step(
        self,
        now_sec: float,
        controller: ControllerSample | None,
        joints: JointSample | None,
    ) -> StepResult:
        settings = self.settings
        input_age = None if controller is None else now_sec - controller.arrival_sec
        joint_age = None if joints is None else now_sec - joints.arrival_sec
        ages = dict(input_age_sec=input_age, joint_age_sec=joint_age)

        # The enable input is tracked even while nothing else is usable, so a
        # release during a fault still counts as the release a re-enable needs.
        input_fresh = controller is not None and input_age <= settings.input_timeout_sec
        value = controller.enable_value(settings.enable_source) if input_fresh else None
        if value is not None and not math.isfinite(value):
            return self._fault("enable input is not finite", **ages)
        pressed = value is not None and value >= settings.press_threshold
        released = value is not None and value <= settings.release_threshold
        ages["pressed"] = pressed

        if self.state == ENGAGED and released:
            self._stop(IDLE)
        if self.state != ENGAGED and released:
            if self.state == LOCKED:
                self._stop(IDLE)
            self._armed = True
            self._refusal = None

        if not input_fresh:
            # The age is reported in input_age_sec, not in the text, so the
            # reason stays the same from one step to the next.
            return self._fault(
                "no controller input" if controller is None
                else "controller input is stale", **ages)
        if joints is None or joint_age > settings.joint_state_timeout_sec:
            return self._fault(
                "no joint state" if joints is None else "joint state is stale", **ages)
        q = np.asarray(joints.q, dtype=float)
        if q.shape != (len(self.chain),) or not np.isfinite(q).all():
            return self._fault("joint state is not finite", **ages)
        if controller.pose is None:
            return self._fault("controller pose is not valid", **ages)
        # A copy: the previous pose is kept for the jump check, and must not
        # alias an array the caller goes on to modify.
        pose = np.array(controller.pose, dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            return self._fault("controller pose is not finite", **ages)

        ages.pop("pressed")
        if self.state == LOCKED:
            return StepResult(
                LOCKED,
                reason=f"{self._lock_reason}; release and press the enable input again",
                **ages,
            )
        if self.state == IDLE:
            if not (pressed and self._armed):
                reason = None
                if pressed:
                    reason = (self._refusal or "enable input was already held") + (
                        "; release and press it again")
                return StepResult(IDLE, reason=reason, **ages)
            return self._engage(pose, q, controller, ages)

        # ENGAGED: the held band between the two thresholds keeps following.
        previous = self._previous_pose
        jump_m = float(np.linalg.norm(pose[:3, 3] - previous[:3, 3]))
        jump_rad = _rotation_angle(previous[:3, :3], pose[:3, :3])
        if jump_m > settings.max_input_jump_m or jump_rad > settings.max_input_jump_rad:
            return self._fault(
                f"controller pose jumped {jump_m * 1000:.0f} mm / {jump_rad:.2f} rad "
                "between samples", **ages)
        self._previous_pose = pose
        return self._follow(pose, q, ages)

    def _engage(self, pose, q, controller, ages) -> StepResult:
        smallest = float(np.linalg.svd(self.chain.jacobian(q), compute_uv=False)[-1])
        if smallest < self.settings.min_enable_singular_value:
            return self._refuse(
                f"cannot enable: the arm is at a singular pose (smallest Jacobian "
                f"singular value {smallest:.3f} < "
                f"{self.settings.min_enable_singular_value:.3f}); move it to a bent "
                "posture first", ages)
        start = self.chain.pose(q)
        try:
            self.mapper.engage(pose, start, controller.reference)
        except MappingError as error:
            return self._refuse(f"cannot enable: {error}", ages)
        self._gate = CommandGate(execute=True, **self._limits)
        # The measured pose seeds both the IK and the gate, so the first
        # command is the pose the arm is already in.
        self._solution = q.copy()
        self._previous_pose = pose
        self._armed = False
        self.state = ENGAGED
        result = self._follow(pose, q, ages)
        if result.state != ENGAGED:
            return result
        return replace(result, just_engaged=True)

    def _refuse(self, reason: str, ages) -> StepResult:
        """A press that cannot start following is spent, and says why."""
        self._armed = False
        self._refusal = reason
        return StepResult(IDLE, reason=f"{reason}; release and press it again", **ages)

    def _follow(self, pose, q, ages) -> StepResult:
        target, clamped = self.mapper.target(pose)
        limits = self._limits
        solved = solve_pose(
            self.chain, target, self._solution, limits["lower"], limits["upper"],
            self.settings.ik,
        )
        if not solved.ok:
            # Still engaged: the hand may come back into reach. Nothing is sent
            # and the previous solution is kept as the seed.
            return StepResult(
                ENGAGED, reason=f"IK failed: {solved.reason}", target=target,
                position_error_m=solved.position_error_m,
                rotation_error_rad=solved.rotation_error_rad, **ages)
        try:
            command, limited = self._gate.follow(
                solved.q, q, limits["command_period_sec"])
        except SafetyError as error:
            return self._fault(f"command gate refused: {error}", **ages)
        self._solution = solved.q
        notes = [note for note in (limited, "workspace limit" if clamped else None) if note]
        return StepResult(
            ENGAGED, command=command, limited=" and ".join(notes) or None,
            target=target, position_error_m=solved.position_error_m,
            rotation_error_rad=solved.rotation_error_rad, **ages)
