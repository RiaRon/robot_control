"""Hand targets from a glove retarget node -> one fake RH56F1 hand controller.

Pure Python and numpy, no ROS import. The ROS node (``ros_adapter``) feeds it
samples and publishes what it returns.

Contract
--------
The producer (today the retarget node of ``inspire_hand-main.zip``) publishes
``sensor_msgs/JointState`` *targets* on ``/inspire_<side>/retarget/joint_states``:
six source joints in rad, 0 = fully open, a per-joint value for fully closed.
The adapter maps them **by name** to the hand's six actuators in manifest order
(``thumb_1, thumb_2, index_1, middle_1, ring_1, pinky_1``) and scales the
closure onto each actuator's canonical URDF range:

    canonical = clamp(source / closed, 0, 1) * upper        (lower is 0)

That scaling is a **fake display mapping**. It is not a raw <-> URDF
calibration and says nothing about the real hand.

States
------
``waiting``    no command yet; following starts with the first valid target
               that arrived after start (or after an enable).
``following``  one rate-limited command per step (robot_control CommandGate).
``locked``     targets or hand state went stale or the gate refused: nothing is
               sent until ``enable()`` is called (a service in the ROS node).
               Following never resumes on its own.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
import yaml

from robot_control.safety import CommandGate, SafetyError

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "glove_adapter.yaml"
ACTUATORS = ("thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "pinky_1")
SIDES = ("right", "left")
THUMB_POLICIES = ("hold", "follow")
WAITING = "waiting"
FOLLOWING = "following"
LOCKED = "locked"
#: Same lead budget as the arm stream (robot_control cli.LEAD_SEC).
LEAD_SEC = 0.1
FAKE_MAPPING_NOTE = (
    "fake display mapping: source closure scaled onto the canonical URDF range; "
    "not a raw<->URDF calibration")


class ContractError(ValueError):
    """The configuration cannot describe a hand target contract."""


class TargetError(ValueError):
    """One target message is unusable; the message says why."""


def load_config(path: str | Path | None = None) -> dict:
    data = yaml.safe_load(Path(path or DEFAULT_CONFIG).read_text())
    for key in ("source", "thumb", "glove", "hand", "timing"):
        if key not in data:
            raise ContractError(f"config has no {key!r} section")
    return data


def fill(template: str, **values: str) -> str:
    return template.format(**values)


@dataclass(frozen=True)
class SourceJoint:
    name: str
    actuator: str
    closed: float


@dataclass(frozen=True)
class Contract:
    """One side's mapping from source target names to canonical actuators."""

    side: str
    canonical: tuple[str, ...]      # manifest order, e.g. r_hj_thumb_1 ...
    upper: np.ndarray               # canonical upper limits (lower are 0)
    lower: np.ndarray
    velocity: np.ndarray
    sources: tuple[SourceJoint, ...]
    thumb_pitch: str
    thumb_yaw: str

    @property
    def prefix(self) -> str:
        return "r_hj_" if self.side == "right" else "l_hj_"

    def index(self, actuator: str) -> int:
        return self.canonical.index(self.prefix + actuator)

    @property
    def held(self) -> set[str]:
        """Actuators held at their seeded value instead of following."""
        held = set()
        by_actuator = {s.actuator: s for s in self.sources}
        if self.thumb_pitch == "hold" and "thumb_2" in by_actuator:
            held.add("thumb_2")
        if self.thumb_yaw == "hold" and "thumb_1" in by_actuator:
            held.add("thumb_1")
        return held


def build_contract(config: dict, profile, manifest_order: Sequence[str], side: str,
                   thumb_pitch: str | None = None, thumb_yaw: str | None = None) -> Contract:
    """Resolve names, order and limits for *side* from the manifest and profile."""
    if side not in SIDES:
        raise ContractError(f"side must be one of {SIDES}")
    prefix = "r_hj_" if side == "right" else "l_hj_"
    canonical = tuple(n for n in manifest_order if n.startswith(prefix))
    if [n[len(prefix):] for n in canonical] != list(ACTUATORS):
        raise ContractError(f"manifest {side} hand joints are {canonical}, expected {ACTUATORS}")
    joints = {joint.canonical: joint for joint in profile.joints}
    missing = [n for n in canonical if n not in joints]
    if missing:
        raise ContractError(f"profile has no limits for {missing}")
    sources = []
    seen = set()
    for name, spec in config["source"]["joints"].items():
        actuator = spec["actuator"]
        closed = float(spec["closed"])
        if actuator not in ACTUATORS:
            raise ContractError(f"source {name!r} maps to unknown actuator {actuator!r}")
        if actuator in seen:
            raise ContractError(f"two source joints map to {actuator!r}")
        if not math.isfinite(closed) or closed <= 0.0:
            raise ContractError(f"source {name!r} needs a positive 'closed' value")
        seen.add(actuator)
        sources.append(SourceJoint(str(name), actuator, closed))
    if seen != set(ACTUATORS):
        raise ContractError(f"source joints cover {sorted(seen)}, not all of {ACTUATORS}")
    pitch = thumb_pitch or config["thumb"]["pitch"]
    yaw = thumb_yaw or config["thumb"]["yaw"]
    for value in (pitch, yaw):
        if value not in THUMB_POLICIES:
            raise ContractError(f"thumb policy must be one of {THUMB_POLICIES}, got {value!r}")
    lower = np.array([joints[n].lower for n in canonical])
    if np.any(lower != 0.0):
        raise ContractError("the closure mapping assumes every hand actuator's lower limit is 0")
    return Contract(
        side=side, canonical=canonical,
        upper=np.array([joints[n].upper for n in canonical]), lower=lower,
        velocity=np.array([joints[n].velocity for n in canonical]),
        sources=tuple(sources), thumb_pitch=pitch, thumb_yaw=yaw)


@dataclass(frozen=True)
class Mapped:
    """A validated target, in canonical order."""

    target: np.ndarray          # canonical rad, manifest order (held axes unset)
    clamped: tuple[str, ...]    # source joints whose finite value was outside [0, closed]
    ignored: tuple[str, ...]    # names in the message the contract does not use


def map_target(contract: Contract, names: Sequence[str], positions: Sequence[float]) -> Mapped:
    """Validate one target message and map it by name. Raises TargetError."""
    names = list(names)
    positions = list(positions)
    if len(names) != len(positions):
        raise TargetError(f"{len(names)} names but {len(positions)} positions")
    if len(set(names)) != len(names):
        duplicates = sorted({n for n in names if names.count(n) > 1})
        raise TargetError(f"duplicate joint names {duplicates}")
    values = {}
    for name, value in zip(names, positions):
        value = float(value)
        if not math.isfinite(value):
            raise TargetError(f"{name} is not finite")
        values[name] = value
    missing = [s.name for s in contract.sources if s.name not in values]
    if missing:
        raise TargetError(f"missing source joints {missing}")
    target = np.full(len(contract.canonical), np.nan)
    clamped = []
    for source in contract.sources:
        ratio = values[source.name] / source.closed
        if ratio < 0.0 or ratio > 1.0:
            clamped.append(source.name)
        i = contract.index(source.actuator)
        target[i] = min(1.0, max(0.0, ratio)) * contract.upper[i]
    used = {s.name for s in contract.sources}
    return Mapped(target, tuple(clamped), tuple(sorted(n for n in names if n not in used)))


@dataclass(frozen=True)
class TargetSample:
    arrival_sec: float
    names: tuple[str, ...]
    positions: tuple[float, ...]


@dataclass(frozen=True)
class JointSample:
    arrival_sec: float
    positions: Mapping[str, float]


@dataclass
class Counters:
    accepted: int = 0
    rejected: int = 0
    clamped: int = 0
    commands: int = 0
    last_rejection: str | None = None
    last_clamped: tuple[str, ...] = ()


@dataclass(frozen=True)
class StepResult:
    state: str
    command: np.ndarray | None = None
    reason: str | None = None
    limited: str | None = None
    target: np.ndarray | None = None
    target_age_sec: float | None = None
    joint_age_sec: float | None = None


@dataclass
class GloveHandAdapter:
    contract: Contract
    command_period_sec: float
    target_timeout_sec: float = 0.5
    joint_state_timeout_sec: float = 0.5
    state: str = WAITING
    counters: Counters = field(default_factory=Counters)
    _gate: CommandGate | None = None
    _held: dict = field(default_factory=dict)
    _since: float = float("-inf")         # targets older than this are not used
    _last_target: TargetSample | None = None
    _mapped: Mapped | None = None
    _lock_reason: str | None = None

    def __post_init__(self):
        for name in ("command_period_sec", "target_timeout_sec", "joint_state_timeout_sec"):
            if not getattr(self, name) > 0.0:
                raise ContractError(f"{name} must be positive")

    # ------------------------------------------------------------- inputs
    def offer_target(self, sample: TargetSample) -> None:
        """Validate a newly arrived target; an invalid one does not refresh."""
        try:
            mapped = map_target(self.contract, sample.names, sample.positions)
        except TargetError as error:
            self.counters.rejected += 1
            self.counters.last_rejection = str(error)
            return
        self.counters.accepted += 1
        if mapped.clamped:
            self.counters.clamped += 1
            self.counters.last_clamped = mapped.clamped
        self._last_target = sample
        self._mapped = mapped

    def enable(self, now_sec: float) -> str:
        """Leave ``locked``; only targets arriving after now will be used."""
        previous = self.state
        self.state = WAITING
        self._gate = None
        self._lock_reason = None
        self._since = now_sec
        return f"{previous} -> waiting for a target newer than this call"

    def _lock(self, reason: str, ages) -> StepResult:
        if self.state == FOLLOWING:
            self.state = LOCKED
            self._gate = None
            self._lock_reason = reason
        return StepResult(self.state, reason=reason if self.state != LOCKED else
                          f"{self._lock_reason}; call enable to resume", **ages)

    # --------------------------------------------------------------- step
    def step(self, now_sec: float, joints: JointSample | None) -> StepResult:
        target = self._last_target
        target_age = None if target is None else now_sec - target.arrival_sec
        joint_age = None if joints is None else now_sec - joints.arrival_sec
        ages = dict(target_age_sec=target_age, joint_age_sec=joint_age)

        if joints is None or joint_age > self.joint_state_timeout_sec:
            return self._lock("no hand joint state" if joints is None
                              else "hand joint state is stale", ages)
        try:
            measured = np.array([float(joints.positions[n]) for n in self.contract.canonical])
        except KeyError as error:
            return self._lock(f"hand joint state lacks {error}", ages)
        if not np.isfinite(measured).all():
            return self._lock("hand joint state is not finite", ages)

        if self.state == LOCKED:
            return StepResult(LOCKED, reason=f"{self._lock_reason}; call enable to resume", **ages)
        fresh = (target is not None and target.arrival_sec > self._since
                 and target_age <= self.target_timeout_sec)
        if not fresh:
            if self.state == FOLLOWING:
                return self._lock("glove target is stale", ages)
            return StepResult(WAITING, reason="no glove target yet" if target is None
                              else "waiting for a fresh glove target", **ages)

        if self.state == WAITING:
            c = self.contract
            self._gate = CommandGate(
                execute=True, lower=c.lower, upper=c.upper, velocity=c.velocity,
                command_period_sec=self.command_period_sec,
                max_lead=c.velocity * LEAD_SEC, names=list(c.canonical))
            # Held thumb axes stay where they measured when following started.
            self._held = {c.prefix + a: measured[c.index(a)] for a in c.held}
            self.state = FOLLOWING

        desired = np.array(self._mapped.target, dtype=float)
        for i, name in enumerate(self.contract.canonical):
            if name in self._held:
                desired[i] = self._held[name]
        try:
            command, limited = self._gate.follow(desired, measured, self.command_period_sec)
        except SafetyError as error:
            return self._lock(f"command gate refused: {error}", ages)
        self.counters.commands += 1
        return StepResult(FOLLOWING, command=command, limited=limited, target=desired, **ages)


def fake_hand_refusal(contract: Contract, manager: str, controller: str, fake_plugin: str,
                      classes: Sequence[str] | None, commands: Sequence[str] | None,
                      controller_state: str | None,
                      controller_joints: Sequence[str] | None) -> str | None:
    """Why ``--execute`` must be refused for this hand manager, or None.

    ``None`` arguments mean the manager did not answer that query; anything not
    positively identified as this hand on fake hardware is refused.
    """
    if classes is None:
        return f"{manager} does not answer, so its hardware cannot be checked"
    if not classes or set(classes) != {fake_plugin}:
        return f"{manager} runs {list(classes)}, not only {fake_plugin}"
    wanted = sorted(f"{n}/position" for n in contract.canonical)
    if commands is None or sorted(commands) != wanted:
        return f"{manager} exports {None if commands is None else sorted(commands)}, " \
               f"not this hand's six positions"
    if controller_state != "active":
        return f"{controller} is {controller_state or 'not loaded'}"
    if controller_joints is None or list(controller_joints) != list(contract.canonical):
        return f"{controller} drives {controller_joints}, not {list(contract.canonical)}"
    return None

