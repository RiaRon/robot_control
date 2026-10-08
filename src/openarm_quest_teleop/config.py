"""Configuration, and binding one arm to the description that is running.

The profile (``profiles/openarm_rh56f1.yaml``) stays the only place joint
names, their order, signs and limits are written. Which naming the running
description uses is read off the description itself: the fake bringup names
the arm joints canonically (``r_aj_1``), the real one by source
(``openarm_right_joint1``), and the palm link is ``*_hl_palm_sensor`` in both.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ElementTree

import numpy as np
import yaml

from robot_control.kinematics import Chain, KinematicsError, chain_from_urdf
from robot_control.profile import RobotProfile, load_builtin_profile
from .ik import IkSettings
from .relative import RelativeTargetMapper
from .teleop import ArmTeleop, TeleopSettings

DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "quest_teleop.yaml"
#: The only hardware plugin a run without the real-hardware confirmation may
#: command.
FAKE_PLUGIN = "mock_components/GenericSystem"
#: Same lead budget the marker servo uses (cli.LEAD_SEC): how far, as travel
#: time at the velocity limit, a streamed command may run ahead of the arm.
LEAD_SEC = 0.1
RUNTIMES = ("fake", "real", "fake_integrated")
#: ``teleop.ik_backend``: dls = ik.solve_pose (numpy, default); pink = pink_ik
#: (Pinocchio QP, needs the pink venv).
IK_BACKENDS = ("dls", "pink")
#: Runtimes that must be driving GenericSystem; anything else there is refused.
FAKE_RUNTIMES = ("fake", "fake_integrated")


class ConfigError(ValueError):
    """The configuration, or the running description, cannot be used."""


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    path = DEFAULT_CONFIG if path is None else Path(path)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or "quest" not in data or "teleop" not in data:
        raise ConfigError(f"{path} must define 'quest' and 'teleop'")
    return data


@dataclass(frozen=True)
class ArmBinding:
    """How one arm's canonical joints appear in the running system."""

    arm: str
    group: str
    canonical_names: tuple[str, ...]
    #: Names used by /joint_states and the controller, in canonical order.
    runtime_names: tuple[str, ...]
    sign: np.ndarray
    naming: str  # "canonical" or "source"
    control_frame: str
    base_frame: str
    plugins: tuple[str, ...]

    @property
    def is_fake(self) -> bool:
        return self.plugins == (FAKE_PLUGIN,)

    def to_canonical(self, positions: dict[str, float]) -> np.ndarray | None:
        """Canonical joint vector from a name -> position map, or None."""
        try:
            values = np.array([positions[name] for name in self.runtime_names], dtype=float)
        except KeyError:
            return None
        return values * self.sign

    def to_runtime(self, q: np.ndarray) -> list[float]:
        return [float(value) for value in np.asarray(q, dtype=float) * self.sign]


def _root_link(root: ElementTree.Element, link: str) -> str:
    parent_of = {
        joint.find("child").get("link"): joint.find("parent").get("link")
        for joint in root.findall("joint")
        if joint.find("child") is not None and joint.find("parent") is not None
    }
    while link in parent_of:
        link = parent_of[link]
    return link


def bind_arm(urdf: str, profile: RobotProfile, config: dict, arm: str) -> ArmBinding:
    """Resolve *arm*'s joints, naming, palm frame and hardware plugin."""
    arms = config["teleop"]["arms"]
    if arm not in arms:
        raise ConfigError(f"unknown arm {arm!r}; configured arms are {sorted(arms)}")
    entry = arms[arm]
    if entry["group"] not in profile.groups:
        raise ConfigError(f"profile {profile.name!r} has no group {entry['group']!r}")
    group = profile.groups[entry["group"]]
    by_canonical = {joint.canonical: joint for joint in profile.joints}
    joints = [by_canonical[name] for name in group.joints]

    root = ElementTree.fromstring(urdf)
    present = {element.get("name") for element in root.findall("joint")}
    if all(joint.source in present for joint in joints):
        naming, names = "source", tuple(joint.source for joint in joints)
    elif all(joint.canonical in present for joint in joints):
        naming, names = "canonical", tuple(joint.canonical for joint in joints)
    else:
        raise ConfigError(
            f"the running description has neither the source nor the canonical "
            f"joints of group {group.name!r}"
        )
    control_frame = entry["control_frame"]
    if control_frame not in {link.get("name") for link in root.findall("link")}:
        raise ConfigError(
            f"the running description has no link {control_frame!r}; the hand "
            "geometry must stay in the description for its palm frame"
        )

    plugins = []
    wanted = set(names)
    for block in root.findall("ros2_control"):
        if wanted & {joint.get("name") for joint in block.findall("joint")}:
            plugin = block.find("hardware/plugin")
            plugins.append("" if plugin is None else (plugin.text or "").strip())
    if not plugins:
        raise ConfigError(f"no ros2_control block exports the joints of {group.name!r}")
    return ArmBinding(
        arm=arm,
        group=group.name,
        canonical_names=tuple(group.joints),
        runtime_names=names,
        sign=np.array([joint.sign for joint in joints], dtype=float),
        naming=naming,
        control_frame=control_frame,
        base_frame=_root_link(root, control_frame),
        plugins=tuple(sorted(set(plugins))),
    )


def runtime_endpoint(config: dict, runtime: str) -> dict:
    """The runtime's joint-state topic and controller manager, with defaults.

    A config written before the split bringup has neither key; it falls back to
    the teleop-wide ``joint_states_topic`` and ``/controller_manager``.
    """
    if runtime not in RUNTIMES:
        raise ConfigError(f"runtime must be one of {RUNTIMES}")
    entry = config["teleop"]["runtimes"].get(runtime)
    if entry is None:
        raise ConfigError(f"the config has no runtime {runtime!r}")
    return {
        "joint_states_topic": entry.get(
            "joint_states_topic", config["teleop"]["joint_states_topic"]),
        "controller_manager": entry.get("controller_manager", "/controller_manager"),
    }


def controller_for(config: dict, profile: RobotProfile, binding: ArmBinding, runtime: str) -> str:
    if runtime not in RUNTIMES:
        raise ConfigError(f"runtime must be one of {RUNTIMES}")
    controllers = config["teleop"]["runtimes"][runtime]["controllers"]
    if controllers == "profile":
        controller = profile.groups[binding.group].controller
        if controller is None:
            raise ConfigError(f"profile group {binding.group!r} names no controller")
        return controller
    return controllers[binding.arm]


def execution_refusal(
    binding: ArmBinding, runtime: str, execute: bool, confirm_real_hardware: bool
) -> str | None:
    """Why this run may not go on, or None. Decided before anything is published.

    Fake hardware needs ``--execute`` to be commanded. Anything else is a real
    arm, and needs ``--confirm-real-hardware`` as well: one flag carried over
    from a fake session must not be enough to move it.
    """
    if runtime not in RUNTIMES:
        return f"runtime must be one of {RUNTIMES}"
    if runtime in FAKE_RUNTIMES and not binding.is_fake:
        return (
            f"--runtime {runtime}, but the arm is driven by {list(binding.plugins)}, "
            f"not {FAKE_PLUGIN}; use --runtime real"
        )
    if execute and not binding.is_fake and not confirm_real_hardware:
        return (
            f"the arm is driven by {list(binding.plugins)}: this is real hardware. "
            "--execute alone is refused; add --confirm-real-hardware once the "
            "checks in docs/quest-teleop.md are done"
        )
    return None


def build_chain(urdf: str, binding: ArmBinding) -> Chain:
    """The arm's seven-joint chain to its palm frame, in canonical order."""
    try:
        return chain_from_urdf(urdf, binding.runtime_names, binding.control_frame)
    except KinematicsError as error:
        raise ConfigError(str(error)) from error


class _SignedChain(Chain):
    """A chain whose joint values are canonical (profile sign applied)."""

    def __init__(self, chain: Chain, sign: np.ndarray):
        super().__init__(chain.joints, chain.links, chain.tip)
        self._sign = sign

    def frames(self, q):
        return super().frames(self._check(q) * self._sign)

    def jacobian(self, q):
        return super().jacobian(q) * self._sign


def build_teleop(
    urdf: str,
    profile: RobotProfile,
    config: dict,
    binding: ArmBinding,
    *,
    orientation_mode: str | None = None,
) -> ArmTeleop:
    """Assemble the core for one arm from the config and the profile limits."""
    teleop = config["teleop"]
    chain = build_chain(urdf, binding)
    if not np.all(binding.sign == 1.0):
        chain = _SignedChain(chain, binding.sign)
    by_canonical = {joint.canonical: joint for joint in profile.joints}
    joints = [by_canonical[name] for name in binding.canonical_names]
    velocity = np.array([joint.velocity for joint in joints])
    mapper = RelativeTargetMapper(
        axis_mapping=teleop["axis_mapping"],
        position_scale=float(teleop["position_scale"]),
        orientation_mode=orientation_mode or teleop["orientation_mode"],
        heading_mode=teleop["heading"]["mode"],
        yaw_offset_rad=math.radians(float(teleop["heading"]["yaw_offset_deg"])),
        max_offset_m=teleop.get("max_target_offset_m"),
    )
    settings = TeleopSettings(
        enable_source=teleop["enable"]["source"],
        press_threshold=float(teleop["enable"]["press_threshold"]),
        release_threshold=float(teleop["enable"]["release_threshold"]),
        input_timeout_sec=float(teleop["input_timeout_sec"]),
        joint_state_timeout_sec=float(teleop["joint_state_timeout_sec"]),
        max_input_jump_m=float(teleop["max_input_jump_m"]),
        max_input_jump_rad=float(teleop["max_input_jump_rad"]),
        min_enable_singular_value=float(teleop.get("min_enable_singular_value", 0.0)),
        ik=IkSettings(**teleop.get("ik", {})),
    )
    lower = np.array([joint.lower for joint in joints])
    upper = np.array([joint.upper for joint in joints])
    return ArmTeleop(
        chain,
        mapper,
        lower=lower,
        upper=upper,
        velocity=velocity,
        command_period_sec=1.0 / profile.endpoint().command_rate_hz,
        max_lead=velocity * LEAD_SEC,
        names=list(binding.canonical_names),
        settings=settings,
        solve=_solver(urdf, teleop, binding, lower, upper, velocity),
    )


def _solver(urdf, teleop: dict, binding: ArmBinding, lower, upper, velocity):
    """The IK the core follows with: None (its default DLS) or pink's solve."""
    backend = teleop.get("ik_backend", "dls")
    if backend not in IK_BACKENDS:
        raise ConfigError(f"ik_backend must be one of {IK_BACKENDS}, got {backend!r}")
    if backend == "dls":
        return None
    from .pink_ik import PinkIk, PinkSettings

    try:
        solver = PinkIk(urdf, binding.runtime_names, binding.control_frame,
                        sign=binding.sign, lower=lower, upper=upper, velocity=velocity,
                        settings=PinkSettings(**teleop.get("pink", {})))
    except ImportError as error:
        raise ConfigError(
            f"ik_backend: pink needs pink and pinocchio ({error}); run with the "
            "robot_control/.venv interpreter (docs/quest-teleop-leap-pink.md)") from error
    except ValueError as error:
        raise ConfigError(str(error)) from error
    return solver.solve


def load_profile_for(config: dict) -> RobotProfile:
    return load_builtin_profile(config["teleop"]["profile"])
