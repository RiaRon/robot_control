"""Quest teleop on OpenArm + LEAP with the pink IK backend (no ROS).

pink and pinocchio live in robot_control/.venv (docs/quest-teleop-leap-pink.md);
outside it the pink tests are skipped. The description is the LEAP split
bringup's fake arm description, built from the canonical urdf asset.
"""

from pathlib import Path
import math
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/launch"
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(LAUNCH_DIR))

import leap_split_description as leap  # noqa: E402
from openarm_quest_teleop.config import (  # noqa: E402
    DEFAULT_CONFIG, ConfigError, bind_arm, build_teleop, load_config, load_profile_for)
from openarm_quest_teleop.ik import IkSettings  # noqa: E402
from openarm_quest_teleop.teleop import ENGAGED, ControllerSample, JointSample  # noqa: E402

LEAP_CONFIG = DEFAULT_CONFIG.with_name("quest_teleop_leap.yaml")
CANONICAL = leap.default_canonical_urdf(str(LAUNCH_DIR / "openarm_leap_arms.launch.py"))
pytestmark = pytest.mark.skipif(not CANONICAL, reason="openarm_leap_bi_rl.urdf not found")
BENT = np.array([0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0])  # the fake start pose of docs/quest-teleop.md


@pytest.fixture(scope="module")
def urdf():
    return leap.fake_arm_description(CANONICAL, leap.default_manifest_for(Path(CANONICAL)))


@pytest.fixture(scope="module")
def setup(urdf):
    config = load_config(LEAP_CONFIG)
    profile = load_profile_for(config)
    return config, profile, bind_arm(urdf, profile, config, "right")


def _pink(urdf, setup):
    pytest.importorskip("pink")
    from openarm_quest_teleop.pink_ik import PinkIk

    _, profile, binding = setup
    joints = {j.canonical: j for j in profile.joints}
    arm = [joints[n] for n in binding.canonical_names]
    lower, upper = np.array([j.lower for j in arm]), np.array([j.upper for j in arm])
    solver = PinkIk(urdf, binding.runtime_names, binding.control_frame, sign=binding.sign,
                    lower=lower, upper=upper, velocity=np.array([j.velocity for j in arm]))
    return solver, lower, upper


def test_leap_config_binds_the_arm_to_the_leap_palm(setup):
    config, profile, binding = setup
    assert profile.name == "openarm_leap" and config["teleop"]["ik_backend"] == "pink"
    assert binding.control_frame == "r_hl_palm" and binding.base_frame == "body_root"
    assert binding.naming == "canonical" and binding.is_fake
    assert binding.runtime_names == tuple(f"r_aj_{i}" for i in range(1, 8))
    assert config["teleop"]["robot_description_topic"] == leap.ARM_DESCRIPTION_TOPIC


def test_pink_forward_kinematics_matches_the_teleop_chain(urdf, setup):
    solver, _, _ = _pink(urdf, setup)
    from openarm_quest_teleop.config import build_chain

    chain = build_chain(urdf, setup[2])
    for q in (np.zeros(7), BENT, BENT + 0.2):
        assert np.allclose(solver.pose(q), chain.pose(q), atol=1e-9)


def test_pink_tracks_a_moving_palm_target_inside_the_limits(urdf, setup):
    solver, lower, upper = _pink(urdf, setup)
    start, q = solver.pose(BENT), BENT.copy()
    for k in range(200):  # 10 cm circle at 100 Hz, orientation held
        angle = 2 * math.pi * k / 200
        target = start.copy()
        target[:3, 3] += [0.0, 0.1 * math.sin(angle), 0.1 * (1 - math.cos(angle))]
        result = solver.solve(target, q, lower, upper, IkSettings())
        assert result.ok, result.reason
        assert result.position_error_m < 1e-3 and result.rotation_error_rad < 1e-2
        assert np.all(result.q >= lower - 1e-9) and np.all(result.q <= upper + 1e-9)
        q = result.q


def test_pink_refuses_an_unreachable_target_and_a_jump(urdf, setup):
    solver, lower, upper = _pink(urdf, setup)
    far = solver.pose(BENT)
    far[0, 3] += 1.5
    result = solver.solve(far, BENT, lower, upper, IkSettings())
    assert not result.ok and "not reachable" in result.reason
    moved = solver.pose(BENT + np.array([0.6, 0, 0, 0, 0, 0, 0]))
    result = solver.solve(moved, BENT, lower, upper, IkSettings(max_seed_distance_rad=0.2))
    assert not result.ok and "refusing a jump" in result.reason


def test_teleop_core_follows_the_controller_with_pink(urdf, setup):
    pytest.importorskip("pink")
    config, profile, binding = setup
    core = build_teleop(urdf, profile, config, binding)
    hand = np.eye(4)
    hand[:3, 3] = [0.3, -0.2, 1.0]
    joints = JointSample(0.0, BENT)
    start = core.chain.pose(BENT)

    def sample(t, grip, offset=(0.0, 0.0, 0.0)):
        pose = hand.copy()
        pose[:3, 3] += offset
        return ControllerSample(arrival_sec=t, pose=pose, grip=grip)

    assert core.step(0.0, sample(0.0, 0.0), joints).state == "idle"  # released: armed
    first = core.step(0.01, sample(0.01, 1.0), JointSample(0.01, BENT))
    assert first.state == ENGAGED and first.just_engaged
    assert np.allclose(first.command, BENT, atol=1e-6)  # starts where the arm is
    q, t = BENT.copy(), 0.01
    for k in range(1, 101):  # controller 5 cm forward over 1 s; the fake arm follows
        t += 0.01
        result = core.step(t, sample(t, 1.0, (0.05 * k / 100, 0.0, 0.0)), JointSample(t, q))
        assert result.state == ENGAGED and result.command is not None, result.reason
        q = result.command
    for _ in range(50):  # hold: the gate lets the command catch up with the target
        t += 0.01
        q = core.step(t, sample(t, 1.0, (0.05, 0.0, 0.0)), JointSample(t, q)).command
    moved = core.chain.pose(q)[:3, 3] - start[:3, 3]
    assert np.allclose(moved, [0.05, 0.0, 0.0], atol=1e-3), moved
    assert np.allclose(core.chain.pose(q)[:3, :3], start[:3, :3], atol=1e-2)  # hold mode


def test_unknown_ik_backend_is_refused(urdf, setup):
    config, profile, binding = setup
    bad = {**config, "teleop": {**config["teleop"], "ik_backend": "fabric"}}
    with pytest.raises(ConfigError, match="ik_backend must be one of"):
        build_teleop(urdf, profile, bad, binding)


def test_the_rh56f1_config_keeps_the_dls_default():
    assert load_config(DEFAULT_CONFIG)["teleop"].get("ik_backend", "dls") == "dls"
