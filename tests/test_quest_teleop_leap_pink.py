"""Quest teleop on OpenArm + LEAP with the pink IK (no ROS).

pink and pinocchio live in robot_control/.venv (docs/quest-teleop-leap-pink.md);
outside it the pink tests are skipped. The description is the LEAP split
bringup's fake arm description, built from the canonical urdf asset.
"""

from pathlib import Path
import math
import sys

import numpy as np
import pytest
import yaml

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
    assert profile.name == "openarm_leap" and "pink" in config["teleop"]
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


def test_without_pink_the_teleop_refuses_to_build(urdf, setup, monkeypatch):
    config, profile, binding = setup
    monkeypatch.setitem(sys.modules, "pinocchio", None)  # import pinocchio -> ImportError
    with pytest.raises(ConfigError, match="needs pink and pinocchio"):
        build_teleop(urdf, profile, config, binding)


def test_both_teleop_configs_carry_pink_settings():
    for path in (DEFAULT_CONFIG, LEAP_CONFIG):
        teleop = load_config(path)["teleop"]
        assert "ik_backend" not in teleop and teleop["pink"]["solver"] == "daqp"


# ------------------------------------------------------------ start pose
SIM2REAL_AGLT = {  # sim2real deploy/policy_control/config/homes/rh56f1_aglt.yaml (2f8a803)
    "right": [-1.2127, 0.2026, 0.6538, 1.7608, 0.3791, 0.5785, 0.6646],
    "left": [1.2127, -0.2026, -0.6538, 1.7608, -0.3791, -0.5785, -0.6646],
}


@pytest.mark.parametrize("arm", ["right", "left"])
def test_start_pose_is_the_rh56f1_robot_home(urdf, setup, arm):
    from openarm_quest_teleop.config import start_pose_for, start_pose_settings

    config, profile, _ = setup
    assert start_pose_settings(config)["name"] == "rh56f1_aglt_home"
    binding = bind_arm(urdf, profile, config, arm)
    q = start_pose_for(profile, binding, "rh56f1_aglt_home")
    assert np.allclose(q, SIM2REAL_AGLT[arm])
    mirror = np.array([-1, -1, -1, 1, -1, -1, -1])
    assert np.allclose(np.array(SIM2REAL_AGLT["left"]), mirror * SIM2REAL_AGLT["right"])


def test_start_pose_refuses_unknown_names_and_poses_outside_the_limits(urdf, setup, tmp_path):
    from openarm_quest_teleop.config import start_pose_for

    _, profile, binding = setup
    with pytest.raises(ConfigError, match="no pose 'nowhere'"):
        start_pose_for(profile, binding, "nowhere")
    store = tmp_path / "poses.yaml"
    joints = {f"r_aj_{i}": 0.0 for i in range(1, 8)} | {"r_aj_4": 3.0}  # elbow past 2.44
    store.write_text(yaml.safe_dump({"schema": 1, "profile": "openarm_leap", "poses": {
        "bad": {"groups": ["openarm_right_arm"], "saved_at": "test", "gravity": None,
                "joints": joints}}}))
    with pytest.raises(ConfigError, match=r"outside the limits of \['r_aj_4'\]"):
        start_pose_for(profile, binding, "bad", store)


def test_start_move_is_slow_and_never_shorter_than_the_minimum(setup):
    from openarm_quest_teleop.config import start_motion_duration, start_pose_settings

    settings = start_pose_settings(setup[0])
    home = np.array(SIM2REAL_AGLT["right"])
    assert start_motion_duration(np.zeros(7), home, settings) == pytest.approx(1.7608 / 0.3)
    assert start_motion_duration(home, home, settings) == 3.0


def test_the_rh56f1_config_names_no_start_pose():
    from openarm_quest_teleop.config import start_pose_settings

    assert start_pose_settings(load_config(DEFAULT_CONFIG))["name"] is None
