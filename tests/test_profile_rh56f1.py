"""26-DOF OpenArm + RH56F1 profile: the canonical <-> source contract.

Radians only (profile.py rejects any other unit); the vendor raw <-> radian
conversion belongs to the rh56f1_hardware backend and is a stage-5 blocker.
"""
from pathlib import Path

import numpy as np
import pytest
import yaml

from robot_control.interface import CanonicalInterface
from robot_control.profile import ProfileError, load_builtin_profile


ROOT = Path(__file__).parents[1]
PROFILE_PATH = ROOT / "src/robot_control/profiles/openarm_rh56f1.yaml"
CANONICAL_URDF = ROOT.parent / "urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"
CANONICAL_MANIFEST = ROOT.parent / "urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml"
CONTROLLERS = (
    ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/config/controllers/"
    "openarm_rh56f1_real_controllers.yaml"
)

PASSIVE_MIMIC_JOINTS = [
    f"{side}_hj_{finger}"
    for side in ("r", "l")
    for finger in ("thumb_3", "thumb_4", "index_2", "middle_2", "ring_2", "pinky_2")
]


@pytest.fixture(scope="module")
def profile():
    return load_builtin_profile("openarm_rh56f1")


def test_profile_has_exactly_26_unique_joints_in_manifest_order(profile):
    manifest = yaml.safe_load(CANONICAL_MANIFEST.read_text())
    expected = manifest["control_joint_order"]

    assert len(profile.joints) == 26
    assert len(set(profile.joint_names)) == 26
    assert list(profile.joint_names) == expected


def test_no_passive_mimic_joint_is_a_command_resource(profile):
    assert set(PASSIVE_MIMIC_JOINTS).isdisjoint(profile.joint_names)
    for group in profile.groups.values():
        assert set(PASSIVE_MIMIC_JOINTS).isdisjoint(group.joints)


def test_four_groups_split_by_side_and_arm_hand(profile):
    assert set(profile.groups) == {
        "rh56f1_right_arm",
        "rh56f1_left_arm",
        "rh56f1_right_hand",
        "rh56f1_left_hand",
    }
    assert [len(profile.groups[name].joints) for name in
            ("rh56f1_right_arm", "rh56f1_right_hand", "rh56f1_left_arm", "rh56f1_left_hand")] == [7, 6, 7, 6]
    # Every joint belongs to exactly one group (profile.py enforces this at load time);
    # this only re-asserts the union covers the full 26.
    covered = set().union(*(set(g.joints) for g in profile.groups.values()))
    assert covered == set(profile.joint_names)


def test_groups_are_all_independently_executable_with_distinct_controllers(profile):
    executable = profile.executable_groups()
    assert set(executable) == set(profile.groups)
    controllers = [g.controller for g in executable.values()]
    assert len(controllers) == len(set(controllers)) == 4


def test_hand_groups_share_the_one_joint_states_topic(profile):
    """Unlike the Tesollo/DG-5F-M profile, RH56F1 hands run on the same
    controller_manager as the arms, so no group overrides state_topic."""
    for group in profile.groups.values():
        assert group.state_topic is None


def test_canonical_source_round_trip_over_the_full_26_vector(profile):
    interface = CanonicalInterface(profile)
    canonical = np.linspace(-0.01, 0.01, 26)

    source = interface.command_to_source(canonical)
    restored = interface.state_to_canonical(source)

    assert list(source) == [joint.source for joint in profile.joints]
    np.testing.assert_allclose(restored, canonical)


@pytest.mark.parametrize("group_name,count", [
    ("rh56f1_right_arm", 7), ("rh56f1_left_arm", 7),
    ("rh56f1_right_hand", 6), ("rh56f1_left_hand", 6),
])
def test_canonical_source_round_trip_per_group(profile, group_name, count):
    interface = CanonicalInterface(profile)
    canonical = np.linspace(-0.02, 0.02, count)

    source = interface.group_command_to_source(group_name, canonical)
    restored = interface.group_state_to_canonical(group_name, source)

    np.testing.assert_allclose(restored, canonical)


def test_hand_actuator_limits_match_the_canonical_urdf_verbatim():
    import xml.etree.ElementTree as ET

    root = ET.parse(CANONICAL_URDF).getroot()
    urdf_joints = {j.get("name"): j for j in root.findall("joint")}
    profile = load_builtin_profile("openarm_rh56f1")

    hand_joints = [j for j in profile.joints if "_hj_" in j.canonical]
    assert len(hand_joints) == 12
    for joint in hand_joints:
        limit = urdf_joints[joint.canonical].find("limit")
        assert joint.lower == pytest.approx(float(limit.get("lower")))
        assert joint.upper == pytest.approx(float(limit.get("upper")))
        assert joint.velocity == pytest.approx(float(limit.get("velocity")))


def test_arm_position_limits_match_the_canonical_urdf_and_velocity_is_capped():
    import xml.etree.ElementTree as ET

    urdf_joints = {j.get("name"): j for j in ET.parse(CANONICAL_URDF).getroot().findall("joint")}
    tesollo = {j.canonical: j for j in load_builtin_profile("openarm_tesollo").joints}
    profile = load_builtin_profile("openarm_rh56f1")
    arm = [j for j in profile.joints if "_aj_" in j.canonical]
    assert len(arm) == 14
    for joint in arm:
        limit = urdf_joints[joint.canonical].find("limit")
        assert joint.lower == pytest.approx(float(limit.get("lower")), abs=1e-5)
        assert joint.upper == pytest.approx(float(limit.get("upper")), abs=1e-5)
        assert joint.velocity <= float(limit.get("velocity"))
        # The same conservative cap as the OpenArm-Tesollo profile.
        assert joint.velocity == tesollo[joint.canonical].velocity == 2.0


def test_source_names_are_the_manifest_source_to_canonical_entries(profile):
    mapping = yaml.safe_load(CANONICAL_MANIFEST.read_text())["source_to_canonical_joints"]
    canonical_to_source = {c: s for s, c in mapping.items()}
    for joint in profile.joints:
        assert joint.source == canonical_to_source[joint.canonical]
        assert joint.sign == 1 and joint.unit == "rad"


def test_per_command_step_matches_the_tesollo_profile(profile):
    """CommandGate steps at most velocity / command_rate_hz per command."""
    raw = yaml.safe_load(PROFILE_PATH.read_text())
    tesollo = yaml.safe_load((PROFILE_PATH.parent / "openarm_tesollo.yaml").read_text())
    for distro, endpoint in raw["ros"].items():
        rate = endpoint["command_rate_hz"]
        assert rate == tesollo["ros"][distro]["command_rate_hz"] == 100
        for joint in profile.joints:
            assert joint.velocity / rate <= 0.02 + 1e-12


def test_profile_names_no_controller_or_topic_the_bringup_does_not_serve():
    raw = yaml.safe_load(PROFILE_PATH.read_text())
    for endpoint in raw["ros"].values():
        assert set(endpoint) == {"command_rate_hz"}
    controllers = yaml.safe_load(CONTROLLERS.read_text())["controller_manager"]["ros__parameters"]
    for group in raw["groups"].values():
        assert "effort_controller" not in group
        assert group["controller"] in controllers


def test_canonical_26_vector_splits_into_the_four_controller_commands(profile):
    """The interface a later Fabric/IK stage hands its 26-D target to."""
    interface = CanonicalInterface(profile)
    controllers = yaml.safe_load(CONTROLLERS.read_text())
    canonical = np.arange(26, dtype=float) / 100.0
    order = list(profile.joint_names)
    reassembled = np.full(26, np.nan)
    for name, group in profile.groups.items():
        command = interface.group_command_to_source(
            name, [canonical[order.index(j)] for j in group.joints])
        # Source names and order are exactly what that controller claims.
        assert list(command) == controllers[group.controller]["ros__parameters"]["joints"]
        state = interface.group_state_to_canonical(name, command)
        for joint, value in zip(group.joints, state):
            reassembled[order.index(joint)] = value
    np.testing.assert_array_equal(reassembled, canonical)
    with pytest.raises(Exception):
        interface.group_command_to_source("rh56f1_right_hand", [0.0] * 5)
    with pytest.raises(Exception):
        interface.command_to_source([0.0] * 25)
    with pytest.raises(Exception):
        interface.command_to_source([float("nan")] + [0.0] * 25)


def test_profile_manifest_hash_is_pinned_to_the_checked_in_canonical_manifest():
    import hashlib

    digest = hashlib.sha256(CANONICAL_MANIFEST.read_bytes()).hexdigest()
    raw = yaml.safe_load(PROFILE_PATH.read_text())
    assert raw["asset"]["manifest_sha256"] == digest


def test_loading_fails_loudly_if_the_canonical_manifest_ever_drifts(tmp_path):
    """Corrupt the pinned hash in a copy (manifest paths made absolute) and
    confirm the loader refuses."""
    text = PROFILE_PATH.read_text().replace(
        "fc72ad75a1231b203f9d101a572ecbdbe5cd6281f51f44575368d2ba82c15780", "0" * 64
    ).replace("../../../../urdf/", str(ROOT.parent / "urdf") + "/")
    bad = tmp_path / "corrupt_openarm_rh56f1.yaml"
    bad.write_text(text)

    from robot_control.profile import load_profile

    with pytest.raises(ProfileError, match="manifest hash mismatch"):
        load_profile(bad)
