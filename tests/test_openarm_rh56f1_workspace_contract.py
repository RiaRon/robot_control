"""Cross-repository OpenArm/RH56F1 naming and fake-state contract.

This is deliberately static and in-process: it never starts controller_manager,
loads the real OpenArm plugin, opens CAN, or publishes a ROS command.  It ties
the RH56F1 canonical asset checked out next to this repository to the existing
OpenArm controller and GenericSystem declarations.
"""

from pathlib import Path
import re

import numpy as np
import pytest
import yaml

from robot_control.interface import CanonicalInterface
from robot_control.profile import load_builtin_profile
from robot_control.srdf import repository_root


KUKU_LAB_ROOT = Path(__file__).resolve().parents[2]
CANONICAL_MANIFEST = (
    KUKU_LAB_ROOT
    / "urdf/generated/fabric/openarm_rh56f1_bi/openarm_rh56f1_bi_manifest.yaml"
)
CONTROLLERS = (
    repository_root()
    / "ros_ws/src/openarm_ros2/openarm_bringup/config/controllers/"
    "openarm_bimanual_controllers.yaml"
)
ROS2_CONTROL_XACRO = (
    repository_root()
    / "ros_ws/src/openarm_description/urdf/ros2_control/"
    "openarm.bimanual.ros2_control.xacro"
)
RH56F1_COMPONENT = repository_root() / "components/rh56f1.yaml"
OPENARM_HARDWARE_SOURCE = (
    repository_root()
    / "ros_ws/src/openarm_ros2/openarm_hardware/src/openarm_simple_hardware.cpp"
)


def _workspace_file(path: Path) -> Path:
    if not path.is_file():
        pytest.skip(f"KUKU workspace integration input is absent: {path}")
    return path


@pytest.fixture(scope="module")
def manifest():
    return yaml.safe_load(_workspace_file(CANONICAL_MANIFEST).read_text())


@pytest.fixture(scope="module")
def controllers():
    return yaml.safe_load(_workspace_file(CONTROLLERS).read_text())


@pytest.mark.parametrize(
    ("side", "canonical_prefix", "source_side"),
    (("left", "l", "left"), ("right", "r", "right")),
)
def test_canonical_arm_order_matches_every_controller_and_fake_hardware(
    manifest, controllers, side, canonical_prefix, source_side
):
    canonical = [
        name
        for name in manifest["cspace_joint_order"]
        if name.startswith(f"{canonical_prefix}_aj_")
    ]
    expected_canonical = [
        f"{canonical_prefix}_aj_{index}" for index in range(1, 8)
    ]
    source = [f"openarm_{source_side}_joint{index}" for index in range(1, 8)]

    assert canonical == expected_canonical
    for controller in (
        f"{side}_forward_position_controller",
        f"{side}_forward_velocity_controller",
        f"{side}_forward_effort_controller",
        f"{side}_joint_trajectory_controller",
    ):
        assert controllers[controller]["ros__parameters"]["joints"] == source

    xacro_text = _workspace_file(ROS2_CONTROL_XACRO).read_text()
    declared_indices = re.findall(
        rf'joint_name="openarm_\$\{{{side}_arm_prefix\}}joint([1-7])"',
        xacro_text,
    )
    assert declared_indices == [str(index) for index in range(1, 8)]


def test_fake_plugin_is_separate_from_the_real_can_plugin():
    text = _workspace_file(ROS2_CONTROL_XACRO).read_text()

    assert text.count("mock_components/GenericSystem") == 2
    assert text.count("openarm_hardware/OpenArmHW") == 2
    assert '<xacro:if value="${use_fake_hardware}">' in text
    assert '<xacro:unless value="${use_fake_hardware}">' in text


def test_real_hardware_exports_the_same_arm_names_and_interface_dimensions():
    text = _workspace_file(OPENARM_HARDWARE_SOURCE).read_text()

    # The C++ backend generates the same 1..7 sequence selected by each
    # left_/right_ ros2_control block.  This is intentionally a source audit:
    # constructing OpenArmHW would open SocketCAN during on_init().
    assert 'for (size_t i = 1; i <= ARM_DOF; ++i)' in text
    assert '"openarm_" + arm_prefix_ + "joint" + std::to_string(i)' in text
    for interface in ("HW_IF_POSITION", "HW_IF_VELOCITY", "HW_IF_EFFORT"):
        assert text.count(f"hardware_interface::{interface}") >= 2
    assert '"temperature_rotor"' in text
    assert '"temperature_mos"' in text


@pytest.mark.parametrize(
    ("group", "canonical_prefix"),
    (("openarm_left_arm", "l"), ("openarm_right_arm", "r")),
)
def test_fake_command_state_round_trip_preserves_canonical_order(
    manifest, group, canonical_prefix
):
    profile = load_builtin_profile("openarm_tesollo")
    interface = CanonicalInterface(profile)
    canonical_names = [
        name
        for name in manifest["cspace_joint_order"]
        if name.startswith(f"{canonical_prefix}_aj_")
    ]
    assert list(profile.groups[group].joints) == canonical_names

    sentinel = np.arange(7, dtype=float)
    fake_state = interface.group_command_to_source(group, sentinel)
    assert list(fake_state) == list(interface.group_source_names(group))
    np.testing.assert_array_equal(
        interface.group_state_to_canonical(group, fake_state), sentinel
    )


def test_rh56f1_actuators_are_not_claimed_by_current_controller_stack(manifest):
    order = manifest["cspace_joint_order"]
    controller_text = _workspace_file(CONTROLLERS).read_text()
    xacro_text = _workspace_file(ROS2_CONTROL_XACRO).read_text()

    for prefix in ("l_hj_", "r_hj_"):
        actuators = [name for name in order if name.startswith(prefix)]
        assert len(actuators) == 6
        assert not any(name in controller_text for name in actuators)
        assert not any(name in xacro_text for name in actuators)

    component = yaml.safe_load(_workspace_file(RH56F1_COMPONENT).read_text())
    assert component["status"] == "static_contract_only"
