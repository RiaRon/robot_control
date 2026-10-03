"""Static/Xacro contract for the fake-only OpenArm + RH56F1 bringup."""

from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
KUKU_LAB = ROOT.parent
LAUNCH_DIR = (
    ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/launch"
)
sys.path.insert(0, str(LAUNCH_DIR))

from rh56f1_description import (  # noqa: E402
    FAKE_HAND_CONTROLLERS,
    HAND_CONFIGURATIONS,
    build_canonical_variant,
    default_manifest_for,
    fake_hand_controller_params,
    render_control_description,
    ros2_control_joint_names,
)


CANONICAL = KUKU_LAB / "urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"
MANIFEST = KUKU_LAB / "urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml"
PROFILE = ROOT / "src/robot_control/profiles/openarm_rh56f1.yaml"
WRAPPER = (
    ROOT
    / "ros_ws/src/openarm_description/urdf/robot/"
    "openarm_rh56f1_bimanual.urdf.xacro"
)
CONTROL_XACRO = (
    ROOT
    / "ros_ws/src/openarm_description/urdf/ros2_control/"
    "openarm_rh56f1.bimanual.ros2_control.xacro"
)
CONTROLLERS = (
    ROOT
    / "ros_ws/src/openarm_ros2/openarm_bringup/config/controllers/"
    "openarm_rh56f1_fake_controllers.yaml"
)

ARM_JOINTS = [
    *(f"r_aj_{index}" for index in range(1, 8)),
    *(f"l_aj_{index}" for index in range(1, 8)),
]
HAND_ACTUATORS = {
    side: [
        f"{prefix}_hj_thumb_1",
        f"{prefix}_hj_thumb_2",
        f"{prefix}_hj_index_1",
        f"{prefix}_hj_middle_1",
        f"{prefix}_hj_ring_1",
        f"{prefix}_hj_pinky_1",
    ]
    for side, prefix in (("right", "r"), ("left", "l"))
}
HAND_MIMICS = {
    side: [
        f"{prefix}_hj_thumb_3",
        f"{prefix}_hj_thumb_4",
        f"{prefix}_hj_index_2",
        f"{prefix}_hj_middle_2",
        f"{prefix}_hj_ring_2",
        f"{prefix}_hj_pinky_2",
    ]
    for side, prefix in (("right", "r"), ("left", "l"))
}


def _render_text(configuration: str, policy: str = "parked") -> str:
    pytest.importorskip("xacro")
    return render_control_description(
        source_urdf=CANONICAL,
        wrapper_xacro=WRAPPER,
        hand_configuration=configuration,
        state_policy=policy,
        use_fake_hardware=True,
    )


def _render(configuration: str, policy: str = "parked") -> ET.Element:
    return ET.fromstring(_render_text(configuration, policy))


def _assert_tree(root: ET.Element) -> None:
    links = [element.get("name") for element in root.findall("link")]
    joints = [element.get("name") for element in root.findall("joint")]
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))
    children = {}
    adjacency = {link: [] for link in links}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        assert parent in adjacency
        assert child in adjacency
        assert child not in children
        children[child] = parent
        adjacency[parent].append(child)
    roots = set(links) - set(children)
    assert roots == {"body_root"}
    visited = set()

    def visit(link, active):
        assert link not in active
        active.add(link)
        visited.add(link)
        for child in adjacency[link]:
            visit(child, active)
        active.remove(link)

    visit("body_root", set())
    assert visited == set(links)


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_all_hand_variants_generate_valid_control_xacro(configuration):
    root = _render(configuration)
    _assert_tree(root)
    links = {link.get("name") for link in root.findall("link")}
    assert ("l_hl_palm_sensor" in links) is (
        configuration in ("left", "both")
    )
    assert ("r_hl_palm_sensor" in links) is (
        configuration in ("right", "both")
    )


@pytest.mark.parametrize(
    ("configuration", "enabled_sides", "expected_count"),
    (
        ("arm_only", set(), 14),
        ("left", {"left"}, 20),
        ("right", {"right"}, 20),
        ("both", {"left", "right"}, 26),
    ),
)
def test_parked_control_resources_are_exactly_independent_dofs(
    configuration, enabled_sides, expected_count
):
    root = _render(configuration)
    resources = root.findall("ros2_control/joint")
    names = [joint.get("name") for joint in resources]
    expected = (
        ARM_JOINTS[:7]
        + (HAND_ACTUATORS["right"] if "right" in enabled_sides else [])
        + ARM_JOINTS[7:]
        + (HAND_ACTUATORS["left"] if "left" in enabled_sides else [])
    )
    assert len(names) == expected_count
    assert names == expected
    assert len(names) == len(set(names))
    assert not any(
        mimic in names for side in HAND_MIMICS.values() for mimic in side
    )


def test_inactive_keeps_geometry_but_exports_no_hand_resources():
    root = _render("both", "inactive")
    names = [joint.get("name") for joint in root.findall("ros2_control/joint")]
    links = {link.get("name") for link in root.findall("link")}
    assert names == ARM_JOINTS
    assert {"l_hl_palm_sensor", "r_hl_palm_sensor"} <= links


def test_fake_only_zero_park_is_within_canonical_actuator_limits():
    root = _render("both")
    kinematic = {joint.get("name"): joint for joint in root.findall("joint")}
    for name in HAND_ACTUATORS["right"] + HAND_ACTUATORS["left"]:
        limit = kinematic[name].find("limit")
        assert float(limit.get("lower")) <= 0.0 <= float(limit.get("upper"))
    for resource in root.findall("ros2_control/joint"):
        if "_hj_" not in resource.get("name"):
            continue
        initial = resource.find("state_interface/param[@name='initial_value']")
        assert float(initial.text) == 0.0

def _joint_contract(joint):
    return (
        joint.get("type"),
        tuple(
            (child.tag, tuple(sorted(child.attrib.items())), (child.text or "").strip())
            for child in joint
        ),
    )



def test_mounts_and_mimics_come_from_canonical_unchanged():
    source = ET.parse(CANONICAL).getroot()
    rendered = _render("both")
    source_joints = {joint.get("name"): joint for joint in source.findall("joint")}
    rendered_joints = {
        joint.get("name"): joint for joint in rendered.findall("joint")
    }
    for name in (
        "l_hj_mount",
        "r_hj_mount",
        *HAND_MIMICS["left"],
        *HAND_MIMICS["right"],
    ):
        assert _joint_contract(rendered_joints[name]) == _joint_contract(source_joints[name])


def test_only_generic_system_is_available_and_no_hand_controller_exists():
    root = _render("both")
    plugins = [element.text for element in root.findall("ros2_control/hardware/plugin")]
    assert plugins == ["mock_components/GenericSystem"]
    assert "OpenArmHW" not in CONTROL_XACRO.read_text()

    controllers = yaml.safe_load(CONTROLLERS.read_text())
    declared = controllers["controller_manager"]["ros__parameters"]
    assert set(declared) == {
        "update_rate",
        "joint_state_broadcaster",
        "left_joint_trajectory_controller",
        "right_joint_trajectory_controller",
    }
    text = CONTROLLERS.read_text()
    assert "_hj_" not in text
    assert "gripper" not in text


def test_disabled_hand_removal_does_not_mutate_canonical_input():
    before = CANONICAL.read_bytes()
    for configuration in HAND_CONFIGURATIONS:
        build_canonical_variant(CANONICAL, configuration)
    assert CANONICAL.read_bytes() == before


# ------------------------------------------------ fake_commandable (fake-only)
_ENABLED = {"arm_only": (), "left": ("left",), "right": ("right",),
            "both": ("right", "left")}


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_fake_commandable_exports_exactly_the_parked_resources(configuration):
    parked = _render_text(configuration, "parked")
    commandable = _render_text(configuration, "fake_commandable")
    assert ros2_control_joint_names(commandable) == ros2_control_joint_names(parked)
    assert ET.tostring(ET.fromstring(commandable).find("ros2_control")) == ET.tostring(
        ET.fromstring(parked).find("ros2_control")
    )
    plugins = [e.text for e in ET.fromstring(commandable).findall("ros2_control/hardware/plugin")]
    assert plugins == ["mock_components/GenericSystem"]


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_fake_hand_controllers_take_manifest_order_and_only_actuators(configuration):
    description = _render_text(configuration, "fake_commandable")
    params = fake_hand_controller_params(MANIFEST, configuration, description)
    order = yaml.safe_load(MANIFEST.read_text())["control_joint_order"]
    assert list(params) == [FAKE_HAND_CONTROLLERS[side] for side in _ENABLED[configuration]]
    for side in _ENABLED[configuration]:
        values = params[FAKE_HAND_CONTROLLERS[side]]["ros__parameters"]
        prefix = "r_hj_" if side == "right" else "l_hj_"
        assert values["joints"] == [name for name in order if name.startswith(prefix)]
        assert values["joints"] == HAND_ACTUATORS[side]
        assert not set(values["joints"]) & set(HAND_MIMICS[side])
        assert values["type"] == "joint_trajectory_controller/JointTrajectoryController"
        assert values["command_interfaces"] == ["position"]
        assert values["state_interfaces"] == ["position"]
    assert default_manifest_for(CANONICAL) == MANIFEST


def test_fake_hand_controllers_refuse_a_description_without_hand_resources():
    inactive = _render_text("both", "inactive")
    with pytest.raises(ValueError, match="does not match"):
        fake_hand_controller_params(MANIFEST, "both", inactive)


def test_fake_controller_groups_reproduce_manifest_and_profile_canonical_order():
    """Canonical <-> fake controller mapping is identity and order preserving."""

    description = _render_text("both", "fake_commandable")
    hands = fake_hand_controller_params(MANIFEST, "both", description)
    arms = yaml.safe_load(CONTROLLERS.read_text())
    groups = {
        "right_arm": arms["right_joint_trajectory_controller"]["ros__parameters"]["joints"],
        "right_hand": hands[FAKE_HAND_CONTROLLERS["right"]]["ros__parameters"]["joints"],
        "left_arm": arms["left_joint_trajectory_controller"]["ros__parameters"]["joints"],
        "left_hand": hands[FAKE_HAND_CONTROLLERS["left"]]["ros__parameters"]["joints"],
    }
    order = yaml.safe_load(MANIFEST.read_text())["control_joint_order"]
    concatenated = [name for group in groups.values() for name in group]
    assert concatenated == order
    assert concatenated == ros2_control_joint_names(description)
    profile = yaml.safe_load(PROFILE.read_text())
    assert [entry["canonical"] for entry in profile["joints"]] == order
    for group, joints in groups.items():
        assert profile["groups"][f"rh56f1_{group}"]["joints"] == joints


def _launch_actions(policy: str, configuration: str = "both"):
    pytest.importorskip("launch")
    ament = pytest.importorskip("ament_index_python.packages")
    try:
        ament.get_package_share_directory("openarm_description")
    except Exception:
        pytest.skip("openarm_description is not installed in this environment")
    import importlib.util
    from launch import LaunchContext
    from launch.actions import TimerAction
    from launch.substitutions import TextSubstitution

    spec = importlib.util.spec_from_file_location(
        "rh56f1_fake_launch", LAUNCH_DIR / "openarm.rh56f1_bimanual.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    text = TextSubstitution
    actions = module._robot_nodes(
        LaunchContext(),
        text(text=str(CANONICAL)),
        text(text=configuration),
        text(text=policy),
        text(text="true"),
        text(text=str(CONTROLLERS)),
        text(text=""),
        text(text=""),
    )
    nodes = []
    for action in actions:
        nodes += action.actions if isinstance(action, TimerAction) else [action]
    return nodes


def _spawned(nodes):
    spawned = []
    for node in nodes:
        if getattr(node, "node_executable", None) != "spawner":
            continue
        arguments = [
            a if isinstance(a, str) else "".join(part.text for part in a)
            for a in node._Node__arguments
        ]
        spawned.append(arguments)
    return spawned


@pytest.mark.parametrize("policy", ("parked", "inactive"))
def test_existing_policies_spawn_no_hand_controller(policy):
    assert _spawned(_launch_actions(policy)) == []


def test_fake_commandable_spawns_one_controller_per_selected_hand():
    spawned = _spawned(_launch_actions("fake_commandable", "both"))
    assert len(spawned) == 1
    arguments = spawned[0]
    assert arguments[:2] == [FAKE_HAND_CONTROLLERS["right"], FAKE_HAND_CONTROLLERS["left"]]
    param_file = Path(arguments[arguments.index("--param-file") + 1])
    try:
        assert yaml.safe_load(param_file.read_text())[FAKE_HAND_CONTROLLERS["left"]][
            "ros__parameters"]["joints"] == HAND_ACTUATORS["left"]
    finally:
        shutil.rmtree(param_file.parent, ignore_errors=True)
    assert _spawned(_launch_actions("fake_commandable", "arm_only")) == []
