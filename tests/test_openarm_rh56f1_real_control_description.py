"""Contract of the REAL OpenArm + RH56F1 runtime description and launch plan.

The real runtime uses the canonical manifest's source joint names (the arm
names OpenArmHW exports, the manifest's rh56f1_* hand names) in a temporary
variant of the canonical URDF; canonical names stay at the policy boundary.
The first half needs no ROS: the variant, the derived control contract, the
launch plan and forward kinematics over source-named joint states. The second
half renders the xacro and is skipped where xacro/ament are unavailable (it
runs in the Humble container).
"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
KUKU_LAB = ROOT.parent
LAUNCH_DIR = ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/launch"
sys.path.insert(0, str(LAUNCH_DIR))

from rh56f1_description import HAND_CONFIGURATIONS  # noqa: E402
import rh56f1_real_description as real  # noqa: E402


CANONICAL = KUKU_LAB / "urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"
MANIFEST = KUKU_LAB / "urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml"
DESCRIPTION_SRC = ROOT / "ros_ws/src/openarm_description"
REAL_CONTROL_XACRO = DESCRIPTION_SRC / "urdf/ros2_control/openarm_rh56f1.bimanual.real.ros2_control.xacro"
FAKE_CONTROL_XACRO = DESCRIPTION_SRC / "urdf/ros2_control/openarm_rh56f1.bimanual.ros2_control.xacro"
SAFETY = DESCRIPTION_SRC / "config/rh56f1/real_bringup_safety.yaml"
CONTROLLERS = (
    ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/config/controllers/"
    "openarm_rh56f1_real_controllers.yaml"
)
REAL_LAUNCH = LAUNCH_DIR / "openarm.rh56f1_bimanual_real.launch.py"

SOURCE_TO_CANONICAL = yaml.safe_load(MANIFEST.read_text())["source_to_canonical_joints"]
CANONICAL_TO_SOURCE = {c: s for s, c in SOURCE_TO_CANONICAL.items()}
CONTROL_ORDER = yaml.safe_load(MANIFEST.read_text())["control_joint_order"]
PASSIVE = [f"{p}_hj_{f}" for p in ("r", "l")
           for f in ("thumb_3", "thumb_4", "index_2", "middle_2", "ring_2", "pinky_2")]


def _variant(configuration="both"):
    return real.build_runtime_variant(CANONICAL, configuration, MANIFEST)


# ---------------------------------------------------------------- runtime variant (no ROS)
@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_every_mapped_joint_carries_its_manifest_source_name(configuration):
    root, _, _ = _variant(configuration)
    canonical_root = real.build_canonical_variant(CANONICAL, configuration)
    names = [j.get("name") for j in root.findall("joint")]
    assert len(names) == len(set(names))
    for original, renamed in zip(canonical_root.findall("joint"), root.findall("joint")):
        expected = CANONICAL_TO_SOURCE.get(original.get("name"), original.get("name"))
        assert renamed.get("name") == expected
    assert not {n for n in names if n in CANONICAL_TO_SOURCE}, "a canonical joint name survived"


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_only_joint_names_change(configuration):
    root, _, _ = _variant(configuration)
    canonical_root = real.build_canonical_variant(CANONICAL, configuration)
    assert [l.get("name") for l in root.findall("link")] == [
        l.get("name") for l in canonical_root.findall("link")]
    for original, renamed in zip(canonical_root.findall("joint"), root.findall("joint")):
        for tag in ("parent", "child", "origin", "axis", "limit"):
            a, b = original.find(tag), renamed.find(tag)
            assert (a is None) == (b is None)
            if a is not None:
                assert a.attrib == b.attrib, (original.get("name"), tag)


def test_mimic_references_follow_the_rename():
    root, _, _ = _variant("both")
    names = {j.get("name") for j in root.findall("joint")}
    mimics = [j for j in root.findall("joint") if j.find("mimic") is not None]
    assert len(mimics) == 12
    for joint in mimics:
        assert joint.find("mimic").get("joint") in names
    assert {SOURCE_TO_CANONICAL[j.get("name")] for j in mimics} == set(PASSIVE)


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_palm_sensor_frames_exist_for_selected_hands_only(configuration):
    root, _, _ = _variant(configuration)
    links = {l.get("name") for l in root.findall("link")}
    hands = real.enabled_hands(configuration)
    assert ("r_hl_palm_sensor" in links) == ("right" in hands)
    assert ("l_hl_palm_sensor" in links) == ("left" in hands)


def test_contract_names_are_manifest_sources_and_limits_are_the_urdfs():
    root, contract, _ = _variant("both")
    canonical_joints = {j.get("name"): j for j in ET.parse(CANONICAL).getroot().findall("joint")}
    entries = [e for side in ("right", "left") for e in contract["arms"][side]]
    entries += [contract["hands"][side][k] for side in ("right", "left") for k in real.HAND_ACTUATORS]
    assert sorted(e["canonical"] for e in entries) == sorted(CONTROL_ORDER)
    for entry in entries:
        assert entry["name"] == CANONICAL_TO_SOURCE[entry["canonical"]]
        limit = canonical_joints[entry["canonical"]].find("limit")
        assert entry["lower"] == float(limit.get("lower"))
        assert entry["upper"] == float(limit.get("upper"))
    assert [e["name"] for e in contract["arms"]["right"]] == [
        f"openarm_right_joint{i}" for i in range(1, 8)]


def test_source_canonical_round_trip_over_the_26_joints():
    sources = [CANONICAL_TO_SOURCE[c] for c in CONTROL_ORDER]
    assert len(set(sources)) == 26
    assert [SOURCE_TO_CANONICAL[s] for s in sources] == CONTROL_ORDER


def test_contract_excludes_passive_joints():
    _, contract, _ = _variant("both")
    commanded = {e["canonical"] for side in ("right", "left") for e in contract["arms"][side]}
    commanded |= {contract["hands"][s][k]["canonical"] for s in ("right", "left")
                  for k in real.HAND_ACTUATORS}
    assert commanded.isdisjoint(PASSIVE)


# ---------------------------------------------------------------- FK over source-named states
def _rot(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def _axis_angle(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def _fk(root, positions, target):
    """base->target homogeneous transform; mimic joints follow their drivers."""
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    by_name = {j.get("name"): j for j in root.findall("joint")}

    def value(joint):
        mimic = joint.find("mimic")
        if mimic is not None:
            return float(mimic.get("multiplier", 1)) * value(by_name[mimic.get("joint")]) + float(
                mimic.get("offset", 0))
        return positions.get(joint.get("name"), 0.0)

    transform = np.eye(4)
    link = target
    while link in joints:
        joint = joints[link]
        origin = joint.find("origin")
        local = np.eye(4)
        if origin is not None:
            local[:3, :3] = _rot([float(v) for v in origin.get("rpy", "0 0 0").split()])
            local[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
        motion = np.eye(4)
        if joint.get("type") in ("revolute", "continuous"):
            axis = [float(v) for v in joint.find("axis").get("xyz").split()]
            motion[:3, :3] = _axis_angle(axis, value(joint))
        transform = local @ motion @ transform
        link = joint.find("parent").get("link")
    return transform


def test_arm_joint_change_moves_both_palm_sensor_frames():
    root, _, _ = _variant("both")
    for side in ("right", "left"):
        palm = f"{side[0]}_hl_palm_sensor"
        before = _fk(root, {}, palm)
        after = _fk(root, {f"openarm_{side}_joint2": 0.25}, palm)
        assert np.linalg.norm(after[:3, 3] - before[:3, 3]) > 1e-3


def test_hand_actuator_change_moves_fingertips_through_mimic_but_not_the_palm():
    root, _, _ = _variant("both")
    for side in ("right", "left"):
        p = side[0]
        driver = CANONICAL_TO_SOURCE[f"{p}_hj_index_1"]
        mimic = CANONICAL_TO_SOURCE[f"{p}_hj_index_2"]
        palm, tip = f"{p}_hl_palm_sensor", f"{p}_hl_index_tip"
        moved = _fk(root, {driver: 0.25}, tip)
        assert np.linalg.norm(moved[:3, 3] - _fk(root, {}, tip)[:3, 3]) > 1e-3
        np.testing.assert_allclose(_fk(root, {driver: 0.25}, palm), _fk(root, {}, palm), atol=1e-12)
        # Without its mimic tag index_2 stays at zero: the tip lands elsewhere,
        # so the passive joint does follow the actuator.
        no_mimic = ET.fromstring(ET.tostring(root))
        for joint in no_mimic.findall("joint"):
            if joint.get("name") == mimic:
                joint.remove(joint.find("mimic"))
        assert np.linalg.norm(_fk(no_mimic, {driver: 0.25}, tip)[:3, 3] - moved[:3, 3]) > 1e-4


# ---------------------------------------------------------------- launch plan (no ROS)
def _plan(**overrides):
    options = dict(hand_configuration="both", enable_right_arm=True, enable_left_arm=True,
                   right_hand_transport="rs485", left_hand_transport="rs485",
                   allow_mock_hands=False)
    options.update(overrides)
    _, contract, _ = _variant(options["hand_configuration"])
    return real.device_plan(contract, **options)


def test_hand_transport_has_no_default():
    with pytest.raises(ValueError, match="no default"):
        _plan(right_hand_transport="")


def test_mock_hand_in_real_launch_needs_explicit_permission():
    with pytest.raises(ValueError, match="allow_mock_hands_in_real_launch"):
        _plan(left_hand_transport="mock")
    plan = _plan(left_hand_transport="mock", allow_mock_hands=True)
    left_hand = [CANONICAL_TO_SOURCE[f"l_hj_{k}"] for k in real.HAND_ACTUATORS]
    assert plan["mock_hand_joints"] == left_hand
    assert set(left_hand).isdisjoint(plan["real_joints"])


def test_disabled_devices_have_no_component_or_controller():
    plan = _plan(hand_configuration="arm_only", enable_left_arm=False,
                 right_hand_transport="", left_hand_transport="")
    assert plan["components"] == ["openarm_rh56f1_right_arm"]
    assert plan["controllers"] == ["rh56f1_right_arm_controller"]
    assert plan["hand_joints"] == []


def test_each_device_has_its_own_broadcasters_and_mock_state_stays_off_joint_states():
    plan = _plan(right_hand_transport="mock", allow_mock_hands=True)
    b = plan["broadcasters"]
    assert set(b) == {"joint_state_broadcaster",
                      "rh56f1_right_hand_state_broadcaster", "rh56f1_right_hand_status_broadcaster",
                      "rh56f1_left_hand_state_broadcaster", "rh56f1_left_hand_status_broadcaster"}
    assert b["joint_state_broadcaster"]["joints"] == [
        e["name"] for s in ("right", "left") for e in _variant()[1]["arms"][s]]
    assert b["rh56f1_right_hand_state_broadcaster"]["use_local_topics"] is True   # mock
    assert b["rh56f1_left_hand_state_broadcaster"]["use_local_topics"] is False   # rs485
    for side in ("right", "left"):
        assert b[f"rh56f1_{side}_hand_status_broadcaster"]["use_local_topics"] is True
    # Exactly the real devices' joints reach /joint_states, each once.
    global_joints = [j for spec in b.values() if not spec["use_local_topics"] for j in spec["joints"]]
    assert sorted(global_joints) == sorted(plan["real_joints"])
    assert len(global_joints) == len(set(global_joints)) == 20
    assert set(plan["mock_hand_joints"]).isdisjoint(global_joints)


def test_controller_params_start_components_inactive():
    static = yaml.safe_load(CONTROLLERS.read_text())
    plan = _plan()
    params = real.controller_params(static, plan)
    manager = params["controller_manager"]["ros__parameters"]
    # Arms configured-but-inactive; hands unconfigured (their configure opens
    # the transport and may fail, which must not abort the whole node).
    assert manager["hardware_components_initial_state"] == {
        "inactive": ["openarm_rh56f1_right_arm", "openarm_rh56f1_left_arm"],
        "unconfigured": ["rh56f1_right_hand", "rh56f1_left_hand"]}
    assert len(plan["components"]) == 4
    assert plan["broadcasters"]["joint_state_broadcaster"]["active_at_launch"] is True
    assert not any(spec["active_at_launch"] for name, spec in plan["broadcasters"].items()
                   if name.startswith("rh56f1_"))
    for name, spec in plan["broadcasters"].items():
        assert manager[name]["type"] == "joint_state_broadcaster/JointStateBroadcaster"
        assert params[name]["ros__parameters"]["joints"] == spec["joints"]


def test_controller_params_drop_unused_controllers_and_broadcasters():
    static = yaml.safe_load(CONTROLLERS.read_text())
    plan = _plan(hand_configuration="arm_only", right_hand_transport="", left_hand_transport="")
    params = real.controller_params(static, plan)
    manager = params["controller_manager"]["ros__parameters"]
    for gone in ("rh56f1_right_hand_controller", "rh56f1_left_hand_controller",
                 "rh56f1_right_hand_state_broadcaster", "rh56f1_left_hand_status_broadcaster"):
        assert gone not in manager and gone not in params
    assert set(n for n in manager if n.endswith("_controller")) == {
        "rh56f1_right_arm_controller", "rh56f1_left_arm_controller"}


def test_controllers_yaml_joints_are_the_contracts_source_names():
    static = yaml.safe_load(CONTROLLERS.read_text())
    _, contract, _ = _variant("both")
    for side in ("right", "left"):
        arm = static[f"rh56f1_{side}_arm_controller"]["ros__parameters"]["joints"]
        hand = static[f"rh56f1_{side}_hand_controller"]["ros__parameters"]["joints"]
        assert arm == [e["name"] for e in contract["arms"][side]]
        assert hand == [contract["hands"][side][k]["name"] for k in real.HAND_ACTUATORS]


def test_real_launch_activates_nothing_and_exposes_no_test_switch():
    text = REAL_LAUNCH.read_text()
    assert "--inactive" in text
    assert "set_hardware_component_state" in text  # documented operator step only
    assert "arm_test_double" not in text and "extra_hardware_params" not in text
    assert "--activate" not in text
    # One spawner per controller, so one failing device cannot block the rest.
    assert text.count('executable="spawner"') == 2 and "for name in plan[" in text


def test_safety_values_are_declared_once():
    safety = yaml.safe_load(SAFETY.read_text())
    assert safety["arm"]["auto_return_to_zero"] is False
    assert safety["hand"]["max_velocity_rad_s"] <= 2.0
    assert safety["hand"]["max_step_rad"] <= 0.02
    # No literal value in the control xacro: every param is a ${...} lookup
    # (safety file, contract, gains) or a macro argument.
    import re
    literals = re.findall(r"<param name=\"[^\"]+\">([^<$][^<]*)</param>", REAL_CONTROL_XACRO.read_text())
    assert literals == ["false"], literals  # hand:=false, the arm has no gripper here


def test_fake_and_real_control_xacros_never_reference_each_others_plugin():
    fake, real_text = FAKE_CONTROL_XACRO.read_text(), REAL_CONTROL_XACRO.read_text()
    assert "mock_components/GenericSystem" in fake and "OpenArmHW" not in fake
    assert "Rh56f1HW" not in fake
    assert "mock_components/GenericSystem" not in real_text


# ---------------------------------------------------------------- rendered xacro (Humble)
def _render(**overrides):
    pytest.importorskip("xacro")
    ament = pytest.importorskip("ament_index_python.packages")
    try:
        share = Path(ament.get_package_share_directory("openarm_description"))
    except Exception:
        pytest.skip("openarm_description is not installed in this environment")
    options = dict(
        source_urdf=CANONICAL, manifest=MANIFEST,
        wrapper_xacro=share / "urdf/robot/openarm_rh56f1_bimanual_real.urdf.xacro",
        hand_configuration="both", right_can_interface="can0", left_can_interface="can1",
        can_fd=True, enable_right_arm=True, enable_left_arm=True,
        right_hand_transport="rs485", left_hand_transport="rs485",
        right_hand_port="/dev/null", left_hand_port="/dev/null",
        right_hand_baudrate=115200, left_hand_baudrate=115200,
        right_hand_device_id=1, left_hand_device_id=2)
    options.update(overrides)
    return ET.fromstring(real.render_real_control_description(**options))


def _components(root):
    return {c.get("name"): c for c in root.findall("ros2_control")}


@pytest.mark.parametrize("configuration", HAND_CONFIGURATIONS)
def test_rendered_resources_all_exist_in_the_runtime_urdf(configuration):
    root = _render(hand_configuration=configuration)
    urdf_joints = {j.get("name") for j in root.findall("joint")}
    resources = [j.get("name") for c in root.findall("ros2_control") for j in c.findall("joint")]
    assert resources and set(resources) <= urdf_joints
    assert len(resources) == len(set(resources))
    expected = 14 + 6 * len(real.enabled_hands(configuration))
    assert len(resources) == expected
    mimic = {j.get("name") for j in root.findall("joint") if j.find("mimic") is not None}
    assert mimic.isdisjoint(resources)


def test_rendered_arm_components_carry_the_safety_file_values():
    root = _render()
    safety = yaml.safe_load(SAFETY.read_text())["arm"]
    for side in ("right", "left"):
        hw = _components(root)[f"openarm_rh56f1_{side}_arm"].find("hardware")
        params = {p.get("name"): p.text for p in hw.findall("param")}
        assert hw.find("plugin").text == "openarm_hardware/OpenArmHW"
        assert params["auto_return_to_zero"].lower() == "false"
        assert params["verify_state_before_enable"].lower() == "true"
        assert float(params["min_inactive_sec_before_activate"]) == safety["min_inactive_sec_before_activate"]
        assert float(params["state_stale_timeout_sec"]) == safety["state_stale_timeout_sec"]


def test_rendered_hand_components_bind_actuators_by_name_with_urdf_limits():
    root = _render()
    _, contract, _ = _variant("both")
    for side in ("right", "left"):
        component = _components(root)[f"rh56f1_{side}_hand"]
        params = {p.get("name"): p.text for p in component.find("hardware").findall("param")}
        assert params["transport"] == "rs485"
        for key in real.HAND_ACTUATORS:
            entry = contract["hands"][side][key]
            assert params[f"actuator_{key}"] == entry["name"]
            joint = component.find(f"joint[@name='{entry['name']}']")
            position = joint.find("command_interface[@name='position']")
            limits = {p.get("name"): float(p.text) for p in position.findall("param")}
            assert limits == {"min": entry["lower"], "max": entry["upper"]}
        assert not any(name.startswith("mock_") for name in params)


def test_disabled_arm_is_absent_from_the_render():
    root = _render(enable_left_arm=False, hand_configuration="right")
    assert set(_components(root)) == {"openarm_rh56f1_right_arm", "rh56f1_right_hand"}


def test_test_double_is_opt_in_and_marked():
    root = _render(arm_test_double=True)
    plugins = {n: c.find("hardware/plugin").text for n, c in _components(root).items()}
    assert plugins["openarm_rh56f1_right_arm"] == "mock_components/GenericSystem"
    assert plugins["rh56f1_right_hand"] == "rh56f1_hardware/Rh56f1HW"
    assert root.get("name").endswith("_TEST_DOUBLE")
    assert "GenericSystem" not in ET.tostring(_render(), encoding="unicode")
