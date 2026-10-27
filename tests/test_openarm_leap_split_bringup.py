"""Split OpenArm + LEAP bringup (fake): descriptions, ownership, controller params.

Static; needs only PyYAML and the canonical LEAP asset in the urdf repo (branch
feat/leap-hand). The runtime counterpart is tests/jazzy_leap_split_probe.py.
"""

from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_DIR = ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/launch/leap_hand"
sys.path.insert(0, str(LAUNCH_DIR))

import leap_split_description as leap  # noqa: E402

CANONICAL = leap.default_canonical_urdf(str(LAUNCH_DIR / "openarm_leap_arms.launch.py"))
pytestmark = pytest.mark.skipif(not CANONICAL, reason="openarm_leap_bi_rl.urdf not found")
MANIFEST = leap.default_manifest_for(Path(CANONICAL)) if CANONICAL else None


def _blocks(description):
    return leap.ros2_control_blocks(description)


def test_canonical_urdf_is_found_relative_to_the_launch_file(monkeypatch, tmp_path):
    assert Path(CANONICAL).resolve() == (ROOT.parent / leap.CANONICAL_RELATIVE).resolve()
    override = tmp_path / leap.CANONICAL_RELATIVE
    override.parent.mkdir(parents=True)
    override.write_text("<robot/>")
    monkeypatch.setenv("KUKU_LAB_ROOT", str(tmp_path))
    assert leap.default_canonical_urdf(str(LAUNCH_DIR / "x.py")) == str(override)


def test_each_device_owns_its_joints_in_manifest_order():
    order = yaml.safe_load(MANIFEST.read_text())["control_joint_order"]
    owned = leap.device_joints(MANIFEST)
    assert [len(owned[d]) for d in ("arms", "right", "left")] == [14, 16, 16]
    assert sorted(sum(owned.values(), [])) == sorted(order)  # every LEAP joint is actuated


def test_model_has_no_hardware_and_every_mesh_resolves():
    description = leap.model_description(CANONICAL)
    root = ET.fromstring(description)
    assert root.findall("ros2_control") == []
    meshes = [m.get("filename") for m in root.iter("mesh")]
    assert meshes and all(Path(m.removeprefix("file://")).is_file() for m in meshes)
    canonical = ET.parse(CANONICAL).getroot()
    assert [j.attrib for j in root.findall("joint")] == [j.attrib for j in canonical.findall("joint")]


def test_arm_and_hand_managers_split_the_46_resources():
    owned = leap.device_joints(MANIFEST)
    descriptions = {"arms": leap.fake_arm_description(CANONICAL, MANIFEST),
                    "right": leap.fake_hand_description("right", CANONICAL, MANIFEST),
                    "left": leap.fake_hand_description("left", CANONICAL, MANIFEST)}
    commands = []
    for device, description in descriptions.items():
        (block,) = _blocks(description).values()
        assert block["plugin"] == leap.FAKE_PLUGIN
        assert block["joints"] == owned[device]
        commands += block["commands"]
        assert {"r_hl_palm", "l_hl_palm"} <= {l.get("name") for l in ET.fromstring(description).iter("link")}
    assert len(commands) == len(set(commands)) == 46


@pytest.mark.parametrize("side", leap.SIDES)
def test_controller_params_are_plain_yaml_over_owned_joints(side):
    hand = leap.fake_hand_description(side, CANONICAL, MANIFEST)
    arms = leap.fake_arm_description(CANONICAL, MANIFEST)
    for params, controllers in ((leap.fake_hand_controllers(side, MANIFEST, hand),
                                 [leap.FAKE_HAND_CONTROLLERS[side]]),
                                (leap.fake_arm_controllers(MANIFEST, arms),
                                 list(leap.ARM_CONTROLLERS.values()))):
        text = yaml.safe_dump(params)
        assert "&id" not in text and "*id" not in text  # rcl refuses YAML aliases
        manager = params["/**/controller_manager"]["ros__parameters"]
        for name in controllers:
            assert manager[name]["type"] == "joint_trajectory_controller/JointTrajectoryController"
            assert params[f"/**/{name}"]["ros__parameters"]["command_interfaces"] == ["position"]
    joints = params["/**/right_joint_trajectory_controller"]["ros__parameters"]["joints"]
    assert joints == [f"r_aj_{i}" for i in range(1, 8)]


def test_merger_sources_and_topics_do_not_loop():
    sources = leap.state_sources(MANIFEST)
    assert [(s["id"], s["topic"]) for s in sources] == [
        ("openarm", "/openarm/joint_states"), ("leap_right", "/leap_right/joint_states"),
        ("leap_left", "/leap_left/joint_states"), ("head", leap.HEAD_STATE_TOPIC)]
    # The head is display-only: no device, so its pan/tilt are always drawn from the placeholder.
    assert sources[-1]["joints"] == ["head_j_pan", "head_j_tilt"] and sources[-1]["display_placeholder"]
    outputs = {leap.MERGED_STATE_TOPIC, leap.DISPLAY_STATE_TOPIC, leap.SOURCE_STATUS_TOPIC,
               leap.ARM_DESCRIPTION_TOPIC}
    assert not {s["id"] for s in sources} - {"openarm", "leap_right", "leap_left", "head"}
    assert not outputs & {s["topic"] for s in sources}
    assert leap.ARM_DESCRIPTION_TOPIC != "/robot_description"  # the model's publisher owns that


# ------------------------------------------------------------- real hand (ROBOTIS plugin)
def _real_settings(**sides):
    settings = leap.load_real_settings()
    for side, port in sides.items():
        settings[side] = {"port": port}
    return settings


@pytest.mark.parametrize("side", leap.SIDES)
def test_real_hand_motor_ids_follow_the_manifest(side):
    motors = leap.motor_joints(side, MANIFEST)
    mapping = yaml.safe_load(MANIFEST.read_text())["source_to_canonical_joints"]
    assert [i for i, _ in motors] == list(range(16))
    assert all(mapping[f"leap_{side}_{i}"] == joint for i, joint in motors)
    assert dict(motors)[1] == f"{side[0]}_hj_index_1"  # LEAP joint 1: index MCP flexion


@pytest.mark.parametrize("side", leap.SIDES)
def test_real_hand_description_is_one_robotis_block(side):
    description = leap.real_hand_description(side, Path(CANONICAL), MANIFEST, _real_settings(**{side: "/dev/x"}))
    blocks = _blocks(description)
    assert list(blocks) == [f"leap_{side}_hand_real"]
    block = blocks[f"leap_{side}_hand_real"]
    assert block["plugin"] == leap.REAL_PLUGIN
    assert block["joints"] == [j for _, j in leap.motor_joints(side, MANIFEST)]
    assert sorted(block["joints"]) == sorted(leap.device_joints(MANIFEST)[side])
    xml = ET.fromstring(description).find("ros2_control")
    params = {p.get("name"): p.text for p in xml.findall("hardware/param")}
    assert params["port_name"] == "/dev/x" and params["disable_torque_at_init"] == "true"
    assert params["number_of_joints"] == params["number_of_transmissions"] == "16"
    matrix = [int(v) for v in params["transmission_to_joint_matrix"].split(",")]
    assert matrix == [int(r == c) for r in range(16) for c in range(16)]
    gpios = xml.findall("gpio")
    assert [int(g.find("param[@name='ID']").text) for g in gpios] == list(range(16))
    first = {p.get("name"): p.text for p in gpios[0].findall("param")}
    assert first["type"] == "dxl" and first["Operating Mode"] == "5" and first["Current Limit"] == "350"


def test_real_hand_refuses_without_a_port_and_starts_inactive():
    with pytest.raises(ValueError, match="right.port is not set"):
        leap.real_hand_description("right", Path(CANONICAL), MANIFEST, _real_settings(right=""))
    description = leap.real_hand_description("right", Path(CANONICAL), MANIFEST, _real_settings(right="/dev/x"))
    params = leap.real_hand_controllers("right", MANIFEST, description, start_inactive=True)
    manager = params["/**/controller_manager"]["ros__parameters"]
    assert manager["hardware_components_initial_state"] == {"inactive": ["leap_right_hand_real"]}
    assert params["/**/right_hand_trajectory_controller"]["ros__parameters"]["joints"] \
        == leap.device_joints(MANIFEST)["right"]


def test_motor_id_swap_exchanges_the_mcp_pairs():
    plain = dict(leap.motor_joints("right", MANIFEST))
    swapped = dict(leap.motor_joints("right", MANIFEST, {0: 1, 4: 5, 8: 9}))
    assert swapped[0] == plain[1] == "r_hj_index_1" and swapped[1] == plain[0]
    assert all(swapped[i] == plain[i] for i in range(16) if i not in (0, 1, 4, 5, 8, 9))
    with pytest.raises(ValueError, match="no such motor"):
        leap.motor_joints("right", MANIFEST, {0: 16})


# ---------------------------------------------------------------- real arms (OpenArmHW)
def _arm_settings(**cans):
    settings = yaml.safe_load(leap.REAL_ARM_SETTINGS.read_text())
    for side, can in cans.items():
        settings[side] = {"can_interface": can}
    return settings


def test_real_arms_are_one_openarmhw_block_per_enabled_arm_with_source_names():
    description = leap.real_arm_description(Path(CANONICAL), MANIFEST, _arm_settings(right="can0"))
    blocks = _blocks(description)
    assert list(blocks) == ["openarm_leap_right_arm"]
    assert blocks["openarm_leap_right_arm"]["plugin"] == leap.REAL_ARM_PLUGIN
    assert blocks["openarm_leap_right_arm"]["joints"] == [f"openarm_right_joint{i}" for i in range(1, 8)]
    root = ET.fromstring(description)
    params = {p.get("name"): p.text for p in root.find("ros2_control/hardware").findall("param")}
    assert params["can_interface"] == "can0" and params["arm_prefix"] == "right_"
    assert params["auto_return_to_zero"] == "false" and params["verify_state_before_enable"] == "true"
    names = {j.get("name") for j in root.findall("joint")}
    assert "openarm_left_joint1" in names and "r_aj_1" not in names and "r_hj_index_1" in names
    with pytest.raises(ValueError, match="no arm enabled"):
        leap.real_arm_description(Path(CANONICAL), MANIFEST, _arm_settings(right="", left=""))


def test_real_arm_controllers_start_inactive_and_states_use_source_names():
    settings = _arm_settings(right="can0", left="can1")
    description = leap.real_arm_description(Path(CANONICAL), MANIFEST, settings)
    params = leap.real_arm_controllers(description, settings)
    manager = params["/**/controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 750
    assert manager["hardware_components_initial_state"]["inactive"] == [
        "openarm_leap_right_arm", "openarm_leap_left_arm"]
    left = params["/**/left_joint_trajectory_controller"]["ros__parameters"]
    assert left["joints"] == [f"openarm_left_joint{i}" for i in range(1, 8)]
    arms = leap.state_sources(MANIFEST, "real")[0]["joints"]
    assert sorted(arms) == sorted(f"openarm_{s}_joint{i}" for s in leap.SIDES for i in range(1, 8))
    model = ET.fromstring(leap.model_description(Path(CANONICAL), MANIFEST, "real"))
    assert {j.get("name") for j in model.findall("joint")} >= set(arms)
