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
LAUNCH_DIR = ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/launch"
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
