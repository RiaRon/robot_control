"""Split OpenArm + RH56F1 bringup: one controller manager per hardware owner.

Arms under /controller_manager, each hand under /rh56f1_<side>/controller_manager,
one robot_state_publisher for the integrated model, and a merger that builds
/joint_states from each device's own joint states. Descriptions that need xacro
or an installed share are skipped outside the Humble overlay.
"""

import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
KUKU_LAB = ROOT.parent
BRINGUP = ROOT / "ros_ws/src/openarm_ros2/openarm_bringup"
LAUNCH_DIR = BRINGUP / "launch"
sys.path.insert(0, str(LAUNCH_DIR))

import device_guard  # noqa: E402
import rh56f1_split_description as split  # noqa: E402

CANONICAL = KUKU_LAB / "urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"
MANIFEST = KUKU_LAB / "urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml"
DESCRIPTION_SRC = ROOT / "ros_ws/src/openarm_description"
FAKE_WRAPPER = DESCRIPTION_SRC / "urdf/robot/openarm_rh56f1_bimanual.urdf.xacro"
REAL_WRAPPER = DESCRIPTION_SRC / "urdf/robot/openarm_rh56f1_bimanual_real.urdf.xacro"
REAL_CONTROLLERS = BRINGUP / "config/controllers/openarm_rh56f1_real_controllers.yaml"
ORDER = yaml.safe_load(MANIFEST.read_text())["control_joint_order"]
SOURCE = {c: s for s, c in yaml.safe_load(MANIFEST.read_text())["source_to_canonical_joints"].items()}
MIMIC = [f"{p}_hj_{f}" for p in "rl" for f in ("thumb_3", "thumb_4", "index_2", "middle_2",
                                                "ring_2", "pinky_2")]
PALMS = {"r_hl_palm_sensor", "l_hl_palm_sensor"}

spec = importlib.util.spec_from_file_location(
    "openarm_rh56f1_joint_state_merger", BRINGUP / "scripts/openarm_rh56f1_joint_state_merger.py")
merger = importlib.util.module_from_spec(spec)
sys.modules["openarm_rh56f1_joint_state_merger"] = merger
spec.loader.exec_module(merger)


def _xacro():
    pytest.importorskip("xacro")


def _links(description):
    return {link.get("name") for link in ET.fromstring(description).findall("link")}


def _joint_contract(root, name):
    joint = next(j for j in root.findall("joint") if j.get("name") == name)
    return joint.get("type"), [(c.tag, sorted(c.attrib.items())) for c in joint]


def _real_arm(**overrides):
    _xacro()
    options = dict(right_can_interface="can0", left_can_interface="can1", can_fd=True,
                   enable_right_arm=True, enable_left_arm=True)
    options.update(overrides)
    try:
        return split.real_arm_description(CANONICAL, MANIFEST, REAL_WRAPPER, **options)
    except Exception as error:  # the real xacro needs the installed openarm_description share
        if "find" in str(error) or "package" in str(error).lower():
            pytest.skip(f"openarm_description is not installed here: {error}")
        raise


# ---------------------------------------------------------------- ownership
def test_each_device_owns_its_joints_and_nothing_else():
    for runtime in split.RUNTIMES:
        owned = split.device_joints(runtime, MANIFEST)
        everything = owned["arms"] + owned["right"] + owned["left"]
        assert len(everything) == len(set(everything)) == 26
        assert not set(everything) & set(MIMIC) and not set(everything) & {SOURCE.get(m) for m in MIMIC}
    fake = split.device_joints("fake", MANIFEST)
    assert fake["arms"] == [n for n in ORDER if "_aj_" in n]
    assert fake["right"] == [n for n in ORDER if n.startswith("r_hj_")]
    real = split.device_joints("real", MANIFEST)
    assert real["arms"][:7] == [f"openarm_right_joint{i}" for i in range(1, 8)]
    assert real["arms"][7:] == [f"openarm_left_joint{i}" for i in range(1, 8)]
    assert real["right"] == [SOURCE[n] for n in fake["right"]]


def test_state_sources_and_namespaces():
    sources = split.state_sources("fake", MANIFEST)
    assert [(s["id"], s["topic"]) for s in sources] == [
        ("openarm", "/openarm/joint_states"),
        ("rh56f1_right", "/rh56f1_right/joint_states"),
        ("rh56f1_left", "/rh56f1_left/joint_states")]
    assert [s["display_placeholder"] for s in sources] == [False, True, True]
    assert split.controller_manager(split.ARM_NAMESPACE) == "/controller_manager"
    assert split.controller_manager(split.HAND_NAMESPACE["right"]) == "/rh56f1_right/controller_manager"
    assert split.controller_manager(split.HAND_NAMESPACE["left"]) == "/rh56f1_left/controller_manager"
    outputs = {split.MERGED_STATE_TOPIC, split.DISPLAY_STATE_TOPIC, split.SOURCE_STATUS_TOPIC}
    assert not outputs & {s["topic"] for s in sources}


# ------------------------------------------------------------- descriptions
@pytest.mark.parametrize("runtime", split.RUNTIMES)
def test_model_has_the_whole_robot_and_no_hardware(runtime):
    description = split.model_description(runtime, CANONICAL, MANIFEST)
    root = ET.fromstring(description)
    assert root.findall("ros2_control") == []
    assert PALMS <= _links(description)
    assert {f"{p}_hl_{f}_tip" for p in "rl" for f in ("thumb", "index", "middle", "ring", "pinky")} \
        <= _links(description)
    canonical = ET.parse(CANONICAL).getroot()
    rename = {} if runtime == "fake" else SOURCE
    for name in ("l_hj_mount", "r_hj_mount", "r_hj_palm_sensor", "l_hj_palm_sensor"):
        assert _joint_contract(root, rename.get(name, name)) == _joint_contract(canonical, name)
    for name in MIMIC:
        kind, children = _joint_contract(root, rename.get(name, name))
        assert kind == _joint_contract(canonical, name)[0]
        # Mimic references follow the runtime names; everything else is unchanged.
        mimic = dict(next(attrs for tag, attrs in children if tag == "mimic"))
        source_mimic = dict(next(attrs for tag, attrs in _joint_contract(canonical, name)[1]
                                 if tag == "mimic"))
        assert mimic["joint"] == rename.get(source_mimic["joint"], source_mimic["joint"])
        assert {k: v for k, v in mimic.items() if k != "joint"} == \
            {k: v for k, v in source_mimic.items() if k != "joint"}
    assert "file:///home/user/rl_ws" not in description


def test_fake_arm_description_loads_only_the_arms():
    _xacro()
    description = split.fake_arm_description(CANONICAL, FAKE_WRAPPER)
    blocks = split.ros2_control_blocks(description)
    assert len(blocks) == 1
    (block,) = blocks.values()
    assert block["plugin"] == split.FAKE_PLUGIN
    assert block["joints"] == [n for n in ORDER if "_aj_" in n]
    assert block["commands"] == [f"{n}/position" for n in block["joints"]]
    assert PALMS <= _links(description)


@pytest.mark.parametrize("side", split.SIDES)
def test_fake_hand_description_loads_only_its_six_actuators(side):
    description = split.fake_hand_description(side, CANONICAL, MANIFEST)
    blocks = split.ros2_control_blocks(description)
    assert list(blocks) == [f"rh56f1_{side}_hand_fake"]
    block = blocks[f"rh56f1_{side}_hand_fake"]
    prefix = "r_hj_" if side == "right" else "l_hj_"
    assert block["plugin"] == split.FAKE_PLUGIN
    assert block["joints"] == [n for n in ORDER if n.startswith(prefix)]
    assert block["commands"] == [f"{n}/position" for n in block["joints"]]
    assert PALMS <= _links(description)  # the whole model, only this hand's hardware
    params = split.fake_hand_controllers(side, MANIFEST, description)
    controller = f"{side}_hand_trajectory_controller"
    assert params["/**/controller_manager"]["ros__parameters"][controller]["type"] == (
        "joint_trajectory_controller/JointTrajectoryController")
    values = params[f"/**/{controller}"]["ros__parameters"]
    assert values["joints"] == block["joints"] and values["command_interfaces"] == ["position"]


def test_three_managers_never_share_a_resource():
    _xacro()
    commands = []
    for description in (split.fake_arm_description(CANONICAL, FAKE_WRAPPER),
                        split.fake_hand_description("right", CANONICAL, MANIFEST),
                        split.fake_hand_description("left", CANONICAL, MANIFEST)):
        commands += [c for block in split.ros2_control_blocks(description).values()
                     for c in block["commands"]]
    assert len(commands) == len(set(commands)) == 26
    assert sorted(c.split("/")[0] for c in commands) == sorted(ORDER)


def test_real_arm_description_has_no_hand_hardware_and_keeps_the_startup_policy():
    description, plan = _real_arm()
    blocks = split.ros2_control_blocks(description)
    assert {b["plugin"] for b in blocks.values()} == {"openarm_hardware/OpenArmHW"}
    assert set(blocks) == {"openarm_rh56f1_right_arm", "openarm_rh56f1_left_arm"}
    assert [j for b in blocks.values() for j in b["joints"]] == \
        split.device_joints("real", MANIFEST)["arms"]
    assert "Rh56f1HW" not in description and PALMS <= _links(description)
    root = ET.fromstring(description)
    for block in root.findall("ros2_control"):
        params = {p.get("name"): (p.text or "").strip() for p in block.findall("hardware/param")}
        assert params["auto_return_to_zero"].lower() == "false"
        assert params["verify_state_before_enable"].lower() == "true"
    assert plan["controllers"] == ["rh56f1_right_arm_controller", "rh56f1_left_arm_controller"]
    assert list(plan["broadcasters"]) == ["joint_state_broadcaster"]
    assert plan["hand_joints"] == [] and plan["components"] == list(blocks)

    from rh56f1_real_description import controller_params

    params = controller_params(yaml.safe_load(REAL_CONTROLLERS.read_text()), plan)
    manager = params["controller_manager"]["ros__parameters"]
    assert manager["hardware_components_initial_state"] == {"inactive": list(blocks)}
    assert not [k for k in list(manager) + list(params) if "hand" in k]


def test_real_arm_description_with_one_arm_and_test_double():
    description, plan = _real_arm(enable_left_arm=False)
    assert list(split.ros2_control_blocks(description)) == ["openarm_rh56f1_right_arm"]
    assert plan["controllers"] == ["rh56f1_right_arm_controller"]
    double, _ = _real_arm(arm_test_double=True)
    assert {b["plugin"] for b in split.ros2_control_blocks(double).values()} == {split.FAKE_PLUGIN}


def test_hardware_hand_configuration_cannot_exceed_the_geometry():
    _xacro()
    from rh56f1_real_description import render_real_control_description

    with pytest.raises(ValueError, match="needs hand geometry"):
        render_real_control_description(
            source_urdf=CANONICAL, manifest=MANIFEST, wrapper_xacro=REAL_WRAPPER,
            hand_configuration="arm_only", hardware_hand_configuration="both",
            right_can_interface="can0", left_can_interface="can1", can_fd=True,
            enable_right_arm=True, enable_left_arm=True,
            right_hand_transport="rs485", left_hand_transport="rs485",
            right_hand_port="", left_hand_port="", right_hand_baudrate=0,
            left_hand_baudrate=0, right_hand_device_id=0, left_hand_device_id=0)


# ------------------------------------------------------------------ merger
def _core(stale=0.5, placeholder=True):
    sources = split.state_sources("fake", MANIFEST)
    if not placeholder:
        for source in sources:
            source["display_placeholder"] = False
    return merger.MergerCore(sources, ["/joint_states", "/display", "/status"], stale)


def test_merger_refuses_shared_owners_and_loops():
    sources = split.state_sources("fake", MANIFEST)
    with pytest.raises(merger.MergerConfigError, match="owned by both"):
        merger.MergerCore(sources + [{"id": "x", "topic": "/x", "joints": ["r_aj_1"]}], [])
    with pytest.raises(merger.MergerConfigError, match="loop"):
        merger.MergerCore(sources, ["/openarm/joint_states"])
    with pytest.raises(merger.MergerConfigError, match="two sources"):
        merger.MergerCore(sources + [{"id": "x", "topic": "/openarm/joint_states", "joints": []}], [])


def test_merger_forwards_by_name_only_what_a_device_owns():
    core = _core()
    arm = split.device_joints("fake", MANIFEST)["arms"]
    shuffled = list(reversed(arm)) + ["r_hj_index_1", "r_hj_index_2"]
    forward = core.receive("openarm", 1.0, shuffled, list(range(len(shuffled))))
    assert forward.names == list(reversed(arm))
    assert forward.positions == [float(i) for i in range(14)]
    status = core.status(1.0)["openarm"]
    assert status["foreign_joints_dropped"] == 2
    assert status["foreign_joint_names"] == ["r_hj_index_1", "r_hj_index_2"]
    assert core.receive("openarm", 1.1, ["r_hj_index_1"], [0.3]) is None
    assert core.receive("openarm", 1.2, arm, [float("nan")] + [0.0] * 13) is None
    assert core.status(1.2)["openarm"]["nonfinite_messages_dropped"] == 1


def test_merger_freshness_and_display_placeholder():
    core = _core(stale=0.5)
    hand = split.device_joints("fake", MANIFEST)["right"]
    assert core.state("rh56f1_right", 0.0) == "never"
    # No hand yet: both hands are drawn open, the arm never is.
    assert sorted(p.source for p in core.placeholders(0.0)) == ["rh56f1_left", "rh56f1_right"]
    core.receive("rh56f1_right", 1.0, hand, [0.1] * 6)
    assert core.state("rh56f1_right", 1.4) == "fresh"
    assert [p.source for p in core.placeholders(1.4)] == ["rh56f1_left"]
    assert core.state("rh56f1_right", 1.6) == "stale"
    stale = [p for p in core.placeholders(1.6) if p.source == "rh56f1_right"]
    assert stale and stale[0].positions == [0.0] * 6 and stale[0].names == hand
    status = core.status(1.6)["rh56f1_right"]
    assert status["state"] == "stale" and status["display_placeholder"] and status["age_sec"] == 0.6
    off = _core(placeholder=False)
    assert off.placeholders(0.0) == []


def test_merger_with_arms_only_never_reports_26():
    core = _core()
    arm = split.device_joints("fake", MANIFEST)["arms"]
    forward = core.receive("openarm", 1.0, arm, [0.0] * 14)
    assert len(forward.names) == 14
    assert {s: v["state"] for s, v in core.status(1.0).items()} == {
        "openarm": "fresh", "rh56f1_right": "never", "rh56f1_left": "never"}


# -------------------------------------------------------------- ownership
def test_device_lock_is_exclusive_across_processes(tmp_path, monkeypatch):
    monkeypatch.setenv(device_guard.LOCK_DIR_ENV, str(tmp_path))
    path = device_guard.hold_device_lock("can_can0")
    assert path.parent == tmp_path
    with pytest.raises(device_guard.OwnershipError, match="already owned"):
        device_guard.hold_device_lock("can_can0")
    code = (
        "import sys; sys.path.insert(0, %r); import device_guard\n"
        "try:\n    device_guard.hold_device_lock('can_can0')\nexcept device_guard.OwnershipError:\n"
        "    sys.exit(3)\n" % str(LAUNCH_DIR))
    env = {**os.environ, device_guard.LOCK_DIR_ENV: str(tmp_path)}
    assert subprocess.run([sys.executable, "-c", code], env=env).returncode == 3
    device_guard.hold_device_lock("can_can1")  # another interface is free


# --------------------------------------------------------- launch structure
def _launch_nodes(file, **values):
    pytest.importorskip("launch")
    ament = pytest.importorskip("ament_index_python.packages")
    try:
        ament.get_package_share_directory("openarm_description")
        ament.get_package_share_directory("openarm_bringup")
    except Exception:
        pytest.skip("openarm_description/openarm_bringup are not installed here")
    from launch import LaunchContext

    spec = importlib.util.spec_from_file_location(file.replace(".", "_"), LAUNCH_DIR / file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        name = getattr(action, "name", None)
        if name is not None and hasattr(action, "default_value"):
            default = action.default_value
            context.launch_configurations[name] = "".join(
                s.perform(context) for s in default) if default else ""
    context.launch_configurations.update(values)
    from launch_ros.actions import Node

    return [action for action in module._setup(context) if isinstance(action, Node)]


def _node_info(node):
    arguments = [a if isinstance(a, str) else "".join(p.text for p in a)
                 for a in (node._Node__arguments or [])]
    remaps = [(a if isinstance(a, str) else "".join(p.text for p in a),
               b if isinstance(b, str) else "".join(p.text for p in b))
              for a, b in (node._Node__remappings or [])]
    namespace = node._Node__node_namespace
    namespace = namespace if namespace is None or isinstance(namespace, str) else \
        "".join(p.text for p in namespace)
    return node.node_executable, namespace, arguments, remaps


def test_fake_arm_launch_owns_the_root_manager_only(tmp_path, monkeypatch):
    monkeypatch.setenv(device_guard.LOCK_DIR_ENV, str(tmp_path))
    nodes = [_node_info(n) for n in _launch_nodes(
        "openarm_rh56f1_arms.launch.py", check_ownership="false")]
    manager = [n for n in nodes if n[0] == "ros2_control_node"]
    assert len(manager) == 1 and manager[0][1] in (None, "")
    assert ("/joint_states", "/openarm/joint_states") in manager[0][3]
    spawned = [n[2] for n in nodes if n[0] == "spawner"]
    assert all(args[-2:] == ["--controller-manager", "/controller_manager"] for args in spawned)
    assert [a[:-2] for a in spawned] == [
        ["joint_state_broadcaster"],
        ["left_joint_trajectory_controller", "right_joint_trajectory_controller"]]
    assert "robot_state_publisher" not in [n[0] for n in nodes]  # that is the model launch's


def test_real_arm_launch_refuses_without_confirmation_and_loads_inactive(tmp_path, monkeypatch):
    monkeypatch.setenv(device_guard.LOCK_DIR_ENV, str(tmp_path))
    with pytest.raises(RuntimeError, match="i_understand_this_moves_real_hardware"):
        _launch_nodes("openarm_rh56f1_arms.launch.py", runtime="real", check_ownership="false")
    nodes = [_node_info(n) for n in _launch_nodes(
        "openarm_rh56f1_arms.launch.py", runtime="real", check_ownership="false",
        i_understand_this_moves_real_hardware="true")]
    spawned = [n[2][:-2] for n in nodes if n[0] == "spawner"]
    assert spawned == [["joint_state_broadcaster"],
                       ["rh56f1_right_arm_controller", "--inactive"],
                       ["rh56f1_left_arm_controller", "--inactive"]]
    assert sorted(p.name for p in tmp_path.iterdir()) == ["can_can0.lock", "can_can1.lock"]
    # A second real arm launch on the same CAN interfaces is refused.
    with pytest.raises(device_guard.OwnershipError, match="can_can0"):
        _launch_nodes("openarm_rh56f1_arms.launch.py", runtime="real", check_ownership="false",
                      i_understand_this_moves_real_hardware="true")


@pytest.mark.parametrize("side", split.SIDES)
def test_hand_launch_owns_its_namespaced_manager_only(side, tmp_path, monkeypatch):
    monkeypatch.setenv(device_guard.LOCK_DIR_ENV, str(tmp_path / side))
    nodes = [_node_info(n) for n in _launch_nodes(
        "rh56f1_hand.launch.py", side=side, check_ownership="false")]
    manager = [n for n in nodes if n[0] == "ros2_control_node"]
    assert len(manager) == 1 and manager[0][1] == f"rh56f1_{side}"
    spawned = [n[2] for n in nodes if n[0] == "spawner"]
    assert spawned == [
        [name, "--controller-manager", f"/rh56f1_{side}/controller_manager"]
        for name in ("joint_state_broadcaster", f"{side}_hand_trajectory_controller")]
    assert {n[0] for n in nodes} == {"ros2_control_node", "spawner"}
    with pytest.raises(RuntimeError, match="no real hand backend"):
        _launch_nodes("rh56f1_hand.launch.py", side=side, runtime="real", check_ownership="false")


def test_model_launch_is_the_only_tf_publisher_and_reads_the_display_topic():
    nodes = _launch_nodes("openarm_rh56f1_model.launch.py")
    infos = [_node_info(n) for n in nodes]
    assert [i[0] for i in infos] == ["robot_state_publisher",
                                     "openarm_rh56f1_joint_state_merger.py"]
    assert ("joint_states", split.DISPLAY_STATE_TOPIC) in infos[0][3]
    arguments = infos[1][2]
    sources = json.loads(arguments[arguments.index("--sources") + 1])
    assert [s["topic"] for s in sources] == [
        "/openarm/joint_states", "/rh56f1_right/joint_states", "/rh56f1_left/joint_states"]
