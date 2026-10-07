"""Nova2 glove -> RH56F1 fake hand adapter: contract, mapping, clutch, vendored code."""

import hashlib
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

from rh56f1_glove_teleop import synth_glove
from rh56f1_glove_teleop.adapter import (
    ACTUATORS,
    FOLLOWING,
    LOCKED,
    WAITING,
    ContractError,
    GloveHandAdapter,
    JointSample,
    TargetError,
    TargetSample,
    build_contract,
    fake_hand_refusal,
    load_config,
    map_target,
)
from robot_control.profile import load_builtin_profile

ROOT = Path(__file__).resolve().parents[1]
KUKU_LAB = ROOT.parent
VENDORED = ROOT / "third_party/inspire_hand_senseglove_teleop"
METADATA = ROOT / "vendor_metadata/inspire_hand_senseglove_teleop/UPSTREAM.yaml"
ORIGINAL = KUKU_LAB / "third_party/inspire_hand-main_2026-10-03/inspire_hand-main"
LAUNCH = ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/launch/rh56f1_glove_input.launch.py"
SOURCE_ORDER = ("pinky_proximal_joint", "ring_proximal_joint", "middle_proximal_joint",
                "index_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint")
CLOSED = {"pinky_proximal_joint": 1.47, "ring_proximal_joint": 1.47,
          "middle_proximal_joint": 1.47, "index_proximal_joint": 1.47,
          "thumb_proximal_pitch_joint": 0.6, "thumb_proximal_yaw_joint": 1.308}
PERIOD = 0.02


@pytest.fixture(scope="module")
def profile():
    return load_builtin_profile("openarm_rh56f1")


@pytest.fixture(scope="module")
def order(profile):
    return yaml.safe_load(Path(profile.manifest_path).read_text())["control_joint_order"]


def _contract(profile, order, side="right", **thumbs):
    return build_contract(load_config(), profile, order, side, **thumbs)


def _upper(contract, actuator):
    return contract.upper[contract.index(actuator)]


# -------------------------------------------------------------- contract
def test_contract_uses_manifest_order_and_profile_limits(profile, order):
    right = _contract(profile, order)
    assert right.canonical == tuple(f"r_hj_{a}" for a in ACTUATORS)
    assert right.canonical == tuple(n for n in order if n.startswith("r_hj_"))
    limits = {j.canonical: (j.lower, j.upper) for j in profile.joints}
    assert [tuple(v) for v in zip(right.lower, right.upper)] == [limits[n] for n in right.canonical]
    left = _contract(profile, order, "left")
    assert left.canonical == tuple(f"l_hj_{a}" for a in ACTUATORS)
    by_source = {s.name: s.actuator for s in right.sources}
    assert by_source == {
        "pinky_proximal_joint": "pinky_1", "ring_proximal_joint": "ring_1",
        "middle_proximal_joint": "middle_1", "index_proximal_joint": "index_1",
        "thumb_proximal_pitch_joint": "thumb_2", "thumb_proximal_yaw_joint": "thumb_1"}
    assert {s.name: s.closed for s in right.sources} == CLOSED
    assert right.held == {"thumb_1", "thumb_2"}          # old bridge: thumb fixed by default
    assert _contract(profile, order, thumb_pitch="follow").held == {"thumb_1"}
    assert _contract(profile, order, thumb_pitch="follow", thumb_yaw="follow").held == set()


@pytest.mark.parametrize("change, match", [
    (lambda c: c["source"]["joints"]["ring_proximal_joint"].update(actuator="pinky_1"),
     "two source joints"),
    (lambda c: c["source"]["joints"].pop("ring_proximal_joint"), "not all of"),
    (lambda c: c["source"]["joints"]["ring_proximal_joint"].update(actuator="ring_2"),
     "unknown actuator"),
    (lambda c: c["source"]["joints"]["ring_proximal_joint"].update(closed=0.0), "positive"),
    (lambda c: c["thumb"].update(pitch="mirror"), "thumb policy"),
])
def test_contract_refuses_bad_config(profile, order, change, match):
    config = load_config()
    change(config)
    with pytest.raises(ContractError, match=match):
        build_contract(config, profile, order, "right")
    with pytest.raises(ContractError):
        build_contract(load_config(), profile, order, "middle")


# --------------------------------------------------------------- mapping
def _message(values: dict, order=SOURCE_ORDER):
    return list(order), [values[n] for n in order]


def test_mapping_is_by_name_whatever_the_order(profile, order):
    contract = _contract(profile, order)
    values = {"pinky_proximal_joint": 0.147, "ring_proximal_joint": 0.735,
              "middle_proximal_joint": 1.47, "index_proximal_joint": 0.0,
              "thumb_proximal_pitch_joint": 0.3, "thumb_proximal_yaw_joint": 0.654}
    forward = map_target(contract, *_message(values))
    backward = map_target(contract, *_message(values, tuple(reversed(SOURCE_ORDER))))
    np.testing.assert_array_equal(forward.target, backward.target)
    expected = {"pinky_1": 0.1, "ring_1": 0.5, "middle_1": 1.0, "index_1": 0.0,
                "thumb_2": 0.5, "thumb_1": 0.5}
    for actuator, closure in expected.items():
        assert forward.target[contract.index(actuator)] == pytest.approx(
            closure * _upper(contract, actuator))
    # Pinky and ring are separate axes: different sources give different targets.
    assert forward.target[contract.index("pinky_1")] != forward.target[contract.index("ring_1")]
    assert forward.clamped == () and forward.ignored == ()


def test_full_closure_reaches_each_canonical_upper_limit(profile, order):
    contract = _contract(profile, order)
    mapped = map_target(contract, *_message(CLOSED))
    np.testing.assert_allclose(mapped.target, contract.upper)
    np.testing.assert_allclose(
        map_target(contract, *_message({n: 0.0 for n in SOURCE_ORDER})).target, 0.0)


def test_out_of_range_values_are_clamped_and_reported(profile, order):
    contract = _contract(profile, order)
    values = {n: 0.5 for n in SOURCE_ORDER}
    values["index_proximal_joint"] = 2.0
    values["pinky_proximal_joint"] = -0.1
    mapped = map_target(contract, *_message(values))
    assert set(mapped.clamped) == {"index_proximal_joint", "pinky_proximal_joint"}
    assert mapped.target[contract.index("index_1")] == pytest.approx(_upper(contract, "index_1"))
    assert mapped.target[contract.index("pinky_1")] == 0.0


@pytest.mark.parametrize("names, positions, match", [
    (list(SOURCE_ORDER), [0.1] * 5, "6 names but 5 positions"),
    (list(SOURCE_ORDER[:5]) + ["ring_proximal_joint"], [0.1] * 6, "duplicate"),
    (list(SOURCE_ORDER), [0.1, float("nan"), 0.1, 0.1, 0.1, 0.1], "not finite"),
    (list(SOURCE_ORDER), [0.1, 0.1, float("inf"), 0.1, 0.1, 0.1], "not finite"),
    (list(SOURCE_ORDER[:5]), [0.1] * 5, "missing source joints"),
])
def test_broken_targets_are_refused(profile, order, names, positions, match):
    with pytest.raises(TargetError, match=match):
        map_target(_contract(profile, order), names, positions)


def test_extra_names_are_ignored_and_left_side_uses_its_own_names(profile, order):
    left = _contract(profile, order, "left")
    names, positions = _message({n: 0.6 for n in SOURCE_ORDER})
    mapped = map_target(left, names + ["wrist"], positions + [9.0])
    assert mapped.ignored == ("wrist",)
    assert left.canonical[0] == "l_hj_thumb_1"


# --------------------------------------------------------------- clutch
class Rig:
    def __init__(self, contract, measured=None):
        self.core = GloveHandAdapter(contract, PERIOD, 0.5, 0.5)
        self.contract = contract
        self.now = 10.0
        self.q = dict(zip(contract.canonical, measured or [0.0] * 6))

    def target(self, values=None, age=0.0):
        values = values or {n: 0.5 * CLOSED[n] for n in SOURCE_ORDER}   # 50 % closure
        names, positions = _message(values)
        self.core.offer_target(TargetSample(self.now - age, tuple(names), tuple(positions)))

    def step(self, joint_age=0.0, **target):
        self.now += PERIOD
        if target is not None and target.get("send", True):
            self.target(target.get("values"))
        result = self.core.step(self.now, JointSample(self.now - joint_age, self.q))
        if result.command is not None:
            self.q = dict(zip(self.contract.canonical, result.command))
        return result


def test_waits_for_a_target_then_follows_with_thumbs_held(profile, order):
    contract = _contract(profile, order)
    rig = Rig(contract, measured=[0.3, 0.1, 0.0, 0.0, 0.0, 0.0])
    first = rig.core.step(rig.now, JointSample(rig.now, rig.q))
    assert first.state == WAITING and first.command is None and "no glove target" in first.reason
    for _ in range(200):
        result = rig.step()
    assert result.state == FOLLOWING and result.command is not None
    for actuator in ("index_1", "middle_1", "ring_1", "pinky_1"):
        assert rig.q[f"r_hj_{actuator}"] == pytest.approx(0.5 * _upper(contract, actuator), abs=1e-9)
    assert rig.q["r_hj_thumb_1"] == 0.3 and rig.q["r_hj_thumb_2"] == 0.1   # held where measured


def test_follow_policy_moves_the_thumbs(profile, order):
    contract = _contract(profile, order, thumb_pitch="follow", thumb_yaw="follow")
    rig = Rig(contract)
    for _ in range(200):
        rig.step()
    assert rig.q["r_hj_thumb_2"] == pytest.approx(0.5 * _upper(contract, "thumb_2"), abs=1e-9)
    assert rig.q["r_hj_thumb_1"] == pytest.approx(0.5 * _upper(contract, "thumb_1"), abs=1e-9)


def test_commands_are_rate_limited(profile, order):
    contract = _contract(profile, order)
    rig = Rig(contract)
    result = rig.step(values=dict(CLOSED))
    budget = contract.velocity * PERIOD
    assert "velocity" in result.limited
    assert np.all(np.abs(result.command) <= budget + 1e-12)


def test_stale_target_locks_and_needs_enable_and_a_newer_target(profile, order):
    contract = _contract(profile, order)
    rig = Rig(contract)
    for _ in range(10):
        rig.step()
    assert rig.core.state == FOLLOWING
    rig.now += 1.0                                       # glove stream stops
    lost = rig.core.step(rig.now, JointSample(rig.now, rig.q))
    assert lost.state == LOCKED and lost.command is None and "stale" in lost.reason
    for _ in range(10):                                  # targets come back: still locked
        back = rig.step()
        assert back.state == LOCKED and back.command is None and "enable" in back.reason
    rig.core.enable(rig.now)
    old = rig.core.step(rig.now + PERIOD, JointSample(rig.now + PERIOD, rig.q))
    assert old.state == WAITING and old.command is None    # the cached target is too old
    assert rig.step().state == FOLLOWING


def test_stale_or_bad_hand_state_locks(profile, order):
    contract = _contract(profile, order)
    rig = Rig(contract)
    rig.step()
    assert rig.step(joint_age=1.0).state == LOCKED
    rig.core.enable(rig.now)
    rig.step()
    rig.q["r_hj_index_1"] = float("nan")
    assert "not finite" in rig.step().reason


def test_invalid_targets_do_not_refresh_the_stream(profile, order):
    contract = _contract(profile, order)
    rig = Rig(contract)
    rig.step()
    for _ in range(40):                                   # 0.8 s of NaN targets only
        rig.now += PERIOD
        rig.core.offer_target(TargetSample(rig.now, SOURCE_ORDER, (float("nan"),) * 6))
        result = rig.core.step(rig.now, JointSample(rig.now, rig.q))
    assert result.state == LOCKED and rig.core.counters.rejected == 40
    assert "not finite" in rig.core.counters.last_rejection


# ------------------------------------- the producer, and the synthetic glove
def _vendored_mapping():
    package = VENDORED / "senseglove_teleop"
    sys.path.insert(0, str(package))
    try:
        spec = importlib.util.spec_from_file_location(
            "senseglove_teleop.mapping", package / "senseglove_teleop/mapping.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(package))


def test_contract_matches_the_vendored_producer():
    mapping = _vendored_mapping()
    assert tuple(mapping.JOINT_NAMES) == SOURCE_ORDER
    assert dict(zip(mapping.JOINT_NAMES, mapping.JOINT_LIMITS_RAD)) == CLOSED
    # The synthetic glove sends what the producer subscribes to.
    for side, prefix in (("right", "r_"), ("left", "l_")):
        assert set(mapping.source_joint_names(prefix)) <= set(synth_glove.joint_names(side))


@pytest.mark.parametrize("side", ["right", "left"])
def test_synthetic_glove_through_the_producer_and_adapter(profile, order, side):
    mapping = _vendored_mapping()
    calibration = synth_glove.load_calibration(None)
    closures = {"index": 0.2, "middle": 0.4, "ring": 0.6, "pinky": 0.8,
                "thumb_pitch": 0.3, "thumb_yaw": 0.7}
    names = synth_glove.joint_names(side)
    glove = dict(zip(names, synth_glove.positions(side, closures, calibration)))
    targets = mapping.direct_map(glove, "r_" if side == "right" else "l_", calibration)
    contract = _contract(profile, order, side, thumb_pitch="follow", thumb_yaw="follow")
    mapped = map_target(contract, mapping.JOINT_NAMES, targets)
    expected = {"index_1": 0.2, "middle_1": 0.4, "ring_1": 0.6, "pinky_1": 0.8,
                "thumb_2": 0.3, "thumb_1": 0.7}
    for actuator, closure in expected.items():
        assert mapped.target[contract.index(actuator)] == pytest.approx(
            closure * _upper(contract, actuator), abs=1e-9)
    assert synth_glove.glove_topic("SYNTH", side) == (
        f"/senseglove/gloveSYNTH/{'rh' if side == 'right' else 'lh'}/joint_states")


def test_synthetic_scenarios():
    def at(scenario, t, closure=0.8):
        return synth_glove.closures_for(scenario, t, closure, 4.0, {})
    assert set(at("open", 1.0).values()) == {0.0} and set(at("close", 1.0).values()) == {1.0}
    assert set(at("hold", 1.0).values()) == {0.8}
    assert at("wave", 0.0)["index"] == pytest.approx(0.0) and at("wave", 2.0)["ring"] == 1.0
    # independent: one finger closed at a time, ring and pinky never together
    for t, finger in ((1.0, "index"), (5.0, "middle"), (9.0, "ring"), (13.0, "pinky"),
                      (17.0, "index")):
        closures = at("independent", t)
        assert closures[finger] == 0.8
        assert all(v == 0.0 for k, v in closures.items() if k != finger)
    assert synth_glove.closures_for("hold", 0.0, 0.5, 4.0, {"ring": 0.1})["ring"] == 0.1


def test_vendored_snapshot_is_unmodified():
    metadata = yaml.safe_load(METADATA.read_text())
    assert metadata["local_modifications"] == []
    present = {p.relative_to(VENDORED).as_posix() for p in VENDORED.rglob("*") if p.is_file()
               and "__pycache__" not in p.parts}
    assert present == set(metadata["files"])
    for relative, digest in metadata["files"].items():
        data = (VENDORED / relative).read_bytes()
        assert hashlib.sha256(data).hexdigest() == digest
        original = ORIGINAL / relative
        if original.is_file():
            assert original.read_bytes() == data
    for absent in ("hand_bridge_node.py", "hand_protocol.py", "package.xml", "setup.py"):
        assert not list(VENDORED.rglob(absent)), absent


def test_adapter_package_never_touches_serial_or_the_arm():
    package = ROOT / "src/rh56f1_glove_teleop"
    for path in package.glob("*.py"):
        text = path.read_text()
        assert "import serial" not in text and "_aj_" not in text, path.name


# ----------------------------------------------------------------- launch
def _launch_actions(**values):
    pytest.importorskip("launch")
    from launch import LaunchContext

    spec = importlib.util.spec_from_file_location("glove_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    defaults = {"side": "right", "glove_serial": "SYNTH", "execute": "false",
                "thumb_pitch": "", "thumb_yaw": "", "retarget_source": "",
                "retarget_params": "", "robot_control_src": str(ROOT / "src")}
    defaults["retarget_source"] = str(VENDORED / "senseglove_teleop")
    defaults.update(values)
    context.launch_configurations.update(defaults)
    return module._setup(context)


def _cmd(action):
    return [c if isinstance(c, str) else "".join(p.text for p in c) for c in action.cmd]


def test_glove_launch_runs_only_retarget_and_adapter():
    actions = _launch_actions()
    commands = [" ".join(_cmd(a)) for a in actions]
    assert len(commands) == 2
    assert "senseglove_teleop.retarget_node" in commands[0]
    assert "input_topic:=/senseglove/gloveSYNTH/rh/joint_states" in commands[0]
    assert "rh56f1_glove_teleop.ros_adapter --side right" in commands[1]
    assert "--execute" not in commands[1]
    joined = " ".join(commands)
    for forbidden in ("hand_bridge", "serial_port", "haptics", "emergency_open", "/dev/"):
        assert forbidden not in joined
    executing = " ".join(_cmd(_launch_actions(execute="true", side="left",
                                              thumb_pitch="follow")[1]))
    assert "--side left" in executing and "--execute" in executing
    assert "--thumb-pitch follow" in executing
    with pytest.raises(ValueError, match="glove_serial is required"):
        _launch_actions(glove_serial="")


def test_execute_needs_this_hand_on_fake_hardware(profile, order):
    contract = _contract(profile, order)
    fake = "mock_components/GenericSystem"
    commands = [f"{n}/position" for n in contract.canonical]
    good = dict(classes=[fake], commands=commands, controller_state="active",
                controller_joints=list(contract.canonical))

    def refusal(**changes):
        return fake_hand_refusal(contract, "/rh56f1_right/controller_manager",
                                 "right_hand_trajectory_controller", fake, **{**good, **changes})

    assert refusal() is None
    assert "does not answer" in refusal(classes=None)
    assert "not only" in refusal(classes=["rh56f1_hardware/Rh56f1HW"])
    assert "not only" in refusal(classes=[fake, "rh56f1_hardware/Rh56f1HW"])
    assert "not only" in refusal(classes=[])
    assert "six positions" in refusal(commands=commands + ["r_aj_1/position"])
    assert "six positions" in refusal(commands=None)
    assert "inactive" in refusal(controller_state="inactive")
    assert "not loaded" in refusal(controller_state=None)
    assert "drives" in refusal(controller_joints=list(reversed(contract.canonical)))

