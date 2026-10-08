"""Quest arm teleoperation: packet, mapping, IK and clutch rules, without ROS."""

import copy
import json
import math
from pathlib import Path
import socket
import time
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from openarm_quest_teleop import packet as pk
from openarm_quest_teleop.config import (
    FAKE_PLUGIN,
    ConfigError,
    _SignedChain,
    bind_arm,
    build_chain,
    build_teleop,
    controller_for,
    execution_refusal,
    load_config,
    load_profile_for,
)
from openarm_quest_teleop.ik import IkSettings
from openarm_quest_teleop.relative import MappingError, RelativeTargetMapper
from openarm_quest_teleop.synth import QuestSender, axis_rotation
from openarm_quest_teleop.teleop import (
    ENGAGED,
    IDLE,
    LOCKED,
    ControllerSample,
    JointSample,
    TeleopSettings,
)
from openarm_quest_teleop.udp_receiver import JsonUdpReceiver, parse_datagram

ROOT = Path(__file__).resolve().parents[1]
KUKU_LAB = ROOT.parent
CANONICAL = KUKU_LAB / "urdf/generated/rl/openarm_rh56f1_bi_rl.urdf"
FAKE_CONTROLLERS = (
    ROOT / "ros_ws/src/openarm_ros2/openarm_bringup/config/controllers/"
    "openarm_rh56f1_fake_controllers.yaml"
)
UPSTREAM = KUKU_LAB / "third_party/dora-openarm-vr/src/dora_openarm_vr"
PACKAGE = ROOT / "src/openarm_quest_teleop"

#: A bent, non-singular right-arm pose near the fake start (all zeros is the
#: arm hanging straight down, which is singular).
BENT = {"right": np.array([0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0]),
        "left": np.array([-0.3, -0.15, 0.0, 1.2, 0.0, 0.0, 0.0])}
PERIOD = 0.01


# ------------------------------------------------------------------ fixtures
def _description(naming: str = "canonical", plugin: str = FAKE_PLUGIN) -> str:
    """The canonical URDF plus a ros2_control block, as a bringup renders it."""
    root = ET.parse(CANONICAL).getroot()
    profile = load_profile_for(load_config())
    arm = [j for j in profile.joints if "_aj_" in j.canonical]
    rename = {j.canonical: j.source for j in arm} if naming == "source" else {}
    for joint in root.findall("joint"):
        joint.set("name", rename.get(joint.get("name"), joint.get("name")))
    for side in ("r", "l"):
        block = ET.SubElement(root, "ros2_control", name=f"{side}_arm", type="system")
        hardware = ET.SubElement(block, "hardware")
        ET.SubElement(hardware, "plugin").text = plugin
        for joint in arm:
            if joint.canonical.startswith(side):
                ET.SubElement(block, "joint", name=rename.get(joint.canonical, joint.canonical))
    return ET.tostring(root, encoding="unicode")


@pytest.fixture(scope="module")
def config():
    return load_config()


@pytest.fixture(scope="module")
def profile(config):
    return load_profile_for(config)


@pytest.fixture(scope="module")
def urdf():
    return _description()


def _teleop(urdf, profile, config, arm="right", **overrides):
    config = copy.deepcopy(config)
    for key, value in overrides.items():
        config["teleop"][key] = value
    binding = bind_arm(urdf, profile, config, arm)
    pytest.importorskip("pink")  # the teleop IK; robot_control/.venv
    return build_teleop(urdf, profile, config, binding), binding


def _pose(position=(0.3, -0.2, 1.0), rotation=None):
    matrix = np.eye(4)
    matrix[:3, 3] = position
    if rotation is not None:
        matrix[:3, :3] = rotation
    return matrix


class Rig:
    """Drives the core against a perfectly tracking fake arm."""

    def __init__(self, core, q):
        self.core = core
        self.q = np.array(q, dtype=float)
        self.now = 0.0
        self.pose = _pose()
        self.commands = []

    def step(self, grip, pose="current", input_age=0.0, joint_age=0.0, q=None, **sample):
        self.now += PERIOD
        controller = ControllerSample(
            arrival_sec=self.now - input_age,
            pose=self.pose if isinstance(pose, str) else pose,
            grip=grip, **sample)
        joints = JointSample(self.now - joint_age, self.q if q is None else q)
        result = self.core.step(self.now, controller, joints)
        if result.command is not None:
            self.commands.append(result.command)
            self.q = result.command.copy()
        return result

    def engage(self):
        self.step(0.0)
        result = self.step(1.0)
        assert result.state == ENGAGED and result.just_engaged
        return result

    def move(self, delta, steps=50, rotation=None):
        start = self.pose.copy()
        result = None
        for index in range(1, steps + 1):
            self.pose = start.copy()
            self.pose[:3, 3] = start[:3, 3] + np.asarray(delta) * index / steps
            if rotation is not None:
                axis, angle = rotation
                self.pose[:3, :3] = axis_rotation(axis, angle * index / steps) @ start[:3, :3]
            result = self.step(1.0)
        return result

    def settle(self, steps=30):
        result = None
        for _ in range(steps):
            result = self.step(1.0)
        return result

    def palm(self):
        return self.core.chain.pose(self.q)


# -------------------------------------------------------------------- packet
#: Unity (x right, y up, z forward) coordinates -> quest_world (x forward, y
#: left, z up). Improper (determinant -1): it changes handedness.
UNITY_TO_ROS = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def _unity(x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0)):
    return {"x": x, "y": y, "z": z, "qx": q[0], "qy": q[1], "qz": q[2], "qw": q[3]}


def _packet(**overrides):
    message = {"t": 1.0, "v": 0, "vl": 0, "vr": 0, "lc": _unity(-0.2, 1.0, 0.3),
               "rc": _unity(0.2, 1.0, 0.3), "rf": _unity(0.0, 1.4, 0.0),
               "lt": 0.1, "rt": 0.2, "lg": 0.3, "rg": 0.9,
               "lsx": 0.0, "lsy": 0.0, "rsx": 0.5, "rsy": -0.5,
               "a": True, "b": False, "x": False, "y": True}
    message.update(overrides)
    return message


def test_position_axes_forward_left_up():
    assert pk.unity_to_ros_pose(_unity(z=1.0)).position == (1.0, 0.0, 0.0)   # forward
    assert pk.unity_to_ros_pose(_unity(x=-1.0)).position == (0.0, 1.0, 0.0)  # left
    assert pk.unity_to_ros_pose(_unity(y=1.0)).position == (0.0, 0.0, 1.0)   # up


def test_conversion_equals_upstream_flip_then_frame_rotation():
    """p = [x, y, -z], q = [-qx, -qy, qz, qw], then _FRAME_ROT (quest_receiver.py)."""
    frame_rot = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    rng = np.random.default_rng(0)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        p = rng.normal(size=3)
        ours = pk.unity_to_ros_pose(_unity(*p, q=q))
        flipped = pk.quaternion_matrix((-q[0], -q[1], q[2], q[3]))
        np.testing.assert_allclose(ours.position, frame_rot @ [p[0], p[1], -p[2]], atol=1e-12)
        np.testing.assert_allclose(
            pk.quaternion_matrix(ours.orientation), frame_rot @ flipped @ frame_rot.T,
            atol=1e-12)


def test_conversion_is_the_change_of_coordinates_from_first_principles():
    """R_ros = C R_unity C^T with C the (improper) axis relabelling."""
    rng = np.random.default_rng(1)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        ours = pk.quaternion_matrix(pk.unity_to_ros_pose(_unity(q=q)).orientation)
        np.testing.assert_allclose(
            ours, UNITY_TO_ROS @ pk.quaternion_matrix(q) @ UNITY_TO_ROS.T, atol=1e-12)


def test_unity_turn_to_the_right_is_negative_yaw_about_ros_up():
    angle = 0.4  # Unity is left-handed: +angle about +y (up) turns forward to the right.
    pose = pk.unity_to_ros_pose(_unity(q=(0.0, math.sin(angle / 2), 0.0, math.cos(angle / 2))))
    forward = pk.quaternion_matrix(pose.orientation) @ [1.0, 0.0, 0.0]
    np.testing.assert_allclose(forward, [math.cos(angle), -math.sin(angle), 0.0], atol=1e-12)


def test_ros_to_unity_is_the_inverse():
    rng = np.random.default_rng(2)
    for _ in range(10):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q) * (1.0 if q[3] >= 0 else -1.0)
        pose = pk.ControllerPose(tuple(rng.normal(size=3)), tuple(q))
        back = pk.unity_to_ros_pose(pk.ros_to_unity_pose(pose))
        np.testing.assert_allclose(back.position, pose.position, atol=1e-12)
        np.testing.assert_allclose(back.orientation, pose.orientation, atol=1e-12)


def test_packet_fields_and_buttons_per_controller():
    frame = pk.parse_packet(_packet())
    right, left = frame.controllers["right"], frame.controllers["left"]
    assert frame.headset_time == 1.0 and frame.reference is not None
    assert (right.grip, right.trigger, right.stick) == (0.9, 0.2, (0.5, -0.5))
    assert right.buttons == (True, False) and left.buttons == (False, True)
    assert right.pose_valid and left.pose_valid
    np.testing.assert_allclose(right.pose.position, (0.3, -0.2, 1.0))


def test_stale_and_invalid_poses_are_not_offered_but_inputs_still_read():
    stale = pk.parse_packet(_packet(v=1, vr=1))
    assert not stale.controllers["right"].pose_valid and stale.reference is None
    assert stale.controllers["right"].grip == 0.9
    one_side = pk.parse_packet(_packet(vr=2))
    assert not one_side.controllers["right"].pose_valid
    assert one_side.controllers["left"].pose_valid
    missing = pk.parse_packet({k: v for k, v in _packet().items() if k != "rc"})
    assert not missing.controllers["right"].pose_valid


def test_side_validity_is_independent_of_overall_validity():
    for overall in (1, 2):
        right = pk.parse_packet(_packet(v=overall, vl=2, vr=0))
        assert right.controllers["right"].pose_valid
        assert not right.controllers["left"].pose_valid
        assert right.reference is None

        left = pk.parse_packet(_packet(v=overall, vl=0, vr=2))
        assert left.controllers["left"].pose_valid
        assert not left.controllers["right"].pose_valid
        assert left.reference is None


def test_missing_side_validity_falls_back_to_overall():
    for overall in (0, 1, 2):
        message = _packet(v=overall)
        message.pop("vl", None)
        message.pop("vr", None)
        frame = pk.parse_packet(message)
        for side in ("left", "right"):
            assert frame.controllers[side].validity == overall
            assert frame.controllers[side].pose_valid == (overall == 0)


@pytest.mark.parametrize("broken, match", [
    (dict(rc=_unity(q=(0.0, 0.0, 0.0, 0.0))), "quaternion norm"),
    (dict(rc=_unity(q=(0.0, 0.0, 0.0, 0.5))), "quaternion norm"),
    (dict(rc={**_unity(), "x": float("nan")}), "not finite"),
    (dict(rc={**_unity(), "y": float("inf")}), "not finite"),
    (dict(rc={**_unity(), "z": "0.1"}), "not a number"),
    (dict(rc={**_unity(), "x": True}), "not a number"),
    (dict(rc={k: v for k, v in _unity().items() if k != "qw"}), "missing field"),
    (dict(rc=[1, 2, 3]), "not an object"),
    (dict(rg=float("nan")), "not finite"),
    (dict(vr=7), "validity code"),
    (dict(a="yes"), "button state"),
    (dict(t=float("inf")), "not finite"),
])
def test_broken_packets_are_refused(broken, match):
    with pytest.raises(pk.PacketError, match=match):
        pk.parse_packet(_packet(**broken))


@pytest.mark.parametrize("data", [
    b"", b"   ", b"not json", b"[1, 2, 3]", b"3.5", b'"text"', b"\xff\xfe\x00",
    b'{"t": NaN}', b'{"rc": {"x": Infinity}}', b'{"t": 1.0',
])
def test_malformed_datagrams_are_dropped(data):
    assert parse_datagram(data) is None


def test_matrix_quaternion_round_trip_all_branches():
    for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 1)):
        for angle in (0.0, 0.5, 2.0, 3.1, math.pi):
            rotation = axis_rotation(axis, angle)
            np.testing.assert_allclose(
                pk.quaternion_matrix(pk.matrix_quaternion(rotation)), rotation, atol=1e-9)


# ------------------------------------------------------------- UDP receiver
def _free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _wait(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_receiver_keeps_the_latest_packet_with_its_own_arrival_time():
    port = _free_port()
    seen = []
    receiver = JsonUdpReceiver("127.0.0.1", port, on_packet=seen.append)
    sender = QuestSender("127.0.0.1", port)
    try:
        assert receiver.latest() is None
        before = time.time_ns()
        assert _wait(lambda: (sender.send(), receiver.latest() is not None)[1])
        first = receiver.latest()
        assert before <= first.recv_ns <= time.time_ns()
        sender.send_raw(b"garbage")
        sender.send_raw(b'{"t": NaN}')
        sender.grip["right"] = 1.0
        sender.send()
        assert _wait(lambda: receiver.latest().message.get("rg") == 1.0)
        latest = receiver.latest()
        accepted, malformed = receiver.counts()
        assert latest.sequence == accepted > first.sequence and malformed == 2
        assert latest.recv_ns >= first.recv_ns and len(seen) == accepted
        assert pk.parse_packet(latest.message).controllers["right"].grip == 1.0
    finally:
        sender.close()
        receiver.close()


def test_receiver_goes_quiet_rather_than_repeating():
    port = _free_port()
    receiver = JsonUdpReceiver("127.0.0.1", port)
    sender = QuestSender("127.0.0.1", port)
    try:
        assert _wait(lambda: (sender.send(), receiver.latest() is not None)[1])
        time.sleep(0.1)
        sequence = receiver.latest().sequence
        time.sleep(0.3)
        assert receiver.latest().sequence == sequence
    finally:
        sender.close()
        receiver.close()


# ------------------------------------------------------------ relative target
def _robot_start():
    return _pose((0.44, -0.19, 0.45), axis_rotation((0.3, 1.0, -0.2), 1.1))


def test_target_at_enable_is_the_robot_start_pose_exactly():
    mapper = RelativeTargetMapper(orientation_mode="relative")
    controller = _pose((0.3, -0.2, 1.0), axis_rotation((1, 2, 3), 0.7))
    mapper.engage(controller, _robot_start())
    target, clamped = mapper.target(controller)
    np.testing.assert_allclose(target, _robot_start(), atol=1e-15)
    assert not clamped


@pytest.mark.parametrize("delta", [(0.05, 0, 0), (-0.05, 0, 0), (0, 0.05, 0),
                                   (0, -0.05, 0), (0, 0, 0.05), (0, 0, -0.05)])
def test_identity_mapping_moves_the_palm_the_way_the_controller_moved(delta):
    mapper = RelativeTargetMapper()
    controller = _pose()
    mapper.engage(controller, _robot_start())
    moved = controller.copy()
    moved[:3, 3] += delta
    target, _ = mapper.target(moved)
    np.testing.assert_allclose(target[:3, 3] - _robot_start()[:3, 3], delta, atol=1e-12)
    np.testing.assert_array_equal(target[:3, :3], _robot_start()[:3, :3])


def test_scale_axis_mapping_and_yaw_offset():
    controller = _pose()
    moved = controller.copy()
    moved[:3, 3] += (0.1, 0.0, 0.0)

    scaled = RelativeTargetMapper(position_scale=0.5)
    scaled.engage(controller, _robot_start())
    np.testing.assert_allclose(
        scaled.target(moved)[0][:3, 3] - _robot_start()[:3, 3], (0.05, 0, 0), atol=1e-12)

    # Robot x is the controller's -x, robot y its -y: an operator facing the robot.
    facing = RelativeTargetMapper(axis_mapping=[[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
    facing.engage(controller, _robot_start())
    np.testing.assert_allclose(
        facing.target(moved)[0][:3, 3] - _robot_start()[:3, 3], (-0.1, 0, 0), atol=1e-12)

    # The operator's forward is quest_world +y (heading +90 deg): moving along
    # quest_world +x is then a move to the operator's right, robot -y.
    turned = RelativeTargetMapper(yaw_offset_rad=math.pi / 2)
    turned.engage(controller, _robot_start())
    np.testing.assert_allclose(
        turned.target(moved)[0][:3, 3] - _robot_start()[:3, 3], (0, -0.1, 0), atol=1e-12)


def test_reference_heading_makes_the_operators_forward_the_robots_forward():
    mapper = RelativeTargetMapper(heading_mode="reference")
    controller = _pose()
    with pytest.raises(MappingError, match="rf pose"):
        mapper.engage(controller, _robot_start(), None)
    with pytest.raises(MappingError, match="vertical"):
        mapper.engage(controller, _robot_start(), _pose(rotation=axis_rotation((0, 1, 0), 1.5)))
    heading = 0.7
    # A tilted reference: only its heading is used, so up stays up.
    reference = _pose(rotation=axis_rotation((0, 0, 1), heading) @ axis_rotation((0, 1, 0), 0.4))
    mapper.engage(controller, _robot_start(), reference)
    moved = controller.copy()
    moved[:3, 3] += 0.1 * np.array([math.cos(heading), math.sin(heading), 0.0])
    np.testing.assert_allclose(
        mapper.target(moved)[0][:3, 3] - _robot_start()[:3, 3], (0.1, 0, 0), atol=1e-12)
    moved[:3, 3] += (0, 0, 0.05)
    assert mapper.target(moved)[0][2, 3] - _robot_start()[2, 3] == pytest.approx(0.05)


def test_hold_mode_keeps_orientation_and_relative_mode_premultiplies_the_turn():
    controller = _pose(rotation=axis_rotation((1, 2, 3), 0.7))
    turn = axis_rotation((0, 0, 1), 0.3)  # a turn to the left about quest_world up
    turned = controller.copy()
    turned[:3, :3] = turn @ controller[:3, :3]

    hold = RelativeTargetMapper(orientation_mode="hold")
    hold.engage(controller, _robot_start())
    np.testing.assert_array_equal(hold.target(turned)[0][:3, :3], _robot_start()[:3, :3])

    relative = RelativeTargetMapper(orientation_mode="relative")
    relative.engage(controller, _robot_start())
    target = relative.target(turned)[0][:3, :3]
    # The same world-frame turn is applied to the palm: turn @ R_start, and
    # not R_start @ turn (which would turn about the palm's own z).
    np.testing.assert_allclose(target, turn @ _robot_start()[:3, :3], atol=1e-12)
    assert not np.allclose(target, _robot_start()[:3, :3] @ turn, atol=1e-3)

    # A turn about the controller's own axis is the world turn R_c L R_c^T.
    local = axis_rotation((1, 0, 0), 0.25)
    rolled = controller.copy()
    rolled[:3, :3] = controller[:3, :3] @ local
    expected = controller[:3, :3] @ local @ controller[:3, :3].T @ _robot_start()[:3, :3]
    np.testing.assert_allclose(relative.target(rolled)[0][:3, :3], expected, atol=1e-12)


def test_relative_rotation_goes_through_the_axis_mapping():
    facing = RelativeTargetMapper(
        axis_mapping=[[-1, 0, 0], [0, -1, 0], [0, 0, 1]], orientation_mode="relative")
    controller = _pose()
    facing.engage(controller, _robot_start())
    turned = controller.copy()
    turned[:3, :3] = axis_rotation((1, 0, 0), 0.3)  # about quest_world forward
    np.testing.assert_allclose(
        facing.target(turned)[0][:3, :3],
        axis_rotation((-1, 0, 0), 0.3) @ _robot_start()[:3, :3], atol=1e-12)


def test_workspace_clamp_and_reengage_without_a_jump():
    mapper = RelativeTargetMapper(max_offset_m=0.1)
    controller = _pose()
    mapper.engage(controller, _robot_start())
    far = controller.copy()
    far[:3, 3] += (0.3, 0.4, 0.0)
    target, clamped = mapper.target(far)
    assert clamped
    np.testing.assert_allclose(
        target[:3, 3] - _robot_start()[:3, 3], (0.06, 0.08, 0.0), atol=1e-12)

    mapper.release()
    with pytest.raises(MappingError):
        mapper.target(far)
    elsewhere = _pose((0.5, -0.3, 0.5))
    mapper.engage(far, elsewhere)  # new start: the controller is where it is now
    np.testing.assert_array_equal(mapper.target(far)[0], elsewhere)


@pytest.mark.parametrize("kwargs", [
    dict(axis_mapping=[[1, 0, 0], [0, 1, 0]]),
    dict(axis_mapping=[[1, 1, 0], [0, 1, 0], [0, 0, 1]]),
    dict(position_scale=0.0),
    dict(position_scale=float("nan")),
    dict(orientation_mode="follow"),
    dict(heading_mode="hmd"),
    dict(max_offset_m=0.0),
])
def test_mapper_refuses_bad_settings(kwargs):
    with pytest.raises(MappingError):
        RelativeTargetMapper(**kwargs)


# ------------------------------------------------------------ binding and IK
def test_binding_reads_naming_frames_and_plugin_from_the_description(profile, config):
    fake = bind_arm(_description(), profile, config, "right")
    assert fake.naming == "canonical" and fake.is_fake
    assert fake.runtime_names == tuple(f"r_aj_{i}" for i in range(1, 8))
    assert (fake.control_frame, fake.base_frame) == ("r_hl_palm_sensor", "body_root")

    real = bind_arm(_description("source", "openarm_hardware/OpenArmHW"), profile, config, "left")
    assert real.naming == "source" and not real.is_fake
    assert real.runtime_names == tuple(f"openarm_left_joint{i}" for i in range(1, 8))
    assert real.plugins == ("openarm_hardware/OpenArmHW",)
    assert real.control_frame == "l_hl_palm_sensor"

    positions = {name: float(i) for i, name in enumerate(real.runtime_names)}
    positions["rh56f1_left_left_thumb_1_joint"] = 9.0
    np.testing.assert_array_equal(real.to_canonical(positions), np.arange(7.0))
    del positions[real.runtime_names[3]]
    assert real.to_canonical(positions) is None

    with pytest.raises(ConfigError, match="unknown arm"):
        bind_arm(_description(), profile, config, "middle")
    without_hand = ET.fromstring(_description())
    for link in without_hand.findall("link"):
        if link.get("name") == "r_hl_palm_sensor":
            without_hand.remove(link)
    with pytest.raises(ConfigError, match="no link"):
        bind_arm(ET.tostring(without_hand, encoding="unicode"), profile, config, "right")


def test_real_hardware_needs_its_own_confirmation(profile, config):
    fake = bind_arm(_description(), profile, config, "right")
    real = bind_arm(_description("source", "openarm_hardware/OpenArmHW"), profile, config,
                    "right")
    assert execution_refusal(fake, "fake", False, False) is None
    assert execution_refusal(fake, "fake", True, False) is None
    assert execution_refusal(fake, "real", True, False) is None  # test doubles
    assert execution_refusal(real, "real", False, False) is None  # dry run only observes
    assert "real hardware" in execution_refusal(real, "real", True, False)
    assert execution_refusal(real, "real", True, True) is None
    for execute in (False, True):
        assert "use --runtime real" in execution_refusal(real, "fake", execute, True)
    assert execution_refusal(fake, "sim", True, True) is not None


def test_controllers_come_from_the_fake_bringup_and_the_profile(profile, config, urdf):
    fake_yaml = yaml.safe_load(FAKE_CONTROLLERS.read_text())
    for arm in ("right", "left"):
        binding = bind_arm(urdf, profile, config, arm)
        fake = controller_for(config, profile, binding, "fake")
        assert fake_yaml[fake]["ros__parameters"]["joints"] == list(binding.runtime_names)
        real = controller_for(config, profile, binding, "real")
        assert real == profile.groups[binding.group].controller == f"rh56f1_{arm}_arm_controller"
    with pytest.raises(ConfigError):
        controller_for(config, profile, binding, "sim")


@pytest.mark.parametrize("arm", ["right", "left"])
def test_chain_is_exactly_the_arms_seven_joints_to_its_palm(profile, config, urdf, arm):
    binding = bind_arm(urdf, profile, config, arm)
    chain = build_chain(urdf, binding)
    prefix = arm[0]
    assert [joint.name for joint in chain.joints] == [f"{prefix}_aj_{i}" for i in range(1, 8)]
    palm = chain.pose(np.zeros(7))[:3, 3]
    # Stage 2 status: body_root -> palm_sensor at zero.
    expected = (-0.001350455, -0.137557902, 0.090184013) if arm == "right" else (
        0.001350499, 0.137557797, 0.090184109)
    np.testing.assert_allclose(palm, expected, atol=1e-6)


def _arm_limits(profile, binding):
    by_name = {joint.canonical: joint for joint in profile.joints}
    joints = [by_name[name] for name in binding.canonical_names]
    return (np.array([j.lower for j in joints]), np.array([j.upper for j in joints]))


def _pink(urdf, profile, config):
    pytest.importorskip("pink")
    from openarm_quest_teleop.pink_ik import PinkIk

    binding = bind_arm(urdf, profile, config, "right")
    lower, upper = _arm_limits(profile, binding)
    velocity = np.full(len(lower), 2.0)
    solver = PinkIk(urdf, binding.runtime_names, binding.control_frame, sign=binding.sign,
                    lower=lower, upper=upper, velocity=velocity)
    return solver, build_chain(urdf, binding), lower, upper


def test_ik_reaches_a_nearby_pose_within_limits(profile, config, urdf):
    solver, chain, lower, upper = _pink(urdf, profile, config)
    goal_q = BENT["right"] + np.array([0.05, -0.04, 0.03, 0.06, -0.05, 0.04, 0.02])
    result = solver.solve(chain.pose(goal_q), BENT["right"], lower, upper)
    assert result.ok and result.reason is None
    assert result.position_error_m <= 1e-3 and result.rotation_error_rad <= 1e-2
    assert np.all(result.q >= lower) and np.all(result.q <= upper)
    reached = chain.pose(result.q)
    assert np.linalg.norm(reached[:3, 3] - chain.pose(goal_q)[:3, 3]) <= 1e-3


def test_ik_reports_unreachable_non_finite_and_jumps(profile, config, urdf):
    solver, chain, lower, upper = _pink(urdf, profile, config)
    seed = BENT["right"]

    far = chain.pose(seed)
    far[:3, 3] += (1.0, 0.0, 0.0)
    unreachable = solver.solve(far, seed, lower, upper)
    assert not unreachable.ok and "not reachable" in unreachable.reason
    assert np.all(unreachable.q >= lower) and np.all(unreachable.q <= upper)

    broken = chain.pose(seed)
    broken[0, 3] = float("nan")
    assert solver.solve(broken, seed, lower, upper).reason == "target is not finite"
    assert solver.solve(chain.pose(seed), seed * float("nan"), lower, upper).reason == (
        "seed is not finite")

    elsewhere = chain.pose(seed + np.array([0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    jump = solver.solve(elsewhere, seed, lower, upper, IkSettings(max_seed_distance_rad=0.2))
    assert not jump.ok and "refusing a jump" in jump.reason


def test_signed_chain_takes_canonical_values(profile, config, urdf):
    binding = bind_arm(urdf, profile, config, "right")
    chain = build_chain(urdf, binding)
    sign = np.array([1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0])
    signed = _SignedChain(chain, sign)
    q = BENT["right"]
    np.testing.assert_allclose(signed.pose(q), chain.pose(q * sign))
    step = 1e-6
    for index in range(7):
        bumped = q.copy()
        bumped[index] += step
        numeric = (signed.pose(bumped)[:3, 3] - signed.pose(q)[:3, 3]) / step
        np.testing.assert_allclose(signed.jacobian(q)[:3, index], numeric, atol=1e-5)


# -------------------------------------------------------------- teleop core
def test_enable_starts_at_the_current_pose_and_sends_the_measured_joints(
        profile, config, urdf):
    core, _ = _teleop(urdf, profile, config, orientation_mode="relative")
    rig = Rig(core, BENT["right"])
    start_palm = rig.palm()
    result = rig.engage()
    np.testing.assert_array_equal(result.command, BENT["right"])
    np.testing.assert_allclose(result.target, start_palm, atol=1e-12)
    assert result.position_error_m <= 1e-12 and result.limited is None  # FK rounding
    for _ in range(20):  # holding the controller still commands no motion
        np.testing.assert_array_equal(rig.step(1.0).command, BENT["right"])


def test_enable_needs_a_press_edge(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config)
    rig = Rig(core, BENT["right"])
    held = rig.step(1.0)  # held when the node started
    assert held.state == IDLE and held.command is None and "already held" in held.reason
    assert rig.step(0.5).state == IDLE          # between the thresholds: not a release
    assert rig.step(1.0).state == IDLE
    assert rig.step(0.2).state == IDLE          # released
    assert rig.step(0.69).state == IDLE         # below the press threshold
    assert rig.step(0.7).state == ENGAGED


@pytest.mark.parametrize("arm", ["right", "left"])
@pytest.mark.parametrize("delta", [(0.05, 0, 0), (-0.05, 0, 0), (0, 0.05, 0),
                                   (0, -0.05, 0), (0, 0, 0.05), (0, 0, -0.05)])
def test_palm_follows_each_direction_with_orientation_held(profile, config, urdf, arm, delta):
    core, binding = _teleop(urdf, profile, config, arm=arm)
    rig = Rig(core, BENT[arm])
    start = rig.palm()
    rig.engage()
    rig.move(delta, steps=100)
    result = rig.settle()
    assert result.state == ENGAGED and result.reason is None
    palm = rig.palm()
    np.testing.assert_allclose(palm[:3, 3] - start[:3, 3], delta, atol=1.5e-3)
    turn = math.acos(min(1.0, (np.trace(start[:3, :3].T @ palm[:3, :3]) - 1) / 2))
    assert turn <= 1.5e-2
    assert len(result.command) == 7 and len(binding.runtime_names) == 7


def test_relative_orientation_turns_the_palm_about_the_same_base_axis(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config, orientation_mode="relative")
    rig = Rig(core, BENT["right"])
    start = rig.palm()
    rig.engage()
    rig.move((0, 0, 0), steps=100, rotation=((0, 0, 1), 0.2))
    rig.settle()
    palm = rig.palm()
    expected = axis_rotation((0, 0, 1), 0.2) @ start[:3, :3]
    error = math.acos(min(1.0, (np.trace(expected.T @ palm[:3, :3]) - 1) / 2))
    assert error <= 1.5e-2
    np.testing.assert_allclose(palm[:3, 3], start[:3, 3], atol=1.5e-3)

    held, _ = _teleop(urdf, profile, config, orientation_mode="hold")
    rig = Rig(held, BENT["right"])
    start = rig.palm()
    rig.engage()
    rig.move((0, 0, 0), steps=100, rotation=((0, 0, 1), 0.2))
    np.testing.assert_allclose(rig.palm(), start, atol=1e-9)


def test_release_holds_and_reenable_starts_from_the_current_pose(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config)
    rig = Rig(core, BENT["right"])
    rig.engage()
    rig.move((0.03, 0, 0))
    rig.settle()
    held_q, sent = rig.q.copy(), len(rig.commands)

    released = rig.step(0.0)
    assert released.state == IDLE and released.command is None
    rig.pose[:3, 3] += (0.0, 0.08, 0.0)  # the hand moves away while released
    for _ in range(20):
        assert rig.step(0.0).command is None
    assert len(rig.commands) == sent
    np.testing.assert_array_equal(rig.q, held_q)

    again = rig.step(1.0)
    assert again.just_engaged
    np.testing.assert_array_equal(again.command, held_q)  # no jump to the old offset
    before = rig.palm()
    rig.move((0, 0, 0.02))
    rig.settle()
    np.testing.assert_allclose(rig.palm()[:3, 3] - before[:3, 3], (0, 0, 0.02), atol=1.5e-3)


def test_input_loss_locks_until_released_and_pressed_again(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config)
    rig = Rig(core, BENT["right"])
    rig.engage()
    rig.move((0.02, 0, 0))
    sent = len(rig.commands)

    lost = rig.step(1.0, input_age=0.5)
    assert lost.state == LOCKED and lost.command is None and "stale" in lost.reason
    rig.pose[:3, 3] += (0.05, 0, 0)
    for _ in range(10):  # input is back, grip still held: must not resume
        back = rig.step(1.0)
        assert back.state == LOCKED and back.command is None
        assert "release and press" in back.reason
    assert len(rig.commands) == sent
    assert rig.step(0.0).state == IDLE
    q_before = rig.q.copy()
    resumed = rig.step(1.0)
    assert resumed.just_engaged
    np.testing.assert_array_equal(resumed.command, q_before)

    assert core.step(rig.now + 1.0, None, JointSample(rig.now + 1.0, rig.q)).state == LOCKED


@pytest.mark.parametrize("fault, match", [
    (dict(pose=None), "pose is not valid"),
    (dict(pose=np.full((4, 4), np.nan)), "pose is not finite"),
    (dict(joint_age=0.5), "joint state is stale"),
    (dict(q=np.full(7, np.nan)), "joint state is not finite"),
    (dict(q=np.zeros(6)), "joint state is not finite"),
])
def test_faults_cut_following_and_send_nothing(profile, config, urdf, fault, match):
    core, _ = _teleop(urdf, profile, config)
    rig = Rig(core, BENT["right"])
    rig.engage()
    sent = len(rig.commands)
    result = rig.step(1.0, **fault)
    assert result.state == LOCKED and result.command is None and match in result.reason
    assert rig.step(1.0).state == LOCKED and len(rig.commands) == sent


def test_a_press_made_during_a_fault_does_not_start_following(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config)
    rig = Rig(core, BENT["right"])
    rig.step(0.0)
    assert rig.step(1.0, joint_age=0.5).reason.startswith("joint state is stale")
    waiting = rig.step(1.0)  # joint state is back, the press is spent
    assert waiting.state == IDLE and waiting.command is None
    assert "joint state is stale" in waiting.reason and "release and press" in waiting.reason
    rig.step(0.0)
    assert rig.step(1.0).state == ENGAGED


def test_missing_joint_state_never_engages(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config)
    sample = ControllerSample(arrival_sec=1.0, pose=_pose(), grip=0.0)
    assert core.step(1.0, sample, None).reason == "no joint state"
    pressed = ControllerSample(arrival_sec=1.01, pose=_pose(), grip=1.0)
    result = core.step(1.01, pressed, None)
    assert result.state == IDLE and result.command is None


def test_input_jump_is_a_fault(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config)
    rig = Rig(core, BENT["right"])
    rig.engage()
    rig.pose[:3, 3] += (0.0, 0.0, 0.25)
    result = rig.step(1.0)
    assert result.state == LOCKED and "jumped" in result.reason and result.command is None


def test_ik_failure_blocks_commands_and_recovers_when_reachable(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config, max_target_offset_m=None)
    rig = Rig(core, BENT["right"])
    rig.engage()
    result = rig.move((0.6, 0.0, 0.0), steps=40)  # well beyond the arm's reach
    assert result.state == ENGAGED and result.command is None
    assert result.reason.startswith("IK failed: target not reachable")
    sent, q_stopped = len(rig.commands), rig.q.copy()
    for _ in range(10):
        assert rig.step(1.0).command is None
    assert len(rig.commands) == sent
    np.testing.assert_array_equal(rig.q, q_stopped)

    rig.move((-0.55, 0.0, 0.0), steps=40)
    recovered = rig.settle(200)
    assert recovered.command is not None and recovered.reason is None


def test_fast_motion_is_rate_limited_and_reported(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config)
    rig = Rig(core, BENT["right"])
    rig.engage()
    previous = rig.q.copy()
    rig.pose[:3, 3] += (0.0, 0.0, 0.09)  # 9 cm in one 10 ms sample
    result = rig.step(1.0)
    assert result.state == ENGAGED and "velocity" in result.limited
    limits = {j.canonical: j.velocity for j in profile.joints}
    budget = np.array([limits[f"r_aj_{i}"] for i in range(1, 8)]) * PERIOD
    assert np.all(np.abs(result.command - previous) <= budget + 1e-12)


def test_workspace_limit_is_reported(profile, config, urdf):
    core, _ = _teleop(urdf, profile, config, max_target_offset_m=0.02)
    rig = Rig(core, BENT["right"])
    start = rig.palm()
    rig.engage()
    rig.move((0.0, 0.0, 0.08))
    result = rig.settle()
    assert "workspace limit" in result.limited
    assert np.linalg.norm(rig.palm()[:3, 3] - start[:3, 3]) <= 0.02 + 1.5e-3


def test_enable_source_and_thresholds_are_configurable(profile, config, urdf):
    core, _ = _teleop(
        urdf, profile, config,
        enable={"source": "button_primary", "press_threshold": 0.5, "release_threshold": 0.4})
    rig = Rig(core, BENT["right"])
    rig.step(1.0, buttons=(False, False))   # grip is ignored now
    assert rig.step(1.0, buttons=(True, False)).state == ENGAGED
    assert rig.step(1.0, buttons=(False, False)).state == IDLE

    trigger, _ = _teleop(
        urdf, profile, config,
        enable={"source": "trigger", "press_threshold": 0.9, "release_threshold": 0.1})
    rig = Rig(trigger, BENT["right"])
    rig.step(0.0, trigger=0.0)
    assert rig.step(1.0, trigger=0.8).state == IDLE
    assert rig.step(0.0, trigger=0.95).state == ENGAGED

    with pytest.raises(ValueError):
        TeleopSettings(press_threshold=0.3, release_threshold=0.5)
    with pytest.raises(ValueError):
        TeleopSettings(enable_source="thumbstick")


def test_straight_arm_engages_without_jump_and_refuses_unreachable_target(
        profile, config, urdf):
    """Default settings permit engagement without an initial pose jump."""
    core, _ = _teleop(urdf, profile, config)
    assert core.settings.min_enable_singular_value == 0.0
    rig = Rig(core, np.zeros(7))

    started = rig.engage()
    assert started.state == "engaged"
    np.testing.assert_array_equal(started.command, np.zeros(7))
    np.testing.assert_array_equal(rig.q, np.zeros(7))

    # A target below the fully extended arm remains unreachable.
    result = rig.move((0.0, 0.0, -0.05))
    assert result.command is None
    assert result.reason is not None and result.reason.startswith("IK failed")
    assert np.max(np.abs(rig.q)) < 1e-3


# ----------------------------------------------------------------- packaging
def test_synthetic_packets_survive_the_real_parser():
    sender = QuestSender("127.0.0.1", _free_port())
    try:
        sender.position["right"] = np.array([0.4, -0.1, 0.9])
        sender.rotation["right"] = axis_rotation((0, 0, 1), 0.3)
        sender.grip["right"] = 0.8
        frame = pk.parse_packet(json.loads(json.dumps(sender.packet())))
    finally:
        sender.close()
    right = frame.controllers["right"]
    np.testing.assert_allclose(right.pose.position, (0.4, -0.1, 0.9), atol=1e-12)
    np.testing.assert_allclose(
        pk.quaternion_matrix(right.pose.orientation), axis_rotation((0, 0, 1), 0.3), atol=1e-12)
    assert right.grip == 0.8 and frame.reference is not None


def test_smoothing_is_the_upstream_file_plus_a_notice():
    upstream = UPSTREAM / "smoothing.py"
    if not upstream.is_file():
        pytest.skip("third_party/dora-openarm-vr is not checked out")
    notice = (
        "#\n"
        "# Verbatim copy of dora-openarm-vr, src/dora_openarm_vr/smoothing.py, at commit\n"
        "# 072ce98d9c1d781e4b42626639c48f5c8f2ba8ee; only this notice was added\n"
        "# (KUKU Robot Lab, 2026-10-01). Optional, and off by default.\n"
    )
    ours = (PACKAGE / "smoothing.py").read_text()
    assert ours.count(notice) == 1
    assert ours.replace(notice, "") == upstream.read_text()


def test_no_hand_or_other_arm_name_is_written_in_the_teleop_package():
    for path in PACKAGE.glob("*.py"):
        text = path.read_text()
        assert "_hj_" not in text and "hand_controller" not in text, path.name
