"""quest_view as the Quest input: page message -> quest_view frame -> bridge (no ROS)."""

from __future__ import annotations

import json
import math
import socket
import threading
import time

import numpy as np
import pytest

from openarm_quest_teleop import packet as pk
from openarm_quest_teleop.config import RH56F1_CONFIG, load_config
from openarm_quest_teleop.quest_view_receiver import QuestViewReceiver, open_source, parser_for
from openarm_quest_teleop.synth import axis_rotation, ros_to_webxr
from openarm_quest_view.convert import handumi_frame
from openarm_quest_view.front import FrontKeeper

from test_quest_teleop_leap_pink import LEAP_CONFIG


def _page(position=(0.3, -0.2, 1.0), rotation=np.eye(3), grip=True, hmd=True):
    p, q = ros_to_webxr(position, pk.matrix_quaternion(rotation))
    message = {"dt": 0.014, "session": "s1",
               "right": {"p": p, "q": q, "axes": [0.0, 0.0, 0.5, -0.25],
                         "buttons": [False, grip, False, False, True, False]},
               "left": None}
    if hmd:
        message["hmd"] = dict(zip(("p", "q"), ros_to_webxr((0.0, 0.0, 1.4), (0, 0, 0, 1))))
    return message


def test_a_page_pose_reaches_the_bridge_unchanged_in_quest_world():
    rotation = axis_rotation((0, 0, 1), math.radians(30)) @ axis_rotation((0, 1, 0), 0.2)
    frame = pk.parse_handumi(json.loads(json.dumps(handumi_frame(_page(rotation=rotation)))))
    right = frame.controllers["right"]
    assert np.allclose(right.pose.position, (0.3, -0.2, 1.0))
    assert np.allclose(pk.quaternion_matrix(right.pose.orientation), rotation, atol=1e-9)
    assert right.grip == 1.0 and right.trigger == 0.0 and right.buttons == (True, False)
    assert right.stick == pytest.approx((0.5, 0.25))  # WebXR y is down, the stick reads up
    assert frame.controllers["left"].pose is None
    assert frame.controllers["left"].validity == pk.VALID_INVALID
    assert np.allclose(frame.hmd.position, (0.0, 0.0, 1.4))
    assert frame.reference is None


def test_the_front_becomes_the_reference_pose():
    keeper = FrontKeeper(clock=iter(np.arange(0, 10, 0.02)).__next__)
    for _ in range(60):
        keeper.observe(_page())
    assert keeper.set()[0]
    frame = handumi_frame(_page())
    frame["front"] = keeper.wire()
    reference = pk.parse_handumi(frame).reference
    assert np.allclose(reference.position, (0.0, 0.0, 1.4))


def test_without_the_headset_pose_the_frame_is_not_ok():
    frame = pk.parse_handumi(handumi_frame(_page(hmd=False)))
    assert frame.hmd is None and frame.validity == pk.VALID_INVALID
    assert frame.controllers["right"].pose is not None  # the controllers still count


@pytest.mark.parametrize("broken", [{"ovrTimeNs": "x"}, {"rightGripPressed": 2},
                                    {"rightControllerRotation": {"x": 0, "y": 0, "z": 0, "w": 0.5}}])
def test_a_broken_frame_is_refused(broken):
    frame = {**handumi_frame(_page()), **broken}
    with pytest.raises(pk.PacketError):
        pk.parse_handumi(frame)


def test_the_receiver_reads_lines_and_reconnects():
    server = socket.socket()
    server.bind(("127.0.0.1", 0))
    server.listen()
    port = server.getsockname()[1]
    seen = []
    receiver = QuestViewReceiver("127.0.0.1", port, on_packet=seen.append)
    try:
        for n in (1, 2):  # second connection: quest_view restarted
            conn, _ = server.accept()
            line = json.dumps(handumi_frame(_page())).encode()
            conn.sendall(line[:40])  # a frame split across reads
            time.sleep(0.05)
            conn.sendall(line[40:] + b"\nnot json\n")
            deadline = time.monotonic() + 2.0
            while receiver.counts() != (n, n) and time.monotonic() < deadline:
                time.sleep(0.01)
            assert receiver.counts() == (n, n)
            conn.close()
        assert pk.parse_handumi(receiver.latest().message).controllers["right"].grip == 1.0
        assert len(seen) == 2
    finally:
        receiver.close()
        server.close()


def test_the_leap_config_reads_quest_view_and_the_rh56f1_config_keeps_udp():
    leap = load_config(LEAP_CONFIG)["quest"]
    assert leap["source"] == "quest_view" and parser_for(leap) is pk.parse_handumi
    rh56f1 = load_config(RH56F1_CONFIG)["quest"]
    assert parser_for(rh56f1) is pk.parse_packet
    with pytest.raises(ValueError, match="quest.source"):
        parser_for({"source": "bluetooth"})
    receiver, parse, label = open_source(leap, port=1)  # nothing listens on port 1
    try:
        assert parse is pk.parse_handumi and label == "quest_view TCP 127.0.0.1:1"
        deadline = time.monotonic() + 2.0
        while receiver.bind_error() is None and time.monotonic() < deadline:
            time.sleep(0.01)
        assert receiver.bind_error()
    finally:
        receiver.close()


def test_neck_joint_states_follow_the_model_axes():
    pytest.importorskip("yaml")
    from openarm_head.config import load_head_config

    config = load_head_config(LEAP_CONFIG, allow_fake_default=False)
    section = load_config(LEAP_CONFIG)["head"]["joint_states"]

    class Source:  # what JointStatePublisher needs from RosHmdSource
        node = None

    from openarm_head import ros_head

    publisher = ros_head.JointStatePublisher.__new__(ros_head.JointStatePublisher)
    publisher.home = (config.retarget.pan.home_deg, config.retarget.tilt.home_deg)
    publisher.signs = tuple(float(section[k]["sign"]) * a.sign
                            for k, a in zip(("pan", "tilt"), (config.retarget.pan, config.retarget.tilt)))
    # operator looks 10 deg left and 5 deg up: head_j_pan (about -z) and head_j_tilt (about +y) go negative
    pan = publisher.home[0] + config.retarget.pan.sign * 10.0
    tilt = publisher.home[1] + config.retarget.tilt.sign * 5.0
    angles = [s * math.radians(m - h) for s, m, h in zip(publisher.signs, (pan, tilt), publisher.home)]
    assert angles == pytest.approx([-math.radians(10.0), -math.radians(5.0)])


def test_the_page_server_relays_a_page_message_to_the_pose_stream():
    pytest.importorskip("aiohttp")
    pytest.importorskip("websockets")
    import asyncio

    from openarm_quest_view import server as qv

    def free_port():
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]

    page, poses = free_port(), free_port()
    args = qv.parse_args(["--port", str(page), "--tcp-port", str(poses), "--sync-port", str(free_port()),
                          "--frame-port", str(free_port()), "--head-status-port", str(free_port()),
                          "--camera", "-1"])
    loop = asyncio.new_event_loop()
    task = loop.create_task(qv.serve(args))
    thread = threading.Thread(target=loop.run_forever, daemon=True)
    thread.start()
    receiver = None
    try:
        from websockets.sync.client import connect

        deadline = time.monotonic() + 5.0
        while True:
            try:
                with connect(f"ws://127.0.0.1:{page}/ws", open_timeout=1.0):
                    break
            except OSError:
                assert time.monotonic() < deadline
                time.sleep(0.05)
        receiver = QuestViewReceiver("127.0.0.1", poses)
        with connect(f"ws://127.0.0.1:{page}/ws", open_timeout=2.0) as ws:
            deadline = time.monotonic() + 3.0
            while receiver.latest() is None and time.monotonic() < deadline:
                ws.send(json.dumps({"type": "pose", **_page()}))
                time.sleep(0.02)
        frame = pk.parse_handumi(receiver.latest().message)
        assert np.allclose(frame.controllers["right"].pose.position, (0.3, -0.2, 1.0))
        assert "front" in receiver.latest().message  # quest_view always says whether a front is set
    finally:
        if receiver is not None:
            receiver.close()
        loop.call_soon_threadsafe(task.cancel)
        time.sleep(0.2)
        loop.call_soon_threadsafe(loop.stop)
        thread.join(timeout=2.0)
