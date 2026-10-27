"""Synthetic and replayed Quest packets, for testing without a headset.

With ``quest.source: quest_view`` it plays the headset page: WebXR pose
messages over quest_view's websocket (ws://127.0.0.1:8787/ws), so quest_view,
its TCP stream, the bridge and the teleop all run as with the headset.
With ``udp`` it sends the dora-openarm-vr app's UDP JSON datagrams. Needs no
ROS and commands nothing by itself.

    # a still right controller, grip released: check the receive path
    python3 -m openarm_quest_teleop.synth --scenario still --seconds 5
    # press grip, trace 5 cm along forward, left, up and back, release
    python3 -m openarm_quest_teleop.synth --scenario axes
    # send a recording made with `monitor --record` (udp only)
    python3 -m openarm_quest_teleop.synth --replay /tmp/quest.jsonl
    # quest_view: turn the headset (yaw, pitch) after setting the robot's front
    .venv/bin/python -m openarm_quest_teleop.synth --config $CFG --scenario head --set-front

Scenario poses are written in ``quest_world`` (x forward, y left, z up) and
converted to the app's Unity convention on the way out.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import sys
import time

import numpy as np

from .config import load_config
from .packet import SIDES, VALID_OK, ControllerPose, matrix_quaternion, ros_to_unity_pose

_POSE_KEY = {"left": "lc", "right": "rc"}
_VALID_KEY = {"left": "vl", "right": "vr"}
_TRIGGER_KEY = {"left": "lt", "right": "rt"}
_GRIP_KEY = {"left": "lg", "right": "rg"}
#: Where the synthetic controllers start, in quest_world: roughly a seated
#: operator's hands. Only motion relative to enable matters to the teleop.
_START = {"left": (0.30, 0.20, 1.00), "right": (0.30, -0.20, 1.00)}
_HMD_START = (0.0, 0.0, 1.40)


def axis_rotation(axis, angle: float) -> np.ndarray:
    """Rotation matrix about *axis* (quest_world) by *angle* radians."""
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


class QuestSender:
    """Holds both controllers' state and sends packets at a fixed rate."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5006, rate_hz: float = 90.0):
        self.address = (host, port)
        self.period = 1.0 / rate_hz
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.position = {side: np.array(_START[side], dtype=float) for side in SIDES}
        self.rotation = {side: np.eye(3) for side in SIDES}
        self.grip = {side: 0.0 for side in SIDES}
        self.trigger = {side: 0.0 for side in SIDES}
        self.validity = {side: VALID_OK for side in SIDES}
        self.overall_validity = VALID_OK
        self.reference_rotation = np.eye(3)
        self.headset_time = 100.0
        self.sent = 0

    def close(self) -> None:
        self.socket.close()

    def packet(self) -> dict:
        message = {"t": round(self.headset_time, 6), "v": self.overall_validity,
                   "a": False, "b": False, "x": False, "y": False,
                   "lsx": 0.0, "lsy": 0.0, "rsx": 0.0, "rsy": 0.0}
        for side in SIDES:
            pose = ControllerPose(tuple(self.position[side]),
                                  matrix_quaternion(self.rotation[side]))
            message[_POSE_KEY[side]] = ros_to_unity_pose(pose)
            message[_VALID_KEY[side]] = self.validity[side]
            message[_GRIP_KEY[side]] = self.grip[side]
            message[_TRIGGER_KEY[side]] = self.trigger[side]
        message["rf"] = ros_to_unity_pose(
            ControllerPose((0.0, 0.0, 1.4), matrix_quaternion(self.reference_rotation)))
        return message

    def send_raw(self, data: bytes) -> None:
        self.socket.sendto(data, self.address)

    def send(self) -> None:
        self.send_raw(json.dumps(self.packet()).encode("utf-8"))
        self.headset_time += self.period
        self.sent += 1

    def hold(self, seconds: float) -> None:
        """Send the current state for *seconds*."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send()
            time.sleep(self.period)

    def silence(self, seconds: float) -> None:
        """Send nothing, as when the Quest drops off the network."""
        time.sleep(seconds)
        self.headset_time += seconds

    def move(self, side: str, delta=(0.0, 0.0, 0.0), seconds: float = 1.0,
             axis=None, angle: float = 0.0) -> None:
        """Translate by *delta* and turn by *angle* about *axis*, linearly."""
        start_position = self.position[side].copy()
        start_rotation = self.rotation[side].copy()
        steps = max(1, int(round(seconds / self.period)))
        for step in range(1, steps + 1):
            fraction = step / steps
            self.position[side] = start_position + fraction * np.asarray(delta, dtype=float)
            if axis is not None:
                self.rotation[side] = axis_rotation(axis, fraction * angle) @ start_rotation
            self.send()
            time.sleep(self.period)


def ros_to_webxr(position, orientation) -> tuple[list[float], list[float]]:
    """quest_world (x forward, y left, z up) -> WebXR (x right, y up, z backward).

    The inverse of quest_view's WebXR -> Unity mirror followed by
    :func:`packet.unity_to_ros_pose`."""
    unity = ros_to_unity_pose(ControllerPose(tuple(position), tuple(orientation)))
    return ([unity["x"], unity["y"], -unity["z"]],
            [-unity["qx"], -unity["qy"], unity["qz"], unity["qw"]])


class QuestViewSender(QuestSender):
    """The headset page: WebXR pose messages to quest_view's websocket."""

    def __init__(self, url: str = "ws://127.0.0.1:8787/ws", rate_hz: float = 72.0,
                 front_url: str | None = None):
        from websockets.sync.client import connect

        super().__init__(rate_hz=rate_hz)
        self.socket.close()
        self._connection = connect(url, open_timeout=5.0)
        self.socket = self._connection.__enter__()
        self.front_url = front_url
        self.hmd_position = np.array(_HMD_START, dtype=float)
        self.hmd_rotation = np.eye(3)
        self.buttons = {side: [False, False] for side in SIDES}

    def close(self) -> None:
        self._connection.__exit__(None, None, None)

    def packet(self) -> dict:
        message = {"type": "pose", "dt": self.period, "session": "synth", "session_cause": "start",
                   "hmd": dict(zip(("p", "q"), ros_to_webxr(
                       self.hmd_position, matrix_quaternion(self.hmd_rotation))))}
        for side in SIDES:
            if self.validity[side] != VALID_OK:
                message[side] = None
                continue
            p, q = ros_to_webxr(self.position[side], matrix_quaternion(self.rotation[side]))
            message[side] = {"p": p, "q": q, "axes": [0.0, 0.0, 0.0, 0.0],
                             "buttons": [self.trigger[side] > 0.5, self.grip[side] > 0.5, False, False,
                                         *self.buttons[side]]}
        return message

    def send_raw(self, data: bytes) -> None:
        self.socket.send(data.decode("utf-8"))

    def turn_head(self, axis, angle: float, seconds: float = 1.0) -> None:
        start = self.hmd_rotation.copy()
        steps = max(1, int(round(seconds / self.period)))
        for step in range(1, steps + 1):
            self.hmd_rotation = axis_rotation(axis, step / steps * angle) @ start
            self.send()
            time.sleep(self.period)

    def set_front(self) -> str:
        """Hold the headset still, then ask quest_view to keep its pose as the robot's front."""
        import urllib.request

        self.hold(1.5)
        request = urllib.request.Request(self.front_url, data=b"", method="POST")
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return json.loads(response.read())["text"]


def _scenario_still(sender: QuestSender, side: str, seconds: float) -> None:
    sender.hold(seconds)


def _scenario_axes(sender: QuestSender, side: str, seconds: float) -> None:
    sender.hold(0.5)
    sender.grip[side] = 1.0
    sender.hold(0.5)
    for delta in ((0.05, 0, 0), (-0.05, 0, 0), (0, 0.05, 0), (0, -0.05, 0),
                  (0, 0, 0.05), (0, 0, -0.05)):
        sender.move(side, delta, seconds=1.0)
        sender.hold(0.5)
    sender.grip[side] = 0.0
    sender.hold(0.5)


def _scenario_head(sender: QuestSender, side: str, seconds: float) -> None:
    """Headset: 20 deg left and back, 10 deg up and back (quest_view only)."""
    sender.hold(0.5)
    for axis, angle in (((0, 0, 1), math.radians(20)), ((0, 0, 1), -math.radians(20)),
                        ((0, 1, 0), -math.radians(10)), ((0, 1, 0), math.radians(10))):
        sender.turn_head(axis, angle, seconds=1.0)
        sender.hold(0.5)


SCENARIOS = {"still": _scenario_still, "axes": _scenario_axes, "head": _scenario_head}


def replay(sender: QuestSender, path: str, speed: float = 1.0) -> int:
    """Send a ``monitor --record`` file again with its original spacing."""
    previous = None
    count = 0
    with open(path, encoding="utf-8") as lines:
        for line in lines:
            if not line.strip():
                continue
            entry = json.loads(line)
            if previous is not None:
                time.sleep(max(0.0, (entry["recv_ns"] - previous) * 1e-9 / speed))
            previous = entry["recv_ns"]
            sender.send_raw(json.dumps(entry["packet"]).encode("utf-8"))
            count += 1
    return count


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Synthetic or replayed Quest packets")
    parser.add_argument("--config", help="teleop config (default: config/leap_hand/quest_teleop_leap.yaml; RH56F1: config/rh56f1/quest_teleop.yaml)")
    parser.add_argument("--host", default="127.0.0.1", help="where the receiver listens")
    parser.add_argument("--port", type=int)
    parser.add_argument("--rate", type=float, help="messages per second (udp 90, quest_view 72)")
    parser.add_argument("--side", choices=SIDES, default="right")
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="still")
    parser.add_argument("--seconds", type=float, default=5.0, help="for --scenario still")
    parser.add_argument("--replay", help="JSON-lines file from `monitor --record`")
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    parser.add_argument("--set-front", action="store_true",
                        help="quest_view: set the robot's front from the still headset first")
    args = parser.parse_args(argv)
    quest = load_config(args.config)["quest"]
    if quest.get("source", "udp") == "quest_view":
        port = args.port or int(quest["quest_view"]["page_port"])
        if args.replay:
            parser.error("--replay sends UDP packets; it needs quest.source: udp")
        sender = QuestViewSender(f"ws://{args.host}:{port}/ws", args.rate or 72.0,
                                 front_url=f"http://{args.host}:{port}/front")
    else:
        port = args.port or int(quest["udp"]["port"])
        if args.scenario == "head" or args.set_front:
            parser.error("--scenario head and --set-front need quest.source: quest_view")
        sender = QuestSender(args.host, port, args.rate or 90.0)
    try:
        if args.set_front:
            print(f"front: {sender.set_front()}")
        if args.replay:
            print(f"replayed {replay(sender, args.replay, args.speed)} packets")
        else:
            SCENARIOS[args.scenario](sender, args.side, args.seconds)
            print(f"sent {sender.sent} packets to {args.host}:{port}")
    except KeyboardInterrupt:
        pass
    finally:
        sender.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
