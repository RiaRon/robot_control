"""Synthetic and replayed Quest packets, for testing without a headset.

Sends the same UDP JSON datagrams the Quest app sends, so the whole input
path (receiver, validation, frame conversion, ROS topics, teleop) is
exercised. Needs no ROS and commands nothing by itself.

    # a still right controller, grip released: check the receive path
    python3 -m openarm_quest_teleop.synth --scenario still --seconds 5
    # press grip, trace 5 cm along forward, left, up and back, release
    python3 -m openarm_quest_teleop.synth --scenario axes
    # send a recording made with `monitor --record`
    python3 -m openarm_quest_teleop.synth --replay /tmp/quest.jsonl

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


SCENARIOS = {"still": _scenario_still, "axes": _scenario_axes}


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
    parser.add_argument("--config", help="quest_teleop.yaml (default: the packaged one)")
    parser.add_argument("--host", default="127.0.0.1", help="where the receiver listens")
    parser.add_argument("--port", type=int)
    parser.add_argument("--rate", type=float, default=90.0)
    parser.add_argument("--side", choices=SIDES, default="right")
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="still")
    parser.add_argument("--seconds", type=float, default=5.0, help="for --scenario still")
    parser.add_argument("--replay", help="JSON-lines file from `monitor --record`")
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    args = parser.parse_args(argv)
    port = args.port or int(load_config(args.config)["quest"]["udp"]["port"])
    sender = QuestSender(args.host, port, args.rate)
    try:
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
