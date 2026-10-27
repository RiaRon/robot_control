"""Receive-only Quest diagnostic. Needs no ROS and commands nothing.

Prints, a few times a second, both controllers' pose (``quest_world``: x
forward, y left, z up; quaternion x y z w), grip, trigger, stick and buttons,
pose validity, when this PC last received a packet and how long ago, and the
receive rate. The input is the config's ``quest.source`` (quest_view or udp).
``--record`` appends every accepted packet to a JSON-lines file; ``synth
--replay`` can send a UDP recording again.

    python3 -m openarm_quest_teleop.monitor
    python3 -m openarm_quest_teleop.monitor --record /tmp/quest.jsonl
"""

from __future__ import annotations

import argparse
import datetime
import json
import sys
import time

from .config import load_config
from .packet import SIDES, VALID_NAMES, PacketError, parse_packet
from .quest_view_receiver import open_source, parser_for
from .udp_receiver import ReceivedPacket


def describe(packet: ReceivedPacket | None, rate_hz: float, counts: tuple[int, int],
             refused: int, now_ns: int, parse=parse_packet) -> str:
    """One status block for the newest packet."""
    accepted, malformed = counts
    lines = [
        f"datagrams: {accepted} accepted, {malformed} malformed, {refused} refused"
        f" | rate {rate_hz:5.1f} Hz"
    ]
    if packet is None:
        lines.append("no packet received yet")
        return "\n".join(lines)
    received = datetime.datetime.fromtimestamp(packet.recv_ns * 1e-9)
    age = (now_ns - packet.recv_ns) * 1e-9
    lines.append(f"last packet: PC time {received:%H:%M:%S.%f} ({age:.3f} s ago)")
    try:
        frame = parse(packet.message)
    except PacketError as error:
        lines.append(f"  REFUSED: {error}")
        return "\n".join(lines)
    headset = "-" if frame.headset_time is None else f"{frame.headset_time:.3f}"
    lines.append(f"  headset clock t={headset} s, overall validity {VALID_NAMES[frame.validity]}")
    for side in SIDES:
        controller = frame.controllers[side]
        if controller.pose is None:
            pose = f"pose {VALID_NAMES[controller.validity]} (not usable)"
        else:
            x, y, z = controller.pose.position
            qx, qy, qz, qw = controller.pose.orientation
            pose = (f"pos [{x:+.3f} {y:+.3f} {z:+.3f}] m  "
                    f"quat [{qx:+.3f} {qy:+.3f} {qz:+.3f} {qw:+.3f}]")
        lines.append(
            f"  {side:5s} {pose}\n"
            f"        grip {controller.grip:.2f}  trigger {controller.trigger:.2f}  "
            f"stick [{controller.stick[0]:+.2f} {controller.stick[1]:+.2f}]  "
            f"buttons {'AB' if side == 'right' else 'XY'}="
            f"{int(controller.buttons[0])}{int(controller.buttons[1])}"
        )
    if frame.reference is not None:
        x, y, z = frame.reference.position
        lines.append(f"  reference (rf / front) pos [{x:+.3f} {y:+.3f} {z:+.3f}] m")
    if frame.hmd is not None:
        x, y, z = frame.hmd.position
        lines.append(f"  hmd pos [{x:+.3f} {y:+.3f} {z:+.3f}] m")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Receive-only Quest packet diagnostic")
    parser.add_argument("--config", help="teleop config (default: config/leap_hand/quest_teleop_leap.yaml; RH56F1: config/rh56f1/quest_teleop.yaml)")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--period", type=float, default=0.5, help="seconds between reports")
    parser.add_argument("--seconds", type=float, help="stop after this long")
    parser.add_argument("--record", help="append accepted datagrams to this JSON-lines file")
    args = parser.parse_args(argv)
    quest = load_config(args.config)["quest"]
    parse = parser_for(quest)

    record = open(args.record, "a", encoding="utf-8") if args.record else None

    refused = [0]

    def on_packet(packet: ReceivedPacket) -> None:
        if record:
            record.write(
                json.dumps({"recv_ns": packet.recv_ns, "packet": packet.message}) + "\n")
        try:
            parse(packet.message)
        except PacketError:
            refused[0] += 1

    receiver, _, source = open_source(quest, args.host, args.port, on_packet)
    print(f"reading {source}; Ctrl-C to stop. Nothing is commanded.")
    deadline = None if args.seconds is None else time.monotonic() + args.seconds
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(args.period)
            if receiver.bind_error():
                print(f"no input from {source}: {receiver.bind_error()}")
                continue
            stamps = receiver.drain_recv_timestamps()
            print(describe(receiver.latest(), len(stamps) / args.period,
                           receiver.counts(), refused[0], time.time_ns(), parse))
            print()
    except KeyboardInterrupt:
        pass
    finally:
        receiver.close()
        if record:
            record.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
