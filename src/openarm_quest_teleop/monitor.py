"""Receive-only Quest diagnostic. Needs no ROS and commands nothing.

Prints, a few times a second, both controllers' pose (``quest_world``: x
forward, y left, z up; quaternion x y z w), grip, trigger, stick and buttons,
pose validity, when this PC last received a packet and how long ago, and the
receive rate. ``--record`` appends every accepted datagram to a JSON-lines
file that ``synth --replay`` can send again.

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
from .udp_receiver import JsonUdpReceiver, ReceivedPacket


def describe(packet: ReceivedPacket | None, rate_hz: float, counts: tuple[int, int],
             refused: int, now_ns: int) -> str:
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
        frame = parse_packet(packet.message)
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
        lines.append(f"  reference (rf) pos [{x:+.3f} {y:+.3f} {z:+.3f}] m")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Receive-only Quest packet diagnostic")
    parser.add_argument("--config", help="quest_teleop.yaml (default: the packaged one)")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--period", type=float, default=0.5, help="seconds between reports")
    parser.add_argument("--seconds", type=float, help="stop after this long")
    parser.add_argument("--record", help="append accepted datagrams to this JSON-lines file")
    args = parser.parse_args(argv)
    udp = load_config(args.config)["quest"]["udp"]
    host, port = args.host or udp["host"], args.port or int(udp["port"])

    record = open(args.record, "a", encoding="utf-8") if args.record else None

    refused = [0]

    def on_packet(packet: ReceivedPacket) -> None:
        if record:
            record.write(
                json.dumps({"recv_ns": packet.recv_ns, "packet": packet.message}) + "\n")
        try:
            parse_packet(packet.message)
        except PacketError:
            refused[0] += 1

    receiver = JsonUdpReceiver(host, port, on_packet=on_packet)
    print(f"listening on UDP {host}:{port}; Ctrl-C to stop. Nothing is commanded.")
    deadline = None if args.seconds is None else time.monotonic() + args.seconds
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(args.period)
            if receiver.bind_error():
                print(f"cannot listen on {host}:{port}: {receiver.bind_error()}")
                continue
            stamps = receiver.drain_recv_timestamps()
            print(describe(receiver.latest(), len(stamps) / args.period,
                           receiver.counts(), refused[0], time.time_ns()))
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
