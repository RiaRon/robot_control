"""Synthetic Nova2 glove JointState, for testing without a glove or its driver.

Publishes what the senseglove_ros driver publishes for one glove,
``sensor_msgs/JointState`` on ``/senseglove/glove<serial>/<rh|lh>/joint_states``,
with the Nova2 description's joint names (``r_index_pip``, ``r_thumb_brake``,
...). Each finger's value is produced from a closure in [0, 1] (0 = open) with
the zero/range calibration the retarget node uses (by default the vendored
``config/mapping.yaml``), so the retarget output spans its whole range.

It publishes glove joint angles only. It commands nothing, and it is not a
model of the real glove's signals.

    python3 -m rh56f1_glove_teleop.synth_glove --side right --serial SYNTH --scenario wave
    python3 -m rh56f1_glove_teleop.synth_glove --side left --serial SYNTH --scenario hold --closure 0.4
    python3 -m rh56f1_glove_teleop.synth_glove --side right --serial SYNTH --scenario independent --closure 0.8
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import yaml

FINGERS = ("index", "middle", "ring", "pinky")
PHALANGES = ("brake", "mcp", "pip", "dip")
VENDORED_MAPPING = (Path(__file__).resolve().parents[2] / "third_party"
                    / "inspire_hand_senseglove_teleop/senseglove_teleop/config/mapping.yaml")
SCENARIOS = ("hold", "open", "close", "wave", "independent")


def joint_names(side: str) -> list[str]:
    """Every revolute joint of the Nova2 description, as the driver names them."""
    p = "r_" if side == "right" else "l_"
    names = [f"{p}thumb_{j}" for j in PHALANGES]
    names += [f"{p}{finger}_{j}" for finger in FINGERS for j in PHALANGES]
    names += [f"{p}palm_strap", f"{p}palm_index", f"{p}palm_pinky"]
    return names


def glove_topic(serial: str, side: str) -> str:
    return f"/senseglove/glove{serial}/{'rh' if side == 'right' else 'lh'}/joint_states"


def load_calibration(path: str | Path | None) -> dict[str, float]:
    data = yaml.safe_load(Path(path or VENDORED_MAPPING).read_text())
    return {k: float(v) for k, v in data["/**"]["ros__parameters"].items()}


def positions(side: str, closures: dict[str, float], calibration: dict[str, float]) -> list[float]:
    """Glove joint angles that the retarget calibration maps to *closures*.

    closures keys: index, middle, ring, pinky, thumb_pitch, thumb_yaw.
    """
    p = "r_" if side == "right" else "l_"
    values = {name: 0.0 for name in joint_names(side)}
    for finger in FINGERS:
        c = closures[finger]
        values[f"{p}{finger}_pip"] = (calibration[f"{finger}_zero_rad"]
                                      + c * calibration[f"{finger}_range_rad"])
    values[f"{p}thumb_pip"] = (calibration["thumb_pitch_zero_rad"]
                               + closures["thumb_pitch"] * calibration["thumb_pitch_range_rad"])
    values[f"{p}thumb_brake"] = (calibration["thumb_yaw_zero_rad"]
                                 + closures["thumb_yaw"] * calibration["thumb_yaw_range_rad"])
    return [values[name] for name in joint_names(side)]


def closures_for(scenario: str, elapsed: float, closure: float, period: float,
                 overrides: dict[str, float]) -> dict[str, float]:
    if scenario == "open":
        c = 0.0
    elif scenario == "close":
        c = 1.0
    elif scenario == "wave":
        c = 0.5 - 0.5 * math.cos(2.0 * math.pi * elapsed / period)
    elif scenario == "independent":
        # One finger at a time, each for one period: index, middle, ring, pinky.
        active = FINGERS[int(elapsed // period) % len(FINGERS)]
        result = {key: 0.0 for key in (*FINGERS, "thumb_pitch", "thumb_yaw")}
        result[active] = closure
        result.update(overrides)
        return result
    else:
        c = closure
    result = {key: c for key in (*FINGERS, "thumb_pitch", "thumb_yaw")}
    result.update(overrides)
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="synthetic Nova2 glove JointState")
    parser.add_argument("--side", choices=("right", "left"), required=True)
    parser.add_argument("--serial", required=True, help="glove serial in the topic name")
    parser.add_argument("--scenario", choices=SCENARIOS, default="hold")
    parser.add_argument("--closure", type=float, default=0.5,
                        help="for --scenario hold, and the closed finger of independent")
    parser.add_argument("--period", type=float, default=4.0,
                        help="for --scenario wave, and each finger's turn in independent")
    parser.add_argument("--seconds", type=float, help="stop after this long")
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=CLOSURE",
                        help="fix one of index, middle, ring, pinky, thumb_pitch, thumb_yaw")
    parser.add_argument("--mapping", help="retarget calibration yaml (default: vendored)")
    args, _ = parser.parse_known_args(argv)
    overrides = {}
    for item in args.set:
        key, value = item.split("=", 1)
        if key not in (*FINGERS, "thumb_pitch", "thumb_yaw"):
            parser.error(f"unknown key {key!r}")
        overrides[key] = float(value)

    import rclpy
    from sensor_msgs.msg import JointState

    calibration = load_calibration(args.mapping)
    rclpy.init()
    node = rclpy.create_node(f"synthetic_nova2_{args.side}")
    topic = glove_topic(args.serial, args.side)
    publisher = node.create_publisher(JointState, topic, 10)
    names = joint_names(args.side)
    node.get_logger().info(f"synthetic glove -> {topic} ({args.scenario})")
    start = time.monotonic()
    try:
        while args.seconds is None or time.monotonic() - start < args.seconds:
            closures = closures_for(args.scenario, time.monotonic() - start, args.closure,
                                    args.period, overrides)
            message = JointState()
            message.header.stamp = node.get_clock().now().to_msg()
            message.name = names
            message.position = positions(args.side, closures, calibration)
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
