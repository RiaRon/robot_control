#!/usr/bin/env python3
"""Integrated joint states for the split OpenArm + RH56F1 bringup.

Each device publishes its own joint states (arms ``/openarm/joint_states``,
hands ``/rh56f1_<side>/joint_states``). This node is the only publisher of the
integrated topics and never commands or activates anything:

``/joint_states`` (measured)
    Every message a device publishes is forwarded once, as it arrived: its own
    stamp, only the joints that device owns, ``header.frame_id`` set to the
    device id. Nothing is combined, held or re-stamped, so a device that stops
    simply stops appearing, and the others are never delayed by it. With every
    device fresh, the latest message of each covers the 26 independent joints
    once each; with the arms alone, only their 14 appear.

``/openarm_rh56f1/display_joint_states`` (what robot_state_publisher draws)
    The same forwarded messages, plus, only for a hand with no fresh state, its
    six actuators at the open pose (0 rad) with ``header.frame_id`` =
    ``display_placeholder``. The placeholder stops the moment that hand's own
    state is fresh again. Fingertip TF drawn from it is not a measurement.

``/openarm_rh56f1/joint_state_sources`` (std_msgs/String, JSON, 2 Hz)
    Per device: fresh / stale / never, age, messages, dropped joints, and
    whether its fingers are drawn from the placeholder.

Rules: a joint has exactly one owning device (by name, never by position in
the array); a joint a device does not own is dropped and counted; a message
with a non-finite position is dropped whole; no input topic may be one of the
outputs.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import math
import sys
import time

DEFAULT_STALE_AFTER_SEC = 0.5
PLACEHOLDER_PERIOD_SEC = 0.1
STATUS_PERIOD_SEC = 0.5


class MergerConfigError(ValueError):
    pass


@dataclass
class Source:
    id: str
    topic: str
    joints: tuple[str, ...]
    display_placeholder: bool
    last_arrival: float | None = None
    messages: int = 0
    foreign_dropped: int = 0
    nonfinite_dropped: int = 0
    foreign_names: set = field(default_factory=set)


@dataclass(frozen=True)
class Forward:
    """One message to publish: names/positions/... of one source."""

    source: str
    names: list[str]
    positions: list[float]
    velocities: list[float]
    efforts: list[float]


class MergerCore:
    """Ownership, freshness and placeholder rules, without ROS."""

    def __init__(self, sources: list[dict], outputs: list[str],
                 stale_after_sec: float = DEFAULT_STALE_AFTER_SEC):
        if not stale_after_sec > 0.0:
            raise MergerConfigError("stale_after_sec must be positive")
        self.stale_after_sec = stale_after_sec
        self.sources: dict[str, Source] = {}
        owner: dict[str, str] = {}
        topics = set()
        for spec in sources:
            source = Source(spec["id"], spec["topic"], tuple(spec["joints"]),
                            bool(spec.get("display_placeholder", False)))
            if source.id in self.sources:
                raise MergerConfigError(f"duplicate source id {source.id!r}")
            if source.topic in topics:
                raise MergerConfigError(f"two sources read {source.topic}")
            if source.topic in outputs:
                raise MergerConfigError(
                    f"source {source.id!r} reads {source.topic}, which this node "
                    "publishes: that would be a loop")
            for name in source.joints:
                if name in owner:
                    raise MergerConfigError(
                        f"joint {name!r} is owned by both {owner[name]!r} and {source.id!r}")
                owner[name] = source.id
            topics.add(source.topic)
            self.sources[source.id] = source
        self.owner = owner

    def receive(self, source_id: str, now: float, names, positions, velocities=(),
                efforts=()) -> Forward | None:
        """Filter one incoming message; None when it must not be forwarded."""
        source = self.sources[source_id]
        names = list(names)
        positions = [float(v) for v in positions]
        if len(positions) != len(names):
            source.nonfinite_dropped += 1
            return None
        if not all(math.isfinite(v) for v in positions):
            source.nonfinite_dropped += 1
            return None
        keep = [i for i, name in enumerate(names) if self.owner.get(name) == source_id]
        if len(keep) != len(names):
            source.foreign_dropped += len(names) - len(keep)
            source.foreign_names.update(n for i, n in enumerate(names) if i not in keep)
        if not keep:
            return None
        velocities = list(velocities)
        efforts = list(efforts)
        pick = lambda values: [float(values[i]) for i in keep] if len(values) == len(names) else []
        source.last_arrival = now
        source.messages += 1
        return Forward(source_id, [names[i] for i in keep], [positions[i] for i in keep],
                       pick(velocities), pick(efforts))

    def state(self, source_id: str, now: float) -> str:
        source = self.sources[source_id]
        if source.last_arrival is None:
            return "never"
        return "fresh" if now - source.last_arrival <= self.stale_after_sec else "stale"

    def placeholders(self, now: float) -> list[Forward]:
        """Display-only open poses for hands that have no fresh state."""
        return [
            Forward(source.id, list(source.joints), [0.0] * len(source.joints), [], [])
            for source in self.sources.values()
            if source.display_placeholder and self.state(source.id, now) != "fresh"
        ]

    def status(self, now: float) -> dict:
        report = {}
        for source in self.sources.values():
            state = self.state(source.id, now)
            report[source.id] = {
                "topic": source.topic,
                "state": state,
                "age_sec": None if source.last_arrival is None
                else round(now - source.last_arrival, 3),
                "messages": source.messages,
                "joints": len(source.joints),
                "foreign_joints_dropped": source.foreign_dropped,
                "foreign_joint_names": sorted(source.foreign_names),
                "nonfinite_messages_dropped": source.nonfinite_dropped,
                "display_placeholder": source.display_placeholder and state != "fresh",
            }
        return report


def main(argv: list[str] | None = None) -> int:
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description="split-bringup joint state merger")
    parser.add_argument("--sources", required=True,
                        help="JSON list of {id, topic, joints, display_placeholder}")
    parser.add_argument("--merged-topic", default="/joint_states")
    parser.add_argument("--display-topic", default="/openarm_rh56f1/display_joint_states")
    parser.add_argument("--status-topic", default="/openarm_rh56f1/joint_state_sources")
    parser.add_argument("--stale-after-sec", type=float, default=DEFAULT_STALE_AFTER_SEC)
    parser.add_argument("--no-display-placeholder", action="store_true")
    args, ros_args = parser.parse_known_args(argv)

    sources = json.loads(args.sources)
    if args.no_display_placeholder:
        for source in sources:
            source["display_placeholder"] = False
    core = MergerCore(sources, [args.merged_topic, args.display_topic, args.status_topic],
                      args.stale_after_sec)

    rclpy.init(args=ros_args)
    node = rclpy.create_node("openarm_rh56f1_joint_state_merger")
    merged = node.create_publisher(JointState, args.merged_topic, 10)
    display = node.create_publisher(JointState, args.display_topic, 10)
    status = node.create_publisher(String, args.status_topic, 10)

    def publish(forward, stamp, frame_id, publishers):
        message = JointState()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.name = forward.names
        message.position = forward.positions
        message.velocity = forward.velocities
        message.effort = forward.efforts
        for publisher in publishers:
            publisher.publish(message)

    def subscribe(source_id, topic):
        def on_message(message):
            forward = core.receive(source_id, time.monotonic(), message.name, message.position,
                                   message.velocity, message.effort)
            if forward is not None:
                publish(forward, message.header.stamp, source_id, (merged, display))
        node.create_subscription(JointState, topic, on_message, qos_profile_sensor_data)

    for spec in sources:
        subscribe(spec["id"], spec["topic"])

    def placeholder_tick():
        stamp = node.get_clock().now().to_msg()
        for forward in core.placeholders(time.monotonic()):
            publish(forward, stamp, "display_placeholder", (display,))

    def status_tick():
        status.publish(String(data=json.dumps(core.status(time.monotonic()))))

    node.create_timer(PLACEHOLDER_PERIOD_SEC, placeholder_tick)
    node.create_timer(STATUS_PERIOD_SEC, status_tick)
    node.get_logger().info(
        "merging " + ", ".join(f"{s['id']}={s['topic']} ({len(s['joints'])})" for s in sources))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
