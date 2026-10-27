"""Meta Quest headset -> Dynamixel pan/tilt neck, as a ROS 2 node.

    .venv/bin/python -m openarm_head.ros_head --config $CFG                  # fake neck
    .venv/bin/python -m openarm_head.ros_head --config $CFG --backend real   # head.port must be set

The loop is motion_acq's ``macq head`` (scripts/head_teleop.py at 01dc218):
torque on in place, walk to home, follow the headset's yaw (pan) and pitch
(tilt) relative to the moment it anchors, hold on headset loss, walk home on
Ctrl+C / SIGTERM / --duration-s. Only the input changed: the headset pose and
the robot's front come from the bridge's ROS topics (``quest.topics.hmd_pose``,
``reference_pose``) instead of the HandUMI TCP stream.

With ``quest.source: quest_view`` the neck waits for the robot's front
(quest_view /front), as upstream does. ``--unlock key`` (default on a
terminal) starts locked at home: Space follows from where you look now,
Space again locks and walks home. Each cycle goes to quest_view's status
port (the headset's status line, and the head camera turns on while the
neck runs) and the measured angles to ``head.joint_states.topic``.
"""

from __future__ import annotations

import argparse
import logging
import math
import select
import signal
import sys
import termios
import threading
import time
import tty
from pathlib import Path

import numpy as np
import yaml

from openarm_head.config import HeadConfig, load_head_config
from openarm_head.dynamixel import (
    FakeHeadBus,
    HeadAlertError,
    HeadBus,
    HeadBusError,
    HeadDriver,
    SdkHeadBus,
    deg_to_tick,
)
from openarm_head.retarget import HeadRetargeter
from openarm_head.session import HeadSession, HmdSample, open_log

log = logging.getLogger("openarm_head")
STATUS_PERIOD_S = 1.0
MAX_RATE_HZ = 200.0
QUEST_VIEW_STATUS = "127.0.0.1:47121"
FRONT_STALE_S = 0.5


class RosHmdSource:
    """The newest headset and front poses from the bridge's topics (rclpy spun in a thread)."""

    def __init__(self, quest: dict, timeout_s: float) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState

        rclpy.init()
        self.node = rclpy.create_node("quest_head")
        self.timeout_s = float(timeout_s)
        self.front_required = quest.get("source") == "quest_view"
        self._hmd: tuple[float, np.ndarray] | None = None
        self._front: tuple[float, np.ndarray] | None = None
        self._lock = threading.Lock()
        topics = quest["topics"]
        self.node.create_subscription(PoseStamped, topics["hmd_pose"], self._on_hmd, 10)
        self.node.create_subscription(PoseStamped, topics["reference_pose"], self._on_front, 10)
        self.JointState = JointState
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    @staticmethod
    def _pose7(message) -> np.ndarray:
        p, q = message.pose.position, message.pose.orientation
        return np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=np.float64)

    def _on_hmd(self, message) -> None:
        with self._lock:
            self._hmd = (time.monotonic(), self._pose7(message))

    def _on_front(self, message) -> None:
        with self._lock:
            self._front = (time.monotonic(), self._pose7(message))

    def latest(self) -> HmdSample:
        now = time.monotonic()
        with self._lock:
            hmd, front = self._hmd, self._front
        tracked = hmd is not None and now - hmd[0] <= self.timeout_s
        pose = hmd[1] if hmd is not None else np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float64)
        fresh_front = front[1] if front is not None and now - front[0] <= FRONT_STALE_S else None
        return HmdSample(pose, tracked, self.front_required, fresh_front)

    def close(self) -> None:
        import rclpy

        self._executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class JointStatePublisher:
    """Measured neck angles as the model's joints: sign * radians(operator angle from home)."""

    def __init__(self, source: RosHmdSource, section: dict, config: HeadConfig) -> None:
        self.source = source
        self.publisher = source.node.create_publisher(source.JointState, section["topic"], 10)
        self.names = (section["pan"]["name"], section["tilt"]["name"])
        axes = (config.retarget.pan, config.retarget.tilt)
        self.home = tuple(a.home_deg for a in axes)
        self.signs = tuple(float(section[k]["sign"]) * a.sign for k, a in zip(("pan", "tilt"), axes))

    def publish(self, measured: tuple[float, float] | None) -> None:
        if measured is None:
            return
        message = self.source.JointState()
        message.header.stamp = self.source.node.get_clock().now().to_msg()
        message.name = list(self.names)
        message.position = [s * math.radians(m - h) for s, m, h in zip(self.signs, measured, self.home)]
        self.publisher.publish(message)


class SpaceKey:
    """Space presses on the terminal (upstream KeyboardSpaceListener, Space and Esc only)."""

    def __init__(self, stop: threading.Event) -> None:
        self.stop = stop
        self.enabled = sys.stdin.isatty()
        self._space = threading.Event()
        self._closed = threading.Event()
        if self.enabled:
            threading.Thread(target=self._run, daemon=True).start()

    def consume(self) -> bool:
        if not self._space.is_set():
            return False
        self._space.clear()
        return True

    def close(self) -> None:
        self._closed.set()

    def _run(self) -> None:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._closed.is_set():
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    char = sys.stdin.read(1)
                    if char == " ":
                        self._space.set()
                    elif char == "\x1b":
                        self.stop.set()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _sigterm_to_interrupt(signum, frame) -> None:  # noqa: ARG001
    raise KeyboardInterrupt


def _udp_target(text: str) -> tuple[str, int]:
    host, sep, port = text.rpartition(":")
    if not sep or not host:
        raise SystemExit(f"--udp-target expects HOST:PORT, not {text!r}")
    return host, int(port)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", type=Path, required=True, help="quest_teleop_leap.yaml (quest + head)")
    p.add_argument("--backend", choices=("fake", "real"), default="fake")
    p.add_argument("--axes", choices=("pan", "tilt", "both"), default="both")
    p.add_argument("--auto-start-delay-s", type=float, default=2.0)
    p.add_argument("--rate-hz", type=float, default=None, help="Override head.rate_hz.")
    p.add_argument("--duration-s", type=float, default=0.0, help="Stop after N s (0 = until Ctrl+C).")
    p.add_argument("--log-dir", type=Path, default=Path("logs/head"))
    p.add_argument("--no-log", action="store_true")
    p.add_argument("--udp-target", action="append", default=[QUEST_VIEW_STATUS], metavar="HOST:PORT",
                   help=f"Also send each cycle here (default {QUEST_VIEW_STATUS}: quest_view); repeatable.")
    p.add_argument("--unlock", choices=("auto", "key"), default=None,
                   help="key: start locked at home, Space toggles (default on a terminal); "
                        "auto: anchor after --auto-start-delay-s of headset tracking.")
    p.add_argument("--torque-off-on-exit", action="store_true")
    return p.parse_args(argv)


def build_bus(backend: str, config: HeadConfig) -> HeadBus:
    if backend == "real":
        if not config.port:
            raise SystemExit("--backend real needs head.port (the neck's U2D2) in the config.")
        return SdkHeadBus(config.port, config.baud)
    hw, retarget = config.hardware, config.retarget
    return FakeHeadBus(model=hw.model_number, present_ticks={
        hw.pan_id: deg_to_tick(retarget.pan.home_deg),
        hw.tilt_id: deg_to_tick(retarget.tilt.home_deg),
    })


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="[%(asctime)s] %(levelname)s - %(message)s", datefmt="%H:%M:%S")
    data = yaml.safe_load(args.config.read_text())
    config = load_head_config(args.config, allow_fake_default=False)
    rate_hz = args.rate_hz if args.rate_hz is not None else config.rate_hz
    if not 0.0 < rate_hz <= MAX_RATE_HZ:
        raise SystemExit(f"head rate {rate_hz} Hz is outside (0, {MAX_RATE_HZ:g}].")
    log.info("Head %s backend, axes=%s, pan window [%.1f, %.1f], tilt window [%.1f, %.1f] deg, %.0f Hz.",
             args.backend, args.axes, *config.pan_window, *config.tilt_window, rate_hz)
    driver = HeadDriver(build_bus(args.backend, config), config.hardware,
                        pan_window=config.pan_window, tilt_window=config.tilt_window)
    source = RosHmdSource(data["quest"], data["teleop"]["input_timeout_sec"])
    joints = JointStatePublisher(source, data["head"]["joint_states"], config)
    log_file = None if args.no_log else open_log(args.log_dir, args.backend)
    signal.signal(signal.SIGTERM, _sigterm_to_interrupt)
    unlock = args.unlock or ("key" if sys.stdin.isatty() else "auto")
    alert = False
    try:
        measured = driver.start()
        joints.publish(measured)
        log.info("Head torque on at pan %.1f / tilt %.1f deg; moving to home %.1f / %.1f at %.0f deg/s.",
                 *measured, *config.home, config.home_speed_deg_s)
        measured = driver.move_to(*config.home, speed_deg_s=config.home_speed_deg_s, rate_hz=rate_hz,
                                  tolerance_deg=config.home_tolerance_deg)
        joints.publish(measured)
        log.info("Head at home (pan %.1f / tilt %.1f deg).", *measured)
        session = HeadSession(
            source, driver, HeadRetargeter(config.retarget.with_axes(args.axes)),
            auto_start_delay_s=args.auto_start_delay_s,
            max_consecutive_faults=config.max_consecutive_faults,
            log_file=log_file,
            udp_targets=tuple(_udp_target(t) for t in args.udp_target),
            unlock=unlock,
            home_speed_deg_s=config.home_speed_deg_s,
        )
        _run(session, joints, rate_hz, args.duration_s)
    except HeadBusError as exc:
        alert = isinstance(exc, HeadAlertError)
        raise SystemExit(f"Head stopped: {exc}") from exc
    except KeyboardInterrupt:
        log.info("Stopping head.")
    finally:
        try:
            _return_home(driver, config, rate_hz, joints, alert=alert)
            driver.stop(torque_off=args.torque_off_on_exit)
        finally:
            source.close()
            if log_file is not None:
                log_file.close()


def _return_home(driver: HeadDriver, config: HeadConfig, rate_hz: float, joints: JointStatePublisher,
                 *, alert: bool) -> None:
    """End pose = home, like the start pose. A hardware alert stops in place."""
    if not driver.started:
        return
    if alert:
        log.error("Hardware alert: the head stays where it is (not returning home).")
        return
    try:
        measured = driver.move_to(*config.home, speed_deg_s=config.home_speed_deg_s, rate_hz=rate_hz,
                                  tolerance_deg=config.home_tolerance_deg)
        joints.publish(measured)
        log.info("Head back at home (pan %.1f / tilt %.1f deg).", *measured)
    except (HeadBusError, KeyboardInterrupt) as exc:
        log.error("Head could not return home: %s", exc or "interrupted")


def _run(session: HeadSession, joints: JointStatePublisher, rate_hz: float, duration_s: float) -> None:
    period = 1.0 / rate_hz
    start = last_status = time.monotonic()
    stop = threading.Event()
    keys = SpaceKey(stop) if session.unlock_mode == "key" else None
    if keys is not None:
        log.info("Head locked at home. Space: follow the headset from where you look now / lock again. "
                 "Esc or Ctrl+C: home and stop.")
    try:
        while (duration_s <= 0.0 or time.monotonic() - start < duration_s) and not stop.is_set():
            cycle = time.monotonic()
            if keys is not None and keys.consume():
                session.toggle_lock()
            step = session.tick()
            joints.publish(session.measured)
            if cycle - last_status >= STATUS_PERIOD_S:
                last_status = cycle
                _status(session, step)
            time.sleep(max(period - (time.monotonic() - cycle), 0.0))
    finally:
        if keys is not None:
            keys.close()
    s = session.stats
    log.info("Head done: %d cycles, %d commands, %d hold cycles, %d re-anchors, %d faults.",
             s.cycles, s.commands, s.holds, s.reanchors, s.faults)


def _status(session: HeadSession, step) -> None:
    def fmt(value: float | None) -> str:
        return "  -  " if value is None else f"{value:+6.1f}"

    meas = session.measured or (None, None)
    state = "locked" if session.locked else (step.state.value if step else session.retargeter.state.value)
    rel = (step.rel_yaw_deg, step.rel_pitch_deg) if step else (None, None)
    cmd = (step.command_pan_deg, step.command_tilt_deg) if step else (None, None)
    log.info("head %-7s rel yaw %s pitch %s | cmd pan %s tilt %s | meas pan %s tilt %s",
             state, fmt(rel[0]), fmt(rel[1]), fmt(cmd[0]), fmt(cmd[1]), fmt(meas[0]), fmt(meas[1]))


if __name__ == "__main__":
    main()
