"""Wrist camera -> LEAP Hand: one hand, MediaPipe -> retarget -> trajectory controller.

    .venv-hand/bin/python -m openarm_leap_tracking.ros_node --side right --camera 0
    .venv-hand/bin/python -m openarm_leap_tracking.ros_node --side right --test --execute

Always publishes the capped target on /leap_hand_tracking/<side>/target (JointState).
With --execute, while following, streams the smoothed target to
/leap_<side>/<side>_hand_trajectory_controller/joint_trajectory. Following starts off:
Space toggles it (Esc quits). Following starts from the hand's measured joints
(/leap_<side>/joint_states), so it never jumps; a lost hand holds the last target.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import threading
import time

from . import retarget as rt

KUKU_LAB = Path(__file__).resolve().parents[3]
DEFAULT_URDF = KUKU_LAB / "urdf/generated/rl/openarm_leap_bi_rl.urdf"
DEFAULT_MODEL = Path(__file__).resolve().parents[2] / "models/hand_landmarker.task"
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/latest/hand_landmarker.task")
STATE_TIMEOUT_S = 0.5


class CameraSource:
    """OpenCV camera + MediaPipe HandLandmarker (Tasks API, VIDEO mode)."""

    def __init__(self, camera: int, model: Path) -> None:
        import cv2
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions, vision

        if not model.is_file():
            raise SystemExit(f"hand model missing: {model}\n  curl -L -o {model} --create-dirs {MODEL_URL}")
        self.cv2, self.mp = cv2, mp
        self.capture = cv2.VideoCapture(camera)
        if not self.capture.isOpened():
            raise SystemExit(f"camera {camera} did not open")
        options = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model)),
            running_mode=vision.RunningMode.VIDEO, num_hands=1,
            min_hand_detection_confidence=0.6, min_tracking_confidence=0.6)
        self.landmarker = vision.HandLandmarker.create_from_options(options)
        self.start = time.monotonic()

    def read(self):
        ok, frame = self.capture.read()
        if not ok:
            return None
        image = self.mp.Image(image_format=self.mp.ImageFormat.SRGB,
                              data=self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB))
        result = self.landmarker.detect_for_video(image, int((time.monotonic() - self.start) * 1000))
        if not result.hand_world_landmarks:
            return None
        return [(p.x, p.y, p.z) for p in result.hand_world_landmarks[0]]

    def close(self) -> None:
        self.landmarker.close()
        self.capture.release()


class TestSource:
    """A synthetic hand closing and opening every 4 s (no camera)."""

    def __init__(self, side: str) -> None:
        self.side, self.start = side, time.monotonic()

    def read(self):
        phase = 0.5 - 0.5 * math.cos(2 * math.pi * (time.monotonic() - self.start) / 4.0)
        return rt.synthetic_hand(self.side, curl=1.2 * phase, spread=0.15 * (1 - phase), thumb=0.9 * phase)

    def close(self) -> None:
        pass


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--side", choices=rt.SIDES, default="right")
    p.add_argument("--config", type=Path, default=rt.DEFAULT_CONFIG)
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="canonical LEAP URDF (joint limits)")
    source = p.add_mutually_exclusive_group()
    source.add_argument("--camera", type=int, default=0, help="OpenCV camera index (wrist camera)")
    source.add_argument("--test", action="store_true", help="synthetic hand instead of the camera")
    p.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="MediaPipe hand_landmarker.task")
    p.add_argument("--execute", action="store_true", help="stream to the hand's trajectory controller")
    p.add_argument("--auto-start", action="store_true", help="follow as soon as the hand's state arrives")
    p.add_argument("--duration-s", type=float, default=0.0, help="stop after this long (0: run)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    import rclpy
    from builtin_interfaces.msg import Duration
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    from openarm_head.ros_head import SpaceKey

    config = rt.load_config(args.config)
    maps = rt.joint_maps(config, rt.urdf_limits(args.urdf, args.side))
    smoother = rt.Smoother(config["filter"]["alpha"], math.radians(config["filter"]["max_step_deg"]))
    rate_hz = float(config["command"]["rate_hz"])
    lead = float(config["command"]["time_from_start_s"])
    names = {s: f"{args.side[0]}_hj_{s}" for s in rt.JOINTS}
    namespace = f"leap_{args.side}"

    rclpy.init()
    node = rclpy.create_node(f"leap_hand_tracking_{args.side}")
    log = node.get_logger()
    measured: dict[str, float] = {}
    measured_at = [0.0]

    def on_state(message: JointState) -> None:
        measured.update(zip(message.name, message.position))
        measured_at[0] = time.monotonic()

    node.create_subscription(JointState, f"/{namespace}/joint_states", on_state, 10)
    target_pub = node.create_publisher(JointState, f"/leap_hand_tracking/{args.side}/target", 10)
    command_pub = node.create_publisher(
        JointTrajectory, f"/{namespace}/{args.side}_hand_trajectory_controller/joint_trajectory", 10)

    source = TestSource(args.side) if args.test else CameraSource(args.camera, args.model)
    stop = threading.Event()
    key = SpaceKey(stop)
    following, target = False, None
    seen = lost = 0
    started = last_status = time.monotonic()
    log.info(f"{'test hand' if args.test else f'camera {args.camera}'} -> {namespace}; "
             f"{'streaming on follow' if args.execute else 'dry run (no command)'}; Space: follow / stop")
    try:
        while rclpy.ok() and not stop.is_set():
            tick = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            fresh = tick - measured_at[0] < STATE_TIMEOUT_S and all(n in measured for n in names.values())
            if key.consume() or (args.auto_start and not following and fresh):
                if following:
                    following = False
                    log.info("stopped following; the hand holds its last command")
                elif not fresh:
                    log.warning(f"no fresh /{namespace}/joint_states: not following")
                else:
                    smoother.reset({s: measured[n] for s, n in names.items()})
                    following = True
                    log.info("following the camera hand")
            landmarks = source.read()
            if landmarks is not None:
                try:
                    target = rt.to_robot(rt.human_angles(landmarks, args.side), maps)
                    seen += 1
                except ValueError:
                    lost += 1
            else:
                lost += 1
            if target is not None:
                stamp = node.get_clock().now().to_msg()
                target_pub.publish(JointState(name=list(names.values()), position=[target[s] for s in names]))
                if following:
                    out = smoother(target)
                    if args.execute:
                        command = JointTrajectory(joint_names=list(names.values()))
                        command.header.stamp = stamp
                        command.points = [JointTrajectoryPoint(
                            positions=[out[s] for s in names],
                            time_from_start=Duration(sec=0, nanosec=int(lead * 1e9)))]
                        command_pub.publish(command)
            if tick - last_status >= 2.0:
                last_status = tick
                log.info(f"hand seen {seen} lost {lost}; following={following}; state "
                         f"{'fresh' if fresh else 'stale'}")
            if args.duration_s and tick - started >= args.duration_s:
                break
            time.sleep(max(0.0, 1.0 / rate_hz - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        key.close()
        source.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
