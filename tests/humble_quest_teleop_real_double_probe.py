#!/usr/bin/env python3
"""Humble runtime probe: the teleop on the split REAL arm description, with test doubles.

Checks what the fake bringup cannot: the real arm process of the split bringup
(openarm_rh56f1_arms.launch.py runtime:=real) with its source joint names
(``openarm_right_joint1..7``), its controllers (``rh56f1_<side>_arm_controller``,
``interpolation_method: none``, 750 Hz), its start states, and its arm state on
``/openarm/joint_states``. No hardware is involved:

  - Arms: the same description, plan and controller parameters the launch file
    renders (rh56f1_split_description.real_arm_description), with each
    OpenArmHW plugin swapped for mock_components/GenericSystem
    (arm_test_double), as in humble_rh56f1_hardware_runtime_probe.py.
  - Hands: none. The description keeps both hands' geometry (palm frames) and
    no hand hardware block; no hand plugin is built or loaded.
  - The integrated model (openarm_rh56f1_model.launch.py runtime:=real) draws
    the robot with the same source names.

The right arm component and controller are activated by explicit steps, the
arm is bent with the existing trajectory action, and the same bridge and
teleop processes as on the fake bringup follow synthetic Quest packets.
Output: CHECK lines, then QUEST_TELEOP_REAL_DOUBLE_PROBE=PASS or FAIL.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, str(Path(__file__).resolve().parent))
import humble_rh56f1_hardware_runtime_probe as rp  # noqa: E402
from humble_quest_teleop_probe import Process  # noqa: E402
import humble_split_bringup_probe as split_probe  # noqa: E402
import rh56f1_split_description as split  # noqa: E402

from openarm_quest_teleop.synth import QuestSender  # noqa: E402
from robot_control.kinematics import chain_from_urdf  # noqa: E402

real = rp.real
check = rp.check
ARM = {side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in ("right", "left")}
CONTROLLER = {side: f"rh56f1_{side}_arm_controller" for side in ("right", "left")}
BENT = [0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0]
POSITION_TOL = 2e-3
ROTATION_TOL = 2e-2


class Probe(rp.Probe):
    def __init__(self):
        super().__init__()
        self.teleop = {}
        self.commands = {"right": [], "left": []}
        # The split arm process publishes on its own topic.
        self.create_subscription(JointState, split.ARM_STATE_TOPIC, self._arm_state, 100)
        for side in ("right", "left"):
            self.create_subscription(
                String, f"/quest_teleop/{side}/status",
                lambda m, side=side: self.teleop.__setitem__(side, json.loads(m.data)), 10)
            self.create_subscription(
                JointTrajectory, f"/{CONTROLLER[side]}/joint_trajectory",
                lambda m, side=side: self.commands[side].append(m), 100)

    def drive(self, action):
        thread = threading.Thread(target=action)
        thread.start()
        while thread.is_alive():
            rclpy.spin_once(self, timeout_sec=0.02)
        thread.join()

    def _arm_state(self, message):
        self.joint_messages.append(message)

    def arm_q(self, side):
        return np.array([self.latest_position(name) for name in ARM[side]])


class ArmGraph(rp.Graph):
    """rp.Graph, with the arm broadcaster on its own topic as the launch file does."""

    def _run(self, label, *args):
        if label == "cm":
            args = (*args, "-r", f"/joint_states:={split.ARM_STATE_TOPIC}",
                    "-r", "/dynamic_joint_states:=/openarm/dynamic_joint_states")
        return super()._run(label, *args)


def main():
    rp.LOG_ROOT.mkdir(parents=True, exist_ok=True)
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or not os.environ.get("ROS_DOMAIN_ID"):
        print("refusing to run: set an isolated ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
        return 2
    rclpy.init()
    node = Probe()
    description, plan = split.real_arm_description(
        rp.CANONICAL, rp.MANIFEST, rp.WRAPPER,
        right_can_interface="vcan_unused0", left_can_interface="vcan_unused1", can_fd=True,
        enable_right_arm=True, enable_left_arm=True, arm_test_double=True)
    plugins = sorted({p.text.strip() for p in
                      rp.ET.fromstring(description).findall("ros2_control/hardware/plugin")})
    check("double.only_arm_test_doubles_no_hand_hardware",
          plugins == ["mock_components/GenericSystem"] and "Rh56f1HW" not in description
          and {"r_hl_palm_sensor", "l_hl_palm_sensor"} <= {
              link.get("name") for link in rp.ET.fromstring(description).findall("link")},
          str(plugins))
    chain = chain_from_urdf(description, ARM["right"], "r_hl_palm_sensor")
    graph = ArmGraph("quest_teleop_real_double", description, plan)
    processes = []
    model = split_probe.Launch("double.model", "openarm_rh56f1_model.launch.py", "runtime:=real")
    sender = QuestSender("127.0.0.1", 5006)
    try:
        spawn = graph.start(node)
        check("double.spawners_ok", all(code == 0 for code in spawn.values()), str(spawn))
        node.spin_for(1.0)
        hardware = node.hardware_states()
        check("double.arms_start_inactive_and_no_hand_component",
              hardware == {"openarm_rh56f1_right_arm": "inactive",
                           "openarm_rh56f1_left_arm": "inactive"}, str(hardware))
        check("double.controllers_loaded_inactive",
              node.controller_states() == {"joint_state_broadcaster": "active",
                                           "rh56f1_right_arm_controller": "inactive",
                                           "rh56f1_left_arm_controller": "inactive"},
              str(node.controller_states()))

        # The left arm is left as launched: its controller is inactive, and
        # the teleop must refuse to execute against it.
        refused = Process("double.teleop_left_inactive", "openarm_quest_teleop.ros_teleop",
                          "--arm", "left", "--runtime", "real", "--execute")
        deadline = time.monotonic() + 40
        while refused.process.poll() is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        code, text = refused.process.poll(), refused.text()
        refused.stop()
        check("double.inactive_controller_refuses_execute",
              code == 2 and "rh56f1_left_arm_controller is inactive, not active" in text,
              f"exit {code}")

        # Explicit operator steps for the right arm only.
        check("double.right_arm_component_activated",
              node.set_component("openarm_rh56f1_right_arm", "active"))
        check("double.right_arm_controller_activated",
              node.switch_controllers(activate=[CONTROLLER["right"]]))
        check("double.bend_goal_accepted",
              node.send_goal(CONTROLLER["right"], ARM["right"], BENT, 2.0))
        node.spin_for(3.5)
        bent = node.arm_q("right")
        check("double.right_arm_bent", np.allclose(bent, BENT, atol=1e-4),
              str(np.round(bent, 4).tolist()))

        processes.append(Process("double.bridge", "openarm_quest_teleop.ros_bridge"))
        teleop = Process("double.teleop_right", "openarm_quest_teleop.ros_teleop",
                         "--arm", "right", "--runtime", "real", "--execute")
        processes.append(teleop)
        deadline = time.monotonic() + 30
        while "right" not in node.teleop and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        status = node.teleop.get("right", {})
        check("double.teleop_bound_to_real_controller",
              status.get("controller") == CONTROLLER["right"] and status.get("executing")
              and status.get("hardware") == ["mock_components/GenericSystem"],
              f"{status.get('controller')} on {status.get('hardware')}")
        node.commands["right"].clear()

        node.drive(lambda: sender.hold(1.0))
        check("double.idle_sends_nothing", not node.commands["right"])
        before_q = node.arm_q("right")
        sender.grip["right"] = 1.0
        node.drive(lambda: sender.hold(1.0))
        status = node.teleop["right"]
        palm = chain.pose(before_q)
        gap = np.linalg.norm(np.array(status["target_xyz"]) - palm[:3, 3])
        drift = float(np.max(np.abs(node.arm_q("right") - before_q)))
        check("double.enable_target_is_current_palm_and_moves_nothing",
              status["state"] == "engaged" and gap < 1e-6 and drift < 1e-9,
              f"|target - palm| {gap:.1e} m, joint change {drift:.1e} rad")

        for label, delta in (("forward", (0.04, 0, 0)), ("left", (0, 0.04, 0)),
                             ("up", (0, 0, 0.04))):
            start = chain.pose(node.arm_q("right"))
            node.drive(lambda: (sender.move("right", delta, seconds=1.0), sender.hold(0.8)))
            end = chain.pose(node.arm_q("right"))
            moved = end[:3, 3] - start[:3, 3]
            cosine = (np.trace(start[:3, :3].T @ end[:3, :3]) - 1.0) / 2.0
            turned = math.acos(max(-1.0, min(1.0, cosine)))
            check(f"double.move_{label}",
                  np.linalg.norm(moved - np.array(delta)) < POSITION_TOL
                  and turned < ROTATION_TOL,
                  f"palm moved {np.round(moved, 4).tolist()} m, turned {turned:.4f} rad")

        # TF comes from the integrated model process, not from the arm manager.
        transform = node.transform("body_root", "r_hl_palm_sensor")
        palm = chain.pose(node.arm_q("right"))[:3, 3]
        tf_palm = None if transform is None else np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z])
        check("double.palm_tf_available_without_hand_state",
              tf_palm is not None and np.linalg.norm(tf_palm - palm) < 1e-4,
              f"TF {None if tf_palm is None else np.round(tf_palm, 4).tolist()}")

        sender.grip["right"] = 0.0
        node.drive(lambda: sender.hold(0.5))
        sent = len(node.commands["right"])
        node.drive(lambda: sender.hold(0.5))
        check("double.release_stops_the_stream",
              node.teleop["right"]["state"] == "idle" and len(node.commands["right"]) == sent)

        names = {tuple(m.joint_names) for m in node.commands["right"]}
        check("double.commands_use_source_names_of_the_right_arm_only",
              names == {tuple(ARM["right"])}, f"{len(node.commands['right'])} messages")
        left = node.arm_q("left")
        check("double.left_arm_untouched",
              not node.commands["left"] and np.all(left == 0.0)
              and node.controller_states().get(CONTROLLER["left"]) == "inactive",
              f"{len(node.commands['left'])} messages, q {left.tolist()}")
        hardware = node.hardware_states()
        controllers = node.controller_states()
        check("double.arm_manager_never_held_a_hand",
              set(hardware) == {"openarm_rh56f1_right_arm", "openarm_rh56f1_left_arm"}
              and not [c for c in controllers if "hand" in c],
              f"{hardware}")
        log = teleop.text()
        check("double.teleop_reads_split_real_endpoints",
              "joints source-named" in log
              and "description from /controller_manager parameter robot_description" in log
              and "joint states from /openarm/joint_states" in log)
    except Exception as error:  # noqa: BLE001
        check("double.completed", False, repr(error))
    finally:
        sender.close()
        for process in processes:
            check(f"{process.name}_stopped_by_sigint", process.stop())
        check("double.model_stopped_by_sigint", model.stop())
        graph.stop()
        node.destroy_node()
        rclpy.shutdown()
        deadline = time.monotonic() + 15
        while split_probe.leftovers() and time.monotonic() < deadline:
            time.sleep(0.5)
        check("double.no_process_left", not split_probe.leftovers(), str(split_probe.leftovers()))
    failed = [name for name, ok, _ in rp.RESULTS if not ok]
    print(f"CHECKS {len(rp.RESULTS) - len(failed)}/{len(rp.RESULTS)} passed; failed={failed}")
    print(f"QUEST_TELEOP_REAL_DOUBLE_PROBE={'PASS' if not failed else 'FAIL'}")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
