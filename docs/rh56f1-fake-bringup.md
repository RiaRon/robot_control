# OpenArm + RH56F1 Humble fake bringup

This bringup is a software-integration target only. It supports the canonical
bimanual OpenArm model with no hand, the left RH56F1, the right RH56F1, or both
RH56F1 hands. It cannot load a real RH56F1 backend and must not be used as a
physical startup procedure.

## Description structure

`openarm.rh56f1_bimanual.launch.py` reads the canonical geometry from
`urdf/generated/rl/openarm_rh56f1_bi_rl.urdf`. The adapter removes only the
disabled hand subtrees and resolves the canonical model's legacy mesh prefix.
It then passes that temporary variant to
`openarm_rh56f1_bimanual.urdf.xacro`, which includes the canonical model and
adds `openarm_rh56f1.bimanual.ros2_control.xacro`.

No geometry, joint origin, axis, limit, or mimic relation is copied into the
control Xacro. The stock `openarm.bimanual.launch.py` and its one-joint OpenArm
grippers remain unchanged. The RH56F1 launch never includes or spawns those
stock grippers.

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `canonical_urdf` | detected `openarm_rh56f1_bi_rl.urdf` | Absolute canonical URDF path |
| `hand_configuration` | `both` | `arm_only`, `left`, `right`, or `both` |
| `rh56f1_state_policy` | `parked` | `inactive`, `parked`, or `fake_commandable` |
| `manifest` | empty | Canonical manifest; default `<canonical_urdf stem>_manifest.yaml` (read only by `fake_commandable`) |
| `use_fake_hardware` | `true` | Fixed to `true`; there is no real backend |
| `namespace` | empty | Optional ROS namespace |
| `use_rviz` | `false` | Optional RViz process |

`inactive` keeps the selected RH56F1 geometry in `robot_description` but adds
no hand actuator to ros2_control. No hand controller exists and no hand command
interface is claimed.

`parked` adds the six independent actuator position command/state interfaces
for each selected hand to `mock_components/GenericSystem`. No hand controller
is declared or spawned, so all hand command interfaces remain unclaimed. Their
initial state is zero. Zero is inside all twelve canonical actuator limits,
but it is a **fake-only test pose**, not an asserted safe physical pose or a
real-hardware startup default. Passive joints are omitted from ros2_control and
follow the canonical URDF mimic relations in `robot_state_publisher`.

## Independent joint contract

The `both` + `parked` profile exposes 26 ros2_control state resources in this
order:

```text
r_aj_1 r_aj_2 r_aj_3 r_aj_4 r_aj_5 r_aj_6 r_aj_7
r_hj_thumb_1 r_hj_thumb_2 r_hj_index_1 r_hj_middle_1 r_hj_ring_1 r_hj_pinky_1
l_aj_1 l_aj_2 l_aj_3 l_aj_4 l_aj_5 l_aj_6 l_aj_7
l_hj_thumb_1 l_hj_thumb_2 l_hj_index_1 l_hj_middle_1 l_hj_ring_1 l_hj_pinky_1
```

The existing real-stack mapping remains unchanged and order preserving:

```text
r_aj_1..7 <-> openarm_right_joint1..7
l_aj_1..7 <-> openarm_left_joint1..7
```

Per hand, `thumb_3`, `thumb_4`, `index_2`, `middle_2`, `ring_2`, and `pinky_2`
are passive mimic joints and are not command/state resources.

## `fake_commandable`: fake-only hand motion tests

`fake_commandable` exports exactly the `parked` resources (same six actuators
per selected hand, same zero initial state, same GenericSystem) and in
addition spawns one fake-only `JointTrajectoryController` per selected hand:

| Controller | Joints (position command/state) |
|---|---|
| `right_hand_trajectory_controller` | `r_hj_thumb_1 r_hj_thumb_2 r_hj_index_1 r_hj_middle_1 r_hj_ring_1 r_hj_pinky_1` |
| `left_hand_trajectory_controller` | `l_hj_thumb_1 l_hj_thumb_2 l_hj_index_1 l_hj_middle_1 l_hj_ring_1 l_hj_pinky_1` |

The joint lists are not written in any file. At launch time they are taken from
the manifest `control_joint_order` (entries with the side's `*_hj_` prefix, in
that order) and must equal the hand resources of the rendered description, or
the launch fails. The launch writes them to a temporary parameter file for the
spawner and removes it on shutdown. The static
`openarm_rh56f1_fake_controllers.yaml` is unchanged and contains no hand
controller, so `parked` and `inactive` behave exactly as before.

The canonical <-> fake controller mapping is the identity: the four fake
controllers' joints, concatenated right arm, right hand, left arm, left hand,
equal the manifest `control_joint_order` and the profile
`profiles/openarm_rh56f1.yaml` groups. The real bringup uses the manifest
source names and `rh56f1_{side}_hand_controller` instead.

Hand targets in this mode are fake-only test values in URDF radians. They are
not physical poses and say nothing about the RH56F1 raw <-> radian conversion.

## Fake runtime flow

```text
left/right JointTrajectoryController
  -> position interfaces for 14 arm joints
  -> mock_components/GenericSystem
  -> joint_state_broadcaster
  -> /joint_states
  -> robot_state_publisher
  -> /tf and /tf_static
```

The parked hand state interfaces are published by the same
`joint_state_broadcaster`, but no controller claims their command interfaces.
`l_hl_palm_sensor` and `r_hl_palm_sensor` remain the canonical palm control
frames.

## Humble launch

Run this only in the isolated Humble container/overlay:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source /humble_overlay/install/setup.bash
export ROS_DOMAIN_ID=229
export ROS_LOCALHOST_ONLY=1
ros2 launch openarm_bringup openarm.rh56f1_bimanual.launch.py \
  canonical_urdf:=/workspace/kuku_lab/urdf/generated/rl/openarm_rh56f1_bi_rl.urdf \
  hand_configuration:=both \
  rh56f1_state_policy:=parked \
  use_fake_hardware:=true \
  use_rviz:=false
```

The runtime probe is:

```bash
python3 /workspace/kuku_lab/robot_control/tests/humble_fake_runtime_probe.py
```

It sends only a `0.03 rad` joint-2 goal to each fake arm. It never sends a hand
goal and must never be run against a real controller manager.

## Fake motion regression (arms and hands)

From the host:

```bash
cd ~/kuku_lab/robot_control
tests/run_humble_fake_motion_regression.sh          # all scenarios
tests/run_humble_fake_motion_regression.sh commandable_right
```

The script runs `thchzh/ros2:openarm-humble` as the host user with
`--network none`, `--cap-drop ALL`, `no-new-privileges`, a read-only root, no
`--device`, and kuku_lab mounted read-only. It builds only
`openarm_description` and `openarm_bringup` into a tmpfs overlay and does not
source the image's `/root/ros2_ws` overlay. It then runs the static tests,
`check_urdf` for every policy and hand configuration, and
`tests/humble_fake_hand_motion_probe.py`, which starts and stops the launch
itself:

- `parked_both`: no hand controller, hand command interfaces unclaimed, hands
  stay at zero, and the existing `humble_fake_runtime_probe.py` passes.
- `commandable_right`: right arm only, right hand only, right arm and hand
  together, then right arm only with a bent hand.
- `commandable_both`: each of the four groups alone, then all four together.

Before sending a goal, the probe requires that the only hardware component is
`mock_components/GenericSystem` and that every target is inside the URDF
limits. After each step it checks:

- every commanded joint reached its target, and every other joint never moved
  during the motion
- runtime TF matches offline FK of the joint states (mimic rules included)
- each mimic joint equals multiplier * master + offset
- palm and fingertip world and palm-relative TF changed or stayed put as the
  step requires
- `/joint_states` has a single publisher and each joint once
- controller names, claimed interfaces and joint order, and the exact set of
  command/state interfaces (no mimic joint)

Each scenario ends with SIGINT, then checks that no ROS process or node is
left. The probe prints `FAKE_MOTION_PROBE=PASS` or `FAIL`.
