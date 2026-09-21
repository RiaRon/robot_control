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
| `rh56f1_state_policy` | `parked` | `inactive` or `parked` |
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
