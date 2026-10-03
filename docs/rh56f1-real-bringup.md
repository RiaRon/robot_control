# OpenArm + RH56F1 real bringup: preflight, read-only check, first motion

This is the procedure for the first physical test of the OpenArm arms and the
RH56F1 hands under one `controller_manager`. It is written for the operator;
nothing in it has been run against real hardware yet.

> **Status (2026-09-23): hands NO-GO for ros2_control, arms software-ready but
> unverified on hardware.** The hands are RS-485 on separate FT232R adapters
> (user confirmation). A read-only raw diagnostic, `rh56f1_diag`, is ready for
> the operator to run (see *RH56F1 read-only raw check*). The ros2_control hand
> transport is still a stub that refuses to connect, because per-hand bus IDs,
> the vendor-angle to URDF-joint conversion, a single official raw range,
> rates, timeout and power-on/comm-loss behavior are open
> ([rh56f1-protocol-evidence.md](rh56f1-protocol-evidence.md)). Every hand
> motion step below is blocked.

## What the software guarantees, and what it does not

| Situation | What happens | Verified how |
|---|---|---|
| `ros2 launch …_real.launch.py` | Arm components configured, **inactive**: state is read, motors are not enabled, nothing is commanded. Hand components **unconfigured**: transport not opened. All four device controllers loaded inactive. | Humble probe (arm stand-in, real hand plugin) |
| Description started without this launch file (no `hardware_components_initial_state`) | `controller_manager` auto-activates at startup; both plugins refuse an activation sooner than 2.0 s after configure; Humble then aborts `ros2_control_node`. No motor is enabled. | Humble probe (hands); OpenArmHW gtest + review |
| Activating an arm component | Refused unless ≥2.0 s after configure. Then: disable command to every motor (never energizes) and wait for all 7 replies; enable; wait for fresh replies from all 7; refuse if any is missing, non-finite, or >0.05 rad outside its URDF limit; seed the command from the measured pose. No zero-return (`auto_return_to_zero: false`). | OpenArmHW gtest (decision logic) + review. **Not run on CAN.** |
| Activating a hand component | Refused unless ≥2.0 s after configure with ≥3 fresh reads and no device error; command seeded from measured pose. | Humble probe |
| Arm state stale >0.25 s, or non-finite state/command | Fault latched: no further command is sent, `read()`/`write()` return ERROR. | gtest (logic) + review |
| Hand stale/transport/device error, non-finite command | Fault latched, writes stop, ERROR returned; Humble then drops the component to `unconfigured`. Clears only on an explicit re-activation with healthy state. | Humble probe |
| Hand command outside limits | Clamped to the URDF limit, reported per joint on `/rh56f1_<side>_hand_status_broadcaster/dynamic_joint_states` (`command_clamped`) and in the log. | Humble probe |
| Hand motion | At most 0.02 rad per device write, writes ≥0.1 s apart (≤0.2 rad/s effective). These are Tesollo-derived placeholders, not RH56F1 values (no official rate exists in the manual). | Humble probe (mock) |

Not guaranteed by software: behavior when the whole process is killed or
`ros2 launch` is stopped with Ctrl-C (see *Stopping*), CAN bus faults the
motors do not report, and anything about the hands' real device.

Safety values live in one file:
`ros_ws/src/openarm_description/config/rh56f1/real_bringup_safety.yaml`
(with the source of each value).

## Joint names

The real runtime uses the canonical manifest's source names; the policy and
Fabric boundary keeps the canonical ones. `robotctl` converts between them
through the `openarm_rh56f1` profile.

| Canonical (policy/Fabric) | Runtime (`/joint_states`, controllers) |
|---|---|
| `r_aj_1..7` / `l_aj_1..7` | `openarm_right_joint1..7` / `openarm_left_joint1..7` |
| `r_hj_thumb_1`, `thumb_2`, `index_1`, `middle_1`, `ring_1`, `pinky_1` | `rh56f1_right_right_thumb_1_joint`, `…_thumb_2_joint`, `…_index_1_joint`, `…_middle_1_joint`, `…_ring_1_joint`, `…_little_1_joint` |
| `l_hj_*` | `rh56f1_left_left_*` (same pattern) |

TF frame names are unchanged: the palm control frames are
`r_hl_palm_sensor` and `l_hl_palm_sensor`.

## Before power-on

Do not start software until every item is confirmed by a person at the robot.

1. **Emergency stop.** Identify the physical E-stop or the power switch that
   cuts motor power for both arms, within reach of the operator for the whole
   session. Test it once with the robot unpowered. This is the only stop that
   does not depend on software.
2. **Support.** Deactivating an arm component disables its motors (torque off),
   so the arm falls under gravity. Have a stand, or a second person, able to
   hold each arm before any deactivation.
3. **Mounting.** Each RH56F1 is bolted to its arm's wrist adapter; no loose
   cables across joints.
4. **Which device is which.** Right arm CAN interface (default `can0`), left
   (`can1`); record which physical arm each interface drives. Hands: the adapters are known (left FT232R `BG0327KL`, right FT232R
   `BG033STU`; paths below). Still record each hand's nameplate model
   (RH56F1-E2L / E2R, with or without -T1) and serial number, and confirm
   which physical hand each adapter cable goes to.
5. **CAN up on the host**, at the bitrate the OpenArm motors use, before the
   container starts (`ip link show can0` shows `UP`). The container must not
   configure CAN itself.
6. **Workspace.** Clear the reach of both arms of people and objects.
7. **Current pose.** Photograph or note the pose of both arms and hands. Each
   joint must be inside its limit (activation refuses a joint more than
   0.05 rad outside).

## Launch (no motion)

Arms only, hands disabled (the only configuration usable today):

```bash
ros2 launch openarm_bringup openarm.rh56f1_bimanual_real.launch.py \
  i_understand_this_moves_real_hardware:=true \
  hand_configuration:=arm_only
```

Options:

- `enable_right_arm:=false` / `enable_left_arm:=false`: bring up without that
  arm (e.g. its CAN adapter is absent).
- `hand_configuration:=right|left|both` with `right_hand_transport:=rs485`
  (and `left_…`): the hand is declared but its transport is not opened at
  launch. Configuring it fails (fail-closed stub) until the Stage 5A gate
  items are resolved; see rh56f1-protocol-evidence.md.
- A mock hand is refused in this launch unless
  `allow_mock_hands_in_real_launch:=true`; its simulated state is then
  published only on `/rh56f1_<side>_hand_state_broadcaster/joint_states`,
  never on `/joint_states`, and its status reports `is_mock = 1`.

## Read-only check (nothing may move)

1. Hardware components:
   ```bash
   ros2 control list_hardware_components
   ```
   Arms `inactive`; hands `unconfigured`. Anything `active` → stop (E-stop)
   and investigate.
2. Controllers:
   ```bash
   ros2 control list_controllers
   ```
   `joint_state_broadcaster` active; `rh56f1_*_arm_controller` and
   `rh56f1_*_hand_controller` inactive.
3. Joint states:
   ```bash
   ros2 topic echo /joint_states --once
   ```
   Exactly the enabled arms' 7 joints each, named as in the table above, each
   once; positions match the recorded pose (radians); no NaN.
4. TF:
   ```bash
   ros2 run tf2_ros tf2_echo body_root r_al_7            # links keep canonical names
   ros2 run tf2_ros tf2_echo body_root r_hl_palm_sensor  # if the right hand is declared
   ```
5. Logs: no `FAULT`, `activation refused` or `no fresh state` lines.

Nothing should have moved. If anything did: E-stop, do not continue.

## RH56F1 read-only raw check (operator-run; not run by any agent)

What it does: it sends six **read** requests (ANGLE_ACT, FORCE_ACT,
CURRENT_ACT, ERROR, STATUS, TEMP) to one hand, once, and prints the request
bytes, reply bytes, status and decoded values in vendor raw units. The tool
has no write path. It does not convert to radians and it does not scan IDs.

Lab adapters (user-confirmed 2026-09-23; use these stable paths, never
`/dev/ttyUSB*`, which on this lab's PCs has also been the head DYNAMIXEL bus):

| Side | Path |
|---|---|
| left | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG0327KL-if00-port0` |
| right | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG033STU-if00-port0` |

`<LEFT_DEVICE_ID>` / `<RIGHT_DEVICE_ID>` are the hands' bus IDs, which are not
yet known. The factory default is 1 but it is not verified; ask the supplier
or check the hand's documentation. Baud 115200 is the official default.

1. Preconditions: arms powered off or their components inactive; hand clear
   of people and objects; hand power switch within reach; no other program
   (vendor GUI, `inspire_control_ros2`, ros2_control) has the adapter open.
   The tool refuses a port that is already open.
2. Dry run (opens nothing, prints the exact frames):

   ```bash
   ros2 run rh56f1_hardware rh56f1_diag --side left \
     --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG0327KL-if00-port0 \
     --device-id <LEFT_DEVICE_ID> --baud 115200 --dry-run
   ```
3. One read-only snapshot, left hand:

   ```bash
   ros2 run rh56f1_hardware rh56f1_diag --side left \
     --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG0327KL-if00-port0 \
     --device-id <LEFT_DEVICE_ID> --baud 115200 --execute-read-only
   ```
4. Right hand:

   ```bash
   ros2 run rh56f1_hardware rh56f1_diag --side right \
     --port /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG033STU-if00-port0 \
     --device-id <RIGHT_DEVICE_ID> --baud 115200 --execute-read-only
   ```
5. Optional bounded repetition: `--samples N --period-sec S`
   (N ≤ 20, 0.5 ≤ S ≤ 10). There is no continuous mode.

How to read the result:
- `status : ok` on every register means the ID and baud match. `timeout` on
  all of them means a wrong ID, wrong baud, wrong port, no power, or a wiring
  fault. Do not "try IDs" in a loop; establish the ID from documentation.
- `ERROR` values are decoded to bit names; `STATUS` to status names.
- Record the raw ANGLE_ACT values of both hands in a known pose (e.g. open).
  They are input to the conversion and limit work, and are not usable as
  radians.
- The tool prints `writes sent: 0` at the end.
- If the hand moves at power-on or when the port opens, cut its power, stop,
  and record what happened. The software cannot prevent that.

## First motion

Order is fixed; do not skip a step or continue past a failed one. Every
motion uses `robotctl rh56f1 smoke-test`, which is a dry run unless given
`--execute`, moves one joint relative to its measured position, and refuses
more than 0.05 rad (arm) / 0.03 rad (hand) or a speed outside
0.005–0.1 rad/s. Run each command once without `--execute` and read the
output first.

### Step 1 — one right-hand actuator: **blocked** (no real transport: IDs, conversion, limits, rates not established)

When a real transport exists (after a read-only preflight has been approved
and passed), the sequence will be: configure
(`ros2 control set_hardware_component_state rh56f1_right_hand inactive`),
activate its broadcasters, wait ≥2 s, activate the component, activate
`rh56f1_right_hand_controller`, then
`robotctl rh56f1 smoke-test --side right --device hand --joint r_hj_thumb_1 --delta 0.02 --velocity 0.02 --execute`.

### Step 2 — one left-hand actuator: **blocked** (same reason)

### Step 3 — one right-arm joint, slow and small

1. Arm the right arm (motors enable and hold the measured pose; nothing else
   should move):
   ```bash
   ros2 control set_hardware_component_state openarm_rh56f1_right_arm active
   ros2 control list_hardware_components        # right arm: active
   ```
   If it is refused, read the log line (`activation refused …`) and fix the
   cause; do not retry blindly.
2. Activate its controller:
   ```bash
   ros2 control switch_controllers --activate rh56f1_right_arm_controller
   ```
3. Move:
   ```bash
   robotctl rh56f1 smoke-test --side right --device arm --joint r_aj_2 --delta 0.05 --velocity 0.02
   robotctl rh56f1 smoke-test --side right --device arm --joint r_aj_2 --delta 0.05 --velocity 0.02 --execute
   ```
   - Expected: `openarm_right_joint2` moves +0.05 rad (≈2.9°) over ≈2.5 s.
   - Check: `openarm_right_joint2` in `/joint_states` converges to start+0.05;
     every other joint unchanged; no temperature spike.
   - Stop: E-stop; or `ros2 control switch_controllers --deactivate
     rh56f1_right_arm_controller` (the arm holds its last command, torque on).
   - Do not continue if: any other joint moved, the joint over- or
     under-shot by more than a few milliradians, or any fault was logged.
   - Return: rerun with `--delta -0.05`.

### Step 4 — one left-arm joint

Same as step 3 with `openarm_rh56f1_left_arm`, `rh56f1_left_arm_controller`,
`--side left --joint l_aj_2`.

### Step 5 — one arm and its hand together: **blocked** until steps 1–2 pass

### Step 6 — both arms and both hands: **blocked** until step 5 passes

## Stopping

| Action | Effect | Motor torque |
|---|---|---|
| Physical E-stop / power cut | Everything stops | Off (hardware) |
| `ros2 control switch_controllers --deactivate <controller>` | No new trajectory; the component keeps sending the last command | **On** — holds position |
| `ros2 control set_hardware_component_state openarm_rh56f1_<side>_arm inactive` | `disable_all()` sent three times | **Off** — the arm falls unless supported |
| `ros2 control set_hardware_component_state rh56f1_<side>_hand inactive` | Writes stop | Device behavior unknown (stage 5) |
| Ctrl-C on `ros2 launch` | `ros2_control_node` exits | **Not guaranteed off.** Whether Humble deactivates components on shutdown was not verified; a motor may keep its last command until its own CAN timeout, if one is configured. Use the E-stop. |

After any fault: stop, read the `FAULT latched` log line, fix the cause, then
deactivate and explicitly activate the component again (it re-verifies fresh
state and reseeds from the measured pose).
