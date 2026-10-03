# Quest 3 컨트롤러 → OpenArm 팔 teleoperation 실행 안내

팔 전용이다. 손(RH56F1) backend, RS485, 손가락 명령은 이 경로에 없다. 손 모델은
palm frame(`r_hl_palm_sensor`, `l_hl_palm_sensor`)과 장착 geometry 때문에 description에만
남아 있고, teleop은 손 hardware와 손 controller에 손대지 않는다.

2026-10-03부터 기본 bringup은 팔·손 분리 구조다(`docs/openarm-rh56f1-split-bringup.md`).
팔은 `/controller_manager`, 손은 손마다 자기 CM을 쓴다. teleop의 `--runtime fake`/`real`은
분리된 팔 CM과 `/openarm/joint_states`를 쓰고, 손 상태가 없어도 동작한다. 이전 단일 CM
fake bringup에는 `--runtime fake_integrated`를 쓴다.

검증 상태 (2026-10-03):

| 항목 | 상태 |
|---|---|
| 합성 UDP 입력 → ROS pose → 상대 목표 → IK → fake 팔 추종 | Humble fake에서 자동 회귀로 검증 (분리 bringup) |
| 실제 Quest 입력 → 분리된 fake 오른팔, RViz 표시 | **사용자가 확인** (2026-10-03). 위치 추종과 `--orientation-mode relative` 회전이 정상 동작 |
| real description(source 관절 이름, `rh56f1_*_arm_controller`)에 대한 같은 경로 | 팔 플러그인을 GenericSystem으로 바꾼 test double에서 검증 |
| 실제 OpenArm 운동 | **미검증, 실행한 적 없음** |

실제 Quest 확인은 사용자가 수동으로 한 것이다. 그 실행의 패킷 기록은 저장소에 없고,
자동 회귀는 합성 패킷만 쓴다.

## 경로

```text
Quest 앱 (UDP JSON, port 5006)
  → openarm_quest_teleop.ros_bridge   /quest/<side>/pose (PoseStamped, frame quest_world)
                                      /quest/<side>/joy  (Joy)
  → openarm_quest_teleop.ros_teleop   enable 시점 기준 상대 palm 목표 → IK(팔 7축)
                                      → CommandGate(속도·lead·위치 제한)
  → /<팔 controller>/joint_trajectory (trajectory_msgs/JointTrajectory, 점 1개)
  → 기존 JointTrajectoryController → GenericSystem(fake) 또는 OpenArmHW(real)
```

코드: `robot_control/src/openarm_quest_teleop/`. 설정:
`robot_control/src/openarm_quest_teleop/config/quest_teleop.yaml`.

## 조작 방식

- **grip을 누르는 동안만 추종한다.** 누르는 순간의 컨트롤러 pose와 로봇 palm pose를
  기준으로 잡고, 그 뒤 컨트롤러가 움직인 만큼 palm이 움직인다. 누르는 순간에는 팔이
  움직이지 않는다.
- **놓으면 즉시 명령 전송을 멈춘다.** 팔은 마지막으로 명령된 관절 위치에 머문다.
- 다시 누르면 그 시점의 자세를 새 기준으로 잡는다. 놓은 동안 손을 옮겨도 점프하지 않는다.
- 입력이 끊기거나(0.2 s), pose가 유효하지 않거나, joint state가 끊기면 추종이 잠긴다
  (`locked`). 자동으로 재개하지 않는다. **grip을 놓았다가 다시 눌러야** 한다.
- 버튼과 threshold는 설정의 `teleop.enable`에서 바꾼다
  (`grip | trigger | button_primary | button_secondary`, 기본 press 0.7 / release 0.3).
- 방향은 기본 `orientation_mode: hold`(enable 시점의 palm 방향 유지)다.
  `--orientation-mode relative`를 주면 컨트롤러의 회전도 따라간다.

### 좌표

`quest_world`는 Unity world를 ROS 관례(x 앞, y 왼쪽, z 위)로 한 번만 바꾼 frame이다.
기본 설정(`heading.mode: world`, `axis_mapping` 단위행렬)에서는 **Quest의 world 앞쪽이
로봇의 앞쪽(`body_root` +x)** 이다. 로봇과 같은 방향을 보고 서서 Quest를 recenter(오른쪽
컨트롤러 Meta 버튼 길게)하면 앞/왼쪽/위가 그대로 맞는다. 맞지 않으면
`heading.yaw_offset_deg` 또는 `axis_mapping`을 고친다. 로봇을 마주 보고 조작하려면
`axis_mapping`을 `[[-1,0,0],[0,-1,0],[0,0,1]]`로 둔다.

### 시작 자세 (중요)

팔이 **곧게 아래로 늘어진 자세(모든 관절 0)는 특이점**이다. 팔꿈치가 하한에 닿아 있어
국소 IK가 거의 벗어나지 못한다. 사용자 결정(2026-10-02)으로 시작 특이점 차단은 꺼져 있다
(`min_enable_singular_value: 0.0`). 따라서 이 자세에서도 enable은 되고 첫 명령은 현재
자세 그대로지만, 많은 방향에서 `IK failed: target not reachable`가 나고 명령이 멈춘다.
**팔꿈치를 굽힌 자세에서 시작하는 것을 권한다.** 기존 `robotctl pose ready`의 자세(팔꿈치
0.8 rad)면 충분하다. 그 자세에서 아래로는 약 3 cm까지만 닿는다.

## 1. Quest 개발자 모드와 앱 설치

공식 앱과 수신 코드는 같은 프로토콜 조합이어야 한다. 이 수신기는
`enactic/dora-openarm-vr`(commit `072ce98`)의 패킷 형식을 그대로 쓴다.

1. [Meta Quest Developer 계정](https://developer.oculus.com/)을 만들고, 휴대폰 Meta Horizon
   앱에서 헤드셋의 개발자 모드를 켠다.
2. PC에 [Meta Quest Developer Hub](https://developer.oculus.com/meta-quest-developer-hub/)를
   설치하고 USB로 헤드셋을 연결해 디버깅을 허용한다.
3. `dora-openarm-vr` README의 teleoperation APK를 내려받는다:
   <https://drive.google.com/file/d/1lLDuoQAcl3YBKPE77F_1Z-8RzXEOFxpm/view?usp=drive_link>
4. Developer Hub에서 APK를 sideload한다 (또는 `adb install <파일>.apk`).

이 APK는 이번 작업에서 내려받거나 실행해 보지 못했다. 앱이 보내는 실제 패킷이 upstream
코드의 설명과 다르면 3단계의 진단 도구가 `REFUSED: …`로 어느 필드가 문제인지 보여 준다.

## 2. Quest와 PC 네트워크

1. Quest와 PC를 같은 네트워크(같은 공유기)에 둔다. PC의 IP를 확인한다: `ip -4 addr`.
2. 헤드셋에서 앱을 실행하고 **왼쪽 컨트롤러 메뉴 버튼**을 눌러 설정 패널을 연다.
   PC의 IP와 port `5006`을 입력한다.
3. 헤드셋을 목에 걸고 쓸 때는 화면이 꺼지지 않게 눈 사이 근접 센서를 테이프로 가린다
   (upstream README).
4. PC 방화벽이 켜져 있으면 UDP 5006을 연다: `sudo ufw allow 5006/udp`.

## 3. 컨트롤러 데이터만 수신 (ROS 불필요, 아무것도 명령하지 않음)

호스트에서 바로 실행한다 (python3와 numpy, PyYAML만 필요):

```bash
cd ~/kuku_lab/robot_control
PYTHONPATH=src python3 -m openarm_quest_teleop.monitor
```

출력: 좌우 컨트롤러 위치와 quaternion(`quest_world`, x y z w), grip/trigger/stick/버튼,
pose 유효 여부, PC 수신 시각과 경과 시간, 수신 빈도, malformed/refused 개수.

확인할 것:

- `rate`가 0이 아니고 `last packet … ago`가 계속 작다.
- 오른손을 **앞으로** 내밀면 `right pos`의 x가 커지고, **왼쪽**으로 옮기면 y, **위**로
  올리면 z가 커진다.
- grip을 쥐면 `grip`이 1에 가까워진다. 글러브에 장착한 상태에서 grip을 누르기 어렵다면
  `teleop.enable.source`를 `trigger`나 버튼으로 바꾼다.

패킷을 기록해 두려면 `--record /tmp/quest.jsonl`을 붙인다. Quest 없이 같은 경로를
확인하려면 다른 터미널에서 합성 입력을 보낸다:

```bash
PYTHONPATH=src python3 -m openarm_quest_teleop.synth --scenario still --seconds 5
PYTHONPATH=src python3 -m openarm_quest_teleop.synth --replay /tmp/quest.jsonl
```

monitor를 끝낸 뒤 다음 단계로 간다 (UDP 5006은 한 프로세스만 받는다).

## 4. ROS pose 확인 (fake 전용 Humble 컨테이너)

호스트는 Jazzy이므로 Humble 컨테이너에서 실행한다. 이 스크립트는 **장치를 하나도
전달하지 않고** UDP 5006만 연다. 실제 로봇에 닿을 수 없다.

```bash
# 터미널 A
cd ~/kuku_lab/robot_control
tools/quest_teleop_fake_container.sh
# (컨테이너 안)
python3 -m openarm_quest_teleop.ros_bridge
```

```bash
# 터미널 B, C, … : 같은 컨테이너에 들어간다
docker exec -it quest-teleop-fake bash --rcfile /tmp/quest_env.sh
ros2 topic echo /quest/right/pose --once
ros2 topic echo /quest/right/joy --once
ros2 topic echo /quest/status --once
ros2 topic hz /quest/right/pose
```

| 토픽 | 형식 | 내용 |
|---|---|---|
| `/quest/<side>/pose` | `geometry_msgs/PoseStamped` | `frame_id: quest_world`. pose가 유효한 패킷에서만 발행 |
| `/quest/<side>/joy` | `sensor_msgs/Joy` | `axes [trigger, grip, stick_x, stick_y]`, `buttons [primary, secondary, pose_valid]` (오른손 A/B, 왼손 X/Y) |
| `/quest/reference/pose` | `PoseStamped` | 패킷의 `rf` |
| `/quest/status` | `std_msgs/String` (JSON) | 수신 빈도, 마지막 패킷 경과 시간, malformed/refused 개수 |

`header.stamp`는 **PC가 패킷을 받은 시각**이다(헤드셋 시계도, 로봇 상태 시각도 아니다).
같은 패킷은 한 번만 발행한다. Quest가 끊기면 토픽이 조용해진다. 토픽 이름과
`frame_id`는 설정의 `quest.topics`, `quest.frame_id`에서 바꾼다.

## 5. fake 팔 추종

같은 컨테이너에서:

```bash
# 터미널 B: 분리 bringup의 팔 (GenericSystem 14축, 통합 모델과 RViz 포함)
ros2 launch openarm_bringup openarm_rh56f1_arms.launch.py runtime:=fake use_rviz:=true
# (선택) 손 fake CM. teleop에는 필요 없다.
ros2 launch openarm_bringup rh56f1_right_hand.launch.py

# 터미널 C: fake 팔을 굽힌 시작 자세로 (fake 전용 값)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names:
  [r_aj_1, r_aj_2, r_aj_3, r_aj_4, r_aj_5, r_aj_6, r_aj_7], points: [{positions:
  [0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"

# 터미널 C: 먼저 dry run (명령을 보내지 않고 상태만 보여 준다)
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime fake
# 이상 없으면 Ctrl-C 후 실행
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime fake --execute

# 터미널 D: 상태와 palm 위치
ros2 topic echo /quest_teleop/right/status
ros2 run tf2_ros tf2_echo body_root r_hl_palm_sensor
```

grip을 누르고 손을 움직이면 `status`의 `state`가 `engaged`가 되고 palm TF가 따라온다.
`reason`에는 명령을 보내지 않는 이유가, `limited`에는 명령을 제한한 요인(velocity / lead /
position / workspace limit)이 나온다. 왼팔은 `--arm left`다 (왼쪽 컨트롤러, 한 프로세스에
한 팔).

Quest 없이 확인하려면 터미널 D에서 합성 입력을 보낸다 (bridge가 받는다):

```bash
python3 -m openarm_quest_teleop.synth --scenario axes
```

자동 회귀 (합성 입력, 사람 개입 없음):

```bash
cd ~/kuku_lab/robot_control
tests/run_humble_split_bringup.sh quest     # 분리 bringup 위의 Quest 회귀
tests/run_humble_split_bringup.sh           # 분리 구조 + Quest + real 대역 전체
```

## 6. real 팔: 상태와 controller 확인

**여기부터는 사용자가 직접 실행한다.** 아래 절차는 실물에서 실행된 적이 없다. real
bringup 자체의 preflight, launch, read-only 확인, 정지 방법은
`docs/rh56f1-real-bringup.md`를 그대로 따른다. E-stop을 손이 닿는 곳에 둔다.

1. `docs/rh56f1-real-bringup.md`의 preflight를 마친다.
2. 분리 bringup의 real 팔을 띄운다. 손 geometry와 palm frame은 모델에 있고, 손
   hardware는 로드하지 않는다. 기존 통합 real launch는 함께 띄우지 않는다(같은 CAN을 잡으면
   거부된다):
   ```bash
   ros2 launch openarm_bringup openarm_rh56f1_arms.launch.py runtime:=real \
     i_understand_this_moves_real_hardware:=true
   ```
3. read-only 확인 (아무것도 움직이면 안 된다):
   ```bash
   ros2 control list_hardware_components -c /controller_manager   # 팔 2개 inactive, 손 없음
   ros2 control list_controllers -c /controller_manager           # joint_state_broadcaster만 active
   ros2 topic echo /openarm/joint_states --once                    # openarm_*_joint1..7, NaN 없음
   ros2 run tf2_ros tf2_echo body_root r_hl_palm_sensor
   ```
4. **팔꿈치를 굽힌 자세로 만든다.** 둘 중 하나:
   - 팔 component를 activate하기 전에(모터 꺼짐) 사람이 팔을 굽힌 자세로 받쳐 든 채
     activate한다. activation은 측정된 자세를 유지한다.
   - 또는 activate 후 기존 도구로 올린다 (이 profile로는 실행해 보지 않았다. `--execute`
     없이 먼저 출력만 확인한다):
     `robotctl pose ready --profile openarm_rh56f1 --group rh56f1_right_arm`
5. 오른팔만 켠다 (`docs/rh56f1-real-bringup.md` Step 3와 같다):
   ```bash
   ros2 control set_hardware_component_state -c /controller_manager openarm_rh56f1_right_arm active
   ros2 control switch_controllers -c /controller_manager --activate rh56f1_right_arm_controller
   ros2 control list_controllers -c /controller_manager   # rh56f1_right_arm_controller active
   ```
6. bridge를 띄우고 4단계처럼 `/quest/right/pose`가 들어오는지 본다:
   ```bash
   cd ~/kuku_lab/robot_control && export PYTHONPATH="$PWD/src:$PYTHONPATH"
   python3 -m openarm_quest_teleop.ros_bridge
   ```
7. **dry run**으로 real에 붙여 본다. 아무것도 발행하지 않는다:
   ```bash
   python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime real
   ros2 topic echo /quest_teleop/right/status
   ```
   grip을 누르고 손을 움직여 `state: engaged`, `target_xyz`가 손을 움직인 방향으로
   변하는지, `hardware`가 `openarm_hardware/OpenArmHW`인지, `reason`이 비어 있는지 본다.
   방향이 다르면 여기서 `axis_mapping` / `yaw_offset_deg`를 고친다.

## 7. real 팔 추종 실행 (명시적 enable)

real 실행은 기본 비활성이다. 플래그 **두 개가 모두** 있어야 명령을 발행한다.
`--execute`만 주면 `this is real hardware … refused`로 종료한다.

```bash
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime real \
  --execute --confirm-real-hardware
```

- 시작 로그에 `EXECUTING: commands go to /rh56f1_right_arm_controller/joint_trajectory`가
  나온다. 그 전에는 아무것도 보내지 않는다.
- controller가 active가 아니거나 관절 이름이 맞지 않으면 시작을 거부한다.
- 처음에는 grip을 누르고 **1~2 cm씩 천천히** 움직인다. 관절 속도는 profile의 2.0 rad/s,
  명령 lead는 0.2 rad, palm 목표는 enable 지점에서 0.4 m로 제한되지만, 이 값들은 실물에서
  튜닝되지 않았다. 처음에는 `position_scale`을 0.3 정도로 낮추는 것을 권한다.

## 8. 중지와 종료

| 동작 | 결과 |
|---|---|
| grip을 놓는다 | 즉시 명령 전송 중지. 팔은 마지막 명령 위치를 유지 (토크 유지) |
| teleop 터미널에서 Ctrl-C | 명령 전송 종료. controller가 마지막 위치를 유지 |
| `ros2 control switch_controllers --deactivate rh56f1_right_arm_controller` | 새 trajectory를 받지 않음. 토크 유지 |
| 물리 E-stop / 전원 차단 | 모든 것이 멈춤. **소프트웨어에 의존하지 않는 유일한 정지** |

팔 component를 `inactive`로 내리면 토크가 꺼져 팔이 떨어진다. 받칠 사람이나 거치대가
있어야 한다. 나머지 종료 절차와 주의는 `docs/rh56f1-real-bringup.md`의 *Stopping*을 따른다.
teleop은 종료할 때 팔을 home/zero로 보내지 않는다.

## 설정 요약

`config/quest_teleop.yaml`을 복사해 고친 뒤 `--config <파일>`로 넘긴다.

| 키 | 기본 | 뜻 |
|---|---|---|
| `quest.udp.port` | 5006 | 수신 port |
| `quest.frame_id`, `quest.topics.*` | `quest_world`, `/quest/...` | 발행 frame과 토픽 |
| `quest.smoothing.enabled` | false | upstream One Euro 필터 (미튜닝) |
| `teleop.enable.*` | grip, 0.7 / 0.3 | enable 입력과 threshold |
| `teleop.position_scale` | 1.0 | 손 이동 대비 palm 이동 비율 |
| `teleop.axis_mapping`, `teleop.heading.*` | 단위행렬, world, 0° | `quest_world` → 로봇 base 축 |
| `teleop.orientation_mode` | hold | `hold` 또는 `relative` |
| `teleop.max_target_offset_m` | 0.4 | enable 지점에서 palm 목표까지의 최대 거리 |
| `teleop.input_timeout_sec`, `joint_state_timeout_sec` | 0.2 | 이보다 오래된 입력/상태는 stale |
| `teleop.arms.<arm>` | | profile group, palm frame, 사용할 컨트롤러 쪽 |
