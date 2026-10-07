# Nova2 글러브 → RH56F1 fake 손 (분리 bringup, Quest 팔과 함께)

SenseGlove Nova2 글러브 입력으로 분리 bringup의 손별 fake trajectory controller를
움직인다. Quest 팔 추종과 동시에 RViz에서 확인할 수 있다. 팔 CM, 오른손 CM, 왼손 CM은 서로
독립이고, 통합 모델(RSP 하나, merger 하나)은 바뀌지 않았다. 분리 bringup 자체는
`docs/openarm-rh56f1-split-bringup.md`, Quest는 `docs/quest-teleop.md`를 본다.

검증 상태 (2026-10-04):

| 항목 | 상태 |
|---|---|
| 합성 Nova2 입력 → 원본 retarget 노드 → adapter → fake 손 (한 손, 양손) | Humble GenericSystem 자동 회귀로 검증 |
| Quest 합성 입력(팔) + 합성 글러브(손) 동시 | 자동 회귀로 검증 |
| dry run(명령 0), 단일 command owner, stale/invalid 잠금과 명시적 재개, kill/재시작 | 자동 회귀로 검증 |
| 실제 Nova2 글러브와 드라이버 | **미검증. 실행한 적 없음.** 이 작업에서 글러브 hardware와 햅틱을 켜지 않았다 |
| 실제 RH56F1 손 | **backend 없음.** 실제 손에 연결하는 경로는 구현하지 않았다 |
| 실제 OpenArm 운동 | **미검증** (변화 없음) |

> 손가락 각도 변환은 **fake 표시용 매핑**이다. 실물 raw↔URDF 보정이 아니며, 실제 손이
> 같은 글러브 입력에 같은 자세를 만든다는 뜻이 아니다.

## 1. 구조

### 1.1 이전 (친구 코드 `inspire_hand-main.zip`, 2026-10-03 수령)

```text
SenseGlove Nova2 ─BT─▶ senseglove_ros ─▶ /senseglove/glove<serial>/<rh|lh>/joint_states
                                              │
                                              ▼
                       senseglove_inspire_retarget (retarget_node.py)
                                              │  /inspire_<side>/retarget/joint_states (목표값 6개)
                                              ▼
                       hand_bridge_node ─RS485(/dev/ttyUSB*)─▶ 실제 Inspire 손
```

브리지는 ros2_control 플러그인이 아니라 serial을 직접 여는 독립 노드다. 시작과 종료 때
손을 연다. 실제 각도(angleAct)를 발행하지 않는다.

### 1.2 이번 (fake)

```text
/senseglove/glove<serial>/<rh|lh>/joint_states    (글러브 드라이버, 또는 합성 글러브)
   ▼
senseglove_inspire_retarget_<side>                (원본 retarget_node.py, 수정 없음)
   ▼  /inspire_<side>/retarget/joint_states       (목표값. joint state가 아님)
rh56f1_glove_teleop.ros_adapter                   (/rh56f1_<side>/glove_adapter)
   ▼  /rh56f1_<side>/<side>_hand_trajectory_controller/joint_trajectory   (--execute일 때만)
/rh56f1_<side>/controller_manager: <side>_hand_trajectory_controller → mock_components/GenericSystem
   ▼  joint_state_broadcaster → /rh56f1_<side>/joint_states (측정값)
openarm_rh56f1_joint_state_merger → /joint_states, display_joint_states → RSP 하나 → /tf → RViz
```

- `hand_bridge_node`, serial, 햅틱, 로거, 캘리브레이션 스크립트, `emergency_open`은 fake
  launch에 없다. 원본에서 실행하는 것은 retarget 노드 하나뿐이다.
- 목표값은 `/joint_states`에도 `display_joint_states`에도 나오지 않는다. 손 TF는 손 CM의
  측정 상태로만 그린다.
- 새 RSP, 새 merger, 새 TF 발행자는 없다. 팔 CM과 반대 손 CM은 이 경로와 관계없다.

### 1.3 파일

| 위치 | 내용 |
|---|---|
| `~/kuku_lab/third_party/inspire_hand-main_2026-10-03/` | 받은 zip 원본(sha256 `f9130984…5668`)과 압축 해제본. 수정하지 않는다. robot_control 저장소 밖 |
| `third_party/inspire_hand_senseglove_teleop/senseglove_teleop/` | 원본의 retarget 부분만 바이트 단위로 같게 복사. `package.xml`/`setup.py`가 없으므로 colcon 패키지가 아니다(중복 패키지 없음) |
| `vendor_metadata/inspire_hand_senseglove_teleop/UPSTREAM.yaml` | 출처, zip 해시, 라이선스(MIT, 원본 `package.xml` 선언. zip에 최상위 LICENSE 없음), 날짜, 파일별 sha256, 가져오지 않은 파일 |
| `src/rh56f1_glove_teleop/adapter.py` | 순수 Python: 이름 기반 매핑, 검증, clutch, CommandGate, fake 손 확인 |
| `src/rh56f1_glove_teleop/ros_adapter.py` | rclpy 노드 |
| `src/rh56f1_glove_teleop/config/glove_adapter.yaml` | 토픽, 이름 매핑, 엄지 정책, 시간 설정 |
| `src/rh56f1_glove_teleop/synth_glove.py` | 합성 Nova2 JointState 발행기 |
| `openarm_bringup/launch/rh56f1_glove_input.launch.py` | 손 하나의 retarget 노드 + adapter |
| `openarm_bringup/scripts/fake_trajectory_controller_prime.py` | fake JTC를 spawn 직후 한 번 현재 위치로 hold (9절) |

## 2. 연결 계약

### 2.1 producer(retarget 노드) 계약. 원본에서 다시 확인한 사실

| 항목 | 값 | 근거 |
|---|---|---|
| 입력 | `/senseglove/glove<serial>/<rh\|lh>/joint_states`, `sensor_msgs/JointState`. `{p}{finger}_pip`(손가락 폐색), `{p}thumb_pip`(엄지 pitch), `{p}thumb_brake`(엄지 yaw)를 쓴다. `p` = `r_`/`l_` | `retarget_node.py`, `mapping.py` |
| 출력 | `/inspire_<side>/retarget/joint_states`. 이름 순서 `pinky_proximal_joint, ring_proximal_joint, middle_proximal_joint, index_proximal_joint, thumb_proximal_pitch_joint, thumb_proximal_yaw_joint` | `mapping.JOINT_NAMES` |
| 단위·범위 | rad, 0 = 완전히 폄, 최대 = 완전히 쥠: 1.47 ×4, 0.6, 1.308 | `mapping.JOINT_LIMITS_RAD` |
| 성질 | **목표값**이다. 측정값이 아니다 | |
| 무효 입력 | 필요한 관절이 빠지거나 값이 유한하지 않으면 그 프레임을 발행하지 않는다(경고만) | `_on_glove` |
| 기본 serial | 노드 기본값 `00795`. 이 launch는 `glove_serial:=`을 필수로 받는다 | |
| 보정 | `config/mapping.yaml`(2026-09-28 자동 보정). pinky와 ring의 zero/range가 같다. 코드에서 두 손가락을 묶지는 않는다 | |

브리지에 대한 사실(fake에는 쓰지 않는다): 시작할 때 speed 1500, force 1000을 쓰고 손을
연다(엄지 굽힘 1350, 회전 950). 종료할 때도 연다. 엄지 회전은 항상 950이다. 굽힘은
`drive_thumb`가 꺼져 있으면 1350이고, 켜지면 [1100, 1350]으로 제한된다. angleAct는 읽어
발행하지 않는다. 오른손 기본 `/dev/ttyUSB1`, ID 2는 확인되지 않았다. 주석에는 angleSet이
1046부터라고 되어 있지만 상수는 `REG_ANGLE_SET = 1040`이다.

### 2.2 adapter 매핑 (`glove_adapter.yaml`)

| source (이름으로 찾음) | canonical actuator | closed (rad) |
|---|---|---|
| `pinky_proximal_joint` | `<p>_hj_pinky_1` (real source 이름은 `little_1`) | 1.47 |
| `ring_proximal_joint` | `<p>_hj_ring_1` | 1.47 |
| `middle_proximal_joint` | `<p>_hj_middle_1` | 1.47 |
| `index_proximal_joint` | `<p>_hj_index_1` | 1.47 |
| `thumb_proximal_pitch_joint` | `<p>_hj_thumb_2` | 0.6 |
| `thumb_proximal_yaw_joint` | `<p>_hj_thumb_1` | 1.308 |

```text
canonical = clamp(source / closed, 0, 1) * upper      (lower = 0, upper는 canonical URDF/profile)
```

- 배열 위치는 보지 않는다. 왼손 joint_state_broadcaster는 이름 순서가 다르지만
  (`thumb_1, index_1, thumb_2, …`) 결과는 같다.
- `pinky`↔`little`, 엄지 대응은 이 표 한 곳에서만 정한다.
- 손가락마다 따로 매핑한다. pinky와 ring을 묶지 않는다.
- mimic 관절(`*_2`, `thumb_3`, `thumb_4`)은 명령하지 않는다. URDF mimic으로 따라온다.

### 2.3 엄지

| 근거 | thumb_1 | thumb_2 |
|---|---|---|
| canonical URDF | parent `<p>_hl_base`, 축 ≈ −z, 0–2.094 rad, mimic 없음 | `thumb_1`의 자식, 0–0.4746 rad, `thumb_3`(×1.1425)와 `thumb_4`(×0.7508)를 구동 |
| 해석 | 회전(yaw) | 굽힘(pitch) |
| producer 이름 | `thumb_proximal_yaw_joint` (글러브 `thumb_brake`) | `thumb_proximal_pitch_joint` (글러브 `thumb_pip`) |
| 원본 브리지 | 회전은 항상 950 고정 | 굽힘은 기본 1350 고정 |

실제 손의 엄지 굽힘/회전 레지스터와 `thumb_1/thumb_2`의 물리 대응, 부호, 영점은 확인되지
않았다. 기본 정책은 원본 브리지 기본 동작에 맞춘 **두 축 모두 `hold`**다. 따라 움직이기
시작할 때의 측정값에 고정한다. `--thumb-pitch follow`, `--thumb-yaw follow`(launch:
`thumb_pitch:=follow`, `thumb_yaw:=follow`)는 fake에서 위 표의 매핑으로 따라가게 한다.

## 3. 안전 동작

| 상황 | 동작 |
|---|---|
| 기본 | **dry run**. 매핑 결과를 `~/status`에만 내고 controller 토픽에는 아무것도 발행하지 않는다 |
| `execute:=true` | 먼저 손 CM에 묻는다. hardware가 `mock_components/GenericSystem`뿐이고, command interface가 이 손의 6개 position이며, `<side>_hand_trajectory_controller`가 active이고 그 joint가 manifest 순서와 같아야 한다. 하나라도 아니거나 CM이 응답하지 않으면(30 s) 거부(exit 2) |
| command owner | 손마다 하나. `rh56f1_<side>_hand_command` flock(손 CM 자신의 `rh56f1_<side>_hand` 잠금과 별개)을 쥐고, 명령 토픽에 이미 publisher가 있으면 거부한다. CM 잠금을 막지 않는다 |
| 목표값 검증 | 길이 불일치, 중복 이름, NaN/Inf, 필요한 이름 누락은 메시지째 거부하고 센다. 유한하지만 범위 밖인 값은 clamp하고 `targets_clamped`, `last_clamped`로 알린다. 모르는 이름은 무시한다 |
| 속도 | robot_control `CommandGate`. profile 속도와 측정값 기준 lead(속도 × 0.1 s)로 제한한다 |
| stale | 목표값이나 손 측정값이 0.5 s 넘게 오래되면 `locked`. 명령을 멈추고 **스스로 재개하지 않는다** |
| 재개 | `ros2 service call /rh56f1_<side>/glove_adapter/enable std_srvs/srv/Trigger`. 그 호출 **이후에** 도착한 목표값부터 쓴다 |
| 손 CM 재시작 | 측정값이 끊기므로 `locked`. 새 CM이 뜬 뒤 enable해야 다시 움직인다 |
| 프로세스 관계 | adapter와 retarget 노드는 다른 프로세스를 죽이거나 시작하지 않는다. adapter가 죽어도 팔, 반대 손, 손 CM은 계속 동작한다 |

시간 설정(`timing`): `target_timeout_sec 0.5`, `joint_state_timeout_sec 0.5`,
`command_rate_hz 50`, `stream_horizon_sec 0.0`.

상태: `/rh56f1_<side>/glove_adapter/status` (`std_msgs/String`, JSON, 5 Hz). `state`,
`reason`, `target`, `command`, `limited`, `targets_accepted/rejected/clamped`,
`last_rejection`, `commands`, `command_subscribers`, `thumb`, `mapping`(fake 매핑 경고)를
담는다.

## 4. 빌드

### 4.1 Docker (권장, 호스트는 Jazzy)

`tools/quest_teleop_fake_container.sh`가 Humble 이미지에서 `openarm_description`과
`openarm_bringup`을 빌드하고 `PYTHONPATH`에 `robot_control/src`를 넣는다. glove 쪽은 별도
빌드가 필요 없다. adapter와 합성 글러브는 `robot_control/src`의 Python 모듈이고, retarget
노드는 launch가 vendored 경로를 `PYTHONPATH`에 넣어 실행한다. 이 컨테이너에는 CAN/USB/serial
장치가 없다.

### 4.2 native Humble

```bash
source /opt/ros/humble/setup.bash
cd ~/kuku_lab/robot_control/ros_ws
colcon build --packages-select openarm_description openarm_bringup
source install/setup.bash
export PYTHONPATH=~/kuku_lab/robot_control/src:$PYTHONPATH
export KUKU_LAB_ROOT=~/kuku_lab
```

필요한 ROS 패키지: `rclpy`, `sensor_msgs`, `std_srvs`, `trajectory_msgs`, `control_msgs`,
`controller_manager_msgs`, `ros2_control`, `ros2_controllers`. retarget 노드에는 rclpy와
sensor_msgs만 필요하다.

## 5. fake 실행 (터미널별)

```bash
# 터미널 A: 컨테이너 (RViz)
cd ~/kuku_lab/robot_control
QUEST_TELEOP_GUI=1 tools/quest_teleop_fake_container.sh
# 다른 터미널은 같은 컨테이너에
docker exec -it quest-teleop-fake bash --rcfile /tmp/quest_env.sh
```

### 5.1 순서 1: 한 손

```bash
# B: 팔 CM + 통합 모델 + RViz (손 geometry 포함)
ros2 launch openarm_bringup openarm_rh56f1_arms.launch.py runtime:=fake use_rviz:=true
# C: 오른손 CM
ros2 launch openarm_bringup rh56f1_right_hand.launch.py
# D: 오른손 글러브 입력. 먼저 dry run
ros2 launch openarm_bringup rh56f1_glove_input.launch.py side:=right glove_serial:=SYNTH
# E: 합성 글러브 (실제 글러브 대신)
python3 -m rh56f1_glove_teleop.synth_glove --side right --serial SYNTH --scenario wave
```

dry run에서 확인한다.

```bash
ros2 topic echo /rh56f1_right/glove_adapter/status --once   # "executing": false, "commands": 0, target 값
ros2 topic info /rh56f1_right/right_hand_trajectory_controller/joint_trajectory  # Publisher count: 0
```

그다음 D를 Ctrl-C로 멈추고 실행 모드로 다시 띄운다.

```bash
ros2 launch openarm_bringup rh56f1_glove_input.launch.py side:=right glove_serial:=SYNTH execute:=true
```

RViz에서 오른손 네 손가락이 wave로 쥐었다 편다. 엄지는 `hold`라 움직이지 않는다. 합성
글러브 시나리오는 다음과 같다(값은 closure 0–1, 0 = 폄).

| 시나리오 | 명령 |
|---|---|
| 유지 | `--scenario hold --closure 0.4` |
| 폄 / 쥠 | `--scenario open` / `--scenario close` |
| 반복 | `--scenario wave --period 4` |
| 손가락 하나씩 | `--scenario independent --closure 0.8 --period 2` (검지 → 중지 → 약지 → 새끼) |
| 특정 손가락 고정 | `--set ring=0.1 --set pinky=0.9` (어느 시나리오에나 덧붙임) |
| 엄지 확인 | adapter를 `thumb_pitch:=follow thumb_yaw:=follow`로 띄우고 `--set thumb_yaw=1.0` |

### 5.2 순서 2: Quest 팔과 함께

`docs/openarm-rh56f1-split-bringup.md` 4.4절과 같다. 글러브 입력과 Quest는 서로 다른 CM,
다른 명령 토픽을 쓴다.

```bash
python3 -m openarm_quest_teleop.ros_bridge
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime fake --execute
```

### 5.3 순서 3: 양손

```bash
ros2 launch openarm_bringup rh56f1_left_hand.launch.py
ros2 launch openarm_bringup rh56f1_glove_input.launch.py side:=left glove_serial:=SYNTH execute:=true
python3 -m rh56f1_glove_teleop.synth_glove --side left --serial SYNTH --scenario independent
```

### 5.4 확인할 토픽

| 토픽 | 기대 |
|---|---|
| `/senseglove/glove<serial>/<rh\|lh>/joint_states` | 글러브(또는 합성) 23개 관절 |
| `/inspire_<side>/retarget/joint_states` | 목표값 6개(producer 이름) |
| `/rh56f1_<side>/glove_adapter/status` | `state: following`, `executing: true`, `command_subscribers ≥ 1` |
| `/rh56f1_<side>/<side>_hand_trajectory_controller/joint_trajectory` | publisher 1개(adapter). execute일 때만 |
| `/rh56f1_<side>/joint_states` | 측정값. 손가락이 목표를 따라간다 |
| `/joint_states` | 26개 측정 관절. producer 이름은 없다 |
| `/openarm_rh56f1/joint_state_sources` | 세 장치 모두 `fresh` |
| `/tf` | publisher 1개(통합 RSP) |

### 5.5 정지와 재실행

- 글러브 입력(D) Ctrl-C: 그 손은 마지막 명령 위치에 머문다(fake). 팔과 반대 손은 계속
  동작한다. 다시 띄우면 처음부터 다시 확인한다.
- 합성 글러브(E) Ctrl-C: 0.5 s 뒤 adapter가 `locked`. 글러브를 다시 켠 다음 enable한다.
  `ros2 service call /rh56f1_right/glove_adapter/enable std_srvs/srv/Trigger`
- 손 CM(C) Ctrl-C: adapter가 `locked`. C를 다시 띄우고 enable한다.
- 같은 손 adapter를 두 개 띄우면 두 번째는 거부된다.

## 6. 실제 Nova2 드라이버 (사용자가 직접. 미검증)

로컬에는 공식 드라이버 `Adjuvo/senseglove_ros`(jazzy 브랜치 @ `30c9624`)가 **호스트
Jazzy** `~/senseglove_ws`에 빌드되어 있다. Humble 이미지에는 SenseGlove 패키지가 없다.

- `senseglove_bringup/launch/senseglove.launch.py`는 `config/gloves.yaml`의 글러브마다
  `hardware.launch.py`를 연다. 각 글러브는 `/senseglove/glove<serial>/<rh|lh>` namespace에
  CM, `joint_state_broadcaster`, `senseglove_state_broadcaster`, `robot_state_publisher`를 둔다.
- **같은 launch가 `haptics_controller`(brake와 palm strap에 effort를 쓰는
  JointTrajectoryController)를 active로 spawn한다.** 이 작업의 어떤 코드도 그 controller에
  명령하지 않는다. 햅틱을 쓰지 않으려면 드라이버를 띄운 뒤
  `ros2 control switch_controllers -c /senseglove/glove<serial>/<rh|lh>/controller_manager --deactivate haptics_controller`를
  실행해야 하는지, 그렇게 해도 되는지를 드라이버 문서로 먼저 확인한다.
- 드라이버의 robot_state_publisher는 글러브 링크 TF를 발행한다. 같은 ROS domain에서는
  `/tf` publisher가 2개가 된다(링크 이름은 다르다). 자동 회귀의 "TF 발행자 1개" 검사는
  드라이버가 없을 때 기준이다.

연결 절차 (미검증):

1. 호스트에서 SenseCom으로 글러브를 연결하고 드라이버를 띄운다.
   `ros2 topic echo /senseglove/glove<serial>/<rh|lh>/joint_states --once`로 serial과 손
   방향을 확인한다.
2. retarget 노드와 adapter가 그 토픽을 받아야 한다.
   - 호스트 Jazzy에서 직접 실행: 4.2절의 native 방식은 Humble 기준이다. Jazzy에서 이
     bringup 전체를 돌린 적은 없다.
   - fake 컨테이너에서 실행: 지금 컨테이너는 bridge 네트워크 + `ROS_LOCALHOST_ONLY=1`이라
     호스트 노드를 볼 수 없다. host network와 같은 `ROS_DOMAIN_ID`가 필요하고, Jazzy↔Humble
     DDS 상호 운용도 확인해야 한다.
3. `glove_serial:=<serial>`로 먼저 dry run을 하고, status의 target이 손 동작과 맞는지 본 뒤
   `execute:=true`로 바꾼다. 이때도 대상은 fake 손이다.

## 7. 친구의 최신 손 코드로 교체할 때

받은 zip은 이전 버전이다. 최신 코드로 바꿀 때 **retarget 출력 계약**이 2.1절과 같으면
adapter는 바꿀 필요가 없다.

교체 방법:

- launch 인자로 최신 checkout을 가리킨다.
  `retarget_source:=<최신 senseglove_teleop 디렉터리>`,
  `retarget_params:=<그 보정 yaml>`. 그 디렉터리 아래에
  `senseglove_teleop/retarget_node.py`가 있어야 한다.
- vendored 사본을 바꾸려면 원본 zip이나 커밋을 `third_party/`에 보관한다. 그다음
  `UPSTREAM.yaml`의 출처, 해시, 날짜, 파일별 sha256을 갱신하고
  `tests/test_rh56f1_glove_teleop.py`의 snapshot 해시 검사를 같이 갱신한다. 사본은 수정하지
  않는다. 차이는 adapter 설정에서 흡수한다.

체크리스트:

- [ ] 입력 토픽과 글러브 관절 이름(`{p}{finger}_pip`, `{p}thumb_pip`, `{p}thumb_brake`)이
      같은가. 다르면 합성 글러브(`synth_glove.joint_names`)도 맞춘다.
- [ ] 출력 토픽 `/inspire_<side>/retarget/joint_states`와 6개 이름이 같은가. 다르면
      `glove_adapter.yaml`의 `source.topic`과 `source.joints`만 고친다.
- [ ] 단위(rad), 0 = 폄, 최대값(`closed`)이 같은가. 다르면 `closed`를 고친다.
- [ ] 출력이 여전히 **목표값**인가(측정값처럼 쓰이지 않는가).
- [ ] 무효 입력을 어떻게 처리하는가(발행 안 함 / NaN 발행). adapter는 둘 다 거부하지만
      stale 판단 시간은 달라진다.
- [ ] 엄지 축의 의미(pitch/yaw)와 범위. 바뀌면 2.3절 표와 `thumb` 정책을 다시 정한다.
- [ ] pinky/ring 보정이 실제로 분리되었는가(adapter는 묶지 않는다).
- [ ] 새 노드가 serial, 햅틱, 로거를 같이 열지 않는가. 연다면 retarget 부분만 실행하는
      진입점이 있어야 한다.
- [ ] `tests/run_humble_split_bringup.sh glove`를 다시 돌려 통과하는가.

## 8. 실제 손 backend가 갖춰야 할 것 (미구현)

이 adapter는 **fake 손에만** 명령한다(GenericSystem이 아니면 거부). 실제 손을 손 CM에
붙이려면 `docs/openarm-rh56f1-split-bringup.md` 7절의 backend가 필요하다. 원본 브리지를
그대로 쓰면 안 되는 이유와 backend 요구사항은 다음과 같다.

| 요구 | 원본 브리지 | 필요한 것 |
|---|---|---|
| 시작·종료 동작 | 시작과 종료 때 자동으로 손을 연다 | 시작 때 움직이지 않는다. 종료 때 동작(유지 / 힘 해제)을 정하고 문서화한다 |
| 실제 각도 | angleAct를 읽어 발행하지 않는다 | angleAct를 읽어 state interface와 `/rh56f1_<side>/joint_states`로 낸다 |
| raw↔rad | 없음(보정 각도를 직접 씀) | 레지스터(0.1°, 클수록 펴짐) ↔ canonical rad 변환을 한 곳에서. 엄지 대응, 부호, 영점 포함 |
| seeding | 없음 | activation 때 측정값으로 명령을 초기화 |
| enable | 노드 시작이 곧 구동 | 명시적 activation 전에는 쓰기 0 |
| 장치 소유 | serial을 직접 연다 | 손 하나에 프로세스 하나, 배타적 serial, `rh56f1_<side>_hand` 잠금 공유 |
| stale/fault | 없음 | 통신 끊김과 장치 오류를 알리고, 오래된 값을 새 stamp로 내지 않는다 |
| 종료 | `destroy_node`에서 손을 연다 | deactivate와 종료 동작을 명시한다 |

속도, 힘, 보정값 같은 튜닝 수치는 정하지 않았다. 실물 시험에서 담당 연구원이 정한다.

## 9. 알려진 문제: joint_trajectory_controller 2.47.0의 토픽 명령 무시

Humble 이미지의 `ros-humble-joint-trajectory-controller` 2.47.0은
`std::atomic<bool> rt_has_pending_goal_;`을 초기화하지 않는다(C++17 빌드에서는 값이
정해지지 않는다). 이 값은 action goal이 끝날 때만 false가 된다. 값이 true로 시작하면
`update()`가 `~/joint_trajectory` 토픽 명령을 **로그 없이 모두 버린다**. controller는
active이고 서비스에 응답하며 상태도 발행하지만, 토픽 명령(Quest teleop, glove adapter,
`ros2 topic pub`)으로는 움직이지 않는다. 어떤 controller 인스턴스가 걸리는지는 실행마다
다르다. 이번 회귀에서는 손이 간헐적으로 움직이지 않는 문제로 나타났다. 왼손 CM과 글러브
입력만 띄운 격리 실험 15번 중 6번 멈췄다. 멈춘 상태에서는 CLI publisher의 명령도
무시되었고, CM과 controller 서비스는 정상 응답했다. upstream은 이후 `{false}`로
초기화한다.

대응(fake만): `rh56f1_<side>_hand.launch.py`와 `openarm_rh56f1_arms.launch.py runtime:=fake`는
trajectory controller spawner가 끝나면 `fake_trajectory_controller_prime.py`를 한 번
실행한다. 이 스크립트는 hardware가 GenericSystem뿐인지 확인하고, controller의 측정 위치를
그대로 담은 FollowJointTrajectory goal을 보내 SUCCEEDED를 기다린다. 움직임은 0이고, 그
뒤로는 토픽 명령이 들어간다. 적용 후 같은 격리 실험은 6번 모두 통과했다(왼손 3, 오른손 3).

**real 팔 launch에는 넣지 않았다.** 실제 팔에서 토픽 명령이 무시되는지는 확인하지 않았다.
real에서 같은 문제가 생기면 controller 패키지 업데이트나 같은 hold goal을 운용 절차에
넣을지를 결정해야 한다.

## 10. 자동 회귀

```bash
cd ~/kuku_lab/robot_control
tests/run_humble_split_bringup.sh glove                          # 정적 테스트 + glove fake probe
GLOVE_SCENARIO=dry_run tests/run_humble_split_bringup.sh glove   # dry run만
tests/run_humble_split_bringup.sh                                # 분리 구조 + Quest + glove + real 대역
python3 -m pytest -q tests/test_rh56f1_glove_teleop.py           # 호스트 단위 테스트
```

`tests/humble_glove_fake_probe.py`가 검사하는 것:

- `dry_run`: 명령 0, serial 미사용, hand bridge 없음, 응답 없는 CM이면 거부.
- `combined`: 팔, 양손, 양손 글러브, Quest를 함께 띄운다.
  - 손별 독립 추종과 mimic, 손만 움직일 때와 팔만 움직일 때의 TF/FK.
  - Quest 팔과 글러브 손 동시 동작, 명령 경로 분리, `/joint_states`에 목표값 없음.
  - stale 잠금과 enable 재개, invalid/NaN 거부, 두 번째 owner 거부.
  - adapter와 손 CM의 kill 및 재시작, 잔여 프로세스와 ROS 노드 없음.
