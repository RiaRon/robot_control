# OpenArm + RH56F1 팔·손 분리 bringup

팔과 손이 실행 프로세스, controller manager(CM), 하드웨어를 따로 가진다. 구조는 선배의
OpenArm-Tesollo bringup(`KUKU-Robot-Lab/robot_control` humble @ `a8328a1`)을 따른다.

- 양팔은 `openarm.bimanual.launch.py`처럼 `/controller_manager` 하나를 공유한다.
- 손마다 `dg5f_right_driver.launch.py`처럼 자기 namespace에 CM을 하나씩 가진다.

통합 모델(`generated/rl/openarm_rh56f1_bi_rl.urdf`)과 palm sensor 기준 IK는 그대로다.
기존 단일 CM bringup(`openarm.rh56f1_bimanual.launch.py`,
`openarm.rh56f1_bimanual_real.launch.py`)은 남아 있지만, 분리 bringup과 **동시에 실행하면
안 된다**(아래 "소유권").

검증 상태 (2026-10-03):

| 항목 | 상태 |
|---|---|
| fake 분리 구조(팔 CM, 손 CM 2개, 통합 모델) | Humble GenericSystem에서 검증 |
| Quest 합성 입력 → 분리된 fake 팔 추종 | 자동 회귀로 검증 |
| 실제 Quest 입력 → 분리된 fake 오른팔, RViz | **사용자가 확인** (2026-10-03). 위치 추종과 `--orientation-mode relative` 회전이 정상 동작 |
| real 팔 launch 구조(손 hardware 없음, inactive 시작) | 정적 검사와 GenericSystem 대역으로만 검증 |
| 실제 OpenArm 운동, 실제 손 | **미검증. 실행한 적 없음** |

## 1. 구조

| 대상 | launch | controller manager | hardware | controller | 원본 상태 토픽 |
|---|---|---|---|---|---|
| 양팔 | `openarm_rh56f1_arms.launch.py` | `/controller_manager` | fake: GenericSystem 14축 / real: `OpenArmHW` 팔당 7축 | fake: `right_/left_joint_trajectory_controller` (active)<br>real: `rh56f1_right_/left_arm_controller` (inactive로 load) | `/openarm/joint_states` |
| 오른손 | `rh56f1_right_hand.launch.py` | `/rh56f1_right/controller_manager` | fake: GenericSystem 6축 | `right_hand_trajectory_controller` | `/rh56f1_right/joint_states` |
| 왼손 | `rh56f1_left_hand.launch.py` | `/rh56f1_left/controller_manager` | fake: GenericSystem 6축 | `left_hand_trajectory_controller` | `/rh56f1_left/joint_states` |
| 통합 모델 | `openarm_rh56f1_model.launch.py` | 없음 | 없음 | 없음 | 입력만 받음 |

- 각 CM에 넘기는 description은 **통합 모델 전체 geometry + 그 CM이 소유한 ros2_control
  블록 하나**다.
  - 팔 CM은 손 resource를 갖지 않고, 손 CM은 팔 resource를 갖지 않는다.
  - passive/mimic 관절(`thumb_3`, `thumb_4`, `*_2`)은 어느 CM의 resource도 아니다.
  - description은 파라미터로 받으므로 각 CM은 다른 프로세스 없이 혼자 뜬다.
- 팔 launch는 손 launch를 시작하지도 기다리지도 않는다. 손 launch도 팔과 무관하다.
- real 손 launch는 없다. `rh56f1_hand.launch.py runtime:=real`은 거부된다(6절).
- 관절 이름은 이렇게 정했다.
  - fake: canonical(`r_aj_1`, `r_hj_thumb_1`), 이전 fake와 같다.
  - real: manifest source 이름(`openarm_right_joint1..7`, `rh56f1_right_right_thumb_1_joint`).
  - fake와 real의 controller 이름은 이전과 같고, namespace만 손 CM으로 바뀌었다.

## 2. 상태와 TF

```text
/openarm/joint_states       팔 14  ─┐
/rh56f1_right/joint_states  손 6   ─┼─▶ openarm_rh56f1_joint_state_merger
/rh56f1_left/joint_states   손 6   ─┘     ├─▶ /joint_states                          (측정값만)
                                          ├─▶ /openarm_rh56f1/display_joint_states   ─▶ robot_state_publisher ─▶ /tf, /tf_static
                                          └─▶ /openarm_rh56f1/joint_state_sources    (장치별 상태, JSON)
```

- **TF 발행자는 통합 모델의 robot_state_publisher 하나뿐이다.** 손 launch는 RSP를 띄우지
  않는다. Tesollo 손 launch와 다른 점이다. 같은 link를 두 RSP가 발행하지 않게 하려는
  것이다.
- **`/joint_states`에는 장치가 보낸 메시지가 도착한 그대로 한 번씩 전달된다.**
  - 그 장치가 소유한 관절만 담고, 원래 stamp를 유지하며, `header.frame_id`에 장치 id
    (`openarm`, `rh56f1_right`, `rh56f1_left`)를 넣는다.
  - 여러 장치를 한 메시지로 합치거나 기다리거나 다시 stamp를 찍지 않는다.
  - 그래서 손이 멈추면 그 손 관절만 더 나오지 않고, 팔 상태는 그대로 흐른다.
  - 세 장치가 모두 fresh이면 각 장치의 최신 메시지가 26개 독립 관절을 한 번씩 덮는다.
  - 팔만 있으면 14개만 나오고, 26개로 꾸미지 않는다.
  - 소유 관계는 이름으로만 판단하고 배열 순서는 보지 않는다.
  - 남의 관절은 버리고 개수를 센다. 비유한 값이 든 메시지는 통째로 버린다.
- **한 메시지에 26개를 다 담지 않는 이유**: 합친 메시지에 한 stamp를 찍으면 오래된 손
  값이 새 stamp를 달거나, 팔 TF가 같은 stamp로 반복되어 tf2가 갱신을 무시한다.
- **표시용 placeholder**: fresh 상태가 없는 손(기본 0.5 s 기준)은
  `display_joint_states`에만 열린 자세(0 rad)로 그린다.
  - 이 메시지의 `header.frame_id`는 `display_placeholder`이고, `/joint_states`에는 절대
    나오지 않는다.
  - 그 손의 실제 상태가 다시 fresh가 되면 placeholder는 즉시 멈춘다.
  - 손이 죽으면 손가락이 마지막 측정 자세로 멈춰 보이지 않고 열린 자세로 바뀌며,
    `joint_state_sources`에 `stale`로 표시된다.
  - `hand_display_placeholder:=false`로 끌 수 있다. 그러면 그 손의 손가락 TF가 나오지
    않는다.
- **FK 주의**: 팔 → `*_hl_palm_sensor`는 팔 관절과 고정 joint만으로 정해진다. 손이 없어도
  정확하고, Quest IK가 쓰는 frame이다. 손끝(`*_hl_<finger>_tip`)은 손가락 상태가 있어야
  정확하며, placeholder 동안의 손끝 TF는 측정이 아니다.
- merger는 아무것도 command하거나 activate하지 않는다. 입력 토픽이 출력 토픽과 같으면
  시작을 거부한다(순환 방지).

## 3. 소유권

같은 CM 이름이나 물리 장치를 두 launch가 동시에 가지면 안 된다. 각 launch는 시작 전에
다음을 확인한다(`launch/device_guard.py`).

| 검사 | 하는 launch | 내용 |
|---|---|---|
| CM 이름 | 분리 팔·손, 기존 fake·real 통합 | 같은 namespace에 `controller_manager`가 이미 있으면 거부. SIGKILL로 죽은 CM이 DDS에 남아 있으면 최대 25 s 기다렸다가 판단 |
| CAN 잠금 | real 팔(분리), real 통합 | `can_<interface>` 파일 잠금(`flock`)을 launch 프로세스가 끝날 때까지 쥔다. 같은 CAN을 두 launch가 열 수 없다 |
| 손 잠금 | 분리 손, real 통합(손 활성 시) | `rh56f1_<side>_hand` 파일 잠금 |

잠금 파일은 `/tmp/openarm_rh56f1_locks/`에 생긴다(`OPENARM_RH56F1_LOCK_DIR`로 변경 가능).
같은 기계, 같은 `/tmp`에서만 유효하다. 서로 다른 컨테이너에서 띄우면 공유 디렉터리를
지정해야 한다. CM 이름 검사는 DDS 발견에 기반하므로 보조 수단이다.

## 4. fake 실행 (Humble 컨테이너, 장치 없음)

호스트는 Jazzy이므로 기존 fake 전용 컨테이너를 쓴다. 이 컨테이너에는 CAN/USB/serial
장치가 없고 UDP 5006만 열려 있다.

```bash
# 터미널 A: 컨테이너 시작 (RViz를 쓰려면 QUEST_TELEOP_GUI=1)
cd ~/kuku_lab/robot_control
QUEST_TELEOP_GUI=1 tools/quest_teleop_fake_container.sh

# 다른 터미널은 모두 같은 컨테이너에 들어간다
docker exec -it quest-teleop-fake bash --rcfile /tmp/quest_env.sh
```

### 4.1 손 geometry를 유지한 팔 전용 fake bringup

```bash
# 터미널 B: 팔 CM + 통합 모델(RSP, merger) + RViz
ros2 launch openarm_bringup openarm_rh56f1_arms.launch.py runtime:=fake use_rviz:=true
```

손 launch가 없어도 RViz에는 장착된 InspireHand가 보이고(손가락은 열린 자세
placeholder), 팔을 움직이면 손 전체와 palm sensor가 따라간다. 통합 모델을 따로 띄우려면
`start_model:=false`를 주고 다음을 실행한다.

```bash
ros2 launch openarm_bringup openarm_rh56f1_model.launch.py runtime:=fake use_rviz:=true
```

### 4.2 손별 fake bringup (서로 다른 터미널)

```bash
# 터미널 C
ros2 launch openarm_bringup rh56f1_right_hand.launch.py
# 터미널 D
ros2 launch openarm_bringup rh56f1_left_hand.launch.py
```

### 4.3 CM, controller, resource 조회

```bash
ros2 control list_controllers -c /controller_manager
ros2 control list_controllers -c /rh56f1_right/controller_manager
ros2 control list_controllers -c /rh56f1_left/controller_manager

ros2 control list_hardware_components -c /controller_manager
ros2 control list_hardware_interfaces -c /controller_manager            # 팔 14 x position
ros2 control list_hardware_interfaces -c /rh56f1_right/controller_manager   # 손 6 x position

ros2 topic echo /openarm/joint_states --once
ros2 topic echo /rh56f1_right/joint_states --once
ros2 topic echo /openarm_rh56f1/joint_state_sources --once   # fresh / stale / never
ros2 topic info /tf                                           # Publisher count: 1
ros2 run tf2_ros tf2_echo body_root r_hl_palm_sensor
```

손 fake 명령 예 (fake 전용 값, 실물 보정 아님):

```bash
ros2 action send_goal /rh56f1_right/right_hand_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names:
  [r_hj_thumb_1, r_hj_thumb_2, r_hj_index_1, r_hj_middle_1, r_hj_ring_1, r_hj_pinky_1],
  points: [{positions: [0.4, 0.2, 0.5, 0.45, 0.55, 0.6], time_from_start: {sec: 2}}]}}"
```

### 4.4 Quest bridge와 teleop

`docs/quest-teleop.md`와 같다. 기본 `--runtime fake`는 분리된 팔을 쓴다. 상태는
`/openarm/joint_states`에서 읽고, description은 `/controller_manager`의
`robot_description` 파라미터에서 읽는다. 손 상태가 없어도 teleop은 동작한다.

```bash
# fake 팔을 굽힌 시작 자세로 (fake 전용 값)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names:
  [r_aj_1, r_aj_2, r_aj_3, r_aj_4, r_aj_5, r_aj_6, r_aj_7], points: [{positions:
  [0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"

python3 -m openarm_quest_teleop.ros_bridge
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime fake --execute
python3 -m openarm_quest_teleop.ros_teleop --arm left  --runtime fake --execute   # 왼팔
```

이전 단일 CM fake(`openarm.rh56f1_bimanual.launch.py`)에 붙이려면
`--runtime fake_integrated`를 쓴다.

### 4.5 독립 정지와 재실행

각 launch 터미널에서 Ctrl-C를 누르면 그 프로세스만 멈춘다.

- 손을 멈춰도 팔 상태와 팔 명령, 반대 손은 계속 동작한다. 멈춘 손은
  `joint_state_sources`에 `stale`로 나오고 손가락은 placeholder로 그려진다.
- 팔을 멈춰도 두 손의 CM은 계속 명령을 받는다. 팔 TF는 마지막 값에서 멈추고
  `openarm`이 `stale`로 나온다.
- 다시 실행할 때는 같은 명령을 쓴다. 프로세스가 비정상 종료한 직후에는 소유권 검사가
  DDS의 잔존 CM이 사라질 때까지 최대 25 s 기다린다.

### 4.6 자동 회귀

```bash
cd ~/kuku_lab/robot_control
tests/run_humble_split_bringup.sh            # 정적 테스트 + 분리 구조 + Quest + real 대역
tests/run_humble_split_bringup.sh split      # 분리 구조만
```

## 5. 정지 방법의 차이

| 동작 | 결과 | 비고 |
|---|---|---|
| controller 정지 (`ros2 control switch_controllers -c <CM> --deactivate <controller>`) | 그 controller가 더는 command interface에 쓰지 않는다 | fake: GenericSystem은 마지막 명령값을 유지한다. real 팔: 코드상 `OpenArmHW::write()`가 마지막 명령을 계속 보낸다. **실물에서 자세 유지인지 토크 해제인지는 확인 필요** |
| hardware deactivate (`ros2 control set_hardware_component_state -c <CM> <component> inactive`) | 그 component의 read/write 중지 | real 팔: 코드상 `on_deactivate`가 `disable_all()`을 호출한다(`openarm_simple_hardware.cpp`). 실물에서 팔이 떨어지는지는 **확인 필요**. 받칠 사람이나 거치대 없이 하지 않는다 |
| 프로세스 종료 (Ctrl-C) | 그 CM과 controller가 사라진다 | Humble이 종료 시 component를 deactivate하는지와 모터가 마지막 명령을 유지하는지는 **확인 필요** (Stage 4부터 미검증) |
| 물리 E-stop / 전원 차단 | 모두 멈춘다 | 소프트웨어에 의존하지 않는 유일한 정지 |

손 controller 정지와 손 hardware deactivate는 fake에서는 위와 같다. 실제 손에는 아직
backend가 없다.

## 6. real 팔 (사용자가 직접, 미검증)

실행 전 `docs/rh56f1-real-bringup.md`의 preflight(E-stop, 받침, CAN 확인, 현재 자세 기록)를
마친다. 기존 통합 real bringup은 띄우지 않는다. 띄워져 있으면 CAN 잠금 때문에 이 launch가
거부된다.

```bash
# 1) 팔만, 손 hardware 없이. 손 geometry와 palm frame은 모델에 있다.
ros2 launch openarm_bringup openarm_rh56f1_arms.launch.py runtime:=real \
  i_understand_this_moves_real_hardware:=true use_rviz:=true

# 2) read-only 확인. 아무것도 움직이면 안 된다.
ros2 control list_hardware_components -c /controller_manager   # 팔 2개 inactive, 손 없음
ros2 control list_controllers -c /controller_manager           # joint_state_broadcaster만 active
ros2 topic echo /openarm/joint_states --once                    # openarm_*_joint1..7, NaN 없음
ros2 run tf2_ros tf2_echo body_root r_hl_palm_sensor

# 3) 명시적 activation. 오른팔만, 측정 자세를 유지한다(auto_return_to_zero:false).
ros2 control set_hardware_component_state -c /controller_manager openarm_rh56f1_right_arm active
ros2 control switch_controllers -c /controller_manager --activate rh56f1_right_arm_controller

# 4) Quest: 먼저 dry run, 그다음 두 플래그를 모두 주고 실행
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime real
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime real --execute --confirm-real-hardware
```

startup 정책은 통합 real bringup과 같다. `real_bringup_safety.yaml`과 `OpenArmHW`는
바꾸지 않았다.

- 팔 component는 `inactive`로 시작한다.
- controller는 inactive로 load된다.
- `auto_return_to_zero:false`, `verify_state_before_enable:true`.
- activation은 2 s 이상 지난 뒤, fresh 상태로 seeding해서 한다.
- launch만으로는 어떤 운동 명령도 나가지 않는다.

## 7. 손 backend를 연결할 때 필요한 것

손 실물 제어는 다른 연구원이 맡는다. 그 코드는 독립 ROS 노드 형태이므로, 파일을 넣는다고
손 CM에 붙지 않는다. 연결하려면 **손 전용 hardware interface 플러그인**을 만들거나,
**검증된 어댑터**(노드 토픽 ↔ ros2_control)를 두어야 한다. 어느 쪽이든 다음을 정해야 한다.

| 항목 | 현재 계약 / 정해야 할 것 |
|---|---|
| 관절 이름·순서 | fake: `r_hj_thumb_1, r_hj_thumb_2, r_hj_index_1, r_hj_middle_1, r_hj_ring_1, r_hj_pinky_1` (manifest 순서)<br>real: manifest source 이름 `rh56f1_<side>_<side>_{thumb_1,thumb_2,index_1,middle_1,ring_1,little_1}_joint` (`pinky`↔`little`)<br>mimic 관절은 명령도 발행도 하지 않는다 |
| command | 손 CM의 `position` command interface 6개, rad, canonical URDF 한계(thumb_1 0–2.0944, thumb_2 0–0.4746, 손가락 0–1.5286) |
| state | `position` state interface 6개, rad. 손 CM의 joint_state_broadcaster가 `/rh56f1_<side>/joint_states`로 낸다. merger가 이 토픽을 읽는다 |
| raw↔rad | 정해지지 않음. 장치는 0.1° 각도 레지스터를 쓰고 값이 클수록 펴진다. 슬롯 순서는 새끼, 약지, 중지, 검지, 엄지 굽힘, 엄지 회전이다. `thumb_1/thumb_2`와 엄지 굽힘/회전의 대응, 부호, 영점을 한 곳에서만 변환해야 한다 |
| 장치 소유권 | 손 하나에 프로세스 하나. serial이면 `flock`+`TIOCEXCL`(기존 `serial_port.*`처럼). 같은 손을 다른 launch가 가지지 못하게 `rh56f1_<side>_hand` 잠금을 공유 |
| startup seeding | activation 시 측정값으로 명령을 초기화한다. 시작할 때 움직이지 않는다 |
| 명령 enable | 명시적 activation 전에는 쓰기 0. 명령 rate/step 제한 |
| stale/fault | 통신 끊김과 장치 오류를 ROS 쪽에 알린다(기존 플러그인의 `comm_ok`, `fault` state interface처럼). stale일 때 상태 발행을 멈추고, 오래된 값을 새 stamp로 내지 않는다 |
| 종료 | deactivate와 프로세스 종료 시 손이 하는 동작(유지 / 힘 해제)을 정하고 문서화 |
| 실행 형태 | `rh56f1_hand.launch.py runtime:=real`이 그 backend를 손 CM에 올리도록 연결. 지금은 거부한다 |

참고로 upstream `KUKU-Robot-Lab/robot_control` humble @ `a8328a1`에는 이미 다음이 있다.
로컬에 가져오지 않았고 검증하지 않았다.

- 벤더 RS485 손 노드(`ros_ws/src/inspire_rh56f1`)와 손별 launch
  (`rh56f1_right_driver.launch.py`). 인터페이스는 `/hand_<side>/angle_set`, `angle_actual`
  (0.1°, `rh56f1_interfaces`)다.
- 2026-10-02 README의 "실기 손은 EtherCAT(sim2real `rh56f1_ecat_node`)로 바꿨다"는 결정
  기록.
- 로컬 미추적 파일과 같은 이름의 `profiles/openarm_rh56f1.yaml`,
  `tests/test_profile_rh56f1.py`. 내용이 다르다. upstream profile은 hdgp asset과 다른
  manifest hash, 팔 group `openarm_<side>_arm`과 tip `*_hl_base`를 쓴다.

손 backend를 이 손 CM에 붙일지, upstream의 독립 노드를 어댑터로 감쌀지는 담당 연구원과
정할 일이다.
