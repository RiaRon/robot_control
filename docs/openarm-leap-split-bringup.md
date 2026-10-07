# OpenArm + LEAP Hand 팔·손 분리 bringup (fake, ROS 2 Jazzy)

OpenArm + RH56F1 분리 bringup(`docs/openarm-rh56f1-split-bringup.md`)과 같은 구조로 LEAP 손을 붙였다.
팔과 손이 실행 프로세스, controller manager(CM), 하드웨어를 따로 가진다. 지금은 **fake만** 있다.

검증 상태 (2026-10-07):

| 항목 | 상태 |
|---|---|
| fake 분리 구조(팔 CM, 손 CM 2개, 통합 모델) on Jazzy | `tests/jazzy_leap_split_probe.py` 19개 항목 PASS |
| description·소유권·controller 설정 | `tests/test_openarm_leap_split_bringup.py` (ROS 불필요) |
| RViz 화면 | 미확인 (`use_rviz:=true`로 띄울 수 있음) |
| 실제 LEAP 손(Dynamixel), 실제 OpenArm | **없음.** real runtime은 아직 구현하지 않았다 |

## 1. 구조

| 대상 | launch | controller manager | hardware | controller | 원본 상태 토픽 |
|---|---|---|---|---|---|
| 양팔 | `openarm_leap_arms.launch.py` | `/controller_manager` | GenericSystem 14축 | `right_/left_joint_trajectory_controller` | `/openarm/joint_states` |
| 오른손 | `leap_hand.launch.py side:=right` | `/leap_right/controller_manager` | GenericSystem 16축 | `right_hand_trajectory_controller` | `/leap_right/joint_states` |
| 왼손 | `leap_hand.launch.py side:=left` | `/leap_left/controller_manager` | GenericSystem 16축 | `left_hand_trajectory_controller` | `/leap_left/joint_states` |
| 통합 모델 | `openarm_leap_model.launch.py` (팔 launch가 기본으로 포함) | 없음 | 없음 | 없음 | 입력만 받음 |

- **모델:** urdf 저장소의 canonical RL 자산 `generated/rl/openarm_leap_bi_rl.urdf`와 그 manifest다(urdf 브랜치 `feat/leap-hand`).
  - Isaac 자산과 같은 파일이고, 메시 URI만 이 checkout으로 다시 잡는다.
  - 기본 경로는 launch 파일 위치에서 위로 올라가며 `urdf/generated/rl/openarm_leap_bi_rl.urdf`를 찾는다. `KUKU_LAB_ROOT`로 위치를 덮어쓸 수 있다.
- **관절 이름:** canonical 이름이다(`r_aj_1`, `r_hj_index_1`). LEAP는 mimic 관절이 없어서, 손마다 16개 모두가 그 손 CM의 resource다.
  - 모터 번호 대응은 manifest `source_to_canonical_joints`에 있다(`leap_right_1 → r_hj_index_1` 등).
- **CM에 넘기는 description:** 통합 모델 전체에, 그 CM이 소유한 ros2_control 블록 하나만 더한 것이다.
- **fake trajectory controller 준비:** RH56F1과 같은 `fake_trajectory_controller_prime.py`로, spawn 직후 한 번 측정 위치에 고정한다.
- **상태 합치기:** RH56F1과 같은 merger(`openarm_rh56f1_joint_state_merger.py`, 손 종류와 무관)를 쓴다. 출력 토픽은 다음과 같다.
  - `/joint_states`
  - `/openarm_leap/display_joint_states` (robot_state_publisher 입력)
  - `/openarm_leap/joint_state_sources`

### Jazzy 차이: description은 토픽으로 전달

Jazzy controller_manager(4.48)는 `robot_description`을 **파라미터로 받지 않고 토픽으로만** 받는다. 그래서 CM마다 `robot_description_publisher.py`를 붙여 자기 description을 latched(transient local)로 발행한다.

- 손 CM: `/leap_<side>/robot_description`
- 팔 CM: `/openarm_leap/arms/robot_description`
  - 루트의 `/robot_description`은 통합 모델의 robot_state_publisher가 발행하는데, ros2_control 블록이 없다. 팔 CM이 그걸 집지 않도록 토픽을 리매핑한다.

## 2. 실행

```bash
cd kuku_lab/robot_control/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/openarm_description src/openarm_ros2/openarm_bringup \
  --packages-select openarm_description openarm_bringup
source install/setup.bash

ros2 launch openarm_bringup openarm_leap_arms.launch.py use_rviz:=true   # 팔 + 통합 모델
ros2 launch openarm_bringup leap_hand.launch.py side:=right              # 별도 터미널
ros2 launch openarm_bringup leap_hand.launch.py side:=left               # 별도 터미널
```

손 명령 예시 (fake). 오른손 검지 MCP를 1.0 rad로 굽힌다:

```bash
ros2 action send_goal /leap_right/right_hand_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [r_hj_thumb_1, r_hj_thumb_2, r_hj_thumb_3, r_hj_thumb_4, r_hj_index_1, r_hj_index_2, r_hj_index_3, r_hj_index_4, r_hj_middle_1, r_hj_middle_2, r_hj_middle_3, r_hj_middle_4, r_hj_ring_1, r_hj_ring_2, r_hj_ring_3, r_hj_ring_4], \
  points: [{positions: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 1}}]}}"
```

컨트롤러가 `allow_partial_joints_goal: false`라서 16개 관절을 모두 보내야 한다. 순서는 manifest `control_joint_order`를 따른다.

## 3. 테스트

```bash
cd kuku_lab/robot_control
python3 -m pytest -q tests/test_openarm_leap_split_bringup.py                   # 정적
source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash
ROS_DOMAIN_ID=173 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 tests/jazzy_leap_split_probe.py
```

probe가 확인하는 것:
- CM 세 개의 controller가 active인지
- resource가 정확히 14 / 16 / 16이고 서로 겹치지 않는지
- `/joint_states`에 46개 관절이 모두 들어오는지
- merger가 세 장치를 fresh로 보고하는지
- `body_root → r_hl_index_tip` TF가 나오는지
- 오른손 `index_1`을 1.0 rad로 굽히면 손끝이 움직이고 손바닥은 그대로인지
- 오른팔 `r_aj_4`를 0.5 rad로 움직이면 손이 같이 따라가는지
- 왼손은 그대로인지
- SIGINT로 세 launch가 모두 종료되는지

## 4. 아직 없는 것

- real runtime. LEAP 손의 Dynamixel ros2_control 하드웨어와, OpenArm 팔의 real 경로 모두 없다.
- Quest·글러브 연동.
- 마운트 위치와 질량은 urdf 쪽 임시값이다(인스파이어 어댑터 높이, 벤더 질량). 어댑터가 확정되면 urdf에서 재생성한다.
