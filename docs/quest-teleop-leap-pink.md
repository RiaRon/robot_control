# Meta Quest 원격조종: OpenArm + LEAP Hand, pink IK (ROS 2 Jazzy, fake)

기존 Quest 팔 원격조종(`docs/quest-teleop.md`, `src/openarm_quest_teleop`)에서 **IK만 pink로 바꿔** OpenArm + LEAP 분리 bringup(`docs/openarm-leap-split-bringup.md`)에 붙였다.

```text
Quest 앱 ─UDP JSON─▶ ros_bridge ─▶ /quest/<side>/pose, /joy
  ─▶ ros_teleop --config quest_teleop_leap.yaml
       클러치(grip) → 시작 시점 기준 상대 손바닥 목표 → pink IK (팔 7축, 손바닥 r_hl_palm) → CommandGate
  ─▶ /<side>_joint_trajectory_controller/joint_trajectory (100 Hz) ─▶ fake OpenArm (GenericSystem)
```

검증 상태 (2026-10-08):

| 항목 | 상태 |
|---|---|
| pink IK 단위 테스트, 원격조종 코어 (ROS 없음) | `tests/test_quest_teleop_leap_pink.py` 7개 통과 |
| 합성 Quest → bridge → pink teleop → fake 팔 (Jazzy) | `tests/jazzy_leap_quest_teleop_probe.py` 15개 항목 PASS |
| 실제 Quest 헤드셋 | 미검증 |
| 실제 OpenArm, 실제 LEAP | **없음.** LEAP 설정에는 fake 런타임만 있다 |

## 1. 무엇이 바뀌었나

- **`pink_ik.py`:** `PinkIk.solve`는 `ik.solve_pose`와 같은 계약을 따른다.
  - `IkResult`를 돌려주고, 한계는 반복 안에서 지키며, 예산 안에 못 맞추거나 시드에서 너무 멀면 거부한다.
  - 반복마다 pink QP 하나를 푼다: 손바닥 FrameTask + 시드 쪽 약한 PostureTask, profile의 위치·속도 한계, 솔버 daqp.
  - 모델은 실행 중인 description을 이 팔 7축으로 축소한 것이다(다른 팔·손·헤드는 0에 고정).
- **`teleop.py`:** `ArmTeleop(..., solve=...)`로 IK를 주입받는다. 기본은 기존 DLS라 RH56F1 동작은 그대로다.
- **`config.py`:** `teleop.ik_backend: dls | pink`와 `teleop.pink` 설정을 받는다. pink가 없으면 설치 방법을 담은 `ConfigError`를 낸다.
- **`config/quest_teleop_leap.yaml`:** LEAP 설정.
  - profile `openarm_leap`, 손바닥 `r_hl_palm` / `l_hl_palm`
  - description 토픽 `/openarm_leap/arms/robot_description` (Jazzy controller manager에는 파라미터가 없다)
  - fake 런타임만 있음, `ik_backend: pink`
- **`src/robot_control/profiles/openarm_leap.yaml` + `components/leap.yaml`:** LEAP 로봇 정의.
  - 46관절이고, 자산 manifest는 urdf 저장소의 `generated/rl/openarm_leap_bi_rl_manifest.yaml`이다.
  - 손 관절의 원래 이름은 LEAP 모터 번호다(`leap_right_0..15`).

DLS와 pink 비교 (같은 LEAP 팔, 100 Hz 10 cm 원 추종):

| | 실패 | 평균 위치 오차 | 시간 평균 / 최대 |
|---|---|---|---|
| DLS | 0/400 | 0.11 mm | 0.22 / 1.92 ms |
| pink | 0/400 | 0.01 mm | 0.19 / 0.57 ms |

닿을 수 없는 목표는 둘 다 거부한다. 관절 한계 근처에서는 pink의 반복 횟수가 더 적었다(0.7회 vs 1.3회).

## 2. pink 설치 (한 번만, sudo 불필요)

Ubuntu 24.04의 시스템 pip는 직접 설치를 막고, `python3-venv`(ensurepip)도 없다. 그래서 pip 없이 venv를 만들고 시스템 pip로 그 venv에 설치한다. ROS(rclpy)와 시스템 numpy를 함께 쓰도록 `--system-site-packages`로 만든다.

```bash
cd kuku_lab/robot_control
python3 -m venv --without-pip --system-site-packages .venv
python3 -m pip --python .venv/bin/python install "pin-pink==4.4.0" daqp
.venv/bin/python -c "import pinocchio, pink; print(pinocchio.__version__, pink.__version__)"   # 4.1.0 4.4.0
```

`.venv/`는 git에서 무시된다. numpy는 시스템 것(2.x)을 그대로 쓴다. 버전을 고정하지 말 것.

## 3. 실행 (fake)

```bash
cd kuku_lab/robot_control
source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash
export PYTHONPATH=$PWD/src:$PYTHONPATH
CFG=src/openarm_quest_teleop/config/quest_teleop_leap.yaml

# 터미널 A: fake 팔 + 모델 + RViz (손은 선택: leap_hand.launch.py side:=right)
ros2 launch openarm_bringup openarm_leap_arms.launch.py use_rviz:=true

# 터미널 B: 오른팔을 굽힌 시작 자세로 (fake 전용 값)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names:
  [r_aj_1, r_aj_2, r_aj_3, r_aj_4, r_aj_5, r_aj_6, r_aj_7], points: [{positions:
  [0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"

# 터미널 C: Quest UDP → ROS
.venv/bin/python -m openarm_quest_teleop.ros_bridge --config $CFG

# 터미널 D: 원격조종 (먼저 dry run, 이상 없으면 --execute)
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake --execute

# Quest 없이: 합성 입력 (grip, 앞/왼쪽/위 5 cm 갔다 오기)
.venv/bin/python -m openarm_quest_teleop.synth --config $CFG --scenario axes
ros2 topic echo /quest_teleop/right/status
```

- 실제 Quest 앱은 이 PC의 UDP 5006으로 보낸다(설정 `quest.udp`).
- 왼팔은 `--arm left`다. 한 프로세스에 한 팔이다.
- **완전히 편 영점 자세(팔이 곧게 아래로)에서는 시작하지 말 것.** 특이 자세라 IK가 빠져나오지 못한다. 위처럼 굽힌 자세에서 grip을 누른다. 필요하면 설정의 `min_enable_singular_value`로 영점 근처에서 연결을 거부하게 할 수 있다(현재 0 = 꺼짐).

## 4. 테스트

```bash
cd kuku_lab/robot_control
.venv/bin/python -m pytest -q tests/test_quest_teleop_leap_pink.py    # pink 없는 python3에서는 pink 항목 skip
source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash
ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python tests/jazzy_leap_quest_teleop_probe.py
```

probe는 UDP 15006을 쓴다(실제 앱의 5006과 겹치지 않게). 확인 항목은 다음과 같다.
- 팔 controller active, 시작 자세 도달
- teleop이 pink와 `r_hl_palm`으로 실행되는지
- 연결 1회, 명령 스트리밍, IK 오차 허용 범위 안, IK 실패 없음
- 손바닥이 앞·왼쪽·위로 각 5 cm 따라가고, 목표와의 차이가 1 cm 미만, 끝나면 원위치
- 놓으면 idle, 왼팔 무변화, 세 프로세스 정상 종료

## 5. 아직 없는 것

- 실제 Quest 헤드셋 확인
- real 런타임 (팔 real launch의 Jazzy 대응, LEAP Dynamixel ros2_control)
- 손가락 원격조종 (Quest 손 추적 또는 글러브 → LEAP 16관절)
- `.rosdistro`는 아직 `humble`이다. profile의 `endpoint()`는 humble 항목을 읽지만, LEAP profile은 humble과 jazzy에 같은 값(100 Hz)을 두었다.
