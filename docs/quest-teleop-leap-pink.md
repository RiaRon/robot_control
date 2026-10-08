# Meta Quest 원격조종: OpenArm + LEAP Hand, pink IK (ROS 2 Jazzy, fake)

Quest 팔 원격조종(`docs/quest-teleop.md`, `src/openarm_quest_teleop`)의 IK를 **pink 하나로 바꾸고**(2026-10-08, 이전의 numpy DLS는 삭제), OpenArm + LEAP 분리 bringup(`docs/openarm-leap-split-bringup.md`)에 붙였다. RH56F1 설정도 같은 pink IK를 쓴다.

```text
Quest 앱 ─UDP JSON─▶ ros_bridge ─▶ /quest/<side>/pose, /joy
  ─▶ ros_teleop --config quest_teleop_leap.yaml
       클러치(grip) → 시작 시점 기준 상대 손바닥 목표 → pink IK (팔 7축, 손바닥 r_hl_palm) → CommandGate
  ─▶ /<side>_joint_trajectory_controller/joint_trajectory (100 Hz) ─▶ fake OpenArm (GenericSystem)
```

검증 상태 (2026-10-08):

| 항목 | 상태 |
|---|---|
| pink IK, 원격조종 코어, 시작 자세 (ROS 없음) | `tests/test_quest_teleop_leap_pink.py` 12개 통과 |
| 합성 Quest → bridge → pink teleop → fake 팔 (Jazzy), 시작 자세 이동 포함 | `tests/jazzy_leap_quest_teleop_probe.py` 17개 항목 PASS |
| 실제 Quest 헤드셋 | 미검증 |
| 실제 OpenArm, 실제 LEAP | **없음.** LEAP 설정에는 fake 런타임만 있다 |

## 1. 무엇이 바뀌었나

- **`pink_ik.py`:** 원격조종의 유일한 IK. `PinkIk.solve`는 `IkResult`(ik.py)를 돌려준다.
  - 한계는 반복 안에서 지키고, 예산 안에 못 맞추거나 시드에서 너무 멀면 거부한다.
  - 반복마다 pink QP 하나를 푼다: 손바닥 FrameTask + 시드 쪽 약한 PostureTask, profile의 위치·속도 한계, 솔버 daqp.
  - 모델은 실행 중인 description을 이 팔 7축으로 축소한 것이다(다른 팔·손·헤드는 0에 고정).
- **`ik.py`:** 설정(`IkSettings`)과 결과(`IkResult`)만 남았다. 이전 DLS(`solve_pose`)는 삭제했고, `ik.damping`과 `ik.max_iteration_step_rad`는 이제 쓰이지 않는다.
- **`teleop.py`:** `ArmTeleop(..., solve=...)`는 IK를 반드시 받는다. `config.build_teleop`이 pink를 만든다.
- **`config.py`:** `teleop.pink` 설정으로 PinkIk를 만든다. pink가 없으면 설치 방법을 담은 `ConfigError`를 낸다.
- **`config/quest_teleop_leap.yaml`:** LEAP 설정.
  - profile `openarm_leap`, 손바닥 `r_hl_palm` / `l_hl_palm`
  - description 토픽 `/openarm_leap/arms/robot_description` (Jazzy controller manager에는 파라미터가 없다)
  - fake 런타임만 있음
- **`src/robot_control/profiles/openarm_leap.yaml` + `components/leap.yaml`:** LEAP 로봇 정의.
  - 46관절이고, 자산 manifest는 urdf 저장소의 `generated/rl/openarm_leap_bi_rl_manifest.yaml`이다.
  - 손 관절의 원래 이름은 LEAP 모터 번호다(`leap_right_0..15`).

pink로 바꾸기 전에 한 비교 (같은 LEAP 팔, 100 Hz 10 cm 원 추종):

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

# 터미널 B: Quest UDP → ROS
.venv/bin/python -m openarm_quest_teleop.ros_bridge --config $CFG

# 터미널 C: 원격조종 (먼저 dry run, 이상 없으면 --execute: 시작 자세로 옮긴 뒤 grip을 기다린다)
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake --execute

# Quest 없이: 합성 입력 (grip, 앞/왼쪽/위 5 cm 갔다 오기)
.venv/bin/python -m openarm_quest_teleop.synth --config $CFG --scenario axes
ros2 topic echo /quest_teleop/right/status
```

- 실제 Quest 앱은 이 PC의 UDP 5006으로 보낸다(설정 `quest.udp`).
- 왼팔은 `--arm left`다. 한 프로세스에 한 팔이다.
- **시작 자세:** `--execute`로 띄우면 grip을 받기 전에 그 팔을 `rh56f1_aglt_home`으로 먼저 옮긴다.
  - 이 자세는 RH56F1 로봇의 홈이다. 출처는 sim2real `deploy/policy_control/config/homes/rh56f1_aglt.yaml`(`2f8a803`, 9/29)로, hdgp `rh_aglt` 시작 자세이고 왼팔은 거울이다. 10/06까지 RH56F1 미션의 home·rehome 단계가 이 자세를 썼다.
  - 값은 robot_control `poses/openarm_leap.yaml`에 있다.
  - 이동 방식: 측정 자세에서 FollowJointTrajectory 한 번. 가장 많이 움직이는 관절 기준 0.3 rad/s, 최소 3초. 도착(모든 관절 0.02 rad 이내)을 확인한 뒤에만 원격조종 루프가 시작된다.
  - 설정은 `teleop.start_pose`다. `--start-pose NAME`으로 바꾸고, `--no-start-pose`로 건너뛴다. dry run은 움직이지 않고 "옮길 예정"만 알린다.
  - 완전히 편 영점 자세(팔이 곧게 아래로)는 특이 자세라 IK가 빠져나오지 못한다. `--no-start-pose`로 영점에서 시작하지 말 것.

### 시작 자세 주변의 작업 공간 (녹화 때 확인)

`rh56f1_aglt_home`은 RH56F1 과제용 자세라, LEAP로 원격조종할 때 여유가 작은 관절이 있다.
같은 동작을 ROS 없이 원격조종 코어에 넣어 재현해 확인했다.

- **손목 `aj_6`:** 0.58 rad라 한계 0.785까지 약 0.2 rad(12°)만 남는다. 그 방향으로 손목을 30° 돌리면 한계에 걸리고, IK가 "닿지 않음"으로 명령을 멈춘다. 반대 방향은 여유가 크다.
- **팔꿈치 `aj_4`:** 1.76 rad이고 완전히 접힌 2.443까지 0.68 rad가 남는다. 손을 위·몸 안쪽으로 크게 옮기면 손바닥이 어깨에 가까워져 팔꿈치가 한계에 닿는다. 그때 IK는 다른 관절을 크게 재배치해야 하는 해(시드에서 0.5 rad 초과)를 거부하고, 팔은 마지막 명령 자세를 유지한다.
- 이 두 경우는 **의도된 안전 동작**이다. 팔이 튀지 않고 멈춘다.
- 녹화 동작(앞 10 cm, 바깥 6 cm, 아래 6 cm, 손목 −20°/+20°, 반지름 5 cm 원)은 양팔 모두 IK 실패와 속도 제한 0회였다. 관절 여유는 최소 0.084 rad였다.

녹화: `figure/openarm_leap_pink_quest_teleop.{mp4,webm}` (약 28초, 20 fps, RViz, 회전 추종 모드).
- 영점 → 시작 자세(5.9초) → 합성 Quest 양손 입력.
- 양손 목표 자세(`/quest_teleop/<arm>/target_pose`)를 좌표축으로 표시해, 손바닥이 그 축을 따라가는 것이 보인다.

## 4. 테스트

```bash
cd kuku_lab/robot_control
.venv/bin/python -m pytest -q tests/test_quest_teleop.py tests/test_quest_teleop_leap_pink.py   # pink 없는 python3에서는 teleop 코어 항목 skip
source /opt/ros/jazzy/setup.bash && source ros_ws/install/setup.bash
ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST .venv/bin/python tests/jazzy_leap_quest_teleop_probe.py
```

probe는 UDP 15006을 쓴다(실제 앱의 5006과 겹치지 않게). 확인 항목은 다음과 같다.
- 팔 controller active, 영점에서 출발 → teleop이 시작 자세로 이동(약 5.9초, 오차 0.02 rad 이내), 도착 전에는 연결 안 됨
- teleop이 pink와 `r_hl_palm`으로 실행되는지
- 연결 1회, 명령 스트리밍, IK 오차 허용 범위 안, IK 실패 없음
- 손바닥이 앞·왼쪽·위로 각 5 cm 따라가고, 목표와의 차이가 1 cm 미만, 끝나면 원위치
- 놓으면 idle, 왼팔 무변화, 세 프로세스 정상 종료

## 5. 아직 없는 것

- 실제 Quest 헤드셋 확인
- real 런타임 (팔 real launch의 Jazzy 대응, LEAP Dynamixel ros2_control)
- 손가락 원격조종 (Quest 손 추적 또는 글러브 → LEAP 16관절)
- Humble 컨테이너 회귀(`tests/run_humble_*.sh`)의 이미지에는 pink가 없어, 그 안의 Quest teleop probe는 pink를 설치하기 전까지 실패한다.
- `.rosdistro`는 아직 `humble`이다. profile의 `endpoint()`는 humble 항목을 읽지만, LEAP profile은 humble과 jazzy에 같은 값(100 Hz)을 두었다.
