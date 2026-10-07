# 다른 PC에서 OpenArm + RH56F1 fake / Quest 환경 재현하기

`openarm_rh56f1_*`, `rh56f1_*` bringup, `openarm_quest_teleop`, 관련 테스트를 다른 PC에서
그대로 돌리기 위한 준비 절차다. 실물 로봇은 다루지 않는다. 실물 절차는
`docs/openarm-rh56f1-split-bringup.md` 6절과 `docs/rh56f1-real-bringup.md`에 있다.

## 1. 필요한 저장소와 commit

robot_control은 canonical 모델을 **형제 디렉터리의 `urdf` 저장소**에서 읽는다.

- profile `src/robot_control/profiles/openarm_rh56f1.yaml`이
  `../../../../urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml`을 가리키고, sha256으로
  검사한다.
- launch는 `urdf/generated/rl/openarm_rh56f1_bi_rl.urdf`를 찾는다.

두 저장소를 같은 부모 디렉터리(아래에서는 `~/kuku_lab`) 밑에 둔다.

| 디렉터리 | 저장소 | 브랜치 / commit | 용도 |
|---|---|---|---|
| `kuku_lab/robot_control` | https://github.com/RiaRon/robot_control.git | `feature/rh56f1-fake-bringup` (이 문서가 들어간 commit 이상) | 코드, launch, 테스트 |
| `kuku_lab/urdf` | https://github.com/KUKU-Robot-Lab/urdf.git | `main` @ `88f886cc0bf2b092adafa67f03e166fe6df39638` | **필수.** canonical URDF, manifest, 메시 78개 모두 이 commit에 커밋되어 있음 |
| `kuku_lab/hdgp` | https://github.com/KUKU-Robot-Lab/hdgp.git | `main` @ `0fd488b6f7042e13a706e67ac017a768b9645518` | 선택. 기존 Tesollo profile과 hdgp export 테스트용. `pytest` 전체를 돌릴 때만 필요 |
| `kuku_lab/third_party/dora-openarm-vr` | https://github.com/enactic/dora-openarm-vr.git | `072ce98d9c1d781e4b42626639c48f5c8f2ba8ee` | 선택. `smoothing.py`가 upstream과 같은지 보는 테스트 1개용. 없으면 그 테스트만 skip |

- `urdf` 저장소의 다른 미커밋 파일(Stage 1 시각화 도구 `display_openarm_rh56f1.launch.py`,
  `rviz/`, `tools/openarm_rh56f1_visualization.py`)은 이 PC에만 있다. robot_control은 이
  파일들을 쓰지 않는다. RViz 설정은 robot_control 안에 복사해 두었다
  (`openarm_bringup/config/rviz/openarm_rh56f1_split.rviz`).
- canonical URDF의 메시 경로는 원래 생성 PC의 `file:///home/user/rl_ws/urdf/...`로
  적혀 있다. launch(`rh56f1_description.build_canonical_variant`)가 실행할 때 이 경로를
  `urdf` 체크아웃 위치로 바꾸고, 원본 파일은 수정하지 않는다.

## 2. 호스트 준비

- Docker. 사용자가 `docker` 그룹에 있어야 한다.
- Humble 이미지: `thchzh/ros2:openarm-humble`. 검증에 쓴 이미지의 manifest digest는
  `sha256:259fb14b6493f7b077ff0f6ece4b8ed0c683137fb0be3ebfa722212f5f8d3614`이다. 2026-10-03
  Docker Hub의 태그와 일치함을 확인했다. 정확히 같은 이미지를 쓰려면 digest로 받는다:
  ```bash
  docker pull thchzh/ros2@sha256:259fb14b6493f7b077ff0f6ece4b8ed0c683137fb0be3ebfa722212f5f8d3614
  docker tag  thchzh/ros2@sha256:259fb14b6493f7b077ff0f6ece4b8ed0c683137fb0be3ebfa722212f5f8d3614 thchzh/ros2:openarm-humble
  ```
- RViz를 띄우려면 데스크톱 세션(`DISPLAY`)과 `xauth`가 필요하다.
- Quest를 쓰려면 Quest와 같은 네트워크에 있어야 하고, UDP 5006이 열려 있어야 한다
  (`sudo ufw allow 5006/udp`).
- 호스트 ROS(Jazzy 등)는 쓰지 않는다. 모든 ROS 실행은 컨테이너 안에서 하고, 호스트와
  Humble overlay를 섞지 않는다.

## 3. clone 또는 pull

```bash
mkdir -p ~/kuku_lab && cd ~/kuku_lab

# 처음
git clone -b feature/rh56f1-fake-bringup https://github.com/RiaRon/robot_control.git
git clone https://github.com/KUKU-Robot-Lab/urdf.git
git -C urdf checkout 88f886cc0bf2b092adafa67f03e166fe6df39638   # 또는 이 commit을 포함한 main

# 이미 있으면
git -C robot_control fetch https://github.com/RiaRon/robot_control.git feature/rh56f1-fake-bringup
git -C robot_control checkout feature/rh56f1-fake-bringup
git -C robot_control merge --ff-only FETCH_HEAD
git -C urdf fetch && git -C urdf checkout 88f886cc0bf2b092adafa67f03e166fe6df39638
```

확인 (manifest hash가 profile과 같아야 한다):

```bash
sha256sum urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml
# fc72ad75a1231b203f9d101a572ecbdbe5cd6281f51f44575368d2ba82c15780
```

## 4. 자동 회귀 (사람 개입 없음, 장치 없음)

```bash
cd ~/kuku_lab/robot_control
tests/run_humble_split_bringup.sh             # 정적 테스트 + 분리 구조 + Quest(합성) + glove(합성) + real 대역
tests/run_humble_fake_motion_regression.sh    # 이전 단일 CM fake 회귀 (Stage 6)
```

각각 `SPLIT_BRINGUP_PROBE=PASS`, `QUEST_TELEOP_PROBE=PASS`, `GLOVE_FAKE_PROBE=PASS`,
`QUEST_TELEOP_REAL_DOUBLE_PROBE=PASS`, `FAKE_MOTION_PROBE=PASS`가 나와야 한다. glove 회귀는
`third_party/inspire_hand_senseglove_teleop`(vendored retarget 노드)이 있어야 한다. 컨테이너는
network none, 장치 없음, 권한 없음으로 실행되며, kuku_lab은 읽기 전용으로 마운트된다.

## 5. fake + RViz 실행

```bash
cd ~/kuku_lab/robot_control
QUEST_TELEOP_GUI=1 tools/quest_teleop_fake_container.sh     # 터미널 A (빌드 후 셸)
docker exec -it quest-teleop-fake bash --rcfile /tmp/quest_env.sh   # 터미널 B, C, ...

# 터미널 B: 팔(손 geometry 포함) + 통합 모델 + RViz
ros2 launch openarm_bringup openarm_rh56f1_arms.launch.py runtime:=fake use_rviz:=true
# 터미널 C, D (선택): 손별 fake CM
ros2 launch openarm_bringup rh56f1_right_hand.launch.py
ros2 launch openarm_bringup rh56f1_left_hand.launch.py
# 글러브(합성 또는 실제) → fake 손: docs/rh56f1-glove-fake.md
```

컨테이너는 스크립트 위치에서 `kuku_lab`을 찾아 `/workspace/kuku_lab`에 마운트하고
`KUKU_LAB_ROOT`를 설정한다. 그래서 clone 위치가 달라도 된다. 컨테이너 밖에서 launch를
직접 쓰려면 `canonical_urdf:=<kuku_lab>/urdf/generated/rl/openarm_rh56f1_bi_rl.urdf`를
주거나 `KUKU_LAB_ROOT`를 설정한다.

## 6. Quest 추종 (fake)

`docs/quest-teleop.md` 1–3절대로 Quest 앱을 설치하고 PC IP와 port 5006을 입력한다. 그다음
같은 컨테이너에서 실행한다.

```bash
# fake 오른팔을 굽힌 시작 자세로 (fake 전용 값)
ros2 action send_goal /right_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names:
  [r_aj_1, r_aj_2, r_aj_3, r_aj_4, r_aj_5, r_aj_6, r_aj_7], points: [{positions:
  [0.3, 0.15, 0.0, 1.2, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}}"

python3 -m openarm_quest_teleop.ros_bridge
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime fake --execute
python3 -m openarm_quest_teleop.ros_teleop --arm right --runtime fake --execute --orientation-mode relative
```

grip을 누르는 동안 RViz의 오른팔이 컨트롤러를 따라온다. 2026-10-03에 사용자가 이 경로로
위치 추종과 relative 회전을 확인했다.

## 7. 재현 범위

- 위 절차는 fake(GenericSystem)와 합성 또는 실제 Quest 입력까지다.
- 실물 OpenArm 운동은 어느 PC에서도 아직 검증되지 않았다.
- 실물 손 backend는 없다.
