# Meta Quest 3로 OpenArm + LEAP Hand 원격조종하기: 처음부터 끝까지

사람이 Meta Quest 3를 쓰고 컨트롤러로 OpenArm 팔을 움직이고, 고개를 돌리면 로봇 목(머리 카메라)이 따라 돌게 하기까지의 전체 순서다. 설치는 한 번만 하고, 매번 하는 실행 순서는 4~11단계다.

- 기술적인 세부는 `docs/leap_hand/quest-teleop-leap-pink.md`(pink IK, quest_view, 목, 테스트)와 `docs/leap_hand/openarm-leap-split-bringup.md`(bringup 구조)에 있다.
- 이 문서는 "사람이 순서대로 따라 하는 것"만 다룬다.

---

## 0. 지금 어디까지 되나 (2026-10-27)

| 단계 | 상태 |
|---|---|
| 합성 헤드셋 페이지 → quest_view → fake OpenArm + LEAP 팔 (RViz) | **동작 확인됨.** 자동 테스트 `tests/jazzy_leap_quest_teleop_probe.py` 23/23 PASS |
| 합성 헤드셋 → fake 목 (RViz에서 머리가 따라 돎) | **동작 확인됨.** 같은 테스트. 왼쪽 20° → `head_j_pan` −0.34 rad, 위 10° → `head_j_tilt` −0.17 rad |
| 실제 Quest 3 (quest_view 페이지) → fake 로봇 | **아직 안 해 봄.** 헤드셋을 연결한 뒤 이 문서대로 확인한다 |
| 실제 머리 카메라 영상 → 헤드셋 | **아직 안 해 봄.** 카메라가 이 PC에 연결되어 있지 않다 |
| 손목 카메라 → LEAP 손가락 (10-5) | **fake 확인됨** (합성 손). 실제 카메라는 아직 |
| 녹화 (10-6) | **fake 확인됨** (rosbag2 MCAP) |
| 실물 목 (Dynamixel) | **아직 불가.** 포트·홈 각도·방향을 이 로봇에서 재지 않았다(9-3) |
| 실물 OpenArm + LEAP | **코드 준비됨, 실물 확인 전.** 팔(OpenArmHW)·손(ROBOTIS) 실물 launch가 있고 플러그인 로딩까지 확인. CAN·U2D2·모터 ID 확인 필요(13단계) |

```text
[Quest 3 브라우저: quest_view 페이지 (WebXR)]
      │ USB (adb reverse, ws://localhost:8787)        ▲ 머리 카메라 영상 (JPEG)
      ▼                                                │
[quest_view 서버] ──────── TCP 127.0.0.1:65432 (자세) ──┼──▶ [ros_bridge] ──/quest/*──┬──▶ [ros_teleop (pink IK)] ──▶ 팔 컨트롤러 ──▶ RViz / 실물(예정)
      ▲ 머리 카메라 (RealSense, V4L2)                  │                            └──▶ [ros_head] ──▶ 목 Dynamixel (fake / 실물)
      └──────────────── UDP 47121 (목 상태: 헤드셋 상태 줄, 카메라 켜기) ◀───────────────────┘
```

이전의 dora-openarm-vr 앱(UDP 5006)은 OpenArm + LEAP에서는 더 이상 쓰지 않는다. 헤드셋에는 앱 설치가 필요 없고, Quest 브라우저가 PC의 quest_view 페이지를 연다.

---

## 1. 준비물

- **PC:** Ubuntu 24.04, ROS 2 Jazzy가 설치되어 있어야 한다. GPU는 필요 없다.
- **헤드셋:** Meta Quest 3 (컨트롤러 2개, 충전된 상태)와 **데이터용 USB-C 케이블**. 기본 연결은 USB다(Wi-Fi도 가능, 3-6).
- **머리 카메라:** RealSense (실물 영상을 볼 때). 없으면 테스트 화면으로 대신한다.
- **저장소:** `kuku_lab` 체크아웃
  - `robot_control`: 브랜치 `Leap_Hand`
  - `urdf`: 브랜치 `feat/leap-hand` (LEAP 로봇 모델이 이 브랜치에만 있다)

```bash
cd ~/cbj_ws/kuku_lab/robot_control && git checkout Leap_Hand && git pull
cd ~/cbj_ws/kuku_lab/urdf && git checkout feat/leap-hand && git pull
```

---

## 2. 한 번만 하는 설치 (PC)

### 2-1. ROS 패키지 (RViz 관절 슬라이더 등)
```bash
sudo apt update
sudo apt install -y ros-jazzy-joint-state-publisher-gui
```
`404 Not Found`가 나오면 `sudo apt update`를 먼저 다시 실행한다. apt 목록이 오래되어 생기는 문제다.

### 2-2. Python 환경 (`.venv`)
pink IK, quest_view 서버, 목 제어에 필요한 패키지는 ROS에 들어 있지 않아서, robot_control 안에 전용 환경을 만든다. sudo는 필요 없다.

```bash
cd ~/cbj_ws/kuku_lab/robot_control
python3 -m venv --without-pip --system-site-packages .venv
python3 -m pip --python .venv/bin/python install "pin-pink==4.4.0" daqp \
  opencv-python-headless aiohttp websockets dynamixel-sdk
.venv/bin/python -c "import pinocchio, pink, cv2, aiohttp, websockets, dynamixel_sdk; print('ok')"
```
시스템 OpenCV(apt `python3-opencv`)는 이 PC의 numpy 2와 맞지 않아서, `.venv`에 `opencv-python-headless`를 따로 넣는다.

### 2-3. bringup 빌드
```bash
cd ~/cbj_ws/kuku_lab/robot_control/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/openarm_description src/openarm_ros2/openarm_bringup \
  --packages-select openarm_description openarm_bringup
```

방화벽 설정은 필요 없다. 헤드셋은 USB(adb)로, 나머지는 모두 PC 안(127.0.0.1)에서 연결된다.

---

## 3. 한 번만 하는 설치 (Quest 3, Ubuntu 24.04)

Meta Quest Developer Hub는 Windows/macOS용만 있어서 Ubuntu에는 설치할 수 없다. Ubuntu에서는 **adb**(Android platform-tools)로 같은 일을 한다.

### 3-1. 헤드셋 개발자 모드
1. [Meta 개발자 사이트](https://developer.oculus.com/)에 헤드셋과 같은 Meta 계정으로 로그인해 조직(organization)을 하나 만든다. 개발자 모드를 켜려면 필요하다.
2. 휴대폰 Meta Horizon 앱 → 기기(헤드셋) → 헤드셋 설정 → **개발자 모드**를 켠다. 헤드셋을 재부팅한다.

### 3-2. PC에 adb 설치
둘 중 하나만 하면 된다. 이 PC(cbj)에는 A가 이미 되어 있다(`~/opt/platform-tools`, 37.0.1).

**A. Google platform-tools (sudo 불필요, 최신판)**
```bash
mkdir -p ~/opt ~/.local/bin && cd ~/opt
wget https://dl.google.com/android/repository/platform-tools-latest-linux.zip
unzip -o platform-tools-latest-linux.zip && rm platform-tools-latest-linux.zip
ln -sf ~/opt/platform-tools/adb ~/.local/bin/adb
adb version
```
`adb: command not found`가 나오면 터미널을 새로 열거나 `export PATH=$HOME/.local/bin:$PATH`.

**B. Ubuntu 패키지**
```bash
sudo apt update && sudo apt install -y adb
```

### 3-3. USB 권한 (한 번만, sudo 필요)
Ubuntu는 기본적으로 일반 사용자에게 Quest USB 접근 권한을 주지 않는다. 이걸 안 하면 `adb devices`에 `no permissions`가 나온다.
```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0660", GROUP="plugdev", TAG+="uaccess"' \
  | sudo tee /etc/udev/rules.d/51-meta-quest.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
groups | grep -q plugdev || sudo usermod -aG plugdev $USER   # 추가했다면 로그아웃 후 다시 로그인
```
`2833`은 Meta(Oculus)의 USB 제조사 번호다. 이 PC(cbj)에는 같은 규칙이 이미 있고(10/03), 사용자도 plugdev 그룹에 들어 있다.

### 3-4. 헤드셋 연결
1. USB-C 케이블(데이터 지원)로 헤드셋을 PC에 연결한다.
2. 헤드셋을 쓰고 **"USB 디버깅을 허용하시겠습니까?"** 창에서 "이 컴퓨터에서 항상 허용"을 체크하고 허용한다.
3. 연결 확인:
   ```bash
   cd ~/cbj_ws/kuku_lab/robot_control && scripts/quest_view.sh status
   ```
   `link: <일련번호> (USB)`가 나오면 된다. `unauthorized`면 2번 창을 다시 확인하고, `no permissions`면 3-3을 다시 본다.

앱(APK)은 설치하지 않는다. 헤드셋의 Quest 브라우저가 PC의 quest_view 페이지를 연다.

### 3-5. 처음 한 번: 브라우저의 VR 허용
처음 페이지에서 VR에 들어갈 때 헤드셋 안에 **VR 허용** 창이 뜬다. 허용을 누른다(이후로는 묻지 않는다).

### 3-6. (선택) Wi-Fi로 연결
케이블 없이 쓰려면, 케이블을 꽂은 상태에서 한 번:
```bash
scripts/quest_view.sh wifi       # 이후 케이블을 뽑아도 된다. 되돌리기: scripts/quest_view.sh wifi-off
```
헤드셋과 PC가 같은 공유기에 있어야 한다. USB보다 지연이 크고, 연결이 끊기면 팔이 멈춘다(입력 끊김). Quest를 재부팅하면 케이블을 꽂고 다시 한다.

---

## 4. 매번: 터미널 환경 준비

아래 블록을 **새 터미널마다** 먼저 붙여 넣는다.

```bash
cd ~/cbj_ws/kuku_lab/robot_control
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH=$PWD/src:$PYTHONPATH
export CFG=src/openarm_quest_teleop/config/leap_hand/quest_teleop_leap.yaml
```

다른 ROS 작업과 섞이지 않게 하려면 모든 터미널에 같은 `export ROS_DOMAIN_ID=<번호>`를 추가한다(0 말고 다른 번호).

---

## 5. 매번: quest_view 서버와 헤드셋 페이지

**터미널 A: quest_view 서버** (끝날 때까지 켜 둔다)
```bash
.venv/bin/python -m openarm_quest_view.server                 # 머리 카메라: RealSense 자동 탐색
.venv/bin/python -m openarm_quest_view.server --test-pattern  # 카메라 없이: 움직이는 테스트 화면
```
`page http://localhost:8787 … poses -> TCP 127.0.0.1:65432`가 나오면 된다.

**헤드셋 페이지 열고 VR 들어가기** (아무 터미널, 한 번)
```bash
scripts/quest_view.sh view
```
- PC가 헤드셋 브라우저로 `http://localhost:8787`을 열고, VR까지 들어간다. 헤드셋을 쓰고 있지 않으면 근접 센서를 끄고 깨운다.
- VR이 안 들어가면 헤드셋 안 페이지의 **VR 시작** 버튼을 누른다.
- 헤드셋 화면: 기본은 **패스스루**(헤드셋 카메라로 방이 보임)다. 머리 카메라 영상을 보려면 서버를 `--view camera`로 띄운다. 머리 카메라는 목(9단계)이 돌 때만 켜진다.
- USB를 다시 꽂으면 링크가 끊긴다. `scripts/quest_view.sh view`(또는 `vr`)를 다시 한다.

---

## 6. 매번: Quest 신호가 들어오는지 확인 (로봇 없이)

```bash
.venv/bin/python -m openarm_quest_teleop.monitor --config $CFG
```

컨트롤러를 움직이며 확인한다.
- `rate`가 0이 아니고(헤드셋 화면 주사율, 72~90 Hz), `last packet … ago`가 계속 작다.
- 오른손을 **앞으로** 내밀면 `right pos`의 x가, **왼쪽**으로 옮기면 y가, **위로** 올리면 z가 커진다.
- grip(가운데 손가락 버튼)을 쥐면 `grip`이 1이 된다(페이지는 눌림/안 눌림만 보내서 0 또는 1).
- `hmd pos`가 보인다(헤드셋 자세).

확인했으면 Ctrl-C. monitor는 켜 둔 채로 다음 단계로 가도 된다(65432에는 여러 프로그램이 같이 붙을 수 있다).

---

## 7. 매번: 로봇 띄우기 (fake, RViz)

**터미널 1: 팔 + 로봇 모델 + RViz**
```bash
ros2 launch openarm_bringup openarm_leap_arms.launch.py use_rviz:=true
```

**터미널 2, 3: 손 (선택)** — 원격조종은 팔만 움직이므로 없어도 된다.
```bash
ros2 launch openarm_bringup leap_hand.launch.py side:=right
ros2 launch openarm_bringup leap_hand.launch.py side:=left
```

RViz에 OpenArm과 LEAP 손이 보이면 된다. 처음에는 팔이 곧게 아래로 내려진 영점 자세다.

---

## 8. 매번: Quest 신호를 ROS로 (브리지)와 로봇 정면

**터미널 4: 브리지**
```bash
.venv/bin/python -m openarm_quest_teleop.ros_bridge --config $CFG
```
`reading Quest input from quest_view TCP 127.0.0.1:65432`가 나오면 된다.

**로봇 정면 맞추기** (VR 세션마다 한 번, 목을 쓰기 전에)
1. 헤드셋을 쓰고 **로봇의 정면 방향을 보고 1초 가만히** 있는다.
2. 다른 사람이(또는 직접) PC에서:
   ```bash
   curl -s -X POST http://127.0.0.1:8787/front
   ```
   `"ok": true, "text": "정면 저장됨 …"`이 나오면 된다. 고개가 움직였으면 다시 한다.
- 목은 정면이 정해질 때까지 기다린다. VR을 다시 시작하거나 Quest가 재중심되면 정면이 풀리므로 다시 한다.

---

## 9. 매번: 목 (머리 카메라)

### 9-1. fake 목 (RViz에서 머리가 따라 돎)
**터미널 5**
```bash
.venv/bin/python -m openarm_head.ros_head --config $CFG
```
1. 목이 홈으로 간 뒤 `Head locked at home. Space: follow …`가 나온다.
2. 이 터미널에서 **Space**를 누르면, 그 순간 보던 방향을 기준으로 고개를 따라간다.
3. 다시 Space: 멈추고 홈으로 돌아간다. Esc 또는 Ctrl-C: 홈으로 돌아간 뒤 끝난다.

움직이는 범위는 홈 기준 좌우 20°, 상하 15°다(`head:` 설정). 헤드셋 추적이 끊기면 그 자리에서 멈춘다.

### 9-2. 머리 카메라 영상 보기
quest_view 서버를 `--view camera`로 띄우면 목이 도는 동안 머리 카메라 영상이 헤드셋 앞에 보인다. 헤드셋 아래쪽 상태 줄에 목 상태(locked / following, 홈에서 얼마나 돌았는지)가 나온다.

### 9-3. 실물 목: 아직 불가 (측정 필요)
`quest_teleop_leap.yaml`의 `head:` 값은 선배 motion_acq의 값을 옮긴 **임시값**이다. 실물로 돌리기 전에 이 로봇에서 재야 한다.
- `port`: 목 U2D2의 `/dev/serial/by-id/...` (지금 비어 있어서 `--backend real`은 시작을 거부한다)
- `pan.id` / `tilt.id`, `home_deg`(로봇 정면을 볼 때의 모터 각), `sign`(고개를 왼쪽·위로 돌릴 때 모터 각이 커지면 +1)
- 잰 뒤: `--backend real`. 시작할 때 토크를 그 자리에서 켜고 홈으로 천천히(20°/s) 간다. 목이 허용 범위 밖에 있으면 시작을 거부한다.

---

## 10. 매번: 팔 원격조종

### 10-1. 먼저 dry run (명령을 보내지 않고 점검)
**터미널 6 (오른팔)**
```bash
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake
```
**터미널 7에서 상태 보기**
```bash
ros2 topic echo /quest_teleop/right/status
```
grip을 쥐고 손을 움직이면 다음이 보여야 한다.
- `state`가 `engaged`가 된다.
- `target_xyz`가 손을 움직인 방향으로 바뀐다.
- `reason`이 비어 있다.

dry run에서는 시작 자세로도 옮기지 않는다("would move to start pose"만 출력). 확인했으면 Ctrl-C.

### 10-2. 실제 실행
**터미널 6 (오른팔)**
```bash
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake --execute
```
1. 실행하면 팔이 **시작 자세(`rh56f1_aglt_home`)로 약 6초에 걸쳐 천천히** 이동한다. RH56F1 로봇에서 쓰던 홈 자세다.
2. 로그에 `at start pose rh56f1_aglt_home; press the enable input to follow`가 나오면 준비 끝.
3. 왼팔도 하려면 새 터미널에서 `--arm left`로 하나 더 띄운다. 한 프로세스에 한 팔이다.

### 10-3. 조작법
| 동작 | 결과 |
|---|---|
| **grip을 쥔다** | 쥔 순간의 손 위치를 기준으로 연결된다. 그 뒤 손을 움직인 만큼 로봇 손바닥이 따라 움직인다 |
| **grip을 놓는다** | 즉시 멈춘다. 팔은 그 자리에 머문다 |
| 다시 grip | 그 자리에서 새로 기준을 잡고 이어서 움직인다(클러치). 손이 불편하면 놓고 손을 옮긴 뒤 다시 쥔다 |
| 손목 회전도 따라가게 | 실행 명령에 `--orientation-mode relative`를 붙인다(기본은 `hold`: 손바닥 방향 고정) |

**처음에는 1~2 cm씩 천천히** 움직인다. 손을 아주 빨리 움직이거나 추적이 튀면(한 번에 10 cm 이상) 안전을 위해 `locked`가 된다. 이때는 grip을 놓았다가 다시 쥐면 풀린다.

### 10-4. 시작 자세에서 주의할 동작
시작 자세는 RH56F1용이라 몇몇 관절에 여유가 작다. 아래처럼 움직이면 로봇이 **멈춘다**. 고장이 아니라 안전하게 멈추는 동작이다.
- **손목을 한쪽으로 크게 돌릴 때:** 한쪽은 약 12°만 여유가 있다.
- **손을 몸 쪽 위로 크게 당길 때:** 팔꿈치가 끝까지 접힌다.

멈추면 `status`의 `reason`에 `IK failed: target not reachable` 등이 보인다. 손을 반대로 되돌리면 다시 따라온다. 녹화 때 문제없던 범위는 앞 10 cm, 바깥 6 cm, 아래 6 cm, 손목 ±20°, 반지름 5 cm 원이다.

### 10-5. 손가락: 손목 카메라로 LEAP 손 조종
조작자 손목에 단 카메라(친구 작업과 같은 D405 위치)로 손가락을 보고, MediaPipe 손 랜드마크를 LEAP 16관절로 바꿔 손 컨트롤러에 보낸다. 손(터미널 2)이 떠 있어야 한다.

**한 번만: 전용 환경** (`.venv`와 따로. MediaPipe가 pink 쪽 패키지와 섞이지 않게)
```bash
python3 -m venv --without-pip --system-site-packages .venv-hand
python3 -m pip --python .venv-hand/bin/python install mediapipe==1.1.1 pyyaml
python3 -m pip --python .venv-hand/bin/python install --ignore-installed matplotlib   # apt matplotlib은 numpy 2와 안 맞음
curl -L --create-dirs -o models/hand_landmarker.task \
  https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
```

**터미널 8**
```bash
.venv-hand/bin/python -m openarm_leap_tracking.ros_node --side right --camera 0            # 점검: 목표만 발행
.venv-hand/bin/python -m openarm_leap_tracking.ros_node --side right --camera 0 --execute  # 손에 명령
.venv-hand/bin/python -m openarm_leap_tracking.ros_node --side right --test --execute      # 카메라 없이 합성 손
```
- 시작 직후는 따라가지 않는다. **Space**로 따라가기 시작/멈춤, Esc로 끝낸다. 시작할 때 손의 현재 관절 각도에서 출발하므로 튀지 않는다.
- 손이 화면에서 사라지면 마지막 목표를 유지한다(펴지지 않는다).
- 목표는 항상 `/leap_hand_tracking/right/target`에 나온다(`ros2 topic echo`로 확인).
- 한계: 친구가 실물에서 쓴 안전 범위(MCP 82.85°, PIP 60°, DIP 79.23°, 벌림 ±10°, 엄지 회전 105°). 설정은 `src/openarm_leap_tracking/config/hand_tracking.yaml`. 방향(scale 부호)과 엄지 벌림 기준(offset)은 카메라를 달고 맞춰야 한다(★).
- fake 확인(10/27): 합성 손으로 fake 손이 따라 움직임. 손가락 최대 68°, PIP는 60°에서 멈춤, 벌림 ±8.6°, 한 번에 최대 0.5° 변화.

### 10-6. 녹화
```bash
scripts/record_teleop.sh cup_grasp          # bags/<날짜_시간>_cup_grasp (MCAP), Ctrl-C로 끝
ros2 bag info bags/<폴더>                     # 확인
```
관절 상태(팔·양손·목), Quest 입력(`/quest/*`), 원격조종 상태·목표, 손목 카메라 손 목표, 모든 팔·손 명령, TF와 모델을 기록한다. 카메라 영상은 ROS 토픽이 아니라서(quest_view가 카메라를 직접 읽음) 들어가지 않는다.

---

## 11. 매번: 끝내기

1. grip을 놓는다. 녹화 중이면 녹화 터미널 Ctrl-C, 손목 카메라(8)는 Space로 멈춘 뒤 Esc.
2. 원격조종 터미널(6, 왼팔이면 그것도)에서 Ctrl-C
3. 목(5) Ctrl-C: 홈으로 돌아간 뒤 끝난다
4. 브리지(4) Ctrl-C
5. 손(2, 3), 팔(1) 순서로 Ctrl-C
6. quest_view 서버(A) Ctrl-C. 헤드셋 페이지는 그대로 두어도 된다(서버를 다시 띄우면 페이지가 다시 붙는다).

원격조종은 끝날 때 팔을 영점이나 홈으로 보내지 않는다. fake에서는 다시 띄우면 영점에서 시작한다.

---

## 12. 헤드셋 관련 명령 (`scripts/quest_view.sh`)

| 명령 | 하는 일 |
|---|---|
| `status` | 연결된 헤드셋(USB/Wi-Fi)과 페이지 링크 |
| `view` | 페이지를 열고 VR까지 들어감 |
| `page` | 페이지만 엶 (VR은 헤드셋에서 버튼) |
| `vr` / `vr --status` | 열린 페이지에서 VR 다시 들어가기 / 페이지 상태 |
| `unworn` / `worn` | 쓰지 않고 쓸 때 근접 센서 끄기(화면 안 꺼짐) / 원래대로 |
| `wifi` / `wifi-off` | Wi-Fi 연결로 바꾸기 / 케이블로 되돌리기 |

---

## 13. 실물로 움직이기: 코드는 준비됨, 실물 확인 전

### 13-1. 지금 상태 (2026-10-27)
| 항목 | 상태 | 실물 전에 남은 일 |
|---|---|---|
| LEAP 손 실물 | `leap_hand.launch.py runtime:=real` (ROBOTIS `dynamixel_hardware_interface`). 설정 `config/leap_hand/leap_hand_real.yaml`. 플러그인이 설정을 읽고 포트 열기까지 가는 것 확인(포트 없음에서 멈춤) | `sudo apt install ros-jazzy-dynamixel-hardware-interface`, 손 U2D2 포트, **모터 ID↔관절 확인**(아래 13-3) |
| 팔 실물 | `openarm_leap_arms.launch.py runtime:=real` (OpenArmHW, Jazzy 빌드됨). 설정 `config/leap_hand/openarm_leap_arms_real.yaml`. 플러그인이 설정을 읽고 CAN 열기까지 가는 것 확인(CAN 없음에서 멈춤) | CAN-FD 어댑터와 인터페이스 이름(can0/can1) |
| 원격조종 실물 | `quest_teleop_leap.yaml`에 `runtimes.real`. 실물 이름(`openarm_right_joint1`…)으로 묶이는 것 테스트 | 실물에서 방향 확인 |
| 목 | fake 확인 | 9-3의 값 측정 |
| 어댑터 | 임시값 (RH56F1 높이) | LEAP 어댑터 치수·질량 반영 후 모델 재생성 |

실물 launch는 둘 다 `i_understand_this_moves_real_hardware:=true`가 없으면 거부한다. 하드웨어와 팔·손 컨트롤러는 **비활성(inactive)으로** 시작한다. 띄우기만 해서는 모터가 켜지지 않는다.

빌드(팔 드라이버 포함, 한 번):
```bash
cd ros_ws && source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/openarm_can src/openarm_ros2/openarm_hardware src/openarm_description src/openarm_ros2/openarm_bringup \
  --packages-up-to openarm_hardware openarm_bringup --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```
`-DPython3_EXECUTABLE=/usr/bin/python3`는 이 PC에서 `~/.local`의 python3.12가 잡혀 CMake가 실패하는 것을 막는다.

### 13-2. 실물 순서 (팔 단독 → 손 → 원격조종 → 녹화)
1. **점검:** E-stop을 손 닿는 곳에 둔다. 주변을 비우고, 팔을 받칠 사람이나 거치대를 준비한다.
2. **팔 띄우기 (움직이지 않음):**
   ```bash
   ros2 launch openarm_bringup openarm_leap_arms.launch.py runtime:=real right_can_interface:=can0 \
     i_understand_this_moves_real_hardware:=true use_rviz:=true
   ```
3. **읽기 전용 확인:** `ros2 control list_hardware_components`가 `inactive`, `/openarm/joint_states`에 NaN이 없고 RViz 팔 자세가 실제와 같은지 본다.
4. **오른팔 활성화:** 받친 상태에서
   ```bash
   ros2 control set_hardware_component_state openarm_leap_right_arm active
   ros2 control set_controller_state right_joint_trajectory_controller active
   ```
5. 브리지 → 원격조종 **dry run**(`--runtime real`)으로 `status`만 본다.
6. **실행:** `--runtime real --execute --confirm-real-hardware`. 둘 중 하나라도 없으면 거부된다. 처음에는 `position_scale` 0.3, 1~2 cm씩.
7. **손:** 아래 13-3을 먼저 하고, 손 launch → 손 하드웨어·컨트롤러 활성화 → 손목 카메라(10-5) 점검 모드 → `--execute`.
8. **녹화:** 10-6.

### 13-3. 손 실물 첫 연결
```bash
ros2 launch openarm_bringup leap_hand.launch.py side:=right runtime:=real \
  port:=/dev/serial/by-id/usb-FTDI_..._<serial>-if00-port0 i_understand_this_moves_real_hardware:=true
```
- 시작할 때 플러그인이 모든 모터의 토크를 끄고(켜져 있으면 끈다) 설정값(전류 기반 위치 모드, P 600 / D 200, 전류 한계 350 mA)을 쓴다. 하드웨어는 inactive로 남는다.
- **모터 ID↔관절 확인(★):** 활성화 전에 손가락 하나씩 손으로 움직이며 `/leap_right/joint_states`에서 어느 관절 값이 바뀌는지 본다. manifest는 ID 0/4/8 = 벌림, 1/5/9 = 굽힘(LEAP URDF)인데, 친구 손은 반대로 측정됐다. 반대면 `leap_hand_real.yaml`에 `motor_id_swap: {0: 1, 4: 5, 8: 9}`.
- 확인 후 `ros2 control set_hardware_component_state leap_right_hand_real active` (이때 토크가 켜진다), `ros2 control set_controller_state right_hand_trajectory_controller active`.
- 플러그인이 비활성 상태에서도 상태를 읽는지, 토크가 활성화 때만 켜지는지는 실물에서 처음 확인한다(문서와 코드로는 그렇게 보이지만 확인 전).

### 13-4. 실물에서 멈추는 방법 (기존 문서와 동일)
| 동작 | 결과 |
|---|---|
| grip을 놓는다 | 명령 중지, 팔은 마지막 위치 유지(토크 유지) |
| 원격조종 Ctrl-C | 명령 종료, 컨트롤러가 위치 유지 |
| 팔 컨트롤러 비활성화 | 새 명령을 받지 않음, 토크 유지 |
| **물리 E-stop / 전원 차단** | 모든 것이 멈춤. **소프트웨어에 의존하지 않는 유일한 정지** |

팔 하드웨어를 inactive로 내리면 **토크가 꺼져 팔이 떨어진다.** 반드시 받친 상태에서 한다.

---

## 14. 문제 해결

| 증상 | 원인과 해결 |
|---|---|
| monitor·브리지에 `no input … Connection refused` | quest_view 서버(5단계, 터미널 A)가 안 떠 있다 |
| monitor에 `rate 0`, 서버 로그에 `no headset poses for … s` | 헤드셋 페이지가 VR에 안 들어갔거나 USB 재연결로 링크가 끊겼다 → `scripts/quest_view.sh view` |
| quest_view 서버가 `TCP 127.0.0.1:65432 is taken` | 다른 quest_view나 HandUMI `adb forward`가 65432를 잡고 있다 → 끄거나 `adb forward --remove tcp:65432` |
| 헤드셋에 영상 대신 방이 보임 | 기본이 패스스루다 → 서버를 `--view camera`로. 영상은 목이 돌 때만 나온다 |
| `camera … did not open` | 머리 카메라(RealSense)가 이 PC에 없다 → 연결하거나 `--test-pattern` |
| 목이 `Head waits: the robot's front is not set` | 8단계의 정면 맞추기를 안 했거나 VR 재시작으로 풀렸다 → `curl -X POST http://127.0.0.1:8787/front` |
| 정면 맞추기가 `고개가 움직였습니다` | 1초 동안 고개를 가만히 두고 다시 |
| `--backend real needs head.port` | 9-3: 목 포트를 아직 설정하지 않았다 |
| `the teleop IK needs pink and pinocchio`, `No module named cv2/aiohttp` | `python3` 대신 **`.venv/bin/python`**으로 실행 (2-2 설치) |
| `no robot description … /openarm_leap/arms/robot_description` | 터미널 1(`openarm_leap_arms.launch.py`)이 안 떠 있거나 `ROS_DOMAIN_ID`가 다르다 |
| `controller … is not loaded / not active` | 팔 launch가 아직 준비 중이다. 로그에 `topic commands are accepted`가 나온 뒤 실행 |
| `state locked` | 입력이 끊겼거나(0.2초), 추적이 튀었다 → grip을 놓았다 다시 쥔다 |
| `IK failed: target not reachable` | 10-4의 작업 공간 한계 → 손을 되돌린다 |
| 시작 자세에서 grip을 눌러도 안 움직임 | grip을 실행 전부터 쥐고 있었다 → 한 번 놓았다가 다시 쥔다 |
| RViz 몸통 기둥이 빨갛다 | 정상이다. 메시 색이 그렇게 그려진다(`RobotModel` 상태는 OK) |
| RViz `RobotModel` 항목이 빨간 오류 | 항목을 펼쳐 오류 메시지를 본다. 대개 TF가 없는 링크 때문이다 → 팔 launch(터미널 1)가 정상인지 확인 |
| `joint_state_publisher_gui` 없음, 404 | `sudo apt update` 후 다시 설치 |
| `adb devices`가 비어 있음 | 케이블이 충전 전용이거나 개발자 모드가 꺼져 있다(3-1) → 데이터 케이블로 바꾸고 헤드셋 재부팅 |
| `adb devices`에 `unauthorized` / `no permissions` | 헤드셋 안의 USB 디버깅 허용 창(3-4) / udev 규칙(3-3) |

---

## 15. 참고 문서
- `docs/leap_hand/quest-teleop-leap-pink.md`: pink IK, quest_view 입력, 목, 시작 자세, 테스트, 작업 공간 분석
- `docs/leap_hand/openarm-leap-split-bringup.md`: OpenArm + LEAP 분리 bringup 구조 (Jazzy)
- `vendor_metadata/motion_acq/UPSTREAM.yaml`: 선배 motion_acq에서 가져온 파일과 바꾼 점
- `docs/rh56f1/quest-teleop.md`: Quest 원격조종 원래 문서 (RH56F1, dora 앱 UDP, 실물 절차 6~8절)
- `docs/rh56f1/rh56f1-real-bringup.md`: 실물 팔 preflight·정지 절차
- 녹화: `figure/leap_hand/openarm_leap_pink_quest_teleop.webm` (이전 UDP 입력으로 찍은 것. 팔 동작은 같다)
