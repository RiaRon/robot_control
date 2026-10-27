# Meta Quest 3로 OpenArm + LEAP Hand 원격조종하기: 처음부터 끝까지

사람이 Meta Quest 3 컨트롤러를 들고 OpenArm 팔을 움직이기까지의 전체 순서다. 설치는 한 번만 하고, 매번 하는 실행 순서는 4~9단계다.

- 기술적인 세부는 `docs/leap_hand/quest-teleop-leap-pink.md`(pink IK, 테스트)와 `docs/leap_hand/openarm-leap-split-bringup.md`(bringup 구조)에 있다.
- 이 문서는 "사람이 순서대로 따라 하는 것"만 다룬다.

---

## 0. 지금 어디까지 되나 (2026-10-08)

| 단계 | 상태 |
|---|---|
| 합성 Quest 입력 → fake OpenArm + LEAP (RViz) | **동작 확인됨.** 자동 테스트 17/17 PASS, 녹화 `figure/openarm_leap_pink_quest_teleop.webm` |
| 실제 Quest 3 → fake OpenArm + LEAP (RViz) | 같은 수신 코드로 RH56F1 fake 팔은 10/03에 사람이 확인함. **LEAP + pink 조합으로는 아직 안 해 봄** |
| 실제 Quest 3 → **실물** OpenArm + LEAP | **아직 불가.** LEAP 손 실물 드라이버와 Jazzy용 실물 팔 실행 경로가 없다(10단계 참고) |

그래서 이 문서로 지금 끝까지 할 수 있는 것은 **"실제 Quest로 RViz 속 로봇을 움직이는 것"**이다. 실물 단계는 무엇을 더 만들어야 하는지와, 만들어진 뒤의 순서·안전 절차를 적어 둔다.

```text
[Meta Quest 3 앱] --Wi-Fi, UDP 5006--> [ros_bridge] --/quest/*--> [ros_teleop (pink IK)] --> [팔 컨트롤러] --> RViz (fake) / 실물(예정)
```

---

## 1. 준비물

- **PC:** Ubuntu 24.04, ROS 2 Jazzy가 설치되어 있어야 한다. GPU는 필요 없다.
- **헤드셋:** Meta Quest 3 (컨트롤러 2개, 충전된 상태)
- **네트워크:** PC와 Quest가 **같은 공유기**에 연결되어 있어야 한다.
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

### 2-2. pink IK용 Python 환경 (`.venv`)
원격조종의 IK(pink, pinocchio)는 ROS에 들어 있지 않아서, robot_control 안에 전용 환경을 만든다. sudo는 필요 없다.

```bash
cd ~/cbj_ws/kuku_lab/robot_control
python3 -m venv --without-pip --system-site-packages .venv
python3 -m pip --python .venv/bin/python install "pin-pink==4.4.0" daqp
.venv/bin/python -c "import pinocchio, pink; print(pinocchio.__version__, pink.__version__)"   # 4.1.0 4.4.0
```

### 2-3. bringup 빌드
```bash
cd ~/cbj_ws/kuku_lab/robot_control/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/openarm_description src/openarm_ros2/openarm_bringup \
  --packages-select openarm_description openarm_bringup
```

### 2-4. PC 방화벽 (켜져 있을 때만)
```bash
sudo ufw allow 5006/udp
```

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

### 3-4. 헤드셋 연결과 앱 설치
1. USB-C 케이블(데이터 지원)로 헤드셋을 PC에 연결한다.
2. 헤드셋을 쓰고 **"USB 디버깅을 허용하시겠습니까?"** 창에서 "이 컴퓨터에서 항상 허용"을 체크하고 허용한다.
3. 연결 확인:
   ```bash
   adb devices
   ```
   `<일련번호>  device`가 나오면 된다. `unauthorized`면 2번 창을 다시 확인하고, `no permissions`면 3-3을 다시 본다.
4. OpenArm 공식 `dora-openarm-vr`의 원격조종 APK를 내려받는다: <https://drive.google.com/file/d/1lLDuoQAcl3YBKPE77F_1Z-8RzXEOFxpm/view?usp=drive_link>
5. 설치(sideload):
   ```bash
   adb install -r ~/Downloads/<파일>.apk   # Success가 나오면 끝. -r: 이미 있으면 덮어씀
   ```
6. 헤드셋의 앱 목록 → 오른쪽 위 필터 → **알 수 없는 출처(Unknown Sources)**에 앱이 있다.
7. 설치가 끝나면 USB는 뽑아도 된다. 원격조종 신호는 Wi-Fi로 간다.

### 3-5. 앱 설정
1. PC의 IP를 확인한다: `ip -4 addr` (예: `192.168.0.23`)
2. 헤드셋에서 앱을 실행하고 **왼쪽 컨트롤러의 메뉴 버튼**으로 설정 패널을 연다. PC IP와 포트 `5006`을 입력한다.
3. 헤드셋을 머리에 쓰지 않고 목에 걸어 쓸 때는, 화면이 꺼지지 않도록 눈 사이의 근접 센서를 테이프로 가린다.

---

## 4. 매번: 터미널 환경 준비

아래 블록을 **새 터미널마다** 먼저 붙여 넣는다.

```bash
cd ~/cbj_ws/kuku_lab/robot_control
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
export PYTHONPATH=$PWD/src:$PYTHONPATH
export CFG=src/openarm_quest_teleop/config/quest_teleop_leap.yaml
```

다른 ROS 작업과 섞이지 않게 하려면 모든 터미널에 같은 `export ROS_DOMAIN_ID=<번호>`를 추가한다(0 말고 다른 번호).

---

## 5. 매번: Quest 신호가 들어오는지 확인 (로봇 없이)

```bash
.venv/bin/python -m openarm_quest_teleop.monitor --config $CFG
```

Quest 앱을 켜고 컨트롤러를 움직이며 확인한다.
- `rate`가 0이 아니고(약 90 Hz), `last packet … ago`가 계속 작다.
- 오른손을 **앞으로** 내밀면 `right pos`의 x가, **왼쪽**으로 옮기면 y가, **위로** 올리면 z가 커진다.
- grip(가운데 손가락 버튼)을 쥐면 `grip`이 1에 가까워진다.

확인했으면 **Ctrl-C로 끈다.** 5006 포트는 한 프로그램만 쓸 수 있어서, monitor를 끄지 않으면 다음 단계의 브리지가 받지 못한다.

---

## 6. 매번: 로봇 띄우기 (fake, RViz)

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

## 7. 매번: Quest 신호를 ROS로 (브리지)

**터미널 4**
```bash
.venv/bin/python -m openarm_quest_teleop.ros_bridge --config $CFG
```
`listening for Quest packets on UDP 0.0.0.0:5006`이 나오면 된다.

---

## 8. 매번: 원격조종 시작

### 8-1. 먼저 dry run (명령을 보내지 않고 점검)
**터미널 5 (오른팔)**
```bash
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake
```
**터미널 6에서 상태 보기**
```bash
ros2 topic echo /quest_teleop/right/status
```
grip을 쥐고 손을 움직이면 다음이 보여야 한다.
- `state`가 `engaged`가 된다.
- `target_xyz`가 손을 움직인 방향으로 바뀐다.
- `reason`이 비어 있다.

dry run에서는 시작 자세로도 옮기지 않는다("would move to start pose"만 출력). 확인했으면 Ctrl-C.

### 8-2. 실제 실행
**터미널 5 (오른팔)**
```bash
.venv/bin/python -m openarm_quest_teleop.ros_teleop --config $CFG --arm right --runtime fake --execute
```
1. 실행하면 팔이 **시작 자세(`rh56f1_aglt_home`)로 약 6초에 걸쳐 천천히** 이동한다. RH56F1 로봇에서 쓰던 홈 자세다.
2. 로그에 `at start pose rh56f1_aglt_home; press the enable input to follow`가 나오면 준비 끝.
3. 왼팔도 하려면 새 터미널에서 `--arm left`로 하나 더 띄운다. 한 프로세스에 한 팔이다.

### 8-3. 조작법
| 동작 | 결과 |
|---|---|
| **grip을 쥔다** | 쥔 순간의 손 위치를 기준으로 연결된다. 그 뒤 손을 움직인 만큼 로봇 손바닥이 따라 움직인다 |
| **grip을 놓는다** | 즉시 멈춘다. 팔은 그 자리에 머문다 |
| 다시 grip | 그 자리에서 새로 기준을 잡고 이어서 움직인다(클러치). 손이 불편하면 놓고 손을 옮긴 뒤 다시 쥔다 |
| 손목 회전도 따라가게 | 실행 명령에 `--orientation-mode relative`를 붙인다(기본은 `hold`: 손바닥 방향 고정) |

**처음에는 1~2 cm씩 천천히** 움직인다. 손을 아주 빨리 움직이거나 추적이 튀면(한 번에 10 cm 이상) 안전을 위해 `locked`가 된다. 이때는 grip을 놓았다가 다시 쥐면 풀린다.

### 8-4. 시작 자세에서 주의할 동작
시작 자세는 RH56F1용이라 몇몇 관절에 여유가 작다. 아래처럼 움직이면 로봇이 **멈춘다**. 고장이 아니라 안전하게 멈추는 동작이다.
- **손목을 한쪽으로 크게 돌릴 때:** 한쪽은 약 12°만 여유가 있다.
- **손을 몸 쪽 위로 크게 당길 때:** 팔꿈치가 끝까지 접힌다.

멈추면 `status`의 `reason`에 `IK failed: target not reachable` 등이 보인다. 손을 반대로 되돌리면 다시 따라온다. 녹화 때 문제없던 범위는 앞 10 cm, 바깥 6 cm, 아래 6 cm, 손목 ±20°, 반지름 5 cm 원이다.

---

## 9. 매번: 끝내기

1. grip을 놓는다.
2. 원격조종 터미널(5, 왼팔이면 그것도)에서 Ctrl-C
3. 브리지(4) Ctrl-C
4. 손(2, 3), 팔(1) 순서로 Ctrl-C

원격조종은 끝날 때 팔을 영점이나 홈으로 보내지 않는다. fake에서는 다시 띄우면 영점에서 시작한다.

---

## 10. 실물로 움직이기: 아직 불가

### 10-1. 실물 전에 만들어야 하는 것
| 항목 | 지금 | 필요한 작업 |
|---|---|---|
| LEAP 손 실물 구동 | 없음 (fake만) | Dynamixel ros2_control 하드웨어 플러그인(Jazzy API), `leap_hand_description`에 ros2_control 파일, `leap_hand.launch.py runtime:=real` |
| 팔 실물 구동 (Jazzy) | RH56F1용 Humble 방식만 있음 | `openarm_leap_arms.launch.py runtime:=real`. Jazzy는 description을 토픽으로 넘겨야 하고, 실물 팔 컨트롤러 이름과 시작 정책(비활성으로 띄우기)도 필요 |
| 원격조종 실물 설정 | `quest_teleop_leap.yaml`에 fake만 있음 | `runtimes.real` 항목, profile의 실물 팔 컨트롤러 이름 |
| 시스템 라이브러리 | CLI11 없음 | `sudo apt install -y libcli11-dev` (팔 CAN 통신 `openarm_can` 빌드에 필요) |
| 어댑터 | 임시값 (RH56F1 높이) | LEAP 어댑터 치수·질량 반영 후 모델 재생성 |

### 10-2. 만들어진 뒤의 순서 (예정, RH56F1 실물 절차와 같은 구조)
아래는 기존 RH56F1 실물 절차(`docs/rh56f1/quest-teleop.md` 6~8절, `docs/rh56f1/rh56f1-real-bringup.md`)를 LEAP에 옮긴 **예정 순서**다. 명령 이름은 구현 때 확정된다.

1. **점검(preflight):** E-stop을 손 닿는 곳에 둔다. CAN 장치 확인, 주변을 비운다. 팔을 받칠 사람이나 거치대를 준비한다.
2. **실물 팔 띄우기:** 모터는 비활성(inactive)으로 시작한다. 띄우는 것만으로는 움직이지 않는다.
3. **읽기 전용 확인:** 관절 상태에 NaN이 없는지, TF가 나오는지 본다. 아무것도 움직이면 안 된다.
4. **오른팔만 활성화:** 하드웨어를 active로 만들고 팔 컨트롤러를 켠다. 활성화 순간에는 측정 자세를 유지한다.
5. 브리지를 켜고, **dry run**으로 `status`만 확인한다. 방향이 다르면 `axis_mapping` / `yaw_offset_deg`를 고친다.
6. **실행:** 실물은 `--execute --confirm-real-hardware` **두 개가 다 있어야** 명령이 나간다. `--execute`만 주면 거부된다.
7. 시작 자세 이동은 fake와 같다. 처음에는 `position_scale`을 0.3 정도로 낮추고 1~2 cm씩 움직인다.

### 10-3. 실물에서 멈추는 방법 (기존 문서와 동일)
| 동작 | 결과 |
|---|---|
| grip을 놓는다 | 명령 중지, 팔은 마지막 위치 유지(토크 유지) |
| 원격조종 Ctrl-C | 명령 종료, 컨트롤러가 위치 유지 |
| 팔 컨트롤러 비활성화 | 새 명령을 받지 않음, 토크 유지 |
| **물리 E-stop / 전원 차단** | 모든 것이 멈춤. **소프트웨어에 의존하지 않는 유일한 정지** |

팔 하드웨어를 inactive로 내리면 **토크가 꺼져 팔이 떨어진다.** 반드시 받친 상태에서 한다.

---

## 11. 문제 해결

| 증상 | 원인과 해결 |
|---|---|
| monitor에 `rate 0`, 패킷 없음 | 앱에 입력한 PC IP/포트, 같은 공유기인지, 방화벽(`sudo ufw allow 5006/udp`)을 확인 |
| 브리지가 포트 사용 중 오류 | monitor나 다른 브리지가 5006을 잡고 있다 → 끄기 |
| `the teleop IK needs pink and pinocchio` | `python3` 대신 **`.venv/bin/python`**으로 실행 (2-2 설치) |
| `no robot description … /openarm_leap/arms/robot_description` | 터미널 1(`openarm_leap_arms.launch.py`)이 안 떠 있거나 `ROS_DOMAIN_ID`가 다르다 |
| `controller … is not loaded / not active` | 팔 launch가 아직 준비 중이다. 로그에 `topic commands are accepted`가 나온 뒤 실행 |
| `state locked` | 입력이 끊겼거나(0.2초), 추적이 튀었다 → grip을 놓았다 다시 쥔다 |
| `IK failed: target not reachable` | 8-4의 작업 공간 한계 → 손을 되돌린다 |
| 시작 자세에서 grip을 눌러도 안 움직임 | grip을 실행 전부터 쥐고 있었다 → 한 번 놓았다가 다시 쥔다 |
| RViz 몸통 기둥이 빨갛다 | 정상이다. 메시 색이 그렇게 그려진다(`RobotModel` 상태는 OK) |
| RViz `RobotModel` 항목이 빨간 오류 | 항목을 펼쳐 오류 메시지를 본다. 대개 TF가 없는 링크 때문이다 → 팔 launch(터미널 1)가 정상인지 확인 |
| `joint_state_publisher_gui` 없음, 404 | `sudo apt update` 후 다시 설치 |
| `adb devices`가 비어 있음 | 케이블이 충전 전용이거나 개발자 모드가 꺼져 있다(3-1) → 데이터 케이블로 바꾸고 헤드셋 재부팅 |
| `adb devices`에 `unauthorized` / `no permissions` | 헤드셋 안의 USB 디버깅 허용 창(3-4) / udev 규칙(3-3) |

---

## 12. 참고 문서
- `docs/leap_hand/quest-teleop-leap-pink.md`: pink IK, 시작 자세, 테스트, 작업 공간 분석
- `docs/leap_hand/openarm-leap-split-bringup.md`: OpenArm + LEAP 분리 bringup 구조 (Jazzy)
- `docs/rh56f1/quest-teleop.md`: Quest 원격조종 원래 문서 (RH56F1, 실물 절차 6~8절)
- `docs/rh56f1/rh56f1-real-bringup.md`: 실물 팔 preflight·정지 절차
- 녹화: `figure/openarm_leap_pink_quest_teleop.webm`
