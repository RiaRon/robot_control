# senseglove_teleop

SenseGlove Nova 2(`senseglove_ros`)로 Inspire Hand RH56F1을 텔레옵하는 브리지.

## 왜 이렇게 만들었나

[`Nova2Dex`](https://github.com/Lvn-7/Nova2Dex-A-ROS-2-Framework-for-Dexterous-Hand-Teleoperation)의
`nova2_inspire_retarget` 패키지 구조(순수 매핑 함수 + 얇은 ROS 노드, 같은
`JOINT_NAMES`/라디안 규약)를 그대로 따랐다. 다만 `Nova2Dex`는 Windows에서
SenseCom SDK → UDP → `manus_ros2_msgs/ManusGlove` 메시지를 거치는데, 우리
`senseglove_ros`는 **Linux에서 블루투스(BLE)로 직접** 연결해 표준
`sensor_msgs/JointState`를 낸다(2026-09-22 확인, `Nova 2-00795-L`/`00782-R`).
그래서 Windows 중계 없이 `senseglove_ros`의 토픽을 바로 입력으로 쓴다.

`Nova2Dex`에도 없는 부분(라디안 → 실제 RH56F1 시리얼 레지스터)은
`hand_bridge_node`로 새로 만들었다.

## 구조

```
senseglove_ros (BLE, 이미 우리 워크스페이스에 있음)
  └─ /senseglove/glove<serial>/<lh|rh>/joint_states  (라디안, mcp/pip/dip)
       │
       ▼
senseglove_teleop / retarget_node   (mapping.py의 순수 함수 direct_map 사용)
  └─ /inspire_<left|right>/retarget/joint_states
       (JOINT_NAMES = pinky/ring/middle/index_proximal_joint,
        thumb_proximal_pitch/yaw_joint — Nova2Dex와 동일 규약)
       │
       ▼
senseglove_teleop / hand_bridge_node
  └─ RH56F1 시리얼(RS-485, 0xEB 0x90 ... 프로토콜) → angleSet(1040), forceSet(1046)
```

## 관절 매핑

- 4손가락(pinky/ring/middle/index)은 senseglove의 `<side>_<finger>_pip` 관절을
  폐색(closure) 신호로 써서 Inspire의 `*_proximal_joint`(0~1.47rad)로 정규화
  매핑한다. **`ring`과 `pinky`는 항상 같은 값이다** — Nova 2 글러브에 새끼손가락
  전용 센서가 없어 두 손가락을 한 센서로 같이 잰다(2026-09-22 실기로 확인,
  하드웨어 특성이지 버그가 아님).
- 엄지(thumb_pitch/yaw)는 `retarget_node`가 항상 둘 다 계산해 발행하지만,
  `hand_bridge_node`가 실제로 쓰는 건 굽힘(thumb_pitch)뿐이다.
  `drive_thumb:=false`(기본)에서는 굽힘도 고정 안전값(1350)을 보내고,
  `drive_thumb:=true`로 켜면 굽힘만 실제로 따라 움직이며 항상 안전 범위로
  클램프한다(아래 참고). 엄지 매핑 방향(어느 쪽이 폄/쥠인지)은 2026-09-22
  기준 아직 실기로 확인 중이다. **엄지 회전(thumb_yaw)은 `drive_thumb` 값과
  무관하게 항상 고정값(950)이다** — 글러브로 엄지를 돌려 맞추는 조작이
  불편하고, 굽힘만 추종하는 편이 조작 일관성 측면에서 낫다고 판단해
  2026-09-28에 그렇게 정함(발행된 thumb_yaw 값 자체는 무시됨).

## 안전 설계

- **`hand_bridge_node`는 기본이 dry-run이다.** `execute:=true`를 명시해야
  실제로 `/dev/ttyUSB*`를 열고 시리얼을 쓴다. dry-run에서는 계산된
  angleSet 목표만 로그로 찍는다.
- 손가락 목표는 항상 `[900, 1740]`으로 클램프한다.
- `forceSet`은 매뉴얼(RH56F1-User-ManualV1.2 표 37) 범위 `0-1000` 이하로
  클램프한다(2026-09-21 검증: 이 매뉴얼 버전은 6개 모두 0-1000).
- **엄지 굽힘은 기본이 고정 안전값이다.** `execute:=true`여도 `drive_thumb:=true`를
  **추가로** 명시해야 엄지 굽힘이 토픽 값을 따라간다. 켜더라도 항상
  `[1100, 1350]`으로 클램프해서 **엄지 굽힘이 900까지 내려가는 일은 계산이
  잘못돼도 일어나지 않는다**(CLAUDE.md 안전 규칙). **엄지 회전은 `drive_thumb`
  값과 무관하게 항상 `[600, 1800]` 범위의 고정값(950)이다**(2026-09-28,
  조작 편의를 위한 사용자 결정).

## 실행 예시

```bash
colcon build --symlink-install --packages-select senseglove_teleop
source install/setup.bash

# 1) dry-run으로 먼저 매핑값 확인 (손 연결 없이도 가능)
ros2 launch senseglove_teleop senseglove_inspire_teleop.launch.py \
    left_enabled:=true right_enabled:=false execute:=false

# 2) 확인 후에만 실제 손 구동 (4손가락만, 엄지는 고정)
ros2 launch senseglove_teleop senseglove_inspire_teleop.launch.py \
    left_enabled:=true right_enabled:=false execute:=true

# 3) 4손가락 동작을 확인한 뒤에만 엄지도 추가로 구동
ros2 launch senseglove_teleop senseglove_inspire_teleop.launch.py \
    left_enabled:=true right_enabled:=false execute:=true drive_thumb:=true
```

오른손 기본 포트/ID(`/dev/ttyUSB1`, ID 2)는 `README.md`(inspire_hand)의
표기를 따른 값이며 **코드로 검증되지 않았다.** 오른손을 실제로 연결하기
전에 포트와 ID를 다시 확인할 것.

## 보정이 필요할 수 있는 것

`config/mapping.yaml`의 `*_zero_rad`/`*_range_rad`는 착용자 손 크기 및 SenseCom 재연결 시마다 드리프트가 발생할 수 있습니다.
이를 위해 자동 캘리브레이션 및 원클릭 텔레옵 런처 스크립트가 준비되어 있습니다:

```bash
# 15초간 펴기/쥐기 반복 후 mapping.yaml 자동 갱신 및 텔레옵 원클릭 실행
python3 monitor_glove.py
```
- 15초 측정 후 자동으로 `mapping.yaml`에 `zero/range` 보정값을 덮어씁니다 (기존 파일은 `.bak` 백업).
- 저장이 완료되면 `[Y/n]` 프롬프트가 뜨며, 엔터를 누르면 실기 텔레옵이 즉시 실행됩니다.
- 옵션: `--duration 15.0`, `--side right`, `--launch` (질문 없이 바로 실행), `--haptics true` (햅틱 및 촉각 센서 수집 활성화).

## 햅틱 피드백 & 촉각 센서 데이터 자동 수집 (CSV & 실시간 모니터)

`haptics_enabled:=true` (또는 `python3 monitor_glove.py --haptics true`)를 지정하면 다음 기능이 완전 자동으로 연동됩니다:

1. **자동 CSV 로깅**:
   - 위치: `~/bumsu_ws/logs/inspire_hand_tactile_<YYYYMMDD_HHMMSS>.csv`
   - 내용: 타임스탬프, 손가락 5개 $F_n/F_t$/방향, 손바닥 3구역(좌/중/우) $F_n/F_t$/방향, SenseGlove 브레이크 저항값.
   - 호환성: `rerun_tactile_visualizer.py` 및 Pandas 즉시 로드 가능.
2. **실시간 터미널 모니터 (ASCII 바 게이지)**:
   - 메인 콘솔에서 0.3초 주기로 손가락별/손바닥 접촉력과 햅틱 저항 바 게이지 출력.
   - `monitor_glove.py --haptics true` 실행 시 전용 모니터 창(`gnome-terminal`)이 자동 팝업되어 전 화면 갱신 모니터링 제공.
   - 수동 실행: `ros2 run senseglove_teleop tactile_visualizer --side left`
3. **종료 통계 보고서**:
   - `Ctrl+C` 종료 시 총 수집 시간, 프레임 수, 평균 Hz, 손가락별 최대 가압력(Peak Fn) 및 CSV 저장 경로 요약 출력.

**힘 피드백(FFB) 트리거 기준 (2026-09-28부터 변경)**: 이전엔 손끝 촉각(Fn)이 클수록
글러브 브레이크가 세지는 방식이었으나, 컵 형상 때문에 손끝 촉각 패드에 실제로 안
닿는 경우가 많다는 게 2026-09-27 forceSet 파일럿 실기로 확인돼(예: 4손가락 중
다수가 Fn=0인 채로 파지 종료), **관절별 실측 모터부하(forceAct)를 forceSet 대비
비율로 매핑**하는 방식으로 교체했다. `haptics_grip_ratio_for_max`(기본 `0.5`, "잡는
세기의 50%")를 채우면 이미 `haptics_max_effort`로 포화된다. 촉각(Fn) 데이터 자체는
CSV/토픽에 그대로 계속 수집되며, 바뀐 건 피드백의 입력 신호뿐이다.



## 테스트

```bash
cd senseglove_teleop
python3 -m pytest tests/ -v
```

`mapping.py`, `hand_protocol.py`는 ROS·하드웨어 의존성이 없는 순수 함수라
`colcon build` 없이 바로 테스트할 수 있다.
