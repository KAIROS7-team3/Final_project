# demo — voice→DB Gate→motion→PLC 통합 데모

소켓 렌치(19mm) fetch/return 시연 패키지.  
voice 노드 없이도 대시보드 버튼으로 단독 실행 가능.

---

## 사전 준비

### 1. 시스템 패키지 (apt)

```bash
# ROS2 Humble (이미 설치된 경우 생략)
sudo apt install -y ros-humble-desktop

# RealSense ROS2 드라이버
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description

# Doosan robot2 (소스 빌드 또는 팀 내부 패키지)
# → ros2_ws/src/doosan-robot2/ 가 있으면 colcon build 시 자동 포함

# 기타 ROS2 의존
sudo apt install -y \
  ros-humble-vision-msgs \
  ros-humble-launch-ros \
  ros-humble-py-trees
```

### 2. Python 패키지 (pip)

```bash
# 대시보드 (필수)
pip install fastapi uvicorn

# 카메라 (필수)
pip install opencv-python-headless

# RealSense SDK Python 바인딩 (탑뷰 카메라 사용 시)
pip install pyrealsense2

# Behavior Tree
pip install py-trees>=2.4.0

# 음성 (voice:=true 사용 시)
pip install openai-whisper
```

> **참고:** `fastapi`·`uvicorn`·`opencv-python-headless`·`pyrealsense2`는 rosdep 미지원 pip 패키지이므로 rosdep install 후 별도 설치 필요.

### 3. udev 카메라 규칙 (카메라 인덱스 고정)

```bash
sudo cp config/99-demo-cameras.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

규칙 적용 후:
- C270 그리퍼 캠 → `/dev/gripper_cam`
- RealSense D455f 탑뷰 → `/dev/top_cam`

---

## 빌드

### 0. 서브모듈 초기화 (최초 1회)

`doosan-robot2`와 `easy_handeye2`는 git 서브모듈이다. 워크트리 클론 직후 반드시 초기화한다.

```bash
# 레포 루트에서 실행
git submodule update --init --recursive
```

### 1. Doosan 패키지 빌드

```bash
cd ros2_ws

# dsr_msgs2: 서비스·메시지 타입 (다른 패키지가 의존)
# dsr_common2: 공통 유틸
# dsr_description2: URDF/xacro 모델
# dsr_bringup2: 런치 유틸리티 (bringup_e0509_with_gripper.launch.py)
# dsr_controller2: 컨트롤러 노드 (motion/move_joint, motion/move_line 서비스 제공)
# dsr_hardware2: 실물 하드웨어 인터페이스 (real 모드 필수)
colcon build --packages-select \
  dsr_msgs2 dsr_common2 dsr_description2 dsr_bringup2 dsr_controller2 dsr_hardware2

source install/setup.bash
```

> `virtual` 모드만 사용한다면 `dsr_hardware2`는 생략 가능하다.

### 2. interfaces 빌드

```bash
colcon build --packages-select interfaces
source install/setup.bash
```

### 3. 나머지 패키지 빌드

```bash
colcon build --packages-select motion orchestrator dashboard demo db plc voice

source install/setup.bash
```

---

## DB 초기화

매 테스트 전 DB를 초기 상태(socket_19mm → in_slot)로 리셋한다.

```bash
python3 scripts/seed_demo_db.py
```

`config/demo.yaml`에서 `tool_id`·`db_path` 변경 가능:

```yaml
demo:
  tool_id: socket_19mm
  db_path: ~/robot_tools.db
```

---

## 실행

### 기본 (대시보드 포함)

```bash
ros2 launch demo demo.launch.py robot_ip:=110.120.1.38
```

### 음성 포함

```bash
ros2 launch demo demo.launch.py \
  robot_ip:=110.120.1.38 \
  voice:=true
```

### PLC LED 포함

```bash
ros2 launch demo demo.launch.py \
  robot_ip:=110.120.1.38 \
  plc:=true \
  plc_port:=/dev/ttyUSB0
```

### 전체 옵션

```bash
ros2 launch demo demo.launch.py \
  robot_ip:=110.120.1.38 \
  voice:=true \
  plc:=true \
  plc_port:=/dev/ttyUSB0 \
  dashboard:=true \
  db_path:=~/robot_tools.db
```

### Launch 인자 목록

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `robot_ip` | `110.120.1.38` | Doosan 컨트롤러 IP |
| `robot_ns` | `dsr01` | Doosan 로봇 네임스페이스 |
| `voice` | `false` | Whisper STT + rule_intent 활성화 |
| `plc` | `false` | PLC 노드 활성화 |
| `plc_port` | `/dev/ttyUSB0` | PLC 시리얼 포트 |
| `dashboard` | `true` | 웹 대시보드 활성화 |
| `db_path` | `~/robot_tools.db` | SQLite DB 경로 |

---

## 대시보드

런치 후 브라우저에서:

```
http://localhost:8080
```

| 탭 | 내용 |
|----|------|
| 실시간 | 카메라 2대 MJPEG, 로봇 상태, fetch/return/home/E-stop 버튼, 그리퍼 전류 파형 |
| DB 상태 | 공구 현재 상태 (in_slot / staged / out / missing) |
| 이벤트 로그 | fetch/return/error/rejected 이력 |

---

## 대시보드 없이 수동 제어

```bash
# fetch (공구 꺼내기)
ros2 topic pub --once /voice/intent interfaces/msg/Intent \
  '{intent_type: "fetch", tool_id: "socket_19mm", raw_utterance: "manual"}'

# return (공구 반납)
ros2 topic pub --once /voice/intent interfaces/msg/Intent \
  '{intent_type: "return", tool_id: "socket_19mm", raw_utterance: "manual"}'

# home
ros2 service call /tool_action_server/home std_srvs/srv/Trigger {}

# E-stop
ros2 service call /tool_action_server/estop std_srvs/srv/Trigger {}

# E-stop 해제
ros2 service call /tool_action_server/estop_reset std_srvs/srv/Trigger {}
```

---

## demo_trigger — 마이크 없이 intent 1회 publish

음성 스택(`voice:=true`) 없이 fetch/return을 시연·테스트할 때 사용한다.
`/voice/intent`에 1회 publish 후 종료하는 1회성 노드.

```bash
# config/demo.yaml의 tool_id(socket_19mm) 기본 사용
ros2 run demo demo_trigger fetch
ros2 run demo demo_trigger return

# tool_id 명시
ros2 run demo demo_trigger fetch socket_19mm
ros2 run demo demo_trigger return socket_19mm
```

DB Gate(S-2)는 orchestrator BT의 `CheckFeasibility`가 그대로 수행하므로,
이 경로로 intent를 주입해도 안전 게이트는 우회되지 않는다.

---

## demo_ui — 독립 모니터링 대시보드 (선택)

`dashboard` 패키지(8080, `dashboard:=true`)와 별개로, `demo` 패키지 자체에
포함된 경량 FastAPI 모니터링 UI다. ROS2 스택과 독립적으로 떠 있어도 되고,
ROS2가 없으면 UI만 단독 실행된다.

```bash
# 의존성 (1회)
pip install --user fastapi "uvicorn[standard]" opencv-python-headless

ros2 run demo demo_ui
# 브라우저: http://localhost:8765
```

표시 항목:
- `/voice/raw_text`, `/voice/intent` — 음성 텍스트 및 파싱된 intent
- `/plc/status` — PLC LED 상태
- `/gripper/state` — 그리퍼 전류(effort[0]) 파형
- DB(`config/demo.yaml`의 `db_path`) — 공구 상태·이벤트 로그 (2초 폴링)
- 카메라 2대 (`/dev/video2` C270, RealSense color via `pyrealsense2`)
- fetch/return 트리거 버튼 → `/voice/intent` publish (orchestrator BT 경유, DB Gate 우회 없음)

---

## 노드 시작 순서

launch가 타이머로 자동 제어:

| 시간 | 노드 |
|------|------|
| t=0s | Doosan bringup (DSR + gripper_node) |
| t=5s | db_service_node |
| t=7s | orchestrator_node, tool_action_server, [plc_node] |
| t=9s | [whisper_node, rule_intent_node] (`voice:=true`) |
| t=10s | [dashboard_node] (`dashboard:=true`) |

---

## 안전 주의사항

- **E-stop 후 자동 재시작 없음** — `estop_reset` 서비스 호출 후 home 복귀 필요
- **동작 중 명령 차단 (S-7)** — `is_moving=True` 중 새 intent는 DB에 rejected 기록 후 무시
- **DB Gate (S-2)** — DB에서 가용 상태 확인 후에만 모션 시작. `out`·`missing`·`fod_alert` 상태 공구는 fetch 거부
- **v1.0 핸드오버 금지 (S-6)** — 모든 전달은 Staging Area 경유
