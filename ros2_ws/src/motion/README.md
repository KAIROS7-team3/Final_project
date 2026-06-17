# motion 패키지

Doosan e0509 + RH-P12-RN 그리퍼 통합 제어 패키지 (Track A/B).

## 구성

| 모듈 | 설명 |
|------|------|
| `motion/gripper_node.py` | RH-P12-RN 그리퍼 ROS2 노드. DRL/TCP 두 가지 transport 지원 |
| `motion/rviz_joint_state_merger_node.py` | 팔 joint_states + 그리퍼 pulse를 합쳐 RViz에 전달 |
| `motion/gripper_conversion.py` | 그리퍼 pulse ↔ rad 변환 유틸리티 |
| `config/gripper_node.yaml` | 그리퍼 노드 파라미터 |
| `launch/bringup_e0509_with_gripper.launch.py` | e0509 + 그리퍼 통합 bringup |
| `urdf/e0509_with_gripper.urdf` | 그리퍼 링크 포함 URDF |

## 실행

```bash
# 빌드
cd ~/Final_project/ros2_ws
colcon build --packages-select motion --symlink-install
source install/setup.bash

# 실물 로봇 bringup (IP 명시 필수)
ros2 launch motion bringup_e0509_with_gripper.launch.py \
  mode:=real model:=e0509 host:=<ROBOT_IP> robot_ip:=<ROBOT_IP>
```

## 그리퍼 명령

launch 실행 후 `[gripper] DRL 초기화 완료` 로그 확인 후 사용.

```bash
# 열기
ros2 topic pub --once /gripper/cmd_direct std_msgs/msg/String "{data: 'open'}"

# 닫기
ros2 topic pub --once /gripper/cmd_direct std_msgs/msg/String "{data: 'close'}"

# 커스텀 (pulse 0~700, current mA)
ros2 topic pub --once /gripper/cmd_direct std_msgs/msg/String "{data: 'custom 420 300'}"

# action (피드백 포함)
ros2 action send_goal /gripper/grasp interfaces/action/Grasp \
  "{tool_id: 'test', grasp_force: 300.0}"
```

## 상태 확인

```bash
# 그리퍼 pulse 상태 (close=700, open=0)
ros2 topic echo /gripper/state

# RViz 입력 (팔 + 그리퍼 합산)
ros2 topic echo /dsr01/joint_states_rviz
```

## 주요 파라미터 (gripper_node.yaml)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `command_transport` | `drl` | 명령 전송 방식: `drl` \| `tcp` |
| `robot_ns` | `dsr01` | 로봇 네임스페이스 |
| `pulse_open` | `0` | 열림 pulse |
| `pulse_closed_preset` | `700` | 닫힘 pulse |
| `init_current` | `400` | 초기화 전류 (mA) |
| `grip_current` | `300` | 파지 전류 (mA) |
| `gripper_command_action_enabled` | `true` | `/gripper/grasp` action 서버 활성 |
| `direct_cmd_topic_enabled` | `true` | `/gripper/cmd_direct` 토픽 활성 |

`robot_ip`는 launch 인자로만 전달 (`host:=<IP> robot_ip:=<IP>`). yaml에 하드코딩 금지.

## 아키텍처

```
/gripper/cmd_direct (String)
        │
        ▼
  gripper_node
        │  DRL transport
        ▼
  /dsr01/drl/drl_start (service)
        │
        ▼
  로봇 DRL → flange serial → Modbus RTU → RH-P12-RN
        │
        │  state (optimistic, DRL 모드에서는 명령 후 업데이트)
        ▼
  /gripper/state (JointState, name=gripper_joint)
        │
        ▼
  rviz_joint_state_merger
        │  pulse → rad 변환
        ▼
  /dsr01/joint_states_rviz → RViz
```

## 현재 상태 및 제한사항

- **DRL 모드**: 명령은 정상 동작. 상태는 실제 피드백 없이 명령값으로 업데이트 (optimistic).
- **TCP 모드**: DRL 서버 코드를 로봇에 배포해 TCP 소켓으로 양방향 통신. 실제 state feedback 가능하나 로봇 환경에 따라 소켓 서버 실행 여부 확인 필요.
- **초기 상태**: 노드 시작 시 실제 그리퍼 위치를 읽지 않아 pulse=0으로 초기화됨.

## vision_drawer open/close 시퀀스 (feat/track-b-vision-sequences)

`unit_actions/toolbox_motion.py`에 정의. `toolbox_seq_runner`에서 실행.

### 공통 구조 (11스텝, open/close 동일)

| 스텝 | open | close |
|------|------|-------|
| ① | GRIP_RELEASE | GRIP_RELEASE |
| ② | MoveJ → SETUP_J | MoveJ → CLOSE_SETUP_J |
| ③ | MoveL → APPROACH (하드코딩) | MoveL → OPENDOWN (하드코딩) |
| ④⑤ | **VISUAL_SERVO_XZ** | **VISUAL_SERVO_XZ** |
| ⑥ | GRIP_BOX | GRIP_BOX |
| ⑦ | MoveL → OPEN (당김) | MoveL → OPEN |
| ⑧ | MoveL → SILENCE (Z -9mm) | MoveL → APPROACH (밀기) |
| ⑨ | GRIP_RELEASE | GRIP_RELEASE |
| ⑩ | MoveL → INNER | MoveL → CLOSE_END |
| ⑪ | JOINT_HOME | JOINT_HOME |

### Visual Servoing (④⑤) 개념

탑뷰 카메라는 서랍 손잡이를 보기 어렵고, 그리퍼 카메라는 화질이 낮아 TF 좌표에 오차 존재.
오차가 얼마인지 정확히 모르므로, **한 번에 이동하지 않고 폐루프로 수렴**시키는 PBVS 방식 사용.

```
비전(TF) 손잡이 좌표 읽기
        ↓
현재 EE 위치와 XZ 오차 계산
        ↓
vx = Kp × err_x,  vz = Kp × err_z,  vy = 0 (Y 고정)
        ↓
movel_delta 로 조금 이동 → 다시 읽기 → 반복
        ↓
|err_xz| ≤ xz_align_thr_mm → GRIP_BOX
```

- **vy = 0 고정**: Y는 서랍 당기는/미는 방향이라 VS로 건드리지 않음
- **XZ만 보정**: 그리퍼 카메라가 Y 방향을 향해 장착 → 이미지 가로=X, 세로=Z
- **파라미터**: `config/visual_servo.yaml` (Kp, 임계값, timeout 등)
- **구현체**: `unit_actions/visual_servoing.py` — `HandleServoController`

### layer 0 / layer 1 웨이포인트

| | layer 0 (1층) | layer 1 (2층) |
|-|--------------|--------------|
| SETUP_J | `[-19.53, 53.85, 110.47, 71.14, 95.19, -75.18]` deg | `[-6.14, 44.85, 116.43, 84.19, 91.97, -71.38]` deg |
| CLOSE_SETUP_J | `[-23.33, 56.66, 107.63, 67.45, 96.16, -75.52]` deg | `[-23.64, 48.6, 110.08, 67.82, 98.38, -70.33]` deg |
| APPROACH | `[378.88, 433.02, 65.45, 90,90,90]` | `[380.57, 427.51, 115.68, 90,90,90]` |
| OPEN | `[378.88, 243.86, 65.46, 90,90,90]` | `[380.56, 237.79, 115.69, 90,90,90]` |
| INNER | `[378.88, 169.1, 50.45, 90,90,90]` | `[380.56, 165.94, 103.69, 90,90,90]` |

layer 1이 layer 0보다 Z 약 +50mm 높음.

### 실행 명령

```bash
# vision open layer 0
ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_open_0

# vision open layer 1
ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_open_1

# vision close layer 0
ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_close_0

# vision close layer 1
ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_close_1
```

> approach_x/y/z 파라미터 불필요 — VS가 실시간으로 정렬하므로 제거됨.

### 비전팀 연결 시 확인 필요 (서랍 open/close)

- `/vision/handle_pose` 토픽명 확정 (현재 `geometry_msgs/PointStamped` 가정)
- 좌표 단위 확인 (runner에서 m → mm 변환 적용 중)
- `config/visual_servo.yaml` `handle.kp`, `handle.xz_align_thr_mm` 실기 튜닝

---

## vision_fetch 시퀀스 (feat/track-b-vision-sequences)

### 구조 (12스텝)

| 스텝 | 동작 | 좌표 출처 |
|------|------|-----------|
| ① | JOINT_HOME | 고정 |
| ② | GRIP_RELEASE | — |
| ③ | MoveL — 공구 위쪽 | **탑뷰 D455f XY** + `TOOL_APPROACH_Z_MM` (고정 234mm) |
| ④ | **VISUAL_SERVO_XY** | 그리퍼 캠 C270 XY P제어 수렴 |
| ⑤ | MoveL — 공구로 하강 | **그리퍼 캠 C270 XYZ** |
| ⑥ | GRIP_SOCKET (pulse=650) | — |
| ⑦ | MoveL — 위로 상승 | 탑뷰 XY + 고정 Z (③과 동일) |
| ⑧ | MoveL — staging 위 | `SOCKET_BOTTOM_XY` 고정 |
| ⑨ | MoveL — staging 하강 | `SOCKET_BOTTOM` 고정 |
| ⑩ | GRIP_RELEASE | — |
| ⑪ | MoveL — staging 위 | `SOCKET_BOTTOM_XY` 고정 |
| ⑫ | JOINT_HOME | 고정 |

### Visual Servoing (④) 개념

탑뷰 rough XY로 공구 위로 이동한 뒤, 그리퍼 카메라로 XY 오차를 폐루프로 수렴.

```
[그리퍼 캠] 공구 XY 좌표 읽기
        ↓
현재 EE XY와 오차 계산
        ↓
vx = Kp × err_x,  vy = Kp × err_y,  vz = 0 (Z 고정)
        ↓
movel_delta_xy 로 조금 이동 → 다시 읽기 → 반복
        ↓
|err_xy| ≤ xy_align_thr_mm → DONE → 그리퍼 캠 Z로 하강
```

- **vz = 0 고정**: Z는 VS 완료 후 그리퍼 캠 Z값으로 별도 하강
- **파라미터**: `config/visual_servo.yaml` `tool` 섹션
- **구현체**: `unit_actions/visual_servoing.py` — `ToolServoController`

### 실행

```bash
ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_fetch -p tool_id:=socket_19mm
```

> 좌표 파라미터 불필요 — 탑뷰/그리퍼 토픽에서 실시간 수신.

### 비전팀 연결 시 확인 필요 (vision_fetch)

| 항목 | 현재 가정 | 확인 필요 |
|------|-----------|-----------|
| 탑뷰 공구 좌표 토픽 | `/vision/tool_top_pose` | 토픽명 확정 |
| 그리퍼 캠 공구 좌표 토픽 | `/vision/tool_gripper_pose` | 토픽명 확정 |
| 메시지 타입 | `geometry_msgs/PointStamped` | 타입 확정 |
| 좌표 단위 | m (runner에서 ×1000 → mm 변환) | 단위 확정 |
| `TOOL_APPROACH_Z_MM` | 234.0 mm (소켓 TW 실측) | 공구별 적정 높이 조정 |
| `TOOL_APPROACH_ORI` | `[53.23, 180.0, -38.07]` deg | 공구별 자세 조정 |
| `config/visual_servo.yaml` `tool.kp` | 1.0 | 실기 튜닝 필요 |
| `config/visual_servo.yaml` `tool.xy_align_thr_mm` | 3.0 mm | 실기 튜닝 필요 |

---

## TODO

- [x] **virtual 모드 대응**: `mode` 파라미터로 virtual 감지 → DRL/flange 초기화 생략 (에뮬레이터 블로킹 버그 수정)
  - launch 파일이 `mode:=virtual` 시 gripper_node에 `mode` 파라미터 전달
  - `_init_drl_server()` 진입 시 virtual이면 즉시 반환
- [ ] **초기 상태 읽기**: 노드 시작 시 DRL로 현재 그리퍼 위치(present_position)를 읽어 `_current_hz_pos` 초기화
  - `_init_drl_server` 완료 후 `_fc03(slave_id, REG_PRESENT_POSITION, 2)` 호출해 실제 pulse 값 반영
  - RViz에서 시작부터 실제 그리퍼 상태 표시 가능
- [ ] **VS 실기 튜닝**: `config/visual_servo.yaml` handle/tool 각 섹션 kp·임계값 실측 보정
- [ ] **비전팀 인터페이스 확정** (아래 항목 비전팀 구현 필요)
  - `/vision/fetch/tool_gripper_pose` (`geometry_msgs/PoseStamped`) — fetch 스캔 자세에서 찍은 공구 XY + rz
  - `/vision/return/tool_gripper_pose` (`geometry_msgs/PoseStamped`) — return 스캔 자세에서 찍은 공구 XY + rz
  - 단위: position (m, robot base frame), orientation (quaternion → yaw=rz 추출)
  - fetch/return 스캔 자세가 다르므로 **토픽을 반드시 분리** 구현할 것
  - `return_z_mm` (config/toolbox.yaml 각 공구별): 실측 후 0.0 → 실제값으로 갱신 필요
- [ ] **vision_return VS 구현**: return 시퀀스도 VS 방식으로 전환 (staging pick + slot place 각각 VS 적용)
