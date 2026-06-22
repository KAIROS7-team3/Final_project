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

### 구조 (14스텝)

Visual Servo 없이 그리퍼 캠 한 번 스캔 → 직접 이동 방식.

| 스텝 | 동작 | 좌표 출처 |
|------|------|-----------|
| ① | JOINT_HOME | 고정 |
| ② | grip(0) — 완전 개방 | pulse=0 |
| ③ | MoveJ — 그리퍼 캠 스캔 자세 | `VISION_FETCH_SCAN_J_DEG` = `[-30.1, 15.5, 74.7, 20.9, 101.2, -27.8]` deg |
| ④ | WAIT_VISION_TOP_XY — 토픽 수신 대기 | `/vision/tool_gripper_pose` 신규 수신 대기 |
| ④-1 | GRIP_RELEASE — 파지 준비 개방 | pulse=450 |
| ⑤ | MoveL — 공구 위쪽 | 그리퍼 캠 XY + `tool_approach_z_mm` (고정 234mm) |
| ⑥ | MoveL — 공구 하강 | 그리퍼 캠 XY + `grasp_z_mm` |
| ⑦ | GRIP_TOOL (pulse=650→grip_stroke) | — |
| ⑧ | MoveL — 위로 상승 | 그리퍼 캠 XY + 234mm (⑤과 동일) |
| ⑨ | MoveL — staging 위 | `SOCKET_BOTTOM_XY` = `[550.0, -172.72, 235.73, ...]` 고정 |
| ⑩ | MoveL — staging 하강 | `SOCKET_BOTTOM` = `[550.0, -172.72, -0.12, ...]` 고정 |
| ⑪ | GRIP_RELEASE | pulse=450 |
| ⑫ | MoveL — staging 위 | `SOCKET_BOTTOM_XY` 고정 |
| ⑬ | JOINT_HOME | 고정 |

### 실행

```bash
ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_fetch -p tool_id:=socket_19mm
```

> 좌표 파라미터 불필요 — 그리퍼 캠 토픽에서 실시간 수신.

### 비전팀 연결 시 확인 필요 (vision_fetch)

| 항목 | 현재 가정 | 확인 필요 |
|------|-----------|-----------|
| 그리퍼 캠 공구 좌표 토픽 | `/vision/tool_gripper_pose` | 토픽명 확정 |
| 메시지 타입 | `geometry_msgs/PoseStamped` (XY + rz) | 타입 확정 |
| 좌표 단위 | m (runner에서 ×1000 → mm 변환) | 단위 확정 |
| theta(rz) | `PoseStamped.pose.orientation` → rz 추출 | **gripper_marker_scan_node에 PCA theta 미구현 — 추가 필요** |
| `tool_approach_z_mm` | 234.0 mm | 공구별 적정 높이 조정 |
| `tool_approach_ori` | `[53.23, 180.0, -38.07]` deg | 공구별 자세 조정 |

### TODO

- [ ] **비전팀 필수 작업**: `gripper_marker_scan_node.py` — PCA theta 계산 추가 + `PointStamped` → `PoseStamped` 퍼블리시 변경 (`docs/interfaces.md` §4 참조, PR 검토 시 확인)

---

## vision_return 시퀀스 (feat/motion-drawer-v2)

공구를 staging area에서 집어 공구함 slot으로 반납.

### 구조 (14스텝)

| 스텝 | 동작 | 좌표 출처 |
|------|------|-----------|
| ① | JOINT_HOME | 고정 |
| ② | grip(0) — 완전 개방 | pulse=0 |
| ③ | MoveJ — 그리퍼 캠 스캔 자세 | `VISION_RETURN_SCAN_J_DEG` = `[-24.60, 32.49, 50.78, 22.42, 105.63, -19.92]` deg |
| ④ | WAIT_VISION_RETURN_XY | `/vision/tool_gripper_pose` 수신 대기 (5초 타임아웃) |
| ④-1 | GRIP_RELEASE — 파지 준비 개방 | pulse=450 |
| ⑤ | MoveL — staging 위 | 그리퍼 캠 XY + rz + 고정 Z 234mm |
| ⑥ | MoveL — staging 파지 하강 | 그리퍼 캠 XY + rz + `staging_pickup_z_mm` |
| ⑦ | GRIP_TOOL (pulse=650→grip_stroke) | — |
| ⑧ | MoveL — staging 위 상승 | 그리퍼 캠 XY + rz + 고정 Z 234mm |
| ⑨ | MoveL — slot 위 | `grasp_pose_base` XY (yaml, m→mm) + 고정 Z 234mm |
| ⑩ | MoveL — slot 반납 하강 | `grasp_pose_base` XY + `return_z_mm` |
| ⑪ | GRIP_RELEASE | pulse=450 |
| ⑫ | MoveL — slot 위 상승 | `grasp_pose_base` XY + 고정 Z 234mm |
| ⑬ | JOINT_HOME | 고정 |

### 좌표 출처 정리

| 단계 | XY 출처 | Z 출처 |
|------|---------|--------|
| ⑤⑥⑧ | `/vision/tool_gripper_pose` (PoseStamped, m→mm) | ⑤⑧: 234mm 고정 / ⑥: `staging_pickup_z_mm` |
| ⑨⑩⑫ | `config/toolbox.yaml` `grasp_pose_base.x/y` × 1000 | ⑨⑫: 234mm 고정 / ⑩: `return_z_mm` |

### 실행

```bash
ros2 run motion toolbox_seq_runner --ros-args -p sequence:=vision_return -p tool_id:=socket_19mm
```

### 공구별 Z값 (config/toolbox.yaml)

| 공구 | staging_pickup_z_mm | return_z_mm | slot XY (mm) |
|------|---------------------|-------------|--------------|
| screwdriver | 45.65 ⚠️ | 45.65 | (392.4, 264.2) |
| utility_knife | 102.87 ⚠️ | 102.87 | (445.5, 347.9) |
| ratchet_wrench | 100.32 ⚠️ | 100.32 | (397.6, 258.5) |
| multi_tool | 52.59 ⚠️ | 52.59 | (261.3, 332.8) |
| spanner_16mm | 56.4 ⚠️임의값 | 56.4 ⚠️임의값 | (437.2, 348.3) |
| socket_19mm | 119.69 ⚠️ | 119.69 | (253.0, 343.1) |

> ⚠️ `staging_pickup_z_mm`은 모두 임시값(return_z_mm과 동일). 실기 테스트 전 직접교시로 실측 필요.

### 비전팀 연결 시 확인 필요 (vision_return)

| 항목 | 현재 가정 | 확인 필요 |
|------|-----------|-----------|
| 토픽명 | `/vision/tool_gripper_pose` | 비전팀 합의 필요 (기존 `/vision/tool_gripper_pose`에서 변경) |
| 메시지 타입 | `geometry_msgs/PoseStamped` | 기존 `PointStamped`에서 변경 — 비전팀 구현 필요 |
| 좌표 단위 | position: m (runner에서 ×1000), orientation: quaternion | 확인 필요 |
| rz 추출 | quaternion → yaw (atan2) | 범위 -185~185° 벗어나면 runner가 거부 |

---

## 핸드오버 테스트 (`place_on_hand_test`)

공구함 픽업 앞부분을 건너뛰고 **이미 공구를 쥔 상태에서 손에 전달만** 테스트하는 시퀀스.
(`handover_place_only_seq` — WAIT_HAND_POSE → APPROACH → PLACE → GRIP(450))

### 실행 순서

**터미널 2 — RealSense 카메라**
```bash
source ~/Final_project/ros2_ws/install/setup.bash
ros2 launch vision realsense_bringup.launch.py
```

**터미널 3 — 핸드 감지 파이프라인**
```bash
source ~/Final_project/ros2_ws/install/setup.bash
ros2 launch vision hand_detection.launch.py
```

**터미널 6 — tool_action_server**
```bash
source ~/Final_project/ros2_ws/install/setup.bash
ros2 run motion tool_action_server
```

### 공구별 명령어

```bash
# handle_first (라쳇·드라이버·칼) — 손잡이 방향 전달
ros2 action send_goal /place_on_hand_test interfaces/action/PlaceOnHand "{tool_id: 'ratchet_wrench'}"
ros2 action send_goal /place_on_hand_test interfaces/action/PlaceOnHand "{tool_id: 'screwdriver'}"
ros2 action send_goal /place_on_hand_test interfaces/action/PlaceOnHand "{tool_id: 'utility_knife'}"

# direct (소켓·스패너·멀티툴) — 손바닥 중심 전달
ros2 action send_goal /place_on_hand_test interfaces/action/PlaceOnHand "{tool_id: 'multi_tool'}"
ros2 action send_goal /place_on_hand_test interfaces/action/PlaceOnHand "{tool_id: 'spanner_16mm'}"
ros2 action send_goal /place_on_hand_test interfaces/action/PlaceOnHand "{tool_id: 'socket_19mm'}"
```

---

## TODO

- [x] **virtual 모드 대응**: `mode` 파라미터로 virtual 감지 → DRL/flange 초기화 생략 (에뮬레이터 블로킹 버그 수정)
  - launch 파일이 `mode:=virtual` 시 gripper_node에 `mode` 파라미터 전달
  - `_init_drl_server()` 진입 시 virtual이면 즉시 반환
- [x] **E-stop 블로킹 개선**: `_movel/_movej` 30초 블로킹 → 0.1s 폴링 × 5초 + E-stop 즉시 감지
- [x] **vision_return 시퀀스 구현**: 토픽 분리, rz 수신, staging/slot Z 분리, slot XY yaml 로드
- [ ] **초기 상태 읽기**: 노드 시작 시 DRL로 현재 그리퍼 위치(present_position)를 읽어 `_current_hz_pos` 초기화
- [ ] **VS 실기 튜닝**: `config/visual_servo.yaml` handle/tool 각 섹션 kp·임계값 실측 보정
- [x] **비전팀 인터페이스 확정**: `/vision/tool_gripper_pose` 단일 토픽 `PoseStamped` (XY + rz) — `docs/interfaces.md` §4 반영 완료
- [x] **staging_pickup_z_mm 실측**: 6종 공구 직접교시 실측 완료 (2026-06-18)
- [ ] **spanner_16mm 전체 Z 실측**: grasp_z_mm / staging_pickup_z_mm / return_z_mm 모두 임의값
- [x] **config/toolbox.yaml workspace_limits z_min**: -31.0mm (2026-06-18 실측 기준 갱신 완료)
- [ ] **vision_return VS 구현**: return 시퀀스도 VS 방식으로 전환 (staging pick + slot place)
- [ ] **핸드오버 그리퍼 열기 후 대기**: grip(450) 실행 후 다음 동작(홈 복귀 등) 전 **최소 10초 대기** 필요
  - 공구를 손에서 실제로 떼어내는 데 시간이 필요 (너무 빠르면 공구 낙하 위험)
  - `handover_place_only_seq` / `handover_fetch_handle_first_seq` 의 GRIP(450) 직후 `Step(kind=StepKind.WAIT, sec=10.0)` 추가 예정

---

## PR 리뷰어 체크리스트

> motion 패키지 PR 머지 전 리뷰어가 반드시 확인할 항목.

- [ ] **`/vision/tool_gripper_pose` 타입 확인**: 비전팀이 `geometry_msgs/PoseStamped`로 퍼블리시하는지 확인 (`PointStamped` 아님)
  - `pose.position.x/y/z` — 공구 위치 (m, base_link frame)
  - `pose.orientation` — quaternion → rz(theta, deg) 추출 (PCA 기반)
  - 미구현 시 runner `WAIT_VISION_TOP_XY` / `WAIT_VISION_RETURN_XY` 스텝에서 rz=0 으로 동작하여 파지 자세 오류 발생
- [ ] `config/toolbox.yaml` `staging_pickup_z_mm` 실측값 반영 여부
- [ ] `config/toolbox.yaml` `workspace_limits.z` 범위가 실제 동작 Z를 포함하는지
