# 시뮬레이션

> Track A/B 전용. Track C는 ROS2 미사용 — 시뮬레이션 환경 해당 없음.
> 시뮬레이터 버전·world 파일·pass/fail 기준을 기록해 재현성을 보장한다.

---

## 1. 시뮬레이터 구성

| 항목 | 선택 |
|------|------|
| 시뮬레이터 | Gazebo Classic 11 (ROS2 Humble 기본) |
| 로봇 모델 | Doosan e0509 URDF/XACRO (`urdf/doosan_e0509.urdf.xacro`) |
| 그리퍼 모델 | RH-P12-RN URDF (tool0에 장착) |
| 카메라 플러그인 | `libgazebo_ros_camera.so` (RGB) + `libgazebo_ros_depth_camera.so` |
| world 파일 | `simulation/worlds/toolbox_scene.world` |
| `use_sim_time` | **true** (시뮬레이션 실행 시 항상) |

---

## 2. 실행 명령

### 기본 시뮬레이션 시작

```bash
# Track A 전체 스택 + Gazebo
./run.sh --track A --sim

# 또는 개별 launch
ros2 launch orchestrator sim_track_a.launch.py use_sim_time:=true

# Track B
ros2 launch orchestrator sim_track_b.launch.py use_sim_time:=true
```

### Gazebo만 띄우기 (드라이버 디버깅)

```bash
ros2 launch description doosan_e0509_gazebo.launch.py
```

### 재현 가능한 시뮬레이션 (deterministic seed)

```bash
# world 파일에 고정 seed 설정 (toolbox_scene.world 내부)
# <physics><seed>42</seed></physics>
ros2 launch orchestrator sim_track_a.launch.py seed:=42
```

---

## 3. `use_sim_time` 정책

- 시뮬레이션 실행 시 **모든 노드**가 `use_sim_time: true`를 받아야 함
- `rclcpp::Clock(RCL_ROS_TIME)` 사용 — `RCL_SYSTEM_TIME` 금지
- launch 파일에서 `SetParametersFromFile`로 일괄 적용 권장

```python
# launch 파일 예시
Node(
    package='voice',
    executable='whisper_node',
    parameters=[{'use_sim_time': use_sim_time}],
)
```

---

## 4. 시뮬레이션 World

### `toolbox_scene.world`

| 요소 | 설명 |
|------|------|
| 공구함 모델 | 9종 슬롯 배치, 실제 치수 기반 |
| 공구 모델 | 9종 (`config/tools.yaml` 기하 참조) |
| 조명 | 실험실 실내 조명 시뮬레이션 |
| Staging Area | 거치대 모델 포함 |
| 카메라 위치 | `config/hand_eye.yaml` 기준 고정 마운트 재현 |

### Bridge 매핑 (Gazebo → ROS2)

| Gazebo 토픽 | ROS2 토픽 | 타입 |
|-------------|-----------|------|
| `/camera/image_raw` | `/camera/image_raw` | `sensor_msgs/Image` |
| `/camera/depth/image_raw` | `/camera/depth/image_raw` | `sensor_msgs/Image` |
| `/camera/camera_info` | `/camera/camera_info` | `sensor_msgs/CameraInfo` |

---

## 5. BT 골든 파일 회귀 테스트

Behavior Tree 노드 변경 시 골든 파일 회귀를 실행해야 한다.

```bash
# 골든 파일 회귀 실행
colcon test --packages-select orchestrator
colcon test-result --verbose

# 새 골든 파일 생성 (의도적 변경 후)
python -m pytest orchestrator/tests/bt_regression/ --update-golden
```

### 골든 파일 구조

```
orchestrator/tests/bt_regression/
├── fetch_tool_golden.json     # FetchTool 서브트리 실행 시퀀스
├── return_tool_golden.json    # ReturnTool 서브트리 실행 시퀀스
└── recovery_golden.json       # 에러 복구 서브트리 실행 시퀀스
```

골든 파일은 BT 노드 실행 순서 + Blackboard 상태 변화를 기록한다.

---

## 6. 시뮬레이션 Pass/Fail 기준

### 단위 시나리오

| 시나리오 | Pass 조건 |
|----------|-----------|
| Fetch — `in_slot` 공구 | Staging Area 거치 완료, DB `staged` 갱신, PLC 초록 |
| Fetch — `out` 공구 | 명령 차단, DB `rejected` 기록 |
| Return — `staged` 공구 | 슬롯 반납 완료, DB `in_slot` 갱신 |
| FOD 타임아웃 | 10분 후 `fod_alert` 전이, PLC 주황 점멸 |
| BT 에러 복구 | 그리퍼 실패 → 복구 서브트리 실행 → 홈 복귀 |

### 수락 기준 (시뮬레이션)

- BT 골든 파일 회귀: **100% 일치**
- Gazebo 내 Staging Area 거치 오차: **±5mm 이내**
- 10종 시나리오 × 3 사이클: **전 통과** 후 HIL 진입

---

## 7. 시뮬레이션에서 실제 하드웨어로 (sim-to-real)

알려진 차이점:

| 항목 | 시뮬레이션 | 실제 하드웨어 |
|------|-----------|-------------|
| 관절 마찰 | 이상적 | 실제 마찰 있음 |
| 카메라 노이즈 | Gazebo 플러그인 기본 | 실제 조명·반사 영향 |
| 그리퍼 파지 | 강체 접촉 | 실제 탄성 변형 |
| 타이밍 | deterministic | 네트워크·OS 지연 있음 |

> sim-to-real 전략은 미결 #24 참조 (`docs/adr/ai-ml.md`).
