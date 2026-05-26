# 시스템 아키텍처

> 이 문서는 구현 중 에이전트가 참조하는 설계 기준서다.
> 설계 결정·미결 사항 → [`adr/index.md`](adr/index.md) | 일정·Phase → [`../robot-arm-project.md`](../robot-arm-project.md)

---

## 1. 시스템 개요

음성 명령으로 공구함에서 공구를 꺼내 **Staging Area**에 거치하고, 반납 명령 시 슬롯에 되돌려놓는 시스템. 모든 이벤트는 DB에 기록(FOD 관리), PLC LED로 실시간 상태 표시.

**핵심 태스크 흐름:**
```
음성 → STT → 의도 분석 + DB 가용성 확인
     → 공구 위치 인식
     → 파지 → Staging Area 거치
     → DB 이벤트 기록 + PLC LED 갱신
```

**v1.0 범위:** Staging Area 간접 전달만 구현. 직접 핸드오버(비전 감지 포함)는 v2.0+.

### 트랙 구조

| 트랙 | 의도 이해 | 결정 레이어 | 모션 레이어 | ROS2 |
|------|-----------|-------------|-------------|------|
| **A — DSR** | Gemma 4 + DB check | Behavior Tree | DSR 좌표 제어 | 전체 7패키지 |
| **B — RL** | Gemma 4 + DB check | Behavior Tree | RL 정책 | 전체 7패키지 |
| **C — VLA** | Python 키워드 파서 + DB gate | VLA 모델 end-to-end | Doosan Python SDK 직접 | 없음 |

트랙은 한 번에 하나씩 실행 (VRAM 제약). `./run.sh --track [A|B|C]`로 전환.

### 공구 목록

9종 (드라이버 3 · 렌치 3 · 플라이어 3). `config/toolbox.yaml`로 관리 (슬롯 좌표 + 공구 기하 동일 파일).

---

## 2. 하드웨어

상세 인벤토리·드라이버 버전·스펙·udev 규칙 → [`hardware.md`](hardware.md)

---

## 3. 공유 vs 트랙별 분리

```
╔══════════════════════════════════════════════════════════════════╗
║            SHARED — 물리 하드웨어 + 코드 라이브러리                ║
║  [Hardware]  Doosan e0509 · RH-P12-RN · D455f · PLC · DB       ║
║  [Code lib]  db_core/ · plc_core/ (순수 Python, ROS2 없음)        ║
║              hal/ · unit_actions/ — Track A/B 전용                ║
╠══════════════════════════════════╦═══════════════════════════════╣
║        TRACK A / B               ║        TRACK C                ║
║        ROS2 Humble 기반           ║        독립 Python 프로세스    ║
╠══════════════════════════════════╬═══════════════════════════════╣
║  realsense-ros (ROS2 노드)        ║  pyrealsense2 직접 접근        ║
║  doosan-robot2 (ROS2 노드)        ║  Doosan Python SDK 직접 제어  ║
║  Whisper STT node (ROS2)          ║  Whisper 직접 호출             ║
║  YOLOv8 · 6D Pose · Tracker       ║  Raw RGB-D + STT text         ║
║  Gemma 4 + DB check               ║   → VLA 모델 추론             ║
║  Behavior Tree (py_trees)         ║   → joint + gripper commands  ║
║  unit_actions/ (구조화된 액션)     ║   → Safety Validator          ║
║  unit_action_server (ROS2 래퍼)   ║   → Doosan Python SDK 실행    ║
║  DB node (ROS2 서비스)             ║  db_core/ (직접 SQL)          ║
║  PLC node (ROS2 노드)             ║  plc_core/ (직접 Modbus)      ║
╚══════════════════════════════════╩═══════════════════════════════╝
```

---

## 4. ROS2 패키지 구조 (Track A/B)

```
프로젝트 루트/
├── hal/                          순수 Python HAL (Track A/B 전용, rclpy 금지)
│   ├── arm_interface.py          ArmInterface 추상 클래스
│   ├── gripper_interface.py      GripperInterface
│   └── simulated/                SimulatedArm/Gripper 구현
│
├── unit_actions/                 순수 Python (Track A/B 전용, rclpy 금지)
│   └── ...                       hal/을 통해 하드웨어 추상 호출
│
├── db_core/, plc_core/           순수 Python (전 트랙 공유)
│
├── track_c_vla.py                Track C 단일 엔트리포인트
│
└── ros2_ws/src/                  Track A/B ROS2 워크스페이스
    ├── interfaces/               커스텀 msg/srv/action (모든 ROS2 패키지가 의존)
    ├── voice/                    Whisper STT + Gemma 4 의도 분류
    ├── vision/                   YOLOv8 + 6D Pose + Tracker + Context Builder
    ├── orchestrator/             Behavior Tree + unit_action_server (ROS2 래퍼, hal/+unit_actions/ 사용)
    ├── db/                       db_core/를 ROS2 서비스로 노출
    ├── motion/                   DSR/RL 제어 + Grasp Planner
    └── plc/                      plc_core/를 ROS2 토픽으로 노출
```

> `hal/`, `unit_actions/`, `db_core/`, `plc_core/`는 ROS2 워크스페이스 **밖** 프로젝트 루트에 위치한다. ROS2 패키지가 이들을 PYTHONPATH로 import한다. 이는 Track C가 ROS2 빌드 없이도 동일 모듈을 사용하기 위함이다.

### hal/ — Hardware Abstraction Layer (Track A/B 전용)

`unit_actions/`가 실제 드라이버(Doosan SDK·RH-P12-RN·RealSense)와 시뮬레이션을 동일 인터페이스로 호출할 수 있게 해주는 얇은 추상화 레이어. 순수 Python으로 작성하며 `rclpy` import 금지 (E-2). Track C는 `hal/`을 사용하지 않고 Doosan Python SDK·`pyrealsense2`를 직접 호출한다.

**설계 의도:**
- **드라이버 교체 격리**: doosan-robot2 버전 변경이나 그리퍼 교체가 `unit_actions/`·BT 코드까지 전파되지 않도록 차단
- **시뮬레이션 동등성**: `SimulatedArm` 등이 실 드라이버와 동일 시그니처를 구현하므로 BT 골든 파일 회귀 테스트가 하드웨어 없이 실행됨 (→ [`simulation.md`](simulation.md))
- **단위 테스트 용이성**: `unit_actions/` 테스트가 mock HAL을 주입해 actuator 호출 없이 검증 가능

| 파일 | 역할 |
|------|------|
| `arm_interface.py` | `ArmInterface` 추상 클래스 — `move_to_joint()`, `move_to_pose()`, `get_joint_state()`, `emergency_stop()` 시그니처 정의 (단위: rad, m — E-1) |
| `gripper_interface.py` | `GripperInterface` 추상 클래스 — `open()`, `close(force_n)`, `get_state()`. RH-P12-RN 최대 170N |
| `camera_interface.py` | `CameraInterface` 추상 클래스 — `get_rgbd_frame()`, `get_intrinsics()`. 컬러+depth+TF |
| `doosan/arm_driver.py` | `DoosanArmDriver` — doosan-robot2 ROS2 액션을 호출하는 실 구현 |
| `doosan/gripper_driver.py` | `RHP12RNDriver` — RH-P12-RN 시리얼/CAN 제어 |
| `realsense/camera_driver.py` | `RealSenseDriver` — realsense-ros 토픽 구독 + 동기화 |
| `simulated/simulated_arm.py` | `SimulatedArm` — 시뮬 모드용. 명세 → `mocks/SPEC.md` |
| `simulated/simulated_gripper.py` | `SimulatedGripper` — open/close 지연·실패율 주입 가능 |
| `simulated/simulated_camera.py` | `SimulatedCamera` — Gazebo RGB-D 토픽 또는 녹화 bag 재생 |

> HAL 시그니처는 Phase 0 ②에서 동결한다. 변경 시 `interface-guardian` 검토 필수 (수락 기준: BT 골든 회귀 통과). 실 드라이버와 `Simulated*` 구현은 항상 같은 추상 클래스를 상속해야 하며, CI에서 ABC 미구현 메서드를 자동 검출한다.

### interfaces/ — 전체 msg/srv/action

| 파일 | 내용 |
|------|------|
| `msg/ToolStatus.msg` | tool_id, slot, status, timestamp |
| `msg/PLCStatus.msg` | led_color, led_mode, system_state |
| `msg/RobotStatus.msg` | is_moving: bool (오디오 게이팅용) |
| `msg/Intent.msg` | intent_type, tool_id, confidence, raw_utterance, timestamp |
| `msg/HandoverEvent.msg` | v2.0+ (S-6 — v1.0 미구현) |
| `srv/CheckToolFeasibility.srv` | intent+tool_id → feasible+reason |
| `srv/UpdateToolStatus.srv` | DB 상태 갱신 |
| `action/MoveToPose.action` | target_pose 이동 |
| `action/Grasp.action` | tool_id + approach_direction + force |
| `action/Release.action` | 현재 잡고 있는 공구 놓기 |
| `action/PlaceAtStaging.action` | tool_id → staging 좌표 거치 |
| `action/PickFromStaging.action` | tool_id staging에서 픽업 |
| `action/ReturnToSlot.action` | tool_id + slot_row/col 반납 |

> 액션은 동작별로 분리되어 있다 (이전 단일 `UnitAction.action` 폐기). 모두 `orchestrator/unit_action_server.py`가 다중 액션 서버로 호스팅. 상세 → [`interfaces.md`](interfaces.md) §3.

### voice/

| 파일 | 역할 |
|------|------|
| `whisper_node.py` | 오디오 → `/voice/raw_text` |
| `gemma_intent_node.py` | raw_text + DB snapshot → `/voice/intent` |
| `wake_word_detector.py` | 웨이크워드 감지 (false positive 방지) |

### vision/

| 파일 | 역할 |
|------|------|
| `yolo_node.py` | RGB → `/vision/detections` |
| `pose_node.py` | detections + depth → `/vision/tool_poses` |
| `tracker_node.py` | 멀티 오브젝트 트래킹 |
| `context_builder.py` | Track A/B용 scene JSON (Track C 미사용) |

### orchestrator/

| 파일 | 역할 |
|------|------|
| `behavior_manager.py` | BT 틱 루프 |
| `bt_nodes/fetch_tool.py` | FetchTool 서브트리 |
| `bt_nodes/return_tool.py` | ReturnTool 서브트리 |
| `bt_nodes/recovery.py` | 에러 복구 서브트리 |
| `unit_action_server.py` | ROS2 action server → unit_actions/ 래핑 |

### motion/

| 파일 | 역할 |
|------|------|
| `dsr_controller.py` | Track A: DSR 좌표 제어 |
| `rl_policy_node.py` | Track B: RL 정책 추론 |
| `grasp_planner.py` | 공구 포즈 → 파지 후보 |

---

## 5. 공유 코어 라이브러리

```
db_core/               ← 순수 Python (rclpy import 금지)
├── client.py          DBClient: get_tool_status(), log_event() 등
└── schema.py          테이블 정의, 쿼리 상수

plc_core/              ← 순수 Python (rclpy import 금지)
├── client.py          PLCClient: set_led_state() 등
└── states.py          LEDState 열거형

ros2_ws/src/db/        ← ROS2 래퍼 (Track A/B)
└── db_node.py         db_core → ROS2 서비스로 노출

ros2_ws/src/plc/       ← ROS2 래퍼 (Track A/B)
└── plc_node.py        plc_core → ROS2 토픽으로 노출

track_c_vla.py         ← db_core, plc_core 직접 import
```

> `db_core/`, `plc_core/`, `unit_actions/`, `track_c_vla.py`에 `rclpy` import 절대 금지.

---

## 6. Unit Action Library (Track A/B 전용)

```python
class UnitActions:  # unit_actions/ 순수 Python
    def move_to(self, target: str | Pose, speed: float = 1.0) -> ActionResult: ...
    def grasp(self, tool_id: str) -> ActionResult: ...
    def release(self) -> ActionResult: ...
    def place_at_staging(self, tool_id: str) -> ActionResult: ...   # config/staging_area.yaml 기준
    def pick_from_staging(self, tool_id: str) -> ActionResult: ...
    def return_to_slot(self, tool_id: str) -> ActionResult: ...
    def check_tool_presence(self, slot: list[int]) -> bool: ...
    def emergency_stop(self) -> None: ...
    # v2.0+: handover_wait()
```

Track C는 unit_actions 미사용 — VLA 모델이 joint commands 직접 출력.

---

## 7. Track A/B — Behavior Tree 구조

```
→ Sequence: Root
   ├── Condition: VoiceCommandReceived (feasible=true)
   └── → Fallback: DispatchByIntent
        ├── → Sequence: FetchTool
        │    ├── Condition: IsFetchIntent
        │    ├── Action: LocalizeTool       ← YOLOv8 + 6D Pose
        │    ├── Action: PlanGrasp
        │    ├── Action: MoveToPreGrasp
        │    ├── Action: CloseGripper
        │    ├── Action: PlaceAtStagingArea
        │    ├── Action: OpenGripper
        │    └── Action: MoveToHome
        └── → Sequence: ReturnTool
             ├── Condition: IsReturnIntent
             ├── Action: PickFromStagingArea
             ├── Action: MoveToToolSlot
             ├── Action: OpenGripper
             └── Action: MoveToHome
```

Blackboard 스키마: `{intent, active_tool_id, tool_pose, staging_state}`

---

## 8. Track C — VLA 파이프라인

```
Microphone → Whisper (직접) → STT text
RealSense  → pyrealsense2  → RGB-D frame
Robot state → Doosan SDK   → joint state
     │
     ▼ check_feasibility() — db_core/ 직접 쿼리
     │  FAIL → 동작 거부 + DB 로그 + PLC 경고
     │  PASS ↓
     ▼ VLA 모델 추론 (RGB-D + STT + robot_state)
     │  → joint trajectory + gripper command
     │
     ▼ SafetyValidator.check(trajectory)
     │  FAIL → 동작 거부 + DB 로그 + PLC 빨간 점멸
     │
     ▼ Doosan Python SDK 직접 실행
       arm.execute(joint_trajectory)
       gripper.set(gripper_command)   # 0~1 연속값
       DB 이벤트 기록 + PLC 상태 갱신
```

**is_moving 게이팅:** `is_moving == True`이면 Whisper 추론 생략. 홈 복귀 후 재개.

---

## 9. DB 스키마

> 상세 컬럼 정의·제약·인덱스·트리거는 [`db-schema.md`](db-schema.md)가 단일 진실. 여기서는 트랙 무관 동작 규칙만 정리.

핵심 테이블:

| 테이블 | 역할 |
|--------|------|
| `tools` | 공구 카탈로그 + 현재 상태 (`current_status`, `home_slot_row/col`) |
| `tool_events` | 모든 이벤트의 append-only 로그 (`fetch`, `placed_at_staging`, `picked_from_staging`, `return`, `missing`, `fod_alert`, `rejected`, `error`) |
| `operators` | 운영자 카탈로그 (v1.0: 단일 row `operator_01`) |
| `system_events` | 부팅·reconciliation·E-stop 등 시스템 이벤트 |

### DB 기반 명령 차단 (전 트랙 공통)

| 명령 | 허용 상태 | 차단 상태 |
|------|-----------|-----------|
| Fetch | `in_slot` | `out` / `staged` / `missing` / `fod_alert` |
| Return | `staged` | `in_slot` / `out` / `missing` / `fod_alert` |

### FOD 상태 전이

```
in_slot ──[fetch]──→ out ──[place_at_staging]──→ staged
                      │                              │
               [10분 초과]                    [return 또는
                      │                       운영자 회수]
                      ▼                              ▼
                   missing ──(≤30초)──→ fod_alert   in_slot
```

임계 시간 기본값 10분, `config/fod.yaml`로 조정.

### DB 장애 폴백

- 연결 실패 → 마지막 캐시 사용 (TTL 5분)
- TTL 초과 → 모든 명령 거부
- 진행 중 작업은 완료까지 계속, 완료 후 retry queue

---

## 10. PLC LED 상태 매핑

| 시스템 상태 | 색상 | 모드 |
|-------------|------|------|
| 대기/준비 | 초록 | Solid |
| STT 수음 중 | 파랑 | Pulse |
| LLM/VLA 추론 중 | 노랑 | Pulse |
| 로봇 이동 중 | 파랑 | 빠른 Pulse |
| Staging Area 거치 중 | 청록 | Solid |
| 반납 진행 중 | 보라 | Pulse |
| 공구 분실/FOD | 주황 | Flash |
| 오류 | 빨강 | Flash |
| E-Stop 활성 | 빨강 | Solid |

---

## 11. 안전 아키텍처

### 안전 계층

| 레벨 | 메커니즘 | 구현 |
|------|---------|------|
| Level 0 | 하드웨어 E-Stop | Doosan 티치 펜던트 물리 버튼 |
| Level 1 | 내장 충돌 감지 | Doosan e0509 코봇 안전 컨트롤러 |
| Level 2 | 소프트웨어 워치독 | SafetyWatchdog — 하트비트 타임아웃 500ms |
| Level 3 | 작업공간 제한 | DSR 관절/Cartesian 소프트 리밋 |
| Level 4 | PLC 시각 피드백 | 운영자가 LED로 상태 파악 |

### Track C 추가 안전 게이트

```
VLA 출력 → SafetyValidator.check()
  ✓ joint limit · 속도 한계 · Cartesian 경계 · self-collision
  PASS → Doosan Python SDK 실행
  FAIL → 동작 거부 + DB 로그 + PLC 빨간 점멸
```

**SafetyValidator는 절대 우회 불가 (.claude/rules/safety.md S-1)**

---

## 12. 퍼셉션 파이프라인

### Track A/B
```
D455f RGB   → YOLOv8 → 공구 ID + 2D bbox
D455f Depth → 포인트 클라우드 → ICP/FoundationPose → 6D 포즈
```

### Track C
```
D455f RGB-D + STT text + robot_state → VLA 모델 → joint trajectory
```

VLA가 비전 이해·의도 파악·동작 계획 전부 end-to-end 처리.

### 타이밍 버짓

| 단계 | Track A/B | Track C |
|------|-----------|---------|
| Whisper STT | < 500ms | < 500ms |
| Gemma 4 + DB 확인 | < 800ms | — (Python gate ~수ms) |
| YOLOv8 + 포즈 추정 | < 150ms | 해당 없음 |
| VLA 추론 | — | 모델 선정 후 확정 |
| **음성 → 모션 시작** | **~1.5초** | **모델 선정 후 확정** |

---

## 13. 트랙 비교

| 지표 | Track A | Track B | Track C |
|------|---------|---------|---------|
| 의도 이해 | Gemma 4 + DB check | Gemma 4 + DB check | 키워드 파서 + DB gate |
| 결정 레이어 | Behavior Tree | Behavior Tree | VLA end-to-end |
| 모션 레이어 | DSR 좌표 제어 | RL 정책 | Doosan Python SDK |
| ROS2 의존성 | 전체 | 전체 | 없음 |
| unit_actions | O | O | X |
| 모호한 명령 | Gemma4 확인 요청 | Gemma4 확인 요청 | 키워드 파서 제한 |
| 새 공구 추가 | YAML + YOLOv8 재학습 | YAML + YOLOv8 재학습 | demo 추가 + VLA 재학습 |
| GPU VRAM | ~5.5–7.5GB | ~6.5–8.5GB | ~5–6GB(Q4) |
| 생산 적합성 | 높음 | 중간 | 연구용 |
| 사이클 타임 목표 | ≤ 10초 | ≤ 10초 | ≤ 13초 |

---

## 관련 문서

- 설계 결정·미결 사항 → [`adr/index.md`](adr/index.md)
- 수락 기준·컨벤션 → [`conventions.md`](conventions.md)
- 개발 일정·Phase → [`../robot-arm-project.md`](../robot-arm-project.md)
- 안전 규칙 → [`../.claude/rules/safety.md`](../.claude/rules/safety.md)
- 엔지니어링 규칙 → [`../.claude/rules/engineering.md`](../.claude/rules/engineering.md)
