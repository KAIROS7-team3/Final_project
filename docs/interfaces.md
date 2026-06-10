# 인터페이스 계약

> **이 문서는 `interfaces/` 패키지의 공개 API 계약이다.**
> msg/srv/action 변경 시 이 문서와 `interfaces/CHANGELOG.md`를 함께 갱신해야 한다.
> 변경 전 `interface-guardian` 에이전트 검토 필수 (`docs/conventions.md` 검증 파이프라인 참조).

---

## 0. 명명 규약 (Quick Reference)

| 대상 | 형식 | 예시 |
|------|------|------|
| msg / action 파일명 | `PascalCase.msg` / `PascalCase.action` | `ToolStatus.msg`, `MoveToPose.action` |
| srv 파일명 | `PascalCase.srv` (verb-first) | `CheckToolFeasibility.srv` |
| 필드명 | `snake_case` | `tool_id`, `is_moving` |
| 상태/이벤트 enum 값 | `snake_case` | `in_slot`, `e_stop` |
| 토픽 경로 | `/group/snake_case` | `/voice/raw_text` |
| **상태 스냅샷** | suffix `Status` 사용 | `ToolStatus`, `PLCStatus`, `RobotStatus` |
| **이벤트** | suffix `Event` 사용 | `HandoverEvent` (v2.0+) |

> `State` suffix는 사용하지 않는다. ROS 표준이 `Status`·`State`를 혼용하지만, 본 프로젝트는 `Status`로 통일한다.

### tool_id 형식

```
정규식: ^[a-z][a-z0-9]*(_[a-z0-9]+)+$
형식:   <type>(_<spec>)+    — 최소 2개 토큰, 권장 3개 (type, detail, size)

예시:
  ✅ wrench_8mm                       (2-part: type + size)
  ✅ screwdriver_phillips_small       (3-part: type + detail + size)
  ✅ allen_key_4mm                    (3-part)
  ❌ screwdriver                      (단일 토큰 금지 — 모호함)
  ❌ Wrench8mm                        (camelCase 금지)
```

---

## 1. 메시지 (msg/)

### `ToolStatus.msg`

| 필드 | 타입 | 의미 |
|------|------|------|
| `tool_id` | `string` | tool_id 형식 (§0 참조) |
| `slot_row` | `int32` | 공구함 슬롯 행 (0-indexed) |
| `slot_col` | `int32` | 공구함 슬롯 열 (0-indexed) |
| `status` | `string` | `in_slot` \| `out` \| `staged` \| `missing` \| `fod_alert` |
| `timestamp` | `builtin_interfaces/Time` | DB 상태 변경 시각 (ROS time) |

### `PLCStatus.msg`

| 필드 | 타입 | 의미 |
|------|------|------|
| `led_color` | `string` | `green` \| `yellow` \| `red` \| `white` |
| `led_mode` | `string` | `solid` \| `pulse` \| `flash` |
| `system_state` | `string` | `idle` \| `listening` \| `inferring` \| `moving` \| `error` \| `e_stop` \| `watchdog` |

> 메시지 자체는 PLC의 현재 스냅샷(`Status`)이지만, 그 안의 `system_state` 필드는 상태머신의 멤버이므로 `State` 표현을 사용한다. 메시지명과 필드명의 차이는 의도된 것이다.

### `RobotStatus.msg`

| 필드 | 타입 | 의미 |
|------|------|------|
| `is_moving` | `bool` | 로봇 모션 진행 중 여부 — Whisper 오디오 게이팅(`S-7`)에 사용 |

### `Intent.msg`

| 필드 | 타입 | 의미 |
|------|------|------|
| `intent_type` | `string` | `fetch` \| `return` \| `cancel` \| `unknown` |
| `tool_id` | `string` | `fetch`/`return`일 때 대상 공구. 없으면 `""` |
| `confidence` | `float32` | Gemma 4 의도 분류 신뢰도, 0.0 ~ 1.0 |
| `raw_utterance` | `string` | 원본 발화 (디버깅·DB 로그·rosbag 분석용) |
| `timestamp` | `builtin_interfaces/Time` | 의도 분류 완료 시각 |

> **이전 설계:** `std_msgs/String`(JSON 직렬화)로 발행. 스키마 미강제·rosbag 분석 불가 문제로 폐기.

### `MarkerMap.msg`

| 필드 | 타입 | 의미 |
|------|------|------|
| `header` | `std_msgs/Header` | stamp: RGB 프레임 타임스탬프 / frame_id: `base_link`(캘리브 완료) 또는 `camera_color_optical_frame` |
| `marker_ids` | `int32[]` | 감지된 ArUco 마커 ID 목록 (DICT_4X4_50, ID 0·1·2 = 작업장 1·2·3) |
| `poses_robot` | `geometry_msgs/Pose[]` | marker_ids[i]의 3D 포즈 — position: base_link 기준 [m], orientation: quaternion [x,y,z,w] |
| `place_zone_radius` | `float32` | 마커 중심 기준 공구 place 허용 반경 [m] |
| `calibrated` | `bool` | hand-eye 캘리브레이션 적용 여부 — false 시 poses_robot은 카메라 좌표계 기준 |

**발행자:** `vision/marker_scan_node` → **구독자:** `orchestrator` BT ScanMarkers 노드
**QoS:** Reliable / depth 1 (트리거성 스캔, 최신값만 필요)

---

## 2. 서비스 (srv/)

### `CheckToolFeasibility.srv`

```
# Request
string intent       # 'fetch' | 'return'
string tool_id
---
# Response
bool feasible
string reason       # 차단 시 운영자 안내 문구 (예: "tool is missing", "tool is checked out")
```

**소유 패키지:** `db/` (서비스 서버), `voice/` `orchestrator/` (클라이언트)

### `UpdateToolStatus.srv`

```
# Request
string tool_id
string new_status   # 'in_slot' | 'out' | 'staged' | 'missing' | 'fod_alert'
string event_type   # 아래 enum 참조
string track        # 'A' | 'B' | 'C'
string notes
---
# Response
bool success
string message
```

**`event_type` 허용 값** (DB `tool_events.event_type` 컬럼과 일치):

| 값 | 의미 |
|----|------|
| `fetch` | 공구를 슬롯에서 꺼내 staging에 거치 완료 |
| `return` | 공구를 staging/외부에서 슬롯에 반납 완료 |
| `rejected` | DB Gate에서 차단된 명령 (S-2) |
| `error` | 모션/그리퍼/PLC 실패 (E-5) |
| `fod_alert` | FOD 임계 시간 초과 → 분실 알림 (S-8) |
| `reconciled` | 부팅 시 YOLOv11s 스캔으로 상태 동기화 (S-9) |

**소유 패키지:** `db/` (서비스 서버), `orchestrator/` (클라이언트)

---

## 3. 액션 (action/)

`UnitAction.action` 단일 액션을 폐기하고 동작별로 분리한다.
타입 안전성을 위해 각 액션이 자신에게 필요한 파라미터만 받는다.
모두 `orchestrator/unit_action_server.py`가 호스팅한다 (한 노드가 다중 액션 서버 등록).

### 공통 Result / Feedback 구조

```
# Result (모든 액션 공통)
bool success
string message      # 실패 사유 또는 성공 메시지
---
# Feedback (모든 액션 공통)
string phase        # 진행 단계 — 예: "moving_to_pregrasp", "closing_gripper"
float32 progress    # 0.0 ~ 1.0
```

### `MoveToPose.action`

```
# Goal
geometry_msgs/Pose target_pose    # base_link 기준
float32 velocity_scale            # 0.0 ~ 1.0 (default 0이면 config 기본값 사용)
```

### `Grasp.action`

```
# Goal
string tool_id
string approach_direction        # 'top' | 'side' | 'front'
float32 grasp_force              # N — 0이면 config/toolbox.yaml 의 tool별 default 사용
```

### `Release.action`

```
# Goal
# (파라미터 없음 — 현재 잡고 있는 공구를 놓는다)
```

### `PlaceAtStaging.action`

```
# Goal
string tool_id                    # 거치 대상
# 거치 좌표는 config/staging_area.yaml 에서 tool_id로 조회
```

### `PickFromStaging.action`

```
# Goal
string tool_id
```

### `ReturnToSlot.action`

```
# Goal
string tool_id
int32 slot_row
int32 slot_col
```

**소유 패키지:**
- 액션 서버: `orchestrator/unit_action_server.py` (Track A/B 전용)
- 클라이언트: `orchestrator/bt_nodes/*` (각 BT 노드가 1개 액션에 대응)

> Track C(VLA)는 이 액션을 사용하지 않는다. VLA 모델이 joint trajectory를 직접 출력하고 Doosan SDK로 실행한다.

---

## 4. 토픽 목록

| 토픽 이름 | 메시지 타입 | 발행자 | 구독자 | QoS |
|-----------|------------|--------|--------|-----|
| `/voice/raw_text` | `std_msgs/String` | `whisper_node` | `gemma_intent_node` | Reliable / depth 10 |
| `/voice/intent` | `interfaces/Intent` | `gemma_intent_node` | `orchestrator` | Reliable / depth 1 |
| `/vision/detections` | `vision_msgs/Detection2DArray` | `yolo_node` | `pose_node` | Best Effort / depth 10 |
| `/vision/tool_poses` | `vision_msgs/Detection3DArray` | `pose_node` | `tracker_node`, `orchestrator` | Best Effort / depth 5 |
| `/vision/tracked_poses` | `vision_msgs/Detection3DArray` | `tracker_node` | `context_builder` | Best Effort / depth 5 |
| `/vision/scene_context` | `std_msgs/String` (JSON) | `context_builder` | `voice/gemma_intent_node`, `orchestrator` (Phase 5a) | Reliable / depth 1 |
| `/robot/status` | `interfaces/RobotStatus` | `dsr_controller` 또는 `rl_policy_node` | `whisper_node` | Reliable / depth 1 |
| `/plc/status` | `interfaces/PLCStatus` | `plc_node` | (모니터링용) | Reliable + Transient Local / depth 1 |
| `/plc/e_stop` | `std_msgs/Bool` | `plc_node` | safety/orchestrator | Reliable + Transient Local / depth 1 |
| `/vision/marker/map` | `interfaces/MarkerMap` | `marker_scan_node` | `orchestrator` BT ScanMarkers | Reliable / depth 1 |
| `/vision/marker/debug/image` | `sensor_msgs/Image` | `marker_scan_node` | (디버그용) | Best Effort / depth 1 |
| `/vision/tool_top_pose` | `geometry_msgs/PointStamped` | `vision` (탑뷰 D455f) | `motion/toolbox_seq_runner` | Best Effort / depth 1 |
| `/vision/tool_gripper_pose` | `geometry_msgs/PointStamped` | `vision` (그리퍼 캠 C270) | `motion/toolbox_seq_runner` | Best Effort / depth 1 |
| `/vision/handle_pose` | `geometry_msgs/PointStamped` | `vision` (그리퍼 캠 C270) | `motion/toolbox_seq_runner` | Best Effort / depth 1 |
| `/vision/slot_top_pose` | `geometry_msgs/PointStamped` | `vision` (탑뷰 D455f) | `motion/toolbox_seq_runner` | Best Effort / depth 1 |

> **QoS 선택 기준:** 센서 데이터(비전, STT)는 Best Effort — 최신 프레임이 중요. 상태/의도 토픽은 Reliable — 손실 허용 불가.
> **비전 좌표 토픽 주의:** `/vision/tool_top_pose`, `/vision/tool_gripper_pose`, `/vision/handle_pose`, `/vision/slot_top_pose` 4종은 비전팀과 잠정 합의된 인터페이스. 단위 m, frame_id `base_link`. runner 내부에서 ×1000 → mm 변환 적용. 확정 전 변경 가능.

---

## 5. frame_id 규약

메시지에 `header.frame_id`가 포함된 경우:

| 토픽 | frame_id | 의미 |
|------|----------|------|
| `/vision/detections` | `camera_optical_frame` | 2D 이미지 좌표 기준 |
| `/vision/tool_poses` | `base_link` | 로봇 베이스 기준 변환 후 발행 (hand-eye 미캘리브 시 `camera_optical_frame` — Phase 1 한정) |

좌표계 상세 → [`frames.md`](frames.md)

---

## 6. timestamp 정책

- 모든 `header.stamp`은 **ROS 시간** 사용 (`node.get_clock().now()`)
- 시뮬레이션 시 `use_sim_time: true` 설정 필수
- `ToolStatus.timestamp`, `Intent.timestamp`는 ROS Time
- DB에 저장할 때는 Python `datetime`로 변환 (Unix epoch 기준)
- 센서 타임스탬프 오프셋 보정은 드라이버 노드 책임

---

## 7. 버전 관리

interfaces 변경 시:
1. `interfaces/CHANGELOG.md` 갱신 (Keep a Changelog 형식)
2. 이 문서 해당 섹션 갱신
3. `interface-guardian` 에이전트 검토 후 머지

---

## 8. 예정 인터페이스 (v2.0+)

아래 인터페이스는 v1.0에서 구현하지 않는다. 신규 코드가 이를 import하면 `interface-guardian`이 차단한다.

### `HandoverEvent.msg` — v2.0+

로봇이 사람 손에 직접 공구를 전달하는 동작 (S-6에 따라 v1.0 금지). v2.0 설계 시 필드 확정.
