# interfaces CHANGELOG

> `interfaces/` 패키지(ROS2 msg/srv/action)의 변경 이력.
> 모든 변경은 `interface-guardian` 에이전트 검토 후 머지 (`.claude/rules/process.md` P-2, P-6).
> 형식: [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/) 기반.

## [0.1.0] — 2026-05-27

> Phase 0 초기 릴리스. 코드 구현 전 설계 단계에서 확정된 모든 인터페이스를 ROS2 패키지로 동결.

### Added
- `msg/ToolStatus.msg` — 공구 상태 스냅샷 (tool_id, slot_row/col, status, timestamp)
- `msg/PLCStatus.msg` — PLC 상태 스냅샷 (led_color, led_mode, system_state)
- `msg/RobotStatus.msg` — 로봇 모션 상태 (is_moving — Whisper 오디오 게이팅 S-7)
- `msg/Intent.msg` — Gemma 4 의도 분류 결과 (intent_type, tool_id, confidence, raw_utterance, timestamp)
- `srv/CheckToolFeasibility.srv` — DB Gate fetch/return 가용성 확인
- `srv/UpdateToolStatus.srv` — 공구 상태 + 이벤트 로그 갱신
- `action/MoveToPose.action`, `Grasp.action`, `Release.action`, `PlaceAtStaging.action`, `PickFromStaging.action`, `ReturnToSlot.action` — 동작별 분리 액션 (UnitAction.action 단일 액션 폐기 대체)

### Deprecated
- `msg/HandoverEvent.msg` — v2.0+ 전용 (S-6: v1.0 직접 핸드오버 금지)

### Removed
- `action/UnitAction.action` — 6개 typed action으로 대체

---

## [Unreleased]

### Added
- 신규 토픽 4종 — vision_fetch / vision_open / vision_close 시퀀스용 비전 좌표 입력 (Track B Phase 1, `feat/track-b-vision-sequences`):
  - `/vision/tool_top_pose` (`geometry_msgs/PointStamped`): 탑뷰 D455f가 제공하는 공구 중심 XY 좌표. 발행자: `vision` 패키지. 구독자: `motion/toolbox_seq_runner`. QoS: Best Effort / depth 1.
    - 단위: m (runner 내부에서 ×1000 → mm 변환). frame_id: `base_link`.
    - rationale: vision_fetch 시퀀스 step ③ 공구 위 이동(MOVE_L_TOP_XY)에 탑뷰 rough XY 좌표 공급.
    - migration: 신규 토픽. 비전팀이 발행 미구현 시 해당 step에서 "좌표 미수신" 오류로 시퀀스 실패.
  - `/vision/tool_gripper_pose` (`geometry_msgs/PoseStamped`): 그리퍼 캠 C270이 제공하는 공구 중심 XYZ + 방향각(rz). 발행자: `vision` 패키지. 구독자: `motion/toolbox_seq_runner`. QoS: Best Effort / depth 10.
    - 단위: m. frame_id: `base_link`.
    - orientation: quaternion (yaw-only, PCA 주축 방향각). `pca_theta - 90°` 변환 후 robot rz로 사용.
    - rationale: PCA theta 전달 필요로 PointStamped → PoseStamped 변경. fetch/return 분리 토픽을 단일 토픽으로 통합.
    - migration: 기존 `PointStamped` 구독자는 `PoseStamped`로 타입 교체 필요. rz는 `pose.orientation` quaternion에서 추출.
  - `/vision/handle_pose` (`geometry_msgs/PointStamped`): 그리퍼 캠 C270이 제공하는 서랍 손잡이 중심 XZ 좌표. 발행자: `vision` 패키지. 구독자: `motion/toolbox_seq_runner`. QoS: Best Effort / depth 1.
    - 단위: m. frame_id: `base_link`. (Y는 HandleServoController에서 미사용 — XZ만 보정)
    - rationale: vision_open/close step ④⑤ HandleServoController XZ VS 정렬 좌표 공급.
    - migration: 신규 토픽.
  - `/vision/slot_top_pose` (`geometry_msgs/PointStamped`): 탑뷰 D455f가 제공하는 슬롯 XY 좌표. 발행자: `vision` 패키지. 구독자: `motion/toolbox_seq_runner`. QoS: Best Effort / depth 1.
    - 단위: m. frame_id: `base_link`.
    - rationale: vision_return 시퀀스 슬롯 복귀 정렬(MOVE_L_SLOT_XY) 좌표 공급.
    - migration: 신규 토픽.
  - `/vision/masks/gripper` (`sensor_msgs/Image`, encoding: mono8): 그리퍼 캠 YOLO 검출 중 최고 신뢰도 검출의 이진 마스크. 발행자: `vision/yolo_node` (camera_type=gripper). 구독자: `vision/gripper_marker_scan_node`. QoS: Best Effort / depth 10.
    - 픽셀값: 255 = 공구 마스크, 0 = 배경. 해상도는 원본 C270 이미지와 동일.
    - rationale: `Detection2DArray`는 마스크 픽셀을 미포함. 마스크 별도 토픽으로 PCA theta 계산 정확도 향상 (Canny ROI 근사 대비).
    - migration: 신규 토픽. seg 모델 사용 시에만 발행 (detection 모델이면 미발행 → marker_scan_node가 Canny ROI 폴백으로 자동 전환).
  > **비전팀 확인 필요**: 토픽명·메시지 타입·단위·frame_id는 비전팀과 합의 전 잠정 정의. 확정 후 이 항목 갱신 필수.
- `srv/GripperSetPosition.srv`: RH-P12-RN 그리퍼 위치 제어 서비스 추가 (Track B Phase 1, PR #35).
  - 필드: request `position`(pulse), `current`(mA), `timeout_sec` / response `success`, `message`, `final_position`, `final_current`.
  - `gripper_node`가 `/gripper/set_position`으로 호스팅. Doosan 컨트롤러 TCP(port 9105) 경유 Modbus RTU로 RH-P12-RN 제어.
  - rationale: 그리퍼 제어를 Track A/B 공용 ROS2 서비스로 노출 — toolbox 시퀀스(`unit_actions/toolbox_motion.py`)의 GRIP step이 이 서비스로 실행됨.
  - 단위 주의: `position`/`current`는 DSR 네이티브 pulse/mA 단위 (E-1의 m/rad과 무관한 하드웨어 원시 단위).
  - migration: 신규 서비스라 마이그레이션 불필요.
- `msg/Intent.msg`: Gemma 4 의도 분류 결과를 구조화된 메시지로 발행. 필드: `intent_type`, `tool_id`, `confidence`, `raw_utterance`, `timestamp`.
  - rationale: 이전 `std_msgs/String`(JSON) 방식은 스키마 미강제·rosbag 분석 불가·`interface-guardian` 추적 불가.
  - migration: 코드 구현 전 단계라 마이그레이션 불필요.
- `action/MoveToPose.action`, `Grasp.action`, `Release.action`, `PlaceAtStaging.action`, `PickFromStaging.action`, `ReturnToSlot.action`: 단일 `UnitAction.action`을 동작별로 분리. 각 액션이 자신에게 필요한 파라미터만 받음.
  - rationale: 단일 액션은 `string action_name` 디스크리미네이터로 6개 동작을 처리했으나 액션별 파라미터(목표 pose, 슬롯 좌표 등)를 받을 필드가 없어 런타임 검증 부담·타입 안전성 상실.
  - migration: 코드 구현 전 단계라 마이그레이션 불필요. 모두 `orchestrator/unit_action_server.py`가 다중 액션 서버로 호스팅.
- `tool_id` 정규식 명시 (`docs/interfaces.md §0`): `^[a-z][a-z0-9]*(_[a-z0-9]+)+$`, 최소 2-part 권장 3-part.
  - rationale: 이전엔 형식 설명이 모호해 `wrench_8mm`(2-part)·`screwdriver_phillips_small`(3-part) 혼재.
- `srv/UpdateToolStatus.srv`의 `event_type` enum 값 명시: `fetch`, `return`, `rejected`, `error`, `fod_alert`, `reconciled`.
  - rationale: 이전엔 "tool_events 테이블 참조"라고만 표기되어 인터페이스 문서에서 허용 값 확인 불가.
- `PlaceAtStaging.action`/`ReturnToSlot.action` 공통 Feedback `phase` 필드에 `"pick"`/`"place"` 마커 의미 추가 (`docs/interfaces.md §3` 갱신).
  - rationale: DB 상태(`in_slot<->out<->staged`)를 BT 완료 대기 없이 물리적 집기/놓기 시점에 즉시 전이시키기 위해, `unit_actions.toolbox_motion.Step.marker`가 설정된 step 실행 직후 `phase="pick"`/`"place"`를 추가 발행한다. 기존 `StepKind` 이름(대문자) 기반 진행 단계 phase와 공존한다.
  - migration: 액션 필드 타입 변경 없음 (additive). 기존 소비자는 무시해도 무방. `orchestrator_node`만 신규 phase 값을 구독해 DB 상태 전이를 트리거.

### Changed
- `msg/PLCState.msg` → `msg/PLCStatus.msg` 리네이밍.
  - rationale: 프로젝트 전반 상태 스냅샷 메시지는 `Status` suffix로 통일 (`ToolStatus`, `RobotStatus`와 일관).
  - migration: 코드 구현 전 단계라 마이그레이션 불필요.
- 토픽 `/plc/state` → `/plc/status` (메시지 리네이밍에 따름).
- `PLCStatus.system_state` enum의 `estop` → `e_stop` (snake_case 통일).
  - rationale: 다른 enum 값(`in_slot`, `fod_alert` 등)이 모두 snake_case.

### Removed
- `action/UnitAction.action` 폐기 (6개 typed action으로 대체, Added 항목 참조).
- `/voice/intent` 토픽의 `std_msgs/String`(JSON 직렬화) 형식 폐기 (`interfaces/Intent`로 대체).

### Deprecated
- `msg/HandoverEvent.msg`: v2.0+ 전용으로 표시 명확화 (`docs/interfaces.md §8`로 분리). v1.0 API 표면에서 제외.
  - rationale: S-6에 따라 v1.0에서 직접 핸드오버 금지. v1.0 API 표에 노출되어 신규 팀원 혼란.

### Fixed
- (해당 없음)

### Security
- (해당 없음)

> 위 변경은 코드 구현 전 설계 단계에서 결정됨. 실제 `interfaces/` 패키지 빌드 시점에 v0.1.0으로 릴리스.

---

## 작성 가이드

각 항목은 다음 형식을 따른다:

```markdown
- `<인터페이스 이름>`: <변경 요약> (#<PR 번호>)
  - rationale: <변경 이유>
  - migration: <기존 소비자가 해야 할 작업, 있으면>
```

예시:
```markdown
- `msg/ToolStatus.msg`: `confidence` 필드 추가 (#42)
  - rationale: YOLOv11s detection score를 다운스트림에 전달하기 위함
  - migration: 신규 소비자는 기본값 0.0으로 처리 가능 (additive change)
```
