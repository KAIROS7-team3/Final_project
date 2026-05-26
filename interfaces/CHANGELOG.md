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
  - rationale: YOLOv8 detection score를 다운스트림에 전달하기 위함
  - migration: 신규 소비자는 기본값 0.0으로 처리 가능 (additive change)
```
