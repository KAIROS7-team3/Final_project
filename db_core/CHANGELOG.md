# Changelog — db_core

Keep a Changelog 형식. `ToolRepository`/`DBClient` API 변경, 스키마 마이그레이션 시 갱신 (P-2).

## [Unreleased]

### Changed
- **(Breaking, safety) `ToolRepository.update_tool_status()`에 상태 전이 화이트리스트 추가 (B2-4, S-8/S-9).** `new_status` 값 검증만으로는 막지 못하던 불법 전이(예: `missing`/`fod_alert` → `in_slot` 무인 자동 수정)를 차단한다.
  - 허용 외부 전이(Track A/B/C): `in_slot→out`(fetch, pick), `out→staged`(fetch, place), `staged→in_slot`(return). v1.0 2단계 fetch 모델(S-6 Staging 경유).
  - `reconciled`: 임의 전이 허용하되 운영자 확인 `notes` 필수 (S-9).
  - `error`: 상태 미변경 기록만 허용 (E-5).
  - `missing`/`fod_alert` 진입은 FOD monitor 전용(`mark_checkout_timeouts`) — 외부 호출 금지 (S-8).
  - 기존에 통과하던 `in_slot→staged`(직접) 등 일부 전이가 이제 거부된다.
  - `UpdateToolStatus.srv` 필드 계약은 불변. 동작(거부 범위)만 변경.

### Notes
- ⚠️ **팀 합의 + interface-guardian/safety-reviewer 정식 검토 대기** 후 머지. 사전 검토에서 접근 승인됨(전이표는 팀 선택: 2단계 모델).
- ⚠️ **Track C 우회 잔존**: `DBClient.log_event()`는 본 화이트리스트를 거치지 않는다. DB Gate 이중 구현 단일화(B2-1) 시 함께 공유 레이어로 끌어올려야 완전히 닫힌다.
- 알려진 한계(범위 외): `intent_status_simulator_node`는 fetch를 `in_slot→out`까지만 수행해 `out→staged` place 단계를 생략하므로, 시뮬레이터 단독 fetch→return 라운드트립은 여전히 DB Gate에서 거부된다(테스트 전용 경로).
