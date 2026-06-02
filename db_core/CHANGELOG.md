# Changelog — db_core

Keep a Changelog 형식. `ToolRepository`/`DBClient` API 변경, 스키마 마이그레이션 시 갱신 (P-2).

## [Unreleased]

### Added
- `ToolRepository.__init__`에 `busy_timeout_ms` 파라미터 추가 (기본 5000) — 다중 프로세스(`db_service_node`+`fod_monitor_node`)의 WAL 락 경합 시 `SQLITE_BUSY` 즉시 실패 방지. 운영 값은 `config/runtime.yaml` (`db.busy_timeout_ms`).

### Changed
- `ToolRepository._connect()`가 연결 시 `SCHEMA_SQL`로 스키마를 부트스트랩한다. 신규/비어 있는 DB 파일에 대해 Track A/B 노드 단독 기동 시 `SchemaValidationError`로 DB Gate가 영구 차단되던 문제 해결. `CREATE TABLE IF NOT EXISTS`라 기존 DB에 멱등이며, 컬럼·FK 누락 검증(`_validate_schema`)은 그대로 유지. `DBClient.connect()`와 동일한 `SCHEMA_SQL`을 공유해 스키마 drift 방지.
- **(Breaking, safety) `ToolRepository.update_tool_status()`에 상태 전이 화이트리스트 추가 (B2-4, S-8/S-9).** `new_status` 값 검증만으로는 막지 못하던 불법 전이(예: `missing`/`fod_alert` → `in_slot` 무인 자동 수정)를 차단한다.
  - 허용 외부 전이(Track A/B/C) — fetch/return **대칭 2단계** pick→place 모델(S-6 Staging 경유):
    - fetch: `in_slot→out`(pick), `out→staged`(place)
    - return: `staged→out`(pick), `out→in_slot`(place)
    - `out`에서의 목적지로 방향 구분(`staged`=fetch place / `in_slot`=return place).
  - `reconciled`: 임의 전이 허용하되 운영자 확인 `notes` 필수 (S-9 boot reconciliation 정정 경로). 현재 production 호출처는 없으나, 화이트리스트가 `missing→in_slot` 같은 운영자 확인 정정을 차단하지 않도록 하는 필수 escape hatch.
  - `error`: 상태 미변경 기록만 허용 (E-5).
  - `missing`/`fod_alert` 진입은 FOD monitor 전용(`mark_checkout_timeouts`) — 외부 호출 금지 (S-8).
  - 기존에 통과하던 직행 전이 `in_slot→staged`, `staged→in_slot`(pick/place 한 단계 생략)이 이제 거부된다.
  - 알려진 속성: `out`은 fetch/return 공용 in-transit 상태이므로 status 단독으로는 방향을 알 수 없다. 구조적 합법성만 검증하며, event_type↔실제 동작 정합은 orchestrator 책임(event 로그에 보존).
  - `UpdateToolStatus.srv` 필드 계약은 불변. 동작(거부 범위)만 변경.

### Notes
- 미해결(팀 합의 대기): DB Gate 이중 구현(`DBClient` vs `ToolRepository`) 단일화 (B2-1). 단일화 시 전이 화이트리스트를 공유 레이어로 끌어올려야 한다.
- ⚠️ **Track C 우회 잔존**: `DBClient.log_event()`는 본 화이트리스트를 거치지 않는다. B2-1에서 공유 레이어로 끌어올려야 완전히 닫힌다.
- 알려진 한계(범위 외): `intent_status_simulator_node`/`intent_status_mapping`은 intent당 update 1건만 보낸다(fetch→`out`, return→`in_slot`). 2단계 대칭 모델에서 fetch는 place(`out→staged`) 단계를, return은 pick(`staged→out`) 단계를 생략하므로 시뮬레이터 단독 라운드트립은 DB Gate(return은 `staged`에서만 진입) 및 직행 거부(`staged→in_slot`)에서 막힌다. production motion-완료 콜백이 intent당 2건(pick 후·place 후)을 emit하도록 구현될 때 정합된다(테스트 전용 경로).
