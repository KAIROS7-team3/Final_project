# Changelog — db_core

Keep a Changelog 형식. `ToolRepository`/`DBClient` API 변경, 스키마 마이그레이션 시 갱신 (P-2).

## [Unreleased]

### Added
- **(B1-1) `ToolRepository.log_rejection(tool_id, reason, track=None)` 추가** — 상태를 바꾸지 않고 `tool_events('rejected')` 한 줄만 남기는 공개 경로. DB Gate(S-2) 거부는 내부 `_record_rejection`(check_feasibility 안)이 이미 기록하지만, **DB Gate에 도달하기도 전에 드롭된 명령**(특히 orchestrator의 S-7 `is_moving` 가드)은 기록 경로가 없었다. `status_after`가 NOT NULL이라 현재 상태를 읽어 `status_before/after`에 그대로 넣고 `current_status`는 불변. 예외를 던지지 않고 `UpdateResult(success, message)` 반환(존재하지 않는 `tool_id`는 FK 충돌 대신 깔끔한 실패). 자유형 `reason`은 `MAX_REJECTION_NOTES_LEN`(1024자)으로 잘라 감사 로그 비대화를 막는다. `db_service_node`가 신규 `LogEvent.srv`(event_type 서버측 `'rejected'` 제한)로 노출. ⚠️ **dual-write 주의**: 이제 `tool_events('rejected')`를 쓰는 경로가 둘(`_record_rejection` 내부/S-2, `log_rejection` 공개/S-7)이다. B2-1(DB Gate 단일화) 해결 시 이 두 경로를 공유 레이어로 통합해야 한다.
- `ToolRepository.__init__`에 `busy_timeout_ms` 파라미터 추가 (기본 5000) — 다중 프로세스(`db_service_node`+`fod_monitor_node`)의 WAL 락 경합 시 `SQLITE_BUSY` 즉시 실패 방지. 운영 값은 `config/runtime.yaml` (`db.busy_timeout_ms`).
- **(E-5) `ToolRepository.log_system_event(event_type, severity, track=None, notes="")` 추가** — `system_events`(운영자/PLC 가시 채널)에 상태와 무관한 시스템 사건(부팅, e-stop, FOD 경보 등)을 기록한다. 기존엔 `DBClient`만 system_events를 쓸 수 있었고 `ToolRepository`(Track A/B 경로)는 아예 쓸 수 없었다. 반환 `UpdateResult(success, message)`. 입력 검증: `VALID_SYSTEM_EVENT_TYPES`/`VALID_SEVERITIES`/`VALID_TRACKS`.
- **(스키마 마이그레이션) `tool_events.event_type`에 `'timeout'`, `system_events.event_type`에 `'fod_alert'` 추가** (`schema.py` CHECK + `repository.py` `VALID_EVENT_TYPES`/`VALID_SYSTEM_EVENT_TYPES`). 멱등 마이그레이션 `db_core/migrations/001_add_timeout_and_fod_alert_event_types.sql` 제공(표준 12-step 테이블 재작성, 데이터·FK·인덱스 보존). ⚠️ 기존 DB 처리는 아래 Notes 참조.
- 회귀 방지 테스트 추가: `VALID_EVENT_TYPES`/`VALID_SYSTEM_EVENT_TYPES`가 `SCHEMA_SQL`의 CHECK enum과 정확히 일치하는지 검증(상수↔스키마 drift 차단).

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

### Fixed
- **(Finding 7) `mark_checkout_timeouts` event_type 오기록 + FOD 경보 가시성(E-5)** — 기존엔 `out/staged → missing`(경보 전 단계)과 `missing → fod_alert`(실제 경보)를 **둘 다** `event_type='fod_alert'`로 기록해 로그에서 두 단계를 구분할 수 없었다. 이제 `→ missing`은 `'timeout'`, `→ fod_alert`만 `'fod_alert'`로 기록한다. 추가로 **실제 경보(`→ fod_alert`) 도달 시 `system_events`에 `event_type='fod_alert', severity='critical'` 행을 기록**(E-5). 단, 이 가시성 쓰기는 **상태 전이가 커밋된 뒤 별도 트랜잭션**으로 수행한다 — 같은 트랜잭션에 두면 `system_events` insert 실패가 FOD 경보 전이까지 롤백시켜 경보돼야 할 공구가 `missing`에 머무는 fail-open이 되기 때문(safety-reviewer HIGH 반영). 가시성 쓰기 실패는 로그로만 남기고 이미 커밋된 전이는 보존한다. `mark_checkout_timeouts` 시그니처·반환(`list[FodUpdate]`) 불변; `fod_monitor_node`는 `fod_alert`만 ROS 로그 error 레벨로 격상.
- **(Finding 8, 동시성) `DBClient` 멀티스레드 안전성** — `connect()`가 `check_same_thread=False`로 연결을 열어 한 인스턴스를 여러 스레드(Track C VLA 워커 + ROS executor 등)에서 공유할 수 있는데, 접근을 직렬화하는 락이 없었다. `log_event()`의 `INSERT → UPDATE → commit`이 동시 호출 시 인터리빙되어 `tools.last_event_id`가 엉뚱한 이벤트를 가리키거나 sqlite `recursive use of cursors`/`no transaction is active` 오류가 났다. `threading.RLock`(`self._lock`)으로 `connect`/`disconnect`/`get_tool_status`/`check_feasibility`/`log_event`/`log_system_event`를 보호. **공개 API·시그니처·반환값 불변(계약 무변경)** — 단일 스레드 동작 동일, 동시 접근만 직렬화. `check_feasibility`가 `get_tool_status`를 락 안에서 중첩 호출하므로 재진입 가능한 RLock 필요. 회귀 테스트(`TestConcurrency`) 추가: 락 제거 시 200건 중 49건만 커밋 + 7 오류로 검출됨.

### Notes
- ⚠️ **스키마 마이그레이션 필요**: `tool_events`/`system_events`의 `event_type` CHECK 제약이 바뀌었다. `*.db`는 `.gitignore` 대상이라 저장소에는 DB 파일이 없고, 새/빈 DB는 `_connect()`의 `SCHEMA_SQL` 부트스트랩으로 새 CHECK를 그대로 갖는다. 다만 **기존 로컬 `robot_arm.db`는 `CREATE TABLE IF NOT EXISTS`로 갱신되지 않으므로** 옛 CHECK가 남아 `'timeout'`/system_events `'fod_alert'` insert 시 `CHECK constraint failed`로 실패한다. 기존 로컬 DB는 **(a)** 멱등 마이그레이션 `db_core/migrations/001_add_timeout_and_fod_alert_event_types.sql`을 적용(데이터 보존, 표준 12-step 테이블 재작성)하거나 **(b)** 개발/부트스트랩 단계라면 삭제 후 재생성하면 된다. 마이그레이션 자동 실행기는 아직 없으므로(부트스트랩만 `SCHEMA_SQL` 사용) 현재는 수동 적용 — 자동 러너는 후속(Phase 7+ 정책).
- 미해결(팀 합의 대기): DB Gate 이중 구현(`DBClient` vs `ToolRepository`) 단일화 (B2-1). 단일화 시 전이 화이트리스트를 공유 레이어로 끌어올려야 한다.
- ⚠️ **Track C 우회 잔존**: `DBClient.log_event()`는 본 화이트리스트를 거치지 않는다. B2-1에서 공유 레이어로 끌어올려야 완전히 닫힌다.
- 알려진 한계(범위 외): `intent_status_simulator_node`/`intent_status_mapping`은 intent당 update 1건만 보낸다(fetch→`out`, return→`in_slot`). 2단계 대칭 모델에서 fetch는 place(`out→staged`) 단계를, return은 pick(`staged→out`) 단계를 생략하므로 시뮬레이터 단독 라운드트립은 DB Gate(return은 `staged`에서만 진입) 및 직행 거부(`staged→in_slot`)에서 막힌다. production motion-완료 콜백이 intent당 2건(pick 후·place 후)을 emit하도록 구현될 때 정합된다(테스트 전용 경로).
