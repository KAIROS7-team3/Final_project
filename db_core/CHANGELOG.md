# Changelog — db_core

Keep a Changelog 형식. `ToolRepository`/`DBClient` API 변경, 스키마 마이그레이션 시 갱신 (P-2).

## [Unreleased]

### Added
- **(issue #44) 서랍(layer) 단위 DB Gate — `drawers` 테이블 + `check_drawer_feasibility` + `update_drawer_state` 추가.**
  - `schema.py`에 `drawers(layer_id, is_open, last_updated)` 테이블 추가. 부트스트랩 시 자동 생성.
  - `ToolRepository.check_drawer_feasibility(intent, layer_id) -> FeasibilityResult`:
    - `open`: `drawers` 테이블에 해당 layer가 이미 열려 있으면 `(False, "already_open")` 반환. 호출자는 reason == "already_open"일 때 open 동작을 **생략**(에러 아님)하고 진행한다. layer row가 없거나 닫힌 상태면 `(True, "ok")`.
    - `close`: 항상 `(True, "ok")` — DB 레벨 차단 없음.
  - `ToolRepository.update_drawer_state(layer_id, intent) -> UpdateResult`: open/close 완료 후 motion이 호출. `INSERT OR REPLACE`로 row 없으면 자동 생성.
  - `DBClient.check_drawer_feasibility()` / `DBClient.update_drawer_state()` 추가 — `ToolRepository`에 위임 (Track C 경로). Track A/B ROS2 경로(`db_service_node`) 노출은 후속 작업.
  - `VALID_DRAWER_INTENTS = frozenset({"open", "close"})` — `VALID_INTENTS`("fetch"/"return")와 명시적으로 분리.
  - ⚠️ PR #46의 `fod_alert`/`out`/`staged` 기반 차단 로직을 교체 — 서랍 열림은 공구 상태와 무관하고 `drawers` 테이블 상태로만 판정.


- **(B1-1) `ToolRepository.log_rejection(tool_id, reason, track=None)` 추가** — 상태를 바꾸지 않고 `tool_events('rejected')` 한 줄만 남기는 공개 경로. DB Gate(S-2) 거부는 내부 `_record_rejection`(check_feasibility 안)이 이미 기록하지만, **DB Gate에 도달하기도 전에 드롭된 명령**(특히 orchestrator의 S-7 `is_moving` 가드)은 기록 경로가 없었다. `status_after`가 NOT NULL이라 현재 상태를 읽어 `status_before/after`에 그대로 넣고 `current_status`는 불변. 예외를 던지지 않고 `UpdateResult(success, message)` 반환(존재하지 않는 `tool_id`는 FK 충돌 대신 깔끔한 실패). 자유형 `reason`은 `MAX_REJECTION_NOTES_LEN`(1024자)으로 잘라 감사 로그 비대화를 막는다. `db_service_node`가 신규 `LogEvent.srv`(event_type 서버측 `'rejected'` 제한)로 노출. ⚠️ **dual-write 주의**: `tool_events('rejected')`를 쓰는 경로가 둘(`_record_rejection` 내부/S-2, `log_rejection` 공개/S-7)이었다 → **B2-1에서 공유 헬퍼 `_append_rejection`으로 통합됨**(아래 Changed 참조).
- `ToolRepository.__init__`에 `busy_timeout_ms` 파라미터 추가 (기본 5000) — 다중 프로세스(`db_service_node`+`fod_monitor_node`)의 WAL 락 경합 시 `SQLITE_BUSY` 즉시 실패 방지. 운영 값은 `config/runtime.yaml` (`db.busy_timeout_ms`).
- **(E-5) `ToolRepository.log_system_event(event_type, severity, track=None, notes="")` 추가** — `system_events`(운영자/PLC 가시 채널)에 상태와 무관한 시스템 사건(부팅, e-stop, FOD 경보 등)을 기록한다. 기존엔 `DBClient`만 system_events를 쓸 수 있었고 `ToolRepository`(Track A/B 경로)는 아예 쓸 수 없었다. 반환 `UpdateResult(success, message)`. 입력 검증: `VALID_SYSTEM_EVENT_TYPES`/`VALID_SEVERITIES`/`VALID_TRACKS`.
- **(스키마 마이그레이션) `tool_events.event_type`에 `'timeout'`, `system_events.event_type`에 `'fod_alert'` 추가** (`schema.py` CHECK + `repository.py` `VALID_EVENT_TYPES`/`VALID_SYSTEM_EVENT_TYPES`). 멱등 마이그레이션 `db_core/migrations/001_add_timeout_and_fod_alert_event_types.sql` 제공(표준 12-step 테이블 재작성, 데이터·FK·인덱스 보존). ⚠️ 기존 DB 처리는 아래 Notes 참조.
- 회귀 방지 테스트 추가: `VALID_EVENT_TYPES`/`VALID_SYSTEM_EVENT_TYPES`가 `SCHEMA_SQL`의 CHECK enum과 정확히 일치하는지 검증(상수↔스키마 drift 차단).

### Changed
- **(B2-1, 안전) DB Gate 이중 구현 단일화 — `DBClient`를 `ToolRepository` 위임 어댑터로 전환.** Track A/B가 `db_service_node`를 통해 `ToolRepository`에 접근하듯, Track C용 `DBClient`도 자체 DB 로직(중복 `check_feasibility`/`log_event`/캐시) 대신 내부 `ToolRepository`(공유 코어)에 포워드한다.
  - **`DBClient.log_event()`가 이제 `ToolRepository.update_tool_status()`로 라우팅** — 호출자의 `status_before`를 무시하고 DB에서 실제 상태를 읽어 **S-8/S-9 전이 화이트리스트를 강제**한다. ⚠️ **Track C 우회 차단**: 불법 전이(`missing→in_slot`, 직행 `in_slot→staged` 등)는 조용히 쓰이지 않고 `DBError`를 던진다. `status_after`는 *목표* 상태로 해석된다. **공개 시그니처·반환(int event_id)은 불변**; `status_before`/`operator_id`는 호환을 위해 시그니처에만 남고 더는 권위 없음(repository 소유).
  - `DBClient.check_feasibility()`/`log_system_event()`도 `ToolRepository`로 위임 → Track C와 Track A/B가 **단일 판정·검증 구현**을 공유(drift 차단). `check_feasibility`는 infeasible 시 `ToolRepository`가 `rejected` 감사 행도 남긴다(`db_service_node` 경로와 동일). `log_system_event`는 `VALID_SYSTEM_EVENT_TYPES` 검증을 거친다(예전 `DBClient`는 무검증).
  - `get_tool_status()`는 slot 좌표 포함 읽기 전용 편의 메서드라 보조 연결(`self._conn`)로 직접 읽는 방식 유지(쓰기 아님 → 우회 위험 없음). `connect()`도 `PRAGMA foreign_keys/journal_mode=WAL`를 `ToolRepository._connect()`와 맞춰 같은 파일에서 journal mode drift를 막는다.
  - **`self._lock`(RLock) 범위 축소**: 쓰기는 `ToolRepository`가 호출마다 새 연결로 처리해 공유 쓰기 연결이 없어졌다. RLock은 이제 보조 읽기 연결(`_conn`) 접근만 직렬화한다(Finding 8 회귀 의도 보존).
  - **`ToolRepository._append_rejection(conn, ...)` 추가**: `_record_rejection`(S-2 내부)과 `log_rejection`(S-7 공개)이 공유하던 `rejected` 쓰기 로직을 단일 헬퍼로 통합(B1-1 dual-write 정리). 호출자가 트랜잭션 경계를 소유.
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
- **(B2-1, 동시성) 쓰기 트랜잭션을 `BEGIN` → `BEGIN IMMEDIATE`로 변경** (`update_tool_status`, `mark_checkout_timeouts`, `log_rejection`, `log_system_event`, `_record_rejection`). deferred `BEGIN`은 트랜잭션 안에서 `SELECT`가 읽기 스냅샷만 잡은 사이 다른 writer가 커밋하면, 이후 `INSERT` 업그레이드가 `SQLITE_BUSY_SNAPSHOT`("database is locked")로 실패한다(busy_timeout으로 해소 안 됨). 다중 writer(`fod_monitor_node` + `db_service_node`, 그리고 B2-1로 합류한 Track C in-process 경로)가 동시에 쓸 때 잠복하던 버그로, Track C 쓰기를 `ToolRepository`로 라우팅한 동시성 회귀 테스트에서 드러났다. `BEGIN IMMEDIATE`는 쓰기 잠금을 트랜잭션 시작 시점에 잡아 동시 writer를 `busy_timeout` 대기로 직렬화한다.
- **(Finding 7) `mark_checkout_timeouts` event_type 오기록 + FOD 경보 가시성(E-5)** — 기존엔 `out/staged → missing`(경보 전 단계)과 `missing → fod_alert`(실제 경보)를 **둘 다** `event_type='fod_alert'`로 기록해 로그에서 두 단계를 구분할 수 없었다. 이제 `→ missing`은 `'timeout'`, `→ fod_alert`만 `'fod_alert'`로 기록한다. 추가로 **실제 경보(`→ fod_alert`) 도달 시 `system_events`에 `event_type='fod_alert', severity='critical'` 행을 기록**(E-5). 단, 이 가시성 쓰기는 **상태 전이가 커밋된 뒤 별도 트랜잭션**으로 수행한다 — 같은 트랜잭션에 두면 `system_events` insert 실패가 FOD 경보 전이까지 롤백시켜 경보돼야 할 공구가 `missing`에 머무는 fail-open이 되기 때문(safety-reviewer HIGH 반영). 가시성 쓰기 실패는 로그로만 남기고 이미 커밋된 전이는 보존한다. `mark_checkout_timeouts` 시그니처·반환(`list[FodUpdate]`) 불변; `fod_monitor_node`는 `fod_alert`만 ROS 로그 error 레벨로 격상.
- **(Finding 8, 동시성) `DBClient` 멀티스레드 안전성** — `connect()`가 `check_same_thread=False`로 연결을 열어 한 인스턴스를 여러 스레드(Track C VLA 워커 + ROS executor 등)에서 공유할 수 있는데, 접근을 직렬화하는 락이 없었다. `log_event()`의 `INSERT → UPDATE → commit`이 동시 호출 시 인터리빙되어 `tools.last_event_id`가 엉뚱한 이벤트를 가리키거나 sqlite `recursive use of cursors`/`no transaction is active` 오류가 났다. `threading.RLock`(`self._lock`)으로 `connect`/`disconnect`/`get_tool_status`/`check_feasibility`/`log_event`/`log_system_event`를 보호. **공개 API·시그니처·반환값 불변(계약 무변경)** — 단일 스레드 동작 동일, 동시 접근만 직렬화. `check_feasibility`가 `get_tool_status`를 락 안에서 중첩 호출하므로 재진입 가능한 RLock 필요. 회귀 테스트(`TestConcurrency`) 추가: 락 제거 시 200건 중 49건만 커밋 + 7 오류로 검출됨.

### Notes
- ⚠️ **스키마 마이그레이션 필요**: `tool_events`/`system_events`의 `event_type` CHECK 제약이 바뀌었다. `*.db`는 `.gitignore` 대상이라 저장소에는 DB 파일이 없고, 새/빈 DB는 `_connect()`의 `SCHEMA_SQL` 부트스트랩으로 새 CHECK를 그대로 갖는다. 다만 **기존 로컬 `robot_arm.db`는 `CREATE TABLE IF NOT EXISTS`로 갱신되지 않으므로** 옛 CHECK가 남아 `'timeout'`/system_events `'fod_alert'` insert 시 `CHECK constraint failed`로 실패한다. 기존 로컬 DB는 **(a)** 멱등 마이그레이션 `db_core/migrations/001_add_timeout_and_fod_alert_event_types.sql`을 적용(데이터 보존, 표준 12-step 테이블 재작성)하거나 **(b)** 개발/부트스트랩 단계라면 삭제 후 재생성하면 된다. 마이그레이션 자동 실행기는 아직 없으므로(부트스트랩만 `SCHEMA_SQL` 사용) 현재는 수동 적용 — 자동 러너는 후속(Phase 7+ 정책).
- ✅ **(B2-1 해결) DB Gate 이중 구현 단일화 완료** — `DBClient`가 `ToolRepository`에 위임하므로 전이 화이트리스트가 단일 코어에 모였다(위 Changed 참조). 이전 "Track C 우회 잔존"(`DBClient.log_event()`가 화이트리스트를 안 거치던 문제)도 함께 닫혔다.
- ⚠️ **팀 합의 사항(문서 갱신 동반)**: spec(`.omc/specs/...`)·`architecture.md`의 "Track C는 `DBClient`를 import"는 여전히 유효하나, 이제 `DBClient`는 얇은 위임 어댑터다. `DBClient` 위에 작업 중인 사람이 있다면 `log_event`가 전이 검증을 강제(불법 전이 시 `DBError`)함, `log_system_event`가 `VALID_SYSTEM_EVENT_TYPES`를 강제함을 공유할 것. 향후 `DBClient`를 완전히 제거하고 Track C가 `ToolRepository`를 직접 쓰는 방향도 가능하나 별도 합의 필요. (현재 `DBClient` 실사용 소비자는 테스트뿐 — `track_c_vla.py`는 빈 스텁이라 동작 변경의 실제 영향 반경 0.)
- **리뷰 반영(safety-reviewer CONDITIONAL PASS / interface-guardian PROCEED-WITH-NOTES)**:
  - (D) `DBClient.log_event`가 `update_tool_status` 쓰기를 `_lock` **밖**에서 수행하고 `_lock`은 `_conn` readback만 감싼다 — 경합 쓰기(busy_timeout 대기)가 동시 `get_tool_status` 읽기를 막지 않게.
  - (C) `get_tool_status`는 advisory(표시용)이고 S-2 권위 판정은 `check_feasibility`임을 docstring에 명시. `DBClient._cache`/`ToolRepository._status_cache` 이중 캐시는 둘 다 TTL 초과 시 fail-closed이나 outage 중 나이가 달라질 수 있음 — 단일 캐시 통합은 후속.
  - 회귀 테스트 추가: 같은 공구 동시 쓰기에서 정확히 1건 성공 + 나머지는 전이 검증 `DBError`(절대 `database is locked` 아님) — `BEGIN IMMEDIATE` 직접 검증.
- 이연(HIL 전 권장, 범위 외): ① `fod_monitor_node._poll`은 이미 `mark_checkout_timeouts`를 `try/except`로 감싸 잠금 타임아웃 sweep을 로깅+다음 틱 재시도로 강등(call-site 완화 확인됨) — 내부 guard 추가는 선택. ② `DBClient.connect()`에 `_validate_schema` 호출을 더해 `ToolRepository._connect()`와 fail-fast 동등화. ③ `mark_checkout_timeouts` × `log_event` 교차 동시성 테스트(실제 fod_monitor/db_service 다중 프로세스 경쟁 재현).
- 알려진 한계(범위 외): `intent_status_simulator_node`/`intent_status_mapping`은 intent당 update 1건만 보낸다(fetch→`out`, return→`in_slot`). 2단계 대칭 모델에서 fetch는 place(`out→staged`) 단계를, return은 pick(`staged→out`) 단계를 생략하므로 시뮬레이터 단독 라운드트립은 DB Gate(return은 `staged`에서만 진입) 및 직행 거부(`staged→in_slot`)에서 막힌다. production motion-완료 콜백이 intent당 2건(pick 후·place 후)을 emit하도록 구현될 때 정합된다(테스트 전용 경로).
