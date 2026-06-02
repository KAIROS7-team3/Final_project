"""공구 상태 DB의 기본 SQLite 스키마.

이 스키마는 런타임 상태를 담는 로컬 DB 파일(`robot_arm.db`)을 초기화할 때
사용한다. 핵심 설계는 두 가지다.

1. `tools`는 현재 상태 snapshot이다.
   DB Gate가 빠르게 `current_status`를 읽고 fetch/return 가능 여부를 판단한다.
2. `tool_events`와 `system_events`는 추적 가능한 append log다.
   거부, 오류, FOD 전이, E-stop 같은 사건을 나중에 재현할 수 있게 남긴다.

CHECK 제약은 잘못된 status/event_type이 DB에 들어가는 것을 막는 마지막 방어선이다.
서비스 레벨 검증이 먼저 실패해야 하지만, DB도 안전 critical 값을 한 번 더 막는다.
"""

SCHEMA_SQL = """
PRAGMA journal_mode=WAL;
PRAGMA foreign_keys=ON;

CREATE TABLE IF NOT EXISTS operators (
    -- 작업자 ID는 로그 추적용이다. 기본 operator는 아래 INSERT에서 보장한다.
    operator_id  TEXT PRIMARY KEY CHECK(operator_id GLOB '[a-z][a-z0-9_]*'),
    display_name TEXT NOT NULL,
    created_at   TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE TABLE IF NOT EXISTS tools (
    -- 공구별 현재 상태 snapshot. DB Gate가 가장 자주 읽는 테이블이다.
    tool_id        TEXT PRIMARY KEY,
    display_name   TEXT NOT NULL,
    current_status TEXT NOT NULL
        -- fetch/return/FOD 정책에서 인정하는 상태만 저장한다.
        CHECK(current_status IN ('in_slot','out','staged','missing','fod_alert')),
    home_slot_row  INTEGER NOT NULL,
    home_slot_col  INTEGER NOT NULL,
    -- 마지막 이벤트를 따라가면 현재 상태가 어떤 사건에서 왔는지 확인할 수 있다.
    last_event_id  INTEGER REFERENCES tool_events(event_id),
    last_updated   TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE TABLE IF NOT EXISTS tool_events (
    -- 공구 상태 변경과 DB Gate 거부를 모두 남기는 append-only 성격의 로그.
    event_id      INTEGER PRIMARY KEY AUTOINCREMENT,
    tool_id       TEXT NOT NULL REFERENCES tools(tool_id),
    event_type    TEXT NOT NULL
        CHECK(event_type IN ('fetch','return','rejected','error','fod_alert','reconciled')),
    -- rejected처럼 특정 track이 없을 수 있어 NULL을 허용한다.
    track         TEXT CHECK(track IN ('A','B','C')),
    operator_id   TEXT NOT NULL REFERENCES operators(operator_id),
    status_before TEXT CHECK(status_before IN ('in_slot','out','staged','missing','fod_alert')),
    status_after  TEXT NOT NULL
        CHECK(status_after IN ('in_slot','out','staged','missing','fod_alert')),
    notes         TEXT,
    timestamp     TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE TABLE IF NOT EXISTS system_events (
    -- 공구 한 개가 아니라 시스템 전체 상태와 관련된 사건을 기록한다.
    event_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    event_type TEXT NOT NULL
        CHECK(event_type IN (
            'boot','boot_complete','reconciliation_mismatch',
            'estop','estop_reset','db_cache_fallback','db_cache_expired','calibration'
        )),
    track      TEXT CHECK(track IN ('A','B','C')),
    severity   TEXT NOT NULL CHECK(severity IN ('info','warning','error','critical')),
    notes      TEXT,
    timestamp  TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE INDEX IF NOT EXISTS idx_tool_events_tool_time
    -- 특정 공구의 최근 이력을 빠르게 확인하기 위한 index.
    ON tool_events(tool_id, timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_tool_events_type_time
    -- rejected/error/fod_alert 같은 사건 유형별 조회에 사용한다.
    ON tool_events(event_type, timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_tool_events_track_time
    -- Track A/B/C별 디버깅과 리포트 생성을 위한 index.
    ON tool_events(track, timestamp DESC);

INSERT OR IGNORE INTO operators(operator_id, display_name)
    -- operator_id를 지정하지 않은 초기 테스트에서도 FK가 깨지지 않게 한다.
    VALUES ('operator_01', 'default operator');
"""
