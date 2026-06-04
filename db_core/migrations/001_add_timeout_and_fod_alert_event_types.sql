-- Migration 001 — Finding 7 + E-5
-- tool_events.event_type 에 'timeout', system_events.event_type 에 'fod_alert' 추가.
--
-- 배경: schema.py 는 CREATE TABLE IF NOT EXISTS 라 "이미 존재하는" DB의 CHECK
-- 제약을 갱신하지 못한다. SQLite 는 CHECK 제약을 ALTER 로 바꿀 수 없으므로
-- 표준 12-step 테이블 재작성으로 새 CHECK 를 적용한다(데이터·FK·인덱스 보존).
--
-- 멱등성: 항상 정규 스키마로 재작성하므로 재실행해도 안전하다(데이터는 매번
-- 복사되어 보존). event_id 값은 명시적으로 복사해 tools.last_event_id FK 참조가
-- 그대로 유지된다.
--
-- 신규/빈 DB 에는 불필요하다(schema.py 부트스트랩이 이미 새 CHECK 를 만든다).
-- v1.0 개발 단계에서는 로컬 robot_arm.db 삭제 후 재생성도 동등한 효과를 낸다.

PRAGMA foreign_keys=OFF;

BEGIN TRANSACTION;

-- 이전 마이그레이션 시도가 중간에 실패해 남았을 수 있는 임시 테이블 정리(멱등성).
DROP TABLE IF EXISTS tool_events_new;
DROP TABLE IF EXISTS system_events_new;

-- ── tool_events: event_type 에 'timeout' 추가 ──────────────────────────────
CREATE TABLE tool_events_new (
    event_id      INTEGER PRIMARY KEY AUTOINCREMENT,
    tool_id       TEXT NOT NULL REFERENCES tools(tool_id),
    event_type    TEXT NOT NULL
        CHECK(event_type IN ('fetch','return','rejected','error','timeout','fod_alert','reconciled')),
    track         TEXT CHECK(track IN ('A','B','C')),
    operator_id   TEXT NOT NULL REFERENCES operators(operator_id),
    status_before TEXT CHECK(status_before IN ('in_slot','out','staged','missing','fod_alert')),
    status_after  TEXT NOT NULL
        CHECK(status_after IN ('in_slot','out','staged','missing','fod_alert')),
    notes         TEXT,
    timestamp     TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

INSERT INTO tool_events_new
    (event_id, tool_id, event_type, track, operator_id, status_before, status_after, notes, timestamp)
SELECT
    event_id, tool_id, event_type, track, operator_id, status_before, status_after, notes, timestamp
FROM tool_events;

DROP TABLE tool_events;
ALTER TABLE tool_events_new RENAME TO tool_events;

CREATE INDEX IF NOT EXISTS idx_tool_events_tool_time
    ON tool_events(tool_id, timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_tool_events_type_time
    ON tool_events(event_type, timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_tool_events_track_time
    ON tool_events(track, timestamp DESC);

-- ── system_events: event_type 에 'fod_alert' 추가 ──────────────────────────
CREATE TABLE system_events_new (
    event_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    event_type TEXT NOT NULL
        CHECK(event_type IN (
            'boot','boot_complete','reconciliation_mismatch',
            'estop','estop_reset','db_cache_fallback','db_cache_expired','calibration',
            'fod_alert'
        )),
    track      TEXT CHECK(track IN ('A','B','C')),
    severity   TEXT NOT NULL CHECK(severity IN ('info','warning','error','critical')),
    notes      TEXT,
    timestamp  TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

INSERT INTO system_events_new
    (event_id, event_type, track, severity, notes, timestamp)
SELECT
    event_id, event_type, track, severity, notes, timestamp
FROM system_events;

DROP TABLE system_events;
ALTER TABLE system_events_new RENAME TO system_events;

COMMIT;

-- FK 무결성 확인 후 재활성화.
PRAGMA foreign_key_check;
PRAGMA foreign_keys=ON;
