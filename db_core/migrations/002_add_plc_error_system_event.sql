-- Migration 002 — add plc_error system event
-- system_events.event_type 에 'plc_error' 추가.
--
-- main의 001 migration(tool_events.timeout + system_events.fod_alert) 다음 단계로
-- PLC actuator/read failure 가시화를 추가한다. 기존 system_events 데이터를 보존하는
-- 12-step 재작성 방식으로 적용한다.

PRAGMA foreign_keys=OFF;

BEGIN TRANSACTION;

DROP TABLE IF EXISTS system_events_new;

CREATE TABLE system_events_new (
    event_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    event_type TEXT NOT NULL
        CHECK(event_type IN (
            'boot','boot_complete','reconciliation_mismatch',
            'estop','estop_reset','db_cache_fallback','db_cache_expired',
            'calibration','fod_alert','plc_error'
        )),
    track      TEXT CHECK(track IN ('A','B','C')),
    severity   TEXT NOT NULL CHECK(severity IN ('info','warning','error','critical')),
    notes      TEXT,
    timestamp  TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

INSERT INTO system_events_new (
    event_id,
    event_type,
    track,
    severity,
    notes,
    timestamp
)
SELECT
    event_id,
    event_type,
    track,
    severity,
    notes,
    timestamp
FROM system_events;

DROP TABLE system_events;
ALTER TABLE system_events_new RENAME TO system_events;

COMMIT;

PRAGMA foreign_key_check;
PRAGMA foreign_keys=ON;
