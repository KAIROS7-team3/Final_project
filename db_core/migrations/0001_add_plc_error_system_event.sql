PRAGMA foreign_keys=OFF;

CREATE TABLE IF NOT EXISTS system_events_new (
    event_id   INTEGER PRIMARY KEY AUTOINCREMENT,
    event_type TEXT NOT NULL
        CHECK(event_type IN (
            'boot','boot_complete','reconciliation_mismatch',
            'estop','estop_reset','db_cache_fallback','db_cache_expired',
            'calibration','plc_error'
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

PRAGMA foreign_keys=ON;
