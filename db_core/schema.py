SCHEMA_SQL = """
PRAGMA journal_mode=WAL;
PRAGMA foreign_keys=ON;

CREATE TABLE IF NOT EXISTS operators (
    operator_id  TEXT PRIMARY KEY CHECK(operator_id GLOB '[a-z][a-z0-9_]*'),
    display_name TEXT NOT NULL,
    created_at   TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE TABLE IF NOT EXISTS tools (
    tool_id        TEXT PRIMARY KEY,
    display_name   TEXT NOT NULL,
    current_status TEXT NOT NULL
        CHECK(current_status IN ('in_slot','out','staged','missing','fod_alert')),
    home_slot_row  INTEGER NOT NULL,
    home_slot_col  INTEGER NOT NULL,
    last_event_id  INTEGER REFERENCES tool_events(event_id),
    last_updated   TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE TABLE IF NOT EXISTS tool_events (
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

CREATE TABLE IF NOT EXISTS system_events (
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

CREATE TABLE IF NOT EXISTS drawers (
    layer_id    INTEGER PRIMARY KEY,
    is_open     INTEGER NOT NULL DEFAULT 0 CHECK(is_open IN (0, 1)),
    last_updated TIMESTAMP NOT NULL DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE INDEX IF NOT EXISTS idx_tool_events_tool_time
    ON tool_events(tool_id, timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_tool_events_type_time
    ON tool_events(event_type, timestamp DESC);
CREATE INDEX IF NOT EXISTS idx_tool_events_track_time
    ON tool_events(track, timestamp DESC);

INSERT OR IGNORE INTO operators(operator_id, display_name)
    VALUES ('operator_01', 'default operator');
"""
