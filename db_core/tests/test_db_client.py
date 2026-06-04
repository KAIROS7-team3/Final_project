import pytest
import sqlite3
from pathlib import Path

from db_core.client import DBCacheExpiredError, DBClient, DBError
from db_core.repository import VALID_SYSTEM_EVENT_TYPES


@pytest.fixture
def db(tmp_path):
    client = DBClient(db_path=str(tmp_path / "test.db"))
    client.connect()
    # Seed a test tool
    client._conn.execute(
        "INSERT INTO tools (tool_id, display_name, current_status, home_slot_row, home_slot_col)"
        " VALUES ('wrench_8mm', '렌치 8mm', 'in_slot', 0, 0)"
    )
    client._conn.commit()
    yield client
    client.disconnect()


class TestGetToolStatus:
    def test_happy_path(self, db):
        status = db.get_tool_status("wrench_8mm")
        assert status.tool_id == "wrench_8mm"
        assert status.current_status == "in_slot"

    def test_unknown_tool_raises(self, db):
        with pytest.raises(DBError):
            db.get_tool_status("nonexistent_tool")


class TestCheckFeasibility:
    def test_fetch_in_slot_feasible(self, db):
        feasible, reason = db.check_feasibility("fetch", "wrench_8mm")
        assert feasible is True
        assert reason == ""

    def test_fetch_out_blocked(self, db):
        db._conn.execute("UPDATE tools SET current_status='out' WHERE tool_id='wrench_8mm'")
        db._conn.commit()
        feasible, reason = db.check_feasibility("fetch", "wrench_8mm")
        assert feasible is False
        assert "out" in reason

    def test_return_staged_feasible(self, db):
        db._conn.execute("UPDATE tools SET current_status='staged' WHERE tool_id='wrench_8mm'")
        db._conn.commit()
        feasible, reason = db.check_feasibility("return", "wrench_8mm")
        assert feasible is True

    def test_return_in_slot_blocked(self, db):
        feasible, reason = db.check_feasibility("return", "wrench_8mm")
        assert feasible is False


class TestLogEvent:
    def test_happy_path(self, db):
        event_id = db.log_event(
            tool_id="wrench_8mm",
            event_type="fetch",
            track="A",
            status_before="in_slot",
            status_after="staged",
        )
        assert event_id is not None and event_id > 0
        status = db.get_tool_status("wrench_8mm")
        assert status.current_status == "staged"

    def test_log_plc_system_error_event(self, db):
        db.log_system_event(
            event_type="plc_error",
            severity="error",
            notes="coil write failed address=3",
        )

        row = db._conn.execute(
            "SELECT event_type, severity, notes FROM system_events"
        ).fetchone()
        assert tuple(row) == (
            "plc_error",
            "error",
            "coil write failed address=3",
        )


def test_plc_error_migration_accepts_new_system_event(tmp_path):
    db_path = tmp_path / "old.db"
    migration = (
        Path(__file__).resolve().parents[1]
        / "migrations"
        / "002_add_plc_error_system_event.sql"
    )
    with sqlite3.connect(db_path) as conn:
        conn.executescript(
            """
            CREATE TABLE system_events (
                event_id INTEGER PRIMARY KEY AUTOINCREMENT,
                event_type TEXT NOT NULL
                    CHECK(event_type IN ('boot','calibration','fod_alert')),
                track TEXT CHECK(track IN ('A','B','C')),
                severity TEXT NOT NULL
                    CHECK(severity IN ('info','warning','error','critical')),
                notes TEXT,
                timestamp TIMESTAMP NOT NULL
                    DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
            );
            INSERT INTO system_events (event_type, severity, notes)
            VALUES ('boot', 'info', 'old row');
            INSERT INTO system_events (event_type, severity, notes)
            VALUES ('fod_alert', 'warning', 'old alert row');
            """
        )
        conn.executescript(migration.read_text())
        conn.execute(
            "INSERT INTO system_events (event_type, severity, notes) "
            "VALUES ('plc_error', 'error', 'after migration')"
        )
        rows = conn.execute(
            "SELECT event_type, severity FROM system_events ORDER BY event_id"
        ).fetchall()

    assert rows == [
        ("boot", "info"),
        ("fod_alert", "warning"),
        ("plc_error", "error"),
    ]


def test_valid_system_event_types_include_plc_error_and_fod_alert():
    assert {"fod_alert", "plc_error"} <= VALID_SYSTEM_EVENT_TYPES
