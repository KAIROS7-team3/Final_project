from __future__ import annotations

import sqlite3
from datetime import datetime, timedelta, timezone

import pytest
from db_core.repository import (
    DB_CACHE_TTL_SECONDS,
    SchemaValidationError,
    ToolRepository,
)
from db_core.schema import SCHEMA_SQL

CONFIGURED_TOOL_IDS = (
    "screwdriver",
    "utility_knife",
    "ratchet_wrench",
    "multi_tool",
    "spanner_16mm",
    "socket_19mm",
)


def _create_db(path: str) -> None:
    conn = sqlite3.connect(path)
    conn.executescript(SCHEMA_SQL)
    conn.execute(
        """
        INSERT INTO tools (
            tool_id, display_name, current_status, home_slot_row, home_slot_col, last_updated
        )
        VALUES (?, ?, ?, ?, ?, ?)
        """,
        (
            "spanner_16mm",
            "스패너 16mm",
            "in_slot",
            0,
            0,
            "2026-01-01T00:00:00+00:00",
        ),
    )
    conn.execute(
        """
        INSERT INTO tools (
            tool_id, display_name, current_status, home_slot_row, home_slot_col, last_updated
        )
        VALUES (?, ?, ?, ?, ?, ?)
        """,
        (
            "socket_19mm",
            "복스 소켓 19mm",
            "out",
            0,
            1,
            "2026-01-01T00:00:00+00:00",
        ),
    )
    conn.commit()
    conn.close()


def _create_configured_tools_db(path: str) -> None:
    _create_db(path)
    conn = sqlite3.connect(path)
    for index, tool_id in enumerate(CONFIGURED_TOOL_IDS):
        conn.execute(
            """
            INSERT OR REPLACE INTO tools (
                tool_id,
                display_name,
                current_status,
                home_slot_row,
                home_slot_col,
                last_event_id,
                last_updated
            )
            VALUES (?, ?, 'in_slot', 0, ?, NULL, ?)
            """,
            (tool_id, tool_id, index, "2026-01-01T00:00:00+00:00"),
        )
    conn.commit()
    conn.close()


def test_fetch_feasible_for_all_configured_tools_when_in_slot(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_configured_tools_db(str(db_path))

    repository = ToolRepository(db_path)

    for tool_id in CONFIGURED_TOOL_IDS:
        result = repository.check_feasibility("fetch", tool_id)
        assert result.feasible is True
        assert result.reason == "ok"


def test_fetch_feasible_when_tool_is_in_slot(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).check_feasibility("fetch", "spanner_16mm")

    assert result.feasible is True
    assert result.reason == "ok"


def test_fetch_rejected_when_tool_is_out(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).check_feasibility("fetch", "socket_19mm")

    assert result.feasible is False
    assert "out" in result.reason


def test_return_rejected_when_tool_is_out(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).check_feasibility("return", "socket_19mm")

    assert result.feasible is False
    assert "expected staged" in result.reason


def test_return_feasible_when_tool_is_staged(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    conn = sqlite3.connect(db_path)
    conn.execute("UPDATE tools SET current_status = 'staged' WHERE tool_id = 'socket_19mm'")
    conn.commit()
    conn.close()

    result = ToolRepository(db_path).check_feasibility("return", "socket_19mm")

    assert result.feasible is True
    assert result.reason == "ok"


def test_rejected_feasibility_writes_rejected_event(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).check_feasibility("fetch", "socket_19mm")

    assert result.feasible is False
    conn = sqlite3.connect(db_path)
    event = conn.execute(
        """
        SELECT event_type, track, status_before, status_after, notes
        FROM tool_events
        """
    ).fetchone()
    conn.close()
    assert event == ("rejected", None, "out", "out", "tool is out")


def test_update_tool_status_writes_event_and_tool_snapshot(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).update_tool_status(
        tool_id="spanner_16mm",
        new_status="staged",
        event_type="fetch",
        track="A",
        notes="placed at staging",
    )

    assert result.success is True
    conn = sqlite3.connect(db_path)
    status = conn.execute(
        "SELECT current_status FROM tools WHERE tool_id = 'spanner_16mm'"
    ).fetchone()[0]
    event = conn.execute(
        "SELECT event_type, status_before, status_after FROM tool_events"
    ).fetchone()
    conn.close()
    assert status == "staged"
    assert event == ("fetch", "in_slot", "staged")


def test_schema_validation_rejects_missing_foreign_keys(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    conn = sqlite3.connect(db_path)
    conn.executescript(
        """
        CREATE TABLE operators (
            operator_id TEXT PRIMARY KEY,
            display_name TEXT NOT NULL,
            created_at TEXT NOT NULL
        );
        CREATE TABLE tools (
            tool_id TEXT PRIMARY KEY,
            display_name TEXT NOT NULL,
            current_status TEXT NOT NULL,
            home_slot_row INTEGER NOT NULL,
            home_slot_col INTEGER NOT NULL,
            last_event_id INTEGER,
            last_updated TEXT NOT NULL
        );
        CREATE TABLE tool_events (
            event_id INTEGER PRIMARY KEY AUTOINCREMENT,
            tool_id TEXT NOT NULL,
            event_type TEXT NOT NULL,
            track TEXT NOT NULL,
            operator_id TEXT NOT NULL,
            status_before TEXT,
            status_after TEXT NOT NULL,
            notes TEXT,
            timestamp TEXT NOT NULL
        );
        """
    )
    conn.close()

    with pytest.raises(SchemaValidationError, match="missing foreign keys"):
        with ToolRepository(db_path)._connect():
            pass


def test_check_feasibility_uses_cache_only_within_ttl(tmp_path, monkeypatch) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    repository = ToolRepository(db_path)

    result = repository.check_feasibility("fetch", "spanner_16mm")
    assert result.feasible is True

    def fail_connect():
        raise sqlite3.OperationalError("offline")

    monkeypatch.setattr(repository, "_connect", fail_connect)

    recent = repository.check_feasibility("fetch", "spanner_16mm")
    assert recent.feasible is True

    loaded_at = repository._status_cache["spanner_16mm"].loaded_at
    monkeypatch.setattr(
        "db_core.repository.time.monotonic",
        lambda: loaded_at + DB_CACHE_TTL_SECONDS + 1.0,
    )
    expired = repository.check_feasibility("fetch", "spanner_16mm")
    assert expired.feasible is False
    assert "cache expired" in expired.reason


def test_update_rejects_invalid_status(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).update_tool_status(
        tool_id="spanner_16mm",
        new_status="lost",
        event_type="error",
        track="A",
        notes="invalid",
    )

    assert result.success is False
    assert "unsupported status" in result.message


def test_update_rejects_system_as_external_track(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).update_tool_status(
        tool_id="spanner_16mm",
        new_status="out",
        event_type="fetch",
        track="system",
        notes="invalid track",
    )

    assert result.success is False
    assert result.message == "unsupported track: system"


def test_connect_bootstraps_schema_on_fresh_db(tmp_path) -> None:
    # 존재하지 않는 DB 파일 — 어떤 노드도 스키마를 미리 만들지 않은 상태.
    db_path = tmp_path / "fresh.db"

    result = ToolRepository(db_path).check_feasibility("fetch", "spanner_16mm")

    # 스키마가 부트스트랩되어 SchemaValidationError 대신 정상 판정 경로를 탄다.
    assert result.feasible is False
    assert result.reason == "unknown tool"


def test_connect_applies_busy_timeout(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"

    with ToolRepository(db_path, busy_timeout_ms=1234)._connect() as conn:
        timeout = conn.execute("PRAGMA busy_timeout").fetchone()[0]

    assert timeout == 1234


def test_fod_monitor_marks_overdue_out_tool_missing(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    overdue = (datetime.now(timezone.utc) - timedelta(minutes=20)).isoformat()
    conn = sqlite3.connect(db_path)
    conn.execute(
        "UPDATE tools SET last_updated = ? WHERE tool_id = 'socket_19mm'",
        (overdue,),
    )
    conn.commit()
    conn.close()

    updates = ToolRepository(db_path).mark_checkout_timeouts(
        checkout_timeout=timedelta(minutes=10),
        alert_grace=timedelta(seconds=30),
    )

    assert len(updates) == 1
    assert updates[0].tool_id == "socket_19mm"
    assert updates[0].new_status == "missing"

    conn = sqlite3.connect(db_path)
    track = conn.execute("SELECT track FROM tool_events WHERE event_type = 'fod_alert'").fetchone()[0]
    conn.close()
    assert track is None
