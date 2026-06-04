from __future__ import annotations

import re
import sqlite3
from datetime import datetime, timedelta, timezone
from pathlib import Path

import db_core
import pytest
from db_core.repository import (
    DB_CACHE_TTL_SECONDS,
    VALID_EVENT_TYPES,
    VALID_SYSTEM_EVENT_TYPES,
    SchemaValidationError,
    ToolRepository,
    UpdateResult,
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

    # 2단계 fetch의 1단계: in_slot -> out (pick).
    result = ToolRepository(db_path).update_tool_status(
        tool_id="spanner_16mm",
        new_status="out",
        event_type="fetch",
        track="A",
        notes="picked from slot",
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
    assert status == "out"
    assert event == ("fetch", "in_slot", "out")


def test_update_allows_out_to_staged_place_step(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))  # socket_19mm 은 'out' 상태로 시작.

    # 2단계 fetch의 2단계: out -> staged (place at staging).
    result = ToolRepository(db_path).update_tool_status(
        tool_id="socket_19mm",
        new_status="staged",
        event_type="fetch",
        track="A",
        notes="placed at staging",
    )

    assert result.success is True


def test_update_rejects_illegal_in_slot_to_staged(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))  # spanner_16mm 은 'in_slot'.

    # 직접 in_slot -> staged 는 2단계 모델에서 불법 (out 경유 필수).
    result = ToolRepository(db_path).update_tool_status(
        tool_id="spanner_16mm",
        new_status="staged",
        event_type="fetch",
        track="A",
        notes="skip the out step",
    )

    assert result.success is False
    assert "illegal transition: in_slot -> staged" in result.message
    conn = sqlite3.connect(db_path)
    status = conn.execute(
        "SELECT current_status FROM tools WHERE tool_id = 'spanner_16mm'"
    ).fetchone()[0]
    events = conn.execute("SELECT COUNT(*) FROM tool_events").fetchone()[0]
    conn.close()
    # 거부 시 상태·이벤트 모두 변경되지 않아야 한다 (rollback).
    assert status == "in_slot"
    assert events == 0


def test_update_allows_staged_to_out_return_pick(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    conn = sqlite3.connect(db_path)
    conn.execute("UPDATE tools SET current_status = 'staged' WHERE tool_id = 'socket_19mm'")
    conn.commit()
    conn.close()

    # 2단계 return의 1단계: staged -> out (pick from staging).
    result = ToolRepository(db_path).update_tool_status(
        tool_id="socket_19mm",
        new_status="out",
        event_type="return",
        track="A",
        notes="picked from staging",
    )

    assert result.success is True
    conn = sqlite3.connect(db_path)
    event = conn.execute(
        "SELECT event_type, status_before, status_after FROM tool_events"
    ).fetchone()
    conn.close()
    assert event == ("return", "staged", "out")


def test_update_allows_out_to_in_slot_return_place(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))  # socket_19mm 은 'out' 상태로 시작.

    # 2단계 return의 2단계: out -> in_slot (place into slot).
    result = ToolRepository(db_path).update_tool_status(
        tool_id="socket_19mm",
        new_status="in_slot",
        event_type="return",
        track="A",
        notes="placed back in slot",
    )

    assert result.success is True


def test_update_rejects_illegal_staged_to_in_slot(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    conn = sqlite3.connect(db_path)
    conn.execute("UPDATE tools SET current_status = 'staged' WHERE tool_id = 'socket_19mm'")
    conn.commit()
    conn.close()

    # 직접 staged -> in_slot 는 2단계 모델에서 불법 (out 경유 필수).
    result = ToolRepository(db_path).update_tool_status(
        tool_id="socket_19mm",
        new_status="in_slot",
        event_type="return",
        track="A",
        notes="skip the out step",
    )

    assert result.success is False
    assert "illegal transition: staged -> in_slot" in result.message
    conn = sqlite3.connect(db_path)
    status = conn.execute(
        "SELECT current_status FROM tools WHERE tool_id = 'socket_19mm'"
    ).fetchone()[0]
    events = conn.execute("SELECT COUNT(*) FROM tool_events").fetchone()[0]
    conn.close()
    # 거부 시 상태·이벤트 모두 변경되지 않아야 한다 (rollback).
    assert status == "staged"
    assert events == 0


def test_update_rejects_external_missing_write(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))  # socket_19mm 은 'out'.

    # missing 진입은 FOD monitor 전용 — 외부 트랙이 직접 설정 금지 (S-8).
    result = ToolRepository(db_path).update_tool_status(
        tool_id="socket_19mm",
        new_status="missing",
        event_type="fod_alert",
        track="A",
        notes="should be blocked",
    )

    assert result.success is False
    assert "FOD monitor" in result.message


def test_reconciled_clears_missing_only_with_notes(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    conn = sqlite3.connect(db_path)
    conn.execute("UPDATE tools SET current_status = 'missing' WHERE tool_id = 'socket_19mm'")
    conn.commit()
    conn.close()

    # S-9: notes 없이 missing 해제 금지.
    blocked = ToolRepository(db_path).update_tool_status(
        tool_id="socket_19mm",
        new_status="in_slot",
        event_type="reconciled",
        track="A",
        notes="",
    )
    assert blocked.success is False
    assert "operator confirmation" in blocked.message

    # 운영자 확인 notes 가 있으면 허용.
    allowed = ToolRepository(db_path).update_tool_status(
        tool_id="socket_19mm",
        new_status="in_slot",
        event_type="reconciled",
        track="A",
        notes="operator confirmed tool back in slot",
    )
    assert allowed.success is True


def test_error_event_must_not_change_status(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))  # spanner_16mm 은 'in_slot'.

    # error 가 상태를 바꾸려 하면 거부.
    changed = ToolRepository(db_path).update_tool_status(
        tool_id="spanner_16mm",
        new_status="out",
        event_type="error",
        track="A",
        notes="motion failed mid-pick",
    )
    assert changed.success is False
    assert "error event must not change status" in changed.message

    # 같은 상태로의 error 기록은 허용 (E-5 실패 로깅).
    logged = ToolRepository(db_path).update_tool_status(
        tool_id="spanner_16mm",
        new_status="in_slot",
        event_type="error",
        track="A",
        notes="gripper fault, no motion",
    )
    assert logged.success is True


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

    # Finding 7: out -> missing 은 경보 전 단계라 event_type='timeout' (not 'fod_alert').
    conn = sqlite3.connect(db_path)
    row = conn.execute(
        "SELECT event_type, track, status_before, status_after FROM tool_events"
    ).fetchone()
    # 경보가 아니므로 system_events 채널에는 아무것도 남지 않아야 한다.
    sys_count = conn.execute("SELECT COUNT(*) FROM system_events").fetchone()[0]
    conn.close()
    assert row == ("timeout", None, "out", "missing")
    assert sys_count == 0


def test_fod_monitor_escalates_missing_to_fod_alert_with_system_event(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    # socket_19mm 을 grace 기간을 넘긴 missing 상태로 둔다.
    overdue = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
    conn = sqlite3.connect(db_path)
    conn.execute(
        "UPDATE tools SET current_status = 'missing', last_updated = ? "
        "WHERE tool_id = 'socket_19mm'",
        (overdue,),
    )
    conn.commit()
    conn.close()

    updates = ToolRepository(db_path).mark_checkout_timeouts(
        checkout_timeout=timedelta(minutes=10),
        alert_grace=timedelta(seconds=30),
    )

    assert len(updates) == 1
    assert updates[0].new_status == "fod_alert"

    conn = sqlite3.connect(db_path)
    # 실제 경보 단계는 event_type='fod_alert'.
    tool_event = conn.execute(
        "SELECT event_type, status_before, status_after FROM tool_events"
    ).fetchone()
    # E-5: 경보가 운영자/PLC 가시 채널(system_events)에 critical 로 남는다.
    sys_event = conn.execute(
        "SELECT event_type, severity, track FROM system_events"
    ).fetchone()
    conn.close()
    assert tool_event == ("fod_alert", "missing", "fod_alert")
    assert sys_event == ("fod_alert", "critical", None)


def test_log_system_event_writes_row(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))

    result = ToolRepository(db_path).log_system_event(
        event_type="estop",
        severity="critical",
        notes="emergency stop pressed",
    )

    assert result.success is True
    conn = sqlite3.connect(db_path)
    row = conn.execute(
        "SELECT event_type, severity, track, notes FROM system_events"
    ).fetchone()
    conn.close()
    assert row == ("estop", "critical", None, "emergency stop pressed")


def test_log_system_event_rejects_invalid_values(tmp_path) -> None:
    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    repository = ToolRepository(db_path)

    bad_type = repository.log_system_event(event_type="bogus", severity="info")
    assert bad_type.success is False
    assert "unsupported system event_type" in bad_type.message

    bad_severity = repository.log_system_event(event_type="boot", severity="loud")
    assert bad_severity.success is False
    assert "unsupported severity" in bad_severity.message

    bad_track = repository.log_system_event(
        event_type="boot", severity="info", track="Z"
    )
    assert bad_track.success is False
    assert "unsupported track" in bad_track.message

    # 잘못된 입력은 어떤 행도 남기지 않는다.
    conn = sqlite3.connect(db_path)
    count = conn.execute("SELECT COUNT(*) FROM system_events").fetchone()[0]
    conn.close()
    assert count == 0


def test_fod_escalation_commits_even_if_system_event_write_fails(
    tmp_path, monkeypatch
) -> None:
    """safety-reviewer HIGH: 가시성(system_events) 쓰기 실패가 FOD 경보 전이를
    롤백해선 안 된다 — 경보돼야 할 공구가 missing 에 머무는 fail-open 방지."""

    db_path = tmp_path / "robot_arm.db"
    _create_db(str(db_path))
    overdue = (datetime.now(timezone.utc) - timedelta(minutes=5)).isoformat()
    conn = sqlite3.connect(db_path)
    conn.execute(
        "UPDATE tools SET current_status = 'missing', last_updated = ? "
        "WHERE tool_id = 'socket_19mm'",
        (overdue,),
    )
    conn.commit()
    conn.close()

    repository = ToolRepository(db_path)
    # system_events insert 실패를 시뮬레이션 (커밋 후 별도 트랜잭션이므로 전이엔 무영향).
    monkeypatch.setattr(
        repository,
        "log_system_event",
        lambda *a, **k: UpdateResult(False, "simulated system_events failure"),
    )

    updates = repository.mark_checkout_timeouts(
        checkout_timeout=timedelta(minutes=10),
        alert_grace=timedelta(seconds=30),
    )

    assert len(updates) == 1
    assert updates[0].new_status == "fod_alert"
    conn = sqlite3.connect(db_path)
    status = conn.execute(
        "SELECT current_status FROM tools WHERE tool_id = 'socket_19mm'"
    ).fetchone()[0]
    tool_event = conn.execute(
        "SELECT event_type FROM tool_events WHERE status_after = 'fod_alert'"
    ).fetchone()
    conn.close()
    # 경보 전이는 커밋, 가시성 행만 누락(로그로 보고).
    assert status == "fod_alert"
    assert tool_event == ("fod_alert",)


def test_valid_event_type_constants_match_schema_check() -> None:
    """drift 방지: repository 상수가 schema.py 의 CHECK enum 과 정확히 일치해야 한다."""

    # SCHEMA_SQL 안의 두 `CHECK(event_type IN (...))` — 순서: tool_events, system_events.
    lists = re.findall(r"CHECK\(event_type IN \((.*?)\)\)", SCHEMA_SQL, re.DOTALL)
    assert len(lists) == 2, "expected exactly two event_type CHECK clauses"
    tool_event_check = set(re.findall(r"'([^']+)'", lists[0]))
    system_event_check = set(re.findall(r"'([^']+)'", lists[1]))

    assert tool_event_check == VALID_EVENT_TYPES
    assert system_event_check == VALID_SYSTEM_EVENT_TYPES


def test_migration_001_upgrades_old_check_and_is_idempotent(tmp_path) -> None:
    """migrations/001 이 옛 CHECK DB 에 새 event_type 을 적용하고, 데이터 보존 +
    재실행 안전(멱등)함을 검증한다."""

    # 새 event_type 이 없는 "옛" 스키마를 만든다.
    old_schema = SCHEMA_SQL.replace(
        "'fetch','return','rejected','error','timeout','fod_alert','reconciled'",
        "'fetch','return','rejected','error','fod_alert','reconciled'",
    ).replace(
        "'db_cache_fallback','db_cache_expired','calibration',\n"
        "            'fod_alert'",
        "'db_cache_fallback','db_cache_expired','calibration'",
    )
    assert "timeout" not in old_schema  # sanity: 치환이 실제로 일어났는지 확인

    db_path = tmp_path / "old.db"
    conn = sqlite3.connect(db_path)
    conn.executescript(old_schema)
    conn.execute(
        "INSERT INTO tools (tool_id, display_name, current_status, home_slot_row,"
        " home_slot_col, last_updated)"
        " VALUES ('t', 'T', 'out', 0, 0, '2026-01-01T00:00:00+00:00')"
    )
    conn.commit()
    # 옛 CHECK 에서는 'timeout' 이 거부돼야 한다.
    with pytest.raises(sqlite3.IntegrityError):
        conn.execute(
            "INSERT INTO tool_events (tool_id, event_type, operator_id, status_after)"
            " VALUES ('t', 'timeout', 'operator_01', 'missing')"
        )
    conn.close()

    migration_sql = (
        Path(db_core.__file__).resolve().parent
        / "migrations"
        / "001_add_timeout_and_fod_alert_event_types.sql"
    ).read_text()

    def apply_and_check() -> None:
        conn = sqlite3.connect(db_path)
        conn.executescript(migration_sql)
        # 마이그레이션 후 새 event_type 삽입 가능.
        conn.execute(
            "INSERT INTO tool_events (tool_id, event_type, operator_id, status_after)"
            " VALUES ('t', 'timeout', 'operator_01', 'missing')"
        )
        conn.execute(
            "INSERT INTO system_events (event_type, severity) VALUES ('fod_alert', 'critical')"
        )
        conn.commit()
        # 기존 tools 데이터 보존.
        assert (
            conn.execute(
                "SELECT current_status FROM tools WHERE tool_id = 't'"
            ).fetchone()[0]
            == "out"
        )
        conn.close()

    apply_and_check()
    # 멱등성: 재실행해도 오류 없이 성공하고 데이터가 보존된다.
    apply_and_check()
