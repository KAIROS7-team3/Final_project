"""SQLite 기반 공구 DB 접근 계층.

ROS2 node는 이 repository를 통해서만 feasibility 확인과 상태 갱신을 수행한다.
여기서는 DB Gate, 이벤트 로그 append, FOD timeout 전이를 한 곳에서 관리한다.
"""

from __future__ import annotations

import sqlite3
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

# DB Gate가 공식적으로 받아들이는 값만 상수로 고정한다.
VALID_INTENTS = frozenset({"fetch", "return"})
VALID_STATUSES = frozenset({"in_slot", "out", "staged", "missing", "fod_alert"})
VALID_EVENT_TYPES = frozenset(
    {"fetch", "return", "rejected", "error", "fod_alert", "reconciled"}
)
VALID_TRACKS = frozenset({"A", "B", "C", "system"})
DEFAULT_OPERATOR_ID = "operator_01"
DB_CACHE_TTL_SECONDS = 300.0

REQUIRED_TABLE_COLUMNS = {
    "operators": frozenset({"operator_id", "display_name", "created_at"}),
    "tools": frozenset(
        {
            "tool_id",
            "display_name",
            "current_status",
            "home_slot_row",
            "home_slot_col",
            "last_event_id",
            "last_updated",
        }
    ),
    "tool_events": frozenset(
        {
            "event_id",
            "tool_id",
            "event_type",
            "track",
            "operator_id",
            "status_before",
            "status_after",
            "notes",
            "timestamp",
        }
    ),
}


@dataclass(frozen=True)
class FeasibilityResult:
    """DB Gate 판정 결과."""

    feasible: bool
    reason: str


@dataclass(frozen=True)
class UpdateResult:
    """상태 갱신 service 응답에 그대로 매핑되는 결과."""

    success: bool
    message: str


@dataclass(frozen=True)
class FodUpdate:
    """FOD monitor가 적용한 상태 전이 한 건."""

    tool_id: str
    previous_status: str
    new_status: str


@dataclass(frozen=True)
class _CachedStatus:
    current_status: str
    loaded_at: float


class SchemaValidationError(sqlite3.DatabaseError):
    """Raised when the DB file does not match the required safety schema."""


class ToolRepository:
    """SQLite adapter for DB Gate, status updates, and event logging."""

    def __init__(self, db_path: str | Path, operator_id: str = DEFAULT_OPERATOR_ID) -> None:
        self.db_path = Path(db_path)
        self.operator_id = operator_id
        self._status_cache: dict[str, _CachedStatus] = {}

    def check_feasibility(self, intent: str, tool_id: str) -> FeasibilityResult:
        """Return whether intent can run for the current tool status."""

        intent = intent.strip()
        tool_id = tool_id.strip()
        if intent not in VALID_INTENTS:
            return FeasibilityResult(False, f"unsupported intent: {intent}")
        if not tool_id:
            return FeasibilityResult(False, "tool_id is required")

        try:
            with self._connect() as conn:
                row = conn.execute(
                    "SELECT current_status FROM tools WHERE tool_id = ?",
                    (tool_id,),
                ).fetchone()
        except SchemaValidationError as exc:
            return FeasibilityResult(False, f"database schema error: {exc}")
        except sqlite3.Error as exc:
            cached_status = self._cached_status(tool_id)
            if cached_status is None:
                return FeasibilityResult(
                    False,
                    f"database unavailable and cache expired: {exc}",
                )
            current_status = cached_status
            return self._check_status(intent, tool_id, current_status)

        if row is None:
            return FeasibilityResult(False, "unknown tool")

        current_status = str(row["current_status"])
        self._status_cache[tool_id] = _CachedStatus(
            current_status=current_status,
            loaded_at=time.monotonic(),
        )
        return self._check_status(intent, tool_id, current_status)

    def _check_status(
        self,
        intent: str,
        tool_id: str,
        current_status: str,
    ) -> FeasibilityResult:
        """Apply DB Gate rules to a known current status."""

        if intent == "fetch":
            # Fetch는 공구가 슬롯 안에 있을 때만 허용한다.
            if current_status == "in_slot":
                return FeasibilityResult(True, "ok")
            return self._reject(tool_id, current_status, f"tool is {current_status}")

        # Return은 staging에 놓인 상태에서만 허용한다. out은 직접 반납 우회라 차단한다.
        if current_status == "staged":
            return FeasibilityResult(True, "ok")
        if current_status == "in_slot":
            return self._reject(tool_id, current_status, "tool is already in slot")
        return self._reject(
            tool_id,
            current_status,
            f"tool is {current_status}, expected staged",
        )

    def update_tool_status(
        self,
        tool_id: str,
        new_status: str,
        event_type: str,
        track: str,
        notes: str,
    ) -> UpdateResult:
        """Write one tool status transition and its append-only event record."""

        tool_id = tool_id.strip()
        new_status = new_status.strip()
        event_type = event_type.strip()
        track = track.strip()

        validation_error = self._validate_update(tool_id, new_status, event_type, track)
        if validation_error:
            return UpdateResult(False, validation_error)

        now_dt = datetime.now(timezone.utc)
        now = now_dt.isoformat()
        try:
            with self._connect() as conn:
                conn.execute("BEGIN")
                row = conn.execute(
                    "SELECT current_status FROM tools WHERE tool_id = ?",
                    (tool_id,),
                ).fetchone()
                if row is None:
                    conn.rollback()
                    return UpdateResult(False, "unknown tool")

                status_before = str(row["current_status"])
                # tools snapshot과 tool_events는 같은 transaction에서 갱신한다.
                event_id = self._insert_tool_event(
                    conn=conn,
                    tool_id=tool_id,
                    event_type=event_type,
                    track=track,
                    status_before=status_before,
                    status_after=new_status,
                    notes=notes,
                    timestamp=now,
                )
                self._update_tool_row(conn, tool_id, new_status, event_id, now)
                conn.commit()
                self._status_cache[tool_id] = _CachedStatus(
                    current_status=new_status,
                    loaded_at=time.monotonic(),
                )
        except sqlite3.Error as exc:
            return UpdateResult(False, f"database error: {exc}")

        return UpdateResult(True, "updated")

    def mark_checkout_timeouts(
        self,
        checkout_timeout: timedelta,
        alert_grace: timedelta,
    ) -> list[FodUpdate]:
        """Apply S-8 transitions for overdue out/staged/missing tools."""

        now_dt = datetime.now(timezone.utc)
        now = now_dt.isoformat()
        updates: list[FodUpdate] = []
        with self._connect() as conn:
            conn.execute("BEGIN")
            for row in conn.execute(
                """
                SELECT tool_id, current_status, last_updated
                FROM tools
                WHERE current_status IN ('out', 'staged', 'missing')
                """
            ).fetchall():
                last_updated = _parse_utc(str(row["last_updated"]))
                current_status = str(row["current_status"])
                new_status: str | None = None
                # out/staged가 오래 지속되면 먼저 missing, 이후 grace가 지나면 fod_alert.
                is_checkout_overdue = (
                    current_status in {"out", "staged"}
                    and now_dt - last_updated >= checkout_timeout
                )
                if is_checkout_overdue:
                    new_status = "missing"
                elif current_status == "missing" and now_dt - last_updated >= alert_grace:
                    new_status = "fod_alert"

                if new_status is None:
                    continue

                tool_id = str(row["tool_id"])
                event_id = self._insert_tool_event(
                    conn=conn,
                    tool_id=tool_id,
                    event_type="fod_alert",
                    track="system",
                    status_before=current_status,
                    status_after=new_status,
                    notes="FOD timeout transition",
                    timestamp=now,
                )
                self._update_tool_row(conn, tool_id, new_status, event_id, now)
                updates.append(FodUpdate(tool_id, current_status, new_status))
            conn.commit()

        return updates

    def _connect(self) -> sqlite3.Connection:
        """Open SQLite with row access by column name and FK enforcement."""

        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        conn.execute("PRAGMA journal_mode = WAL")
        _validate_schema(conn)
        return conn

    def _cached_status(self, tool_id: str) -> str | None:
        """Return cached status only while it is within the S-2 TTL window."""

        cached = self._status_cache.get(tool_id)
        if cached is None:
            return None
        age_s = time.monotonic() - cached.loaded_at
        if age_s > DB_CACHE_TTL_SECONDS:
            return None
        return cached.current_status

    def _reject(
        self,
        tool_id: str,
        current_status: str,
        reason: str,
    ) -> FeasibilityResult:
        """Return a rejected result and try to append a rejected event."""

        logging_error = self._record_rejection(tool_id, current_status, reason)
        if logging_error:
            return FeasibilityResult(False, f"{reason}; {logging_error}")
        return FeasibilityResult(False, reason)

    def _record_rejection(self, tool_id: str, current_status: str, reason: str) -> str:
        """Persist DB Gate rejection without changing current_status."""

        now = datetime.now(timezone.utc).isoformat()
        try:
            with self._connect() as conn:
                conn.execute("BEGIN")
                event_id = self._insert_tool_event(
                    conn=conn,
                    tool_id=tool_id,
                    event_type="rejected",
                    track="system",
                    status_before=current_status,
                    status_after=current_status,
                    notes=reason,
                    timestamp=now,
                )
                if event_id is not None:
                    self._update_last_event(conn, tool_id, event_id)
                conn.commit()
        except sqlite3.Error as exc:
            return f"failed to log rejected event: {exc}"
        return ""

    @staticmethod
    def _validate_update(
        tool_id: str,
        new_status: str,
        event_type: str,
        track: str,
    ) -> str:
        """Validate externally supplied service values before writing DB."""

        if not tool_id:
            return "tool_id is required"
        if new_status not in VALID_STATUSES:
            return f"unsupported status: {new_status}"
        if event_type not in VALID_EVENT_TYPES:
            return f"unsupported event_type: {event_type}"
        if track not in VALID_TRACKS:
            return f"unsupported track: {track}"
        return ""

    def _insert_tool_event(
        self,
        conn: sqlite3.Connection,
        tool_id: str,
        event_type: str,
        track: str,
        status_before: str,
        status_after: str,
        notes: str,
        timestamp: str,
    ) -> int | None:
        """Insert a tool_events row using the required schema."""

        values: dict[str, object] = {
            "tool_id": tool_id,
            "event_type": event_type,
            "track": track,
            "operator_id": self.operator_id,
            "status_before": status_before,
            "status_after": status_after,
            "notes": notes,
            "timestamp": timestamp,
        }
        insert_columns = list(values)
        placeholders = ", ".join("?" for _ in insert_columns)
        column_sql = ", ".join(insert_columns)
        cursor = conn.execute(
            f"INSERT INTO tool_events ({column_sql}) VALUES ({placeholders})",
            tuple(values[column] for column in insert_columns),
        )
        return int(cursor.lastrowid) if cursor.lastrowid else None

    @staticmethod
    def _update_tool_row(
        conn: sqlite3.Connection,
        tool_id: str,
        new_status: str,
        event_id: int | None,
        timestamp: str,
    ) -> None:
        """Update the tools snapshot row after an event has been inserted."""

        assignments = ["current_status = ?", "last_updated = ?", "last_event_id = ?"]
        values: list[object] = [new_status, timestamp, event_id]
        values.append(tool_id)
        conn.execute(
            f"UPDATE tools SET {', '.join(assignments)} WHERE tool_id = ?",
            tuple(values),
        )

    @staticmethod
    def _update_last_event(
        conn: sqlite3.Connection,
        tool_id: str,
        event_id: int,
    ) -> None:
        """Update last_event_id after a rejected DB Gate event."""

        conn.execute(
            "UPDATE tools SET last_event_id = ? WHERE tool_id = ?",
            (event_id, tool_id),
        )


def _table_columns(conn: sqlite3.Connection, table_name: str) -> set[str]:
    """Return the column names for compatibility with evolving DB schemas."""

    return {str(row["name"]) for row in conn.execute(f"PRAGMA table_info({table_name})")}


def _validate_schema(conn: sqlite3.Connection) -> None:
    """Fail fast when a DB file does not match the required schema."""

    for table_name, required_columns in REQUIRED_TABLE_COLUMNS.items():
        columns = _table_columns(conn, table_name)
        missing = sorted(required_columns - columns)
        if missing:
            raise SchemaValidationError(
                f"{table_name} missing required columns: {', '.join(missing)}"
            )

    _require_foreign_keys(conn, "tools", {"last_event_id"})
    _require_foreign_keys(conn, "tool_events", {"tool_id", "operator_id"})


def _require_foreign_keys(
    conn: sqlite3.Connection,
    table_name: str,
    constrained_columns: set[str],
) -> None:
    rows = conn.execute(f"PRAGMA foreign_key_list({table_name})").fetchall()
    actual = {str(row["from"]) for row in rows}
    missing = sorted(constrained_columns - actual)
    if missing:
        raise SchemaValidationError(
            f"{table_name} missing foreign keys: {', '.join(missing)}"
        )


def _parse_utc(value: str) -> datetime:
    """Parse an ISO timestamp and normalize it to UTC."""

    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)
