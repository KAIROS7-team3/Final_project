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

from db_core.schema import SCHEMA_SQL

# DB Gate가 공식적으로 받아들이는 값만 상수로 고정한다.
VALID_INTENTS = frozenset({"fetch", "return"})
VALID_STATUSES = frozenset({"in_slot", "out", "staged", "missing", "fod_alert"})
VALID_EVENT_TYPES = frozenset(
    {"fetch", "return", "rejected", "error", "fod_alert", "reconciled"}
)
VALID_TRACKS = frozenset({"A", "B", "C"})
DEFAULT_OPERATOR_ID = "operator_01"
DB_CACHE_TTL_SECONDS = 300.0
# db_service_node·fod_monitor_node가 같은 WAL 파일에 동시 쓰기를 시도할 때
# 즉시 SQLITE_BUSY로 실패하지 않고 대기할 시간(ms). 운영 값은 config/runtime.yaml.
DEFAULT_BUSY_TIMEOUT_MS = 5000

# update_tool_status가 허용하는 외부(Track A/B/C, 정상 motion 완료) 상태 전이.
# v1.0 상태 모델: fetch/return 모두 pick -> place 2단계 (S-6 Staging 경유).
#   out    = 로봇 그리퍼에 들려 이동 중(in-transit, 짧음)
#   staged = Staging Area 거치 / operator 사용 중(길음, S-8 timeout 주 대상)
#   fetch:  in_slot --pick--> out --place--> staged
#   return: staged  --pick--> out --place--> in_slot
# 직행 전이(in_slot<->staged)는 pick/place 한 단계를 건너뛰므로 허용하지 않는다.
# out에서의 목적지(staged=fetch place / in_slot=return place)로 방향을 구분한다.
# (status_before, new_status) -> 허용 event_type.
# missing/fod_alert 진입(out|staged -> missing, missing -> fod_alert)은
# FOD monitor가 mark_checkout_timeouts에서 track=None으로 직접 기록하므로
# 외부 호출 화이트리스트에는 포함하지 않는다 (S-8).
_ALLOWED_TRANSITIONS: dict[tuple[str, str], frozenset[str]] = {
    ("in_slot", "out"): frozenset({"fetch"}),   # fetch pick: 슬롯에서 집어듦
    ("out", "staged"): frozenset({"fetch"}),    # fetch place: Staging Area에 거치
    ("staged", "out"): frozenset({"return"}),   # return pick: Staging에서 집어듦
    ("out", "in_slot"): frozenset({"return"}),  # return place: 슬롯에 되돌림
}

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

    def __init__(
        self,
        db_path: str | Path,
        operator_id: str = DEFAULT_OPERATOR_ID,
        busy_timeout_ms: int = DEFAULT_BUSY_TIMEOUT_MS,
    ) -> None:
        self.db_path = Path(db_path)
        self.operator_id = operator_id
        self._busy_timeout_ms = busy_timeout_ms
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
                # S-8/S-9: 불법 전이(예: missing -> in_slot 자동 수정)를 차단한다.
                transition_error = self._validate_transition(
                    status_before, new_status, event_type, notes
                )
                if transition_error:
                    conn.rollback()
                    return UpdateResult(False, transition_error)
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
                    track=None,
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
        """Open SQLite with row access by column name and FK enforcement.

        스키마가 없으면 SCHEMA_SQL로 부트스트랩한다. SCHEMA_SQL은 모두
        `CREATE TABLE IF NOT EXISTS`라 기존 DB에 대해 멱등하며, 기존 테이블의
        컬럼·FK 누락은 그대로 _validate_schema가 잡아낸다. DBClient.connect()와
        동일한 SCHEMA_SQL을 공유해 두 트랙 간 스키마 drift를 방지한다.
        """

        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        conn.execute("PRAGMA journal_mode = WAL")
        # 다중 프로세스 동시 쓰기 시 즉시 SQLITE_BUSY로 실패하지 않도록 대기시킨다.
        conn.execute(f"PRAGMA busy_timeout = {self._busy_timeout_ms}")
        conn.executescript(SCHEMA_SQL)
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
                    track=None,
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

    @staticmethod
    def _validate_transition(
        status_before: str,
        new_status: str,
        event_type: str,
        notes: str,
    ) -> str:
        """Reject status transitions that would bypass S-8/S-9.

        값 자체는 _validate_update가 검증했다. 여기서는 (현재 상태 -> 새 상태)
        조합의 합법성을 본다.
        """

        # S-9: missing/fod_alert 해제는 운영자 확인(notes)을 동반한 reconciled로만.
        # 자동(무인) 수정 금지 — 부팅 reconciliation도 이 경로를 쓴다.
        if event_type == "reconciled":
            if not notes.strip():
                return "reconciled transition requires operator confirmation notes (S-9)"
            return ""
        # E-5: error는 상태를 바꾸지 않는 기록만 허용한다.
        if event_type == "error":
            if new_status != status_before:
                return f"error event must not change status ({status_before} -> {new_status})"
            return ""
        # S-8: missing/fod_alert 진입은 FOD monitor 전용. 외부 트랙은 설정 금지.
        if new_status in {"missing", "fod_alert"}:
            return f"{new_status} is set only by the FOD monitor (S-8)"
        allowed_events = _ALLOWED_TRANSITIONS.get((status_before, new_status))
        if allowed_events is None:
            return f"illegal transition: {status_before} -> {new_status}"
        if event_type not in allowed_events:
            return f"event_type {event_type} not allowed for {status_before} -> {new_status}"
        return ""

    def _insert_tool_event(
        self,
        conn: sqlite3.Connection,
        tool_id: str,
        event_type: str,
        track: str | None,
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
