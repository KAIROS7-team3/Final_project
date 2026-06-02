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
VALID_TRACKS = frozenset({"A", "B", "C"})
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
    """DB 일시 장애 때만 쓰는 공구 상태 캐시.

    `loaded_at`은 wall clock이 아니라 monotonic time을 사용한다. 시스템 시간이
    바뀌어도 TTL 계산이 흔들리지 않게 하기 위함이다.
    """

    current_status: str
    loaded_at: float


class SchemaValidationError(sqlite3.DatabaseError):
    """DB 파일이 안전 스키마 요구사항과 맞지 않을 때 발생한다."""


class ToolRepository:
    """DB Gate, 상태 갱신, FOD timeout을 한 곳에서 처리하는 repository.

    ROS2 service node는 이 객체만 호출한다. 이렇게 분리하면 `db_core`는 ROS2를
    몰라도 되고, DB Gate 규칙을 Track A/B와 테스트에서 같은 방식으로 검증할 수 있다.
    """

    def __init__(self, db_path: str | Path, operator_id: str = DEFAULT_OPERATOR_ID) -> None:
        self.db_path = Path(db_path)
        self.operator_id = operator_id
        self._status_cache: dict[str, _CachedStatus] = {}

    def check_feasibility(self, intent: str, tool_id: str) -> FeasibilityResult:
        """현재 DB 상태에서 fetch/return 명령을 실행해도 되는지 확인한다.

        이 함수가 프로젝트의 DB Gate다. downstream motion이나 simulator는 이
        판정을 통과한 명령만 처리해야 하며, 실패한 명령은 가능하면 rejected
        event로 남긴다.
        """

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
            # 스키마 자체가 잘못된 DB는 상태를 신뢰할 수 없으므로 즉시 거부한다.
            return FeasibilityResult(False, f"database schema error: {exc}")
        except sqlite3.Error as exc:
            # DB가 잠겼거나 일시적으로 unavailable한 경우, 최근에 읽은 같은 공구의
            # 상태만 TTL 안에서 사용한다. 캐시가 없거나 오래됐으면 안전하게 거부한다.
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
        """이미 확보한 `current_status`에 DB Gate 규칙을 적용한다."""

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
        """공구 상태 전이와 append-only event 기록을 함께 저장한다.

        이 함수는 motion 완료 후 호출되는 쓰기 경로다. command를 받자마자 상태를
        바꾸는 것이 아니라, 실제 동작이 끝났다는 상위 계층의 보고 이후에 호출되어야 한다.
        """

        tool_id = tool_id.strip()
        new_status = new_status.strip()
        event_type = event_type.strip()
        track = track.strip()

        validation_error = self._validate_update(tool_id, new_status, event_type, track)
        if validation_error:
            # 외부 service request 값은 신뢰하지 않는다. DB CHECK 제약 전에 한 번 더
            # 명시적으로 검증해 caller에게 사람이 읽을 수 있는 사유를 돌려준다.
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
        """오래 방치된 공구 상태를 S-8 규칙에 따라 자동 전이한다.

        `out` 또는 `staged`가 checkout timeout을 넘기면 `missing`으로 바꾸고,
        `missing` 상태가 추가 grace 시간을 넘기면 `fod_alert`로 격상한다.
        """

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
        """SQLite 연결을 열고 필수 안전 스키마를 검증한다.

        row_factory를 켜서 column name으로 값을 읽고, FK를 켜서 tool/event 관계가
        깨지지 않게 한다. `_validate_schema()`는 오래된 DB 파일이 섞였을 때
        조용히 잘못 동작하지 않도록 실패를 앞당긴다.
        """

        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys = ON")
        conn.execute("PRAGMA journal_mode = WAL")
        _validate_schema(conn)
        return conn

    def _cached_status(self, tool_id: str) -> str | None:
        """S-2 TTL 안에 있는 캐시 상태만 반환한다."""

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
        """DB Gate 거부 결과를 만들고, 가능한 경우 rejected event도 남긴다."""

        logging_error = self._record_rejection(tool_id, current_status, reason)
        if logging_error:
            return FeasibilityResult(False, f"{reason}; {logging_error}")
        return FeasibilityResult(False, reason)

    def _record_rejection(self, tool_id: str, current_status: str, reason: str) -> str:
        """현재 상태는 바꾸지 않고 DB Gate 거부 이력만 저장한다."""

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
        """외부 service request 값을 DB에 쓰기 전에 검증한다."""

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
        track: str | None,
        status_before: str,
        status_after: str,
        notes: str,
        timestamp: str,
    ) -> int | None:
        """현재 스키마에 맞춰 `tool_events` 한 행을 삽입한다.

        insert column 목록을 dict에서 만들기 때문에 나중에 nullable column이 늘어도
        이 helper 안에서만 조정하면 된다.
        """

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
        """event 삽입 후 `tools` snapshot을 최신 상태로 맞춘다."""

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
        """상태는 그대로 두고 마지막 event pointer만 갱신한다."""

        conn.execute(
            "UPDATE tools SET last_event_id = ? WHERE tool_id = ?",
            (event_id, tool_id),
        )


def _table_columns(conn: sqlite3.Connection, table_name: str) -> set[str]:
    """스키마 검증에 사용할 table column 이름 집합을 반환한다."""

    return {str(row["name"]) for row in conn.execute(f"PRAGMA table_info({table_name})")}


def _validate_schema(conn: sqlite3.Connection) -> None:
    """DB 파일이 현재 안전 스키마와 맞지 않으면 빠르게 실패시킨다."""

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
    """필수 FK 제약이 실제 DB 파일에 존재하는지 확인한다."""

    rows = conn.execute(f"PRAGMA foreign_key_list({table_name})").fetchall()
    actual = {str(row["from"]) for row in rows}
    missing = sorted(constrained_columns - actual)
    if missing:
        raise SchemaValidationError(
            f"{table_name} missing foreign keys: {', '.join(missing)}"
        )


def _parse_utc(value: str) -> datetime:
    """ISO timestamp를 UTC aware datetime으로 정규화한다."""

    parsed = datetime.fromisoformat(value)
    if parsed.tzinfo is None:
        return parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)
