import logging
import sqlite3
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from db_core.schema import SCHEMA_SQL

logger = logging.getLogger(__name__)

_DB_CACHE_TTL_SECONDS = 300  # S-2: 5분 캐시 TTL
_DEFAULT_OPERATOR_ID = "operator_01"


@dataclass
class ToolStatus:
    tool_id: str
    current_status: str
    home_slot_row: int
    home_slot_col: int
    last_updated: str


class DBError(Exception):
    pass


class DBCacheExpiredError(DBError):
    """DB 연결 실패 + TTL 초과 — 모든 명령 거부 (S-2)."""


class DBClient:
    """SQLite WAL client for tool status and event logging.

    No rclpy dependency — usable from Track A/B and Track C.

    Thread-safety: ``connect()`` opens the connection with
    ``check_same_thread=False`` so a single client may be shared across
    threads (e.g. Track C VLA worker + ROS executor). All connection and
    cache access is serialized by ``self._lock`` (reentrant) so multi-step
    transactions like ``log_event`` (INSERT → UPDATE → commit) cannot
    interleave with concurrent writers and corrupt ``last_event_id`` /
    ``current_status``.
    """

    def __init__(self, db_path: str = "robot_arm.db") -> None:
        self._db_path = db_path
        self._conn: sqlite3.Connection | None = None
        self._cache: dict[str, ToolStatus] = {}
        self._cache_loaded_at: float = 0.0
        # Reentrant: check_feasibility() nests get_tool_status() under the lock.
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
            self._conn.row_factory = sqlite3.Row
            self._conn.executescript(SCHEMA_SQL)
            self._conn.commit()
            logger.info("[DBClient] connected - path=%s", self._db_path)

    def disconnect(self) -> None:
        with self._lock:
            if self._conn:
                self._conn.close()
                self._conn = None

    def get_tool_status(self, tool_id: str) -> ToolStatus:
        """Return current status for tool_id. Falls back to cache on DB error (S-2)."""
        with self._lock:
            try:
                row = self._query_one(
                    "SELECT tool_id, current_status, home_slot_row, home_slot_col, last_updated "
                    "FROM tools WHERE tool_id = ?",
                    (tool_id,),
                )
                if row is None:
                    raise DBError(f"tool_id not found: {tool_id}")
                status = ToolStatus(**dict(row))
                self._cache[tool_id] = status
                self._cache_loaded_at = time.monotonic()
                return status
            except DBError:
                raise
            except Exception as e:
                logger.warning("[DBClient] DB error, falling back to cache - error=%s", e)
                return self._cache_fallback(tool_id)

    def check_feasibility(self, intent: str, tool_id: str) -> tuple[bool, str]:
        """Return (feasible, reason). Implements DB Gate (S-2)."""
        # Status read + verdict run under one lock; the reentrant RLock lets
        # get_tool_status() reacquire it.
        with self._lock:
            status = self.get_tool_status(tool_id)
            if intent == "fetch":
                if status.current_status == "in_slot":
                    return True, ""
                return False, f"tool is {status.current_status}"
            if intent == "return":
                if status.current_status == "staged":
                    return True, ""
                return False, f"tool is {status.current_status}, expected staged"
            return False, f"unknown intent: {intent}"

    def log_event(
        self,
        tool_id: str,
        event_type: str,
        track: str,
        status_before: str | None,
        status_after: str,
        notes: str = "",
        operator_id: str = _DEFAULT_OPERATOR_ID,
    ) -> int:
        """Insert event row and update tools.current_status. Returns event_id."""
        # Lock so INSERT → UPDATE → commit is atomic vs concurrent writers
        # (otherwise an interleaved log_event could overwrite last_event_id).
        with self._lock:
            if not self._conn:
                raise DBError("DBClient not connected — call connect() first")
            cur = self._conn.execute(
                "INSERT INTO tool_events"
                " (tool_id, event_type, track, operator_id, status_before, status_after, notes)"
                " VALUES (?, ?, ?, ?, ?, ?, ?)",
                (tool_id, event_type, track, operator_id, status_before, status_after, notes),
            )
            event_id = cur.lastrowid
            self._conn.execute(
                "UPDATE tools SET current_status=?, last_event_id=?, last_updated=? WHERE tool_id=?",
                (status_after, event_id, datetime.now(timezone.utc).isoformat(), tool_id),
            )
            self._conn.commit()
            logger.info(
                "[DBClient] log_event - tool_id=%s event_type=%s track=%s status=%s→%s",
                tool_id, event_type, track, status_before, status_after,
            )
            return event_id

    def log_system_event(
        self,
        event_type: str,
        severity: str,
        track: str | None = None,
        notes: str = "",
    ) -> None:
        with self._lock:
            if not self._conn:
                raise DBError("DBClient not connected — call connect() first")
            self._conn.execute(
                "INSERT INTO system_events (event_type, track, severity, notes) VALUES (?,?,?,?)",
                (event_type, track, severity, notes),
            )
            self._conn.commit()

    def _query_one(self, sql: str, params: tuple) -> sqlite3.Row | None:
        if not self._conn:
            raise DBError("DBClient not connected — call connect() first")
        return self._conn.execute(sql, params).fetchone()

    def _cache_fallback(self, tool_id: str) -> ToolStatus:
        elapsed = time.monotonic() - self._cache_loaded_at
        if elapsed > _DB_CACHE_TTL_SECONDS:
            raise DBCacheExpiredError(
                f"DB unreachable and cache expired ({elapsed:.0f}s > {_DB_CACHE_TTL_SECONDS}s) — all commands rejected (S-2)"
            )
        if tool_id not in self._cache:
            raise DBError(f"no cache entry for tool_id={tool_id}")
        logger.warning("[DBClient] using stale cache for tool_id=%s (%.0fs old)", tool_id, elapsed)
        return self._cache[tool_id]
