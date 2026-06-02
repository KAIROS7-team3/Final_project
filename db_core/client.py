"""Track 공통 SQLite DB client.

이 모듈은 초기 Phase에서 만든 낮은 수준의 DB client다. ROS2 node가 직접
의존하지 않아도 Track A/B 래퍼와 Track C Python 코드가 같은 DB 스키마를
사용할 수 있게 유지한다.

현재 운영 경로의 DB Gate는 `db_core.repository.ToolRepository`가 담당하지만,
이 client도 같은 안전 원칙을 따른다.
- DB 연결 실패 시 최근 캐시만 제한적으로 사용한다.
- 캐시가 오래됐거나 없는 경우에는 명령을 거부한다.
- 상태 변경과 event log 기록은 한 트랜잭션으로 묶는다.
"""

import logging
import sqlite3
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
    """`tools` 테이블의 현재 snapshot 한 행.

    `current_status`는 DB Gate의 핵심 입력이다. 로봇이 공구를 집거나 반납할지
    결정하기 전에 반드시 이 값을 확인해야 한다.
    """

    tool_id: str
    current_status: str
    home_slot_row: int
    home_slot_col: int
    last_updated: str


class DBError(Exception):
    """DB 계층에서 복구 불가능하거나 상위로 알려야 하는 오류."""

    pass


class DBCacheExpiredError(DBError):
    """DB 연결 실패 + TTL 초과 — 모든 명령 거부 (S-2)."""


class DBClient:
    """공구 상태 조회와 이벤트 로그 기록을 담당하는 SQLite client.

    `rclpy`를 import하지 않으므로 순수 Python 코드에서 재사용할 수 있다.
    연결은 `connect()`에서 명시적으로 열고, 조회 실패 시에는 `_cache_fallback()`이
    안전 TTL을 확인한다.
    """

    def __init__(self, db_path: str = "robot_arm.db") -> None:
        self._db_path = db_path
        self._conn: sqlite3.Connection | None = None
        self._cache: dict[str, ToolStatus] = {}
        self._cache_loaded_at: float = 0.0

    def connect(self) -> None:
        """SQLite 연결을 열고 스키마를 보장한다.

        WAL 모드는 `SCHEMA_SQL`에서 설정한다. WAL을 사용하면 읽기와 쓰기가
        섞이는 ROS2 bring-up 상황에서 lock 충돌을 줄일 수 있다.
        """

        self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(SCHEMA_SQL)
        self._conn.commit()
        logger.info("[DBClient] connected - path=%s", self._db_path)

    def disconnect(self) -> None:
        """열려 있는 SQLite 연결을 닫는다."""

        if self._conn:
            self._conn.close()
            self._conn = None

    def get_tool_status(self, tool_id: str) -> ToolStatus:
        """공구의 현재 상태를 조회하고, DB 장애 시 제한적으로 캐시를 사용한다.

        안전 규칙상 DB를 못 읽는 상태에서 임의로 동작을 계속하면 안 된다. 그래서
        캐시가 TTL 안에 있을 때만 같은 공구의 최근 상태를 반환하고, 그 외에는
        예외를 발생시켜 상위 로직이 명령을 거부하게 한다.
        """

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
        """DB Gate 판정을 `(가능 여부, 사유)` 형태로 반환한다.

        - fetch: 공구가 `in_slot`일 때만 허용한다.
        - return: 공구가 `staged`일 때만 허용한다.

        `out` 상태에서 바로 return을 허용하지 않는 이유는 v1.0에서 직접
        핸드오버를 금지하고 Staging Area를 거치게 하기 위함이다.
        """

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
        """이벤트 로그를 append하고 `tools` snapshot을 같은 transaction으로 갱신한다.

        `tool_events`는 추적성을 위한 append-only 기록이고, `tools`는 현재 상태를
        빠르게 읽기 위한 snapshot이다. 둘이 어긋나면 복구가 어려우므로 반드시
        같은 commit 안에서 처리한다.
        """

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
        """부팅, E-stop, DB fallback 같은 시스템 이벤트를 기록한다."""

        if not self._conn:
            raise DBError("DBClient not connected — call connect() first")
        self._conn.execute(
            "INSERT INTO system_events (event_type, track, severity, notes) VALUES (?,?,?,?)",
            (event_type, track, severity, notes),
        )
        self._conn.commit()

    def _query_one(self, sql: str, params: tuple) -> sqlite3.Row | None:
        """단일 row 조회 helper. 연결 누락을 명확한 DBError로 바꾼다."""

        if not self._conn:
            raise DBError("DBClient not connected — call connect() first")
        return self._conn.execute(sql, params).fetchone()

    def _cache_fallback(self, tool_id: str) -> ToolStatus:
        """DB 장애 시 캐시 사용 가능 여부를 판단한다.

        캐시는 편의 기능이 아니라 DB 일시 장애 중에도 같은 상태를 짧게 유지하기
        위한 안전 완충 장치다. TTL을 넘으면 상태가 바뀌었을 가능성을 배제할 수
        없으므로 모든 명령이 거부되도록 예외를 던진다.
        """

        elapsed = time.monotonic() - self._cache_loaded_at
        if elapsed > _DB_CACHE_TTL_SECONDS:
            raise DBCacheExpiredError(
                f"DB unreachable and cache expired ({elapsed:.0f}s > {_DB_CACHE_TTL_SECONDS}s) — all commands rejected (S-2)"
            )
        if tool_id not in self._cache:
            raise DBError(f"no cache entry for tool_id={tool_id}")
        logger.warning("[DBClient] using stale cache for tool_id=%s (%.0fs old)", tool_id, elapsed)
        return self._cache[tool_id]
