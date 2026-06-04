import logging
import sqlite3
import threading
import time
from dataclasses import dataclass

from db_core.repository import ToolRepository
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
    """Track C의 in-process DB 어댑터 (B2-1).

    Track A/B가 ``db_service_node``를 통해 ``ToolRepository``에 접근하듯, Track C는
    ROS2를 우회해 이 클라이언트를 직접 import한다. DB Gate 판정·상태 전이·이벤트
    기록 같은 실제 로직은 더 이상 중복 구현하지 않고 내부 ``ToolRepository``(공유
    코어)에 위임한다. 따라서:

    * ``check_feasibility``/``log_event``/``log_system_event``는 ``ToolRepository``로
      포워드되어 Track A/B와 **동일한** 검증·전이 화이트리스트(S-8/S-9)를 거친다.
      특히 ``log_event``가 임의 전이를 그대로 쓰던 Track C 우회가 닫힌다.
    * ``rclpy`` 의존성 없음 — 순수 Python.

    스레드 안전성: 쓰기는 ``ToolRepository``가 호출마다 새 연결(WAL + busy_timeout)을
    열어 처리하므로 스레드가 공유하는 쓰기 연결이 없다. 본 클래스는 읽기 전용 보조
    연결(``self._conn``, ``get_tool_status`` 및 ``log_event`` 후 event_id 조회용)만
    유지하며, 그 접근만 ``self._lock``(RLock)으로 직렬화한다.
    """

    def __init__(self, db_path: str = "robot_arm.db") -> None:
        self._db_path = db_path
        self._conn: sqlite3.Connection | None = None
        # 모든 검증/쓰기 로직의 단일 출처. Track A/B의 db_service_node와 같은 코어.
        self._repo = ToolRepository(db_path)
        self._cache: dict[str, ToolStatus] = {}
        self._cache_loaded_at: float = 0.0
        # 읽기 보조 연결(_conn) 접근만 직렬화한다.
        self._lock = threading.RLock()

    def connect(self) -> None:
        with self._lock:
            self._conn = sqlite3.connect(self._db_path, check_same_thread=False)
            self._conn.row_factory = sqlite3.Row
            # ToolRepository의 _connect()와 같은 PRAGMA로 맞춰 같은 파일에서
            # journal mode가 어긋나지 않게 한다(WAL은 DB 파일 속성).
            self._conn.execute("PRAGMA foreign_keys = ON")
            self._conn.execute("PRAGMA journal_mode = WAL")
            self._conn.executescript(SCHEMA_SQL)
            self._conn.commit()
            logger.info("[DBClient] connected - path=%s", self._db_path)

    def disconnect(self) -> None:
        with self._lock:
            if self._conn:
                self._conn.close()
                self._conn = None

    def get_tool_status(self, tool_id: str) -> ToolStatus:
        """Return current status for tool_id. Falls back to cache on DB error (S-2).

        읽기 전용 편의 메서드 — slot 좌표까지 포함한 ToolStatus를 돌려준다
        (ToolRepository엔 대응 메서드가 없어 보조 연결로 직접 읽는다). 쓰기가
        아니므로 전이 화이트리스트 우회 위험은 없다.

        ⚠️ advisory 전용: S-2 권위 판정은 check_feasibility(ToolRepository에 위임,
        자체 캐시·TTL)가 한다. 본 메서드는 별도 보조 캐시(self._cache)를 쓰므로 DB
        outage 중 두 경로의 캐시 나이가 달라 상태가 어긋나 보일 수 있다(둘 다 TTL
        초과 시 fail-closed). 게이팅엔 check_feasibility를 쓰고 본 메서드는 표시용으로
        만 쓴다 (safety-reviewer C).
        """
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
        """DB Gate (S-2) 판정을 공유 ToolRepository 코어에 위임한다 (B2-1).

        Track C와 Track A/B의 db_service_node가 단일 판정 구현을 공유하게 된다.
        infeasible 판정 시 ToolRepository가 tool_events('rejected') 감사 한 줄도
        남긴다(db_service_node 경로와 동일). 반환은 (feasible, reason)이며 feasible
        일 때 reason은 빈 문자열로 맞춰 기존 DBClient 계약을 보존한다.

        쓰기 연결(_conn)을 건드리지 않으므로 _lock을 잡지 않는다.
        """
        result = self._repo.check_feasibility(intent, tool_id)
        if result.feasible:
            return True, ""
        return False, result.reason

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
        """상태 전이 이벤트를 공유 ToolRepository 코어로 기록하고 event_id를 돌려준다.

        ⚠️ status_after는 *목표* 상태로 해석된다. 쓰기는
        ToolRepository.update_tool_status로 라우팅되며, 거기서 실제 status_before를
        DB에서 읽어 S-8/S-9 전이 화이트리스트를 강제한다(B2-1). 불법 전이는 조용히
        쓰지 않고 DBError를 던진다 — DBClient가 임의 전이를 그대로 쓸 수 있던 Track C
        우회가 이로써 닫힌다. 호출자가 준 status_before/operator_id는 더 이상
        권위가 없으며(repository가 둘 다 소유) API 호환을 위해 시그니처에만 남는다.
        """
        if not self._conn:
            raise DBError("DBClient not connected — call connect() first")
        # 쓰기는 repository가 자체 연결로 처리하므로 _lock(보조 읽기 연결 _conn 보호용)
        # 밖에서 수행한다. busy_timeout만큼 대기할 수 있는 경합 쓰기가 동시 호출되는
        # get_tool_status 읽기를 막지 않게 하기 위함이다 (safety-reviewer D).
        result = self._repo.update_tool_status(
            tool_id=tool_id,
            new_status=status_after,
            event_type=event_type,
            track=track,
            notes=notes,
        )
        if not result.success:
            raise DBError(result.message)
        logger.info(
            "[DBClient] log_event - tool_id=%s event_type=%s track=%s status→%s",
            tool_id, event_type, track, status_after,
        )
        # update_tool_status가 tools.last_event_id를 방금 쓴 이벤트로 갱신했다.
        # 보조 연결(_conn) 접근만 _lock으로 직렬화한다.
        with self._lock:
            if not self._conn:
                raise DBError("DBClient not connected — call connect() first")
            row = self._conn.execute(
                "SELECT last_event_id FROM tools WHERE tool_id = ?",
                (tool_id,),
            ).fetchone()
        if row is None or row["last_event_id"] is None:
            raise DBError(f"event committed but last_event_id missing for {tool_id}")
        return int(row["last_event_id"])

    def log_system_event(
        self,
        event_type: str,
        severity: str,
        track: str | None = None,
        notes: str = "",
    ) -> None:
        """system_events 기록을 공유 ToolRepository 코어에 위임한다 (E-5).

        ToolRepository.log_system_event가 event_type/severity/track을 검증하므로,
        DBClient도 동일한 화이트리스트를 거친다(예전 DBClient는 검증 없이 썼다).
        실패 시 DBError를 던져 기존 None-반환 계약을 유지한다.
        """
        result = self._repo.log_system_event(
            event_type=event_type,
            severity=severity,
            track=track,
            notes=notes,
        )
        if not result.success:
            raise DBError(result.message)

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
