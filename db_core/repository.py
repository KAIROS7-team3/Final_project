"""SQLite 기반 공구 DB 접근 계층.

ROS2 node는 이 repository를 통해서만 feasibility 확인과 상태 갱신을 수행한다.
여기서는 DB Gate, 이벤트 로그 append, FOD timeout 전이를 한 곳에서 관리한다.
"""

from __future__ import annotations

import logging
import sqlite3
import time
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

from db_core.schema import SCHEMA_SQL

logger = logging.getLogger(__name__)

# DB Gate가 공식적으로 받아들이는 값만 상수로 고정한다.
VALID_INTENTS = frozenset({"fetch", "return"})
# 서랍(layer) 단위 DB Gate — 공구 단위 VALID_INTENTS와 분리 (issue #44, Option B).
VALID_DRAWER_INTENTS = frozenset({"open", "close"})
VALID_STATUSES = frozenset({"in_slot", "out", "staged", "missing", "fod_alert"})
VALID_EVENT_TYPES = frozenset(
    {"fetch", "return", "rejected", "error", "timeout", "fod_alert", "reconciled"}
)
# system_events 채널(운영자/PLC 가시성)이 허용하는 값. schema.py의 CHECK와 일치해야 한다.
VALID_SYSTEM_EVENT_TYPES = frozenset(
    {
        "boot",
        "boot_complete",
        "reconciliation_mismatch",
        "estop",
        "estop_reset",
        "db_cache_fallback",
        "db_cache_expired",
        "calibration",
        "fod_alert",
    }
)
VALID_SEVERITIES = frozenset({"info", "warning", "error", "critical"})
VALID_TRACKS = frozenset({"A", "B", "C"})
# 외부 노드가 보내는 거부 사유(notes) 자유형 문자열의 상한 — 감사 로그 비대화 방지.
MAX_REJECTION_NOTES_LEN = 1024
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
# missing/fod_alert 진입(out|staged -> missing[event_type=timeout],
# missing -> fod_alert[event_type=fod_alert])은 FOD monitor가
# mark_checkout_timeouts에서 track=None으로 직접 기록하므로 외부 호출
# 화이트리스트에는 포함하지 않는다 (S-8).
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

    def check_drawer_feasibility(self, intent: str, layer_id: int) -> FeasibilityResult:
        """서랍(layer) 단위 DB Gate (issue #44).

        - open:  drawers 테이블에 해당 layer가 이미 열려 있으면 (False, "already_open") 반환.
                 호출자는 reason == "already_open"일 때 open 동작을 생략하고 진행한다.
        - close: 항상 (True, "ok") — DB 레벨 차단 없음. 닫기 완료 후 update_drawer_state 호출.
        """
        intent = intent.strip()
        if intent not in VALID_DRAWER_INTENTS:
            return FeasibilityResult(False, f"unsupported drawer intent: {intent}")

        if intent == "close":
            return FeasibilityResult(True, "ok")

        # open: drawers 테이블에서 현재 열림 여부 확인
        try:
            with self._connect() as conn:
                row = conn.execute(
                    "SELECT is_open FROM drawers WHERE layer_id = ?",
                    (layer_id,),
                ).fetchone()
        except SchemaValidationError as exc:
            return FeasibilityResult(False, f"database schema error: {exc}")
        except sqlite3.Error as exc:
            return FeasibilityResult(False, f"database unavailable: {exc}")

        if row and row["is_open"]:
            return FeasibilityResult(False, "already_open")
        return FeasibilityResult(True, "ok")

    def update_drawer_state(self, layer_id: int, intent: str) -> UpdateResult:
        """서랍 열림/닫힘 상태를 drawers 테이블에 기록한다.

        open/close 동작 완료 후 motion이 호출한다.
        layer_id 행이 없으면 자동 생성(INSERT OR REPLACE).
        """
        intent = intent.strip()
        if intent not in VALID_DRAWER_INTENTS:
            return UpdateResult(False, f"unsupported drawer intent: {intent}")

        is_open = 1 if intent == "open" else 0
        now = datetime.now(timezone.utc).isoformat()
        try:
            with self._connect() as conn:
                conn.execute("BEGIN IMMEDIATE")
                conn.execute(
                    "INSERT OR REPLACE INTO drawers (layer_id, is_open, last_updated)"
                    " VALUES (?, ?, ?)",
                    (layer_id, is_open, now),
                )
                conn.commit()
        except sqlite3.Error as exc:
            return UpdateResult(False, f"failed to update drawer state: {exc}")
        return UpdateResult(True, "ok")

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
                # BEGIN IMMEDIATE: 쓰기 잠금을 트랜잭션 시작 시점에 잡는다. deferred
                # BEGIN이면 SELECT가 읽기 스냅샷만 잡은 사이 다른 writer가 커밋해
                # 이후 INSERT 업그레이드가 SQLITE_BUSY_SNAPSHOT("database is locked")로
                # 실패한다(busy_timeout으로 해소 안 됨). 동시 writer(fod_monitor·
                # db_service·Track C)를 busy_timeout 대기로 직렬화한다 (B2-1).
                conn.execute("BEGIN IMMEDIATE")
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
            conn.execute("BEGIN IMMEDIATE")
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
                # out/staged -> missing 은 아직 경보 전(timeout) 단계이고,
                # missing -> fod_alert 만 실제 FOD 경보다 (Finding 7). 이전에는
                # 두 전이 모두 event_type='fod_alert'로 기록돼 로그에서 구분 불가했다.
                event_type = "timeout" if new_status == "missing" else "fod_alert"
                event_id = self._insert_tool_event(
                    conn=conn,
                    tool_id=tool_id,
                    event_type=event_type,
                    track=None,
                    status_before=current_status,
                    status_after=new_status,
                    notes="FOD timeout transition",
                    timestamp=now,
                )
                self._update_tool_row(conn, tool_id, new_status, event_id, now)
                updates.append(FodUpdate(tool_id, current_status, new_status))
            conn.commit()

        # E-5: 운영자/PLC 가시 채널(system_events) 기록은 상태 전이가 "커밋된 뒤"
        # 별도 트랜잭션으로 한다. 안전 우선순위상 FOD 경보(missing→fod_alert) 상태
        # 전이는 가시성 기록의 성공 여부와 무관하게 반드시 커밋돼야 한다. 같은
        # 트랜잭션에 두면 system_events insert 실패가 전이까지 롤백시켜, 경보돼야 할
        # 공구가 missing에 머무는 fail-open이 된다(safety-reviewer HIGH). 따라서
        # 가시성 쓰기 실패는 로그로만 남기고 이미 커밋된 전이는 보존한다.
        for update in updates:
            if update.new_status != "fod_alert":
                continue
            result = self.log_system_event(
                event_type="fod_alert",
                severity="critical",
                track=None,
                notes=(
                    f"FOD alert: {update.tool_id} "
                    f"{update.previous_status} -> fod_alert"
                ),
            )
            if not result.success:
                logger.error(
                    "[ToolRepository] FOD escalation committed but system_events "
                    "write failed for tool_id=%s: %s",
                    update.tool_id,
                    result.message,
                )

        return updates

    def log_rejection(
        self,
        tool_id: str,
        reason: str,
        track: str | None = None,
    ) -> UpdateResult:
        """Persist a rejected command as a tool_events('rejected') row without
        changing current_status (B1-1).

        DB Gate(S-2) 거부는 내부 경로 _record_rejection(check_feasibility 안에서
        호출)이 이미 기록한다. 본 메서드는 DB Gate에 도달하기도 전에 드롭된 명령을
        외부 ROS 노드가 기록하기 위한 공개 경로다 — 특히 orchestrator의 S-7
        is_moving 가드. status_after 컬럼이 NOT NULL이라 현재 상태를 읽어
        status_before/after에 그대로 넣고 current_status는 바꾸지 않는다.
        예외를 던지지 않고 UpdateResult(success, message)를 반환한다.
        """

        if not tool_id:
            return UpdateResult(False, "tool_id is required")
        # orchestrator는 "트랙 없음"을 빈 문자열로 보낸다 — None으로 정규화한 뒤
        # 비어 있지 않은 값만 화이트리스트로 검증한다.
        track = track or None
        if track is not None and track not in VALID_TRACKS:
            return UpdateResult(False, f"unsupported track: {track}")
        # 외부에서 들어오는 자유형 문자열이라 감사 로그가 비대해지지 않게 길이를
        # 제한한다(기록은 보존하되 잘라낸다).
        reason = (reason or "")[:MAX_REJECTION_NOTES_LEN]

        now = datetime.now(timezone.utc).isoformat()
        try:
            with self._connect() as conn:
                # 존재 확인을 트랜잭션 밖(SELECT는 트랜잭션을 열지 않음)에서 먼저
                # 하고, 쓸 게 확정된 뒤에만 BEGIN한다 — 조기 반환 시 롤백이 필요 없다.
                row = conn.execute(
                    "SELECT current_status FROM tools WHERE tool_id = ?",
                    (tool_id,),
                ).fetchone()
                if row is None:
                    return UpdateResult(False, f"unknown tool_id: {tool_id}")
                current_status = str(row["current_status"])
                conn.execute("BEGIN IMMEDIATE")
                self._append_rejection(conn, tool_id, current_status, reason, track, now)
                conn.commit()
        except sqlite3.Error as exc:
            return UpdateResult(False, f"failed to log rejection: {exc}")
        return UpdateResult(True, "logged")

    def log_system_event(
        self,
        event_type: str,
        severity: str,
        track: str | None = None,
        notes: str = "",
    ) -> UpdateResult:
        """Append a system-level event to the operator/PLC visibility channel (E-5).

        tools/tool_events 상태와 무관한 시스템 사건(부팅, e-stop, FOD 경보 등)을
        기록한다. 정상 상태 전이는 update_tool_status를 쓰고, FOD 경보는
        mark_checkout_timeouts가 상태 전이를 커밋한 뒤 본 메서드를 별도 트랜잭션으로
        호출한다(전이가 가시성 쓰기에 의존하지 않도록). 이 메서드는 그 외 독립
        시스템 이벤트용 공개 경로이기도 하다 (DBClient.log_system_event와 동등).
        """

        event_type = event_type.strip()
        severity = severity.strip()
        if event_type not in VALID_SYSTEM_EVENT_TYPES:
            return UpdateResult(False, f"unsupported system event_type: {event_type}")
        if severity not in VALID_SEVERITIES:
            return UpdateResult(False, f"unsupported severity: {severity}")
        if track is not None and track not in VALID_TRACKS:
            return UpdateResult(False, f"unsupported track: {track}")

        now = datetime.now(timezone.utc).isoformat()
        try:
            with self._connect() as conn:
                conn.execute("BEGIN IMMEDIATE")
                self._insert_system_event(
                    conn=conn,
                    event_type=event_type,
                    severity=severity,
                    track=track,
                    notes=notes,
                    timestamp=now,
                )
                conn.commit()
        except sqlite3.Error as exc:
            return UpdateResult(False, f"database error: {exc}")
        return UpdateResult(True, "logged")

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
                conn.execute("BEGIN IMMEDIATE")
                self._append_rejection(conn, tool_id, current_status, reason, None, now)
                conn.commit()
        except sqlite3.Error as exc:
            return f"failed to log rejected event: {exc}"
        return ""

    def _append_rejection(
        self,
        conn: sqlite3.Connection,
        tool_id: str,
        current_status: str,
        reason: str,
        track: str | None,
        timestamp: str,
    ) -> None:
        """Append one rejected event (status unchanged) and bump last_event_id.

        거부 한 줄을 tool_events에 남기는 단일 로직 — DB Gate(S-2) 내부 경로
        (_record_rejection)와 외부 공개 경로(log_rejection, S-7)가 공유한다(B2-1).
        호출자가 connect()/BEGIN/commit 트랜잭션 경계를 소유한다. status_after는
        NOT NULL이라 current_status를 status_before/after에 그대로 넣어 상태를
        바꾸지 않는다.
        """

        event_id = self._insert_tool_event(
            conn=conn,
            tool_id=tool_id,
            event_type="rejected",
            track=track,
            status_before=current_status,
            status_after=current_status,
            notes=reason,
            timestamp=timestamp,
        )
        if event_id is not None:
            self._update_last_event(conn, tool_id, event_id)

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
    def _insert_system_event(
        conn: sqlite3.Connection,
        event_type: str,
        severity: str,
        track: str | None,
        notes: str,
        timestamp: str,
    ) -> None:
        """Insert one system_events row (operator/PLC visibility channel)."""

        conn.execute(
            "INSERT INTO system_events (event_type, track, severity, notes, timestamp)"
            " VALUES (?, ?, ?, ?, ?)",
            (event_type, track, severity, notes, timestamp),
        )

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
