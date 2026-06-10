import threading

import pytest

from db_core.client import DBCacheExpiredError, DBClient, DBError


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


class TestCheckDrawerFeasibility:
    """issue #44 Option B — 서랍(layer) 단위 DB Gate."""

    def test_open_empty_layer_ok(self, db):
        feasible, reason = db.check_drawer_feasibility("open", 1)
        assert feasible is True
        assert reason == ""

    def test_open_blocked_by_fod_alert(self, db):
        db._conn.execute(
            "UPDATE tools SET current_status='fod_alert' WHERE tool_id='wrench_8mm'"
        )
        db._conn.commit()
        feasible, reason = db.check_drawer_feasibility("open", 0)
        assert feasible is False
        assert "FOD" in reason
        assert "wrench_8mm" in reason

    def test_open_other_layer_unaffected(self, db):
        db._conn.execute(
            "UPDATE tools SET current_status='fod_alert' WHERE tool_id='wrench_8mm'"
        )
        db._conn.commit()
        # wrench_8mm은 layer 0; layer 1은 비어있으므로 통과
        feasible, _ = db.check_drawer_feasibility("open", 1)
        assert feasible is True

    def test_close_in_slot_ok(self, db):
        # wrench_8mm이 in_slot이면 닫기 허용
        feasible, reason = db.check_drawer_feasibility("close", 0)
        assert feasible is True
        assert reason == ""

    def test_close_blocked_tool_out(self, db):
        db._conn.execute(
            "UPDATE tools SET current_status='out' WHERE tool_id='wrench_8mm'"
        )
        db._conn.commit()
        feasible, reason = db.check_drawer_feasibility("close", 0)
        assert feasible is False
        assert "wrench_8mm" in reason
        assert "out" in reason

    def test_close_blocked_tool_staged(self, db):
        db._conn.execute(
            "UPDATE tools SET current_status='staged' WHERE tool_id='wrench_8mm'"
        )
        db._conn.commit()
        feasible, reason = db.check_drawer_feasibility("close", 0)
        assert feasible is False
        assert "staged" in reason

    def test_invalid_intent_rejected(self, db):
        feasible, reason = db.check_drawer_feasibility("fetch", 0)
        assert feasible is False
        assert "unsupported" in reason


# 한 공구를 합법적으로 반복 순환시키는 fetch/return 전이열 (S-6 2단계 모델).
#   in_slot --fetch--> out --fetch--> staged --return--> out --return--> in_slot ...
# status_before는 무시되고(repository가 DB에서 직접 읽음) status_after만 목표로 쓰인다.
_LEGAL_CYCLE = [
    ("fetch", "out"),       # in_slot -> out  (fetch pick)
    ("fetch", "staged"),    # out -> staged   (fetch place)
    ("return", "out"),      # staged -> out   (return pick)
    ("return", "in_slot"),  # out -> in_slot  (return place)
]


class TestLogEvent:
    def test_happy_path(self, db):
        # B2-1: log_event는 이제 ToolRepository.update_tool_status로 라우팅되어
        # 전이 화이트리스트를 강제한다. in_slot->out(fetch pick)은 합법 전이.
        event_id = db.log_event(
            tool_id="wrench_8mm",
            event_type="fetch",
            track="A",
            status_before="in_slot",
            status_after="out",
        )
        assert event_id is not None and event_id > 0
        status = db.get_tool_status("wrench_8mm")
        assert status.current_status == "out"

    def test_illegal_transition_raises(self, db):
        # B2-1 핵심: 우회 차단. in_slot->staged는 pick/place 한 단계를 건너뛰는
        # 불법 전이라 DBClient.log_event가 조용히 쓰지 않고 DBError를 던진다.
        with pytest.raises(DBError):
            db.log_event(
                tool_id="wrench_8mm",
                event_type="fetch",
                track="A",
                status_before="in_slot",
                status_after="staged",
            )
        # 상태는 그대로여야 한다.
        assert db.get_tool_status("wrench_8mm").current_status == "in_slot"


class TestConcurrency:
    """B2-1: 쓰기는 더 이상 공유 연결을 RLock으로 보호하지 않는다 — ToolRepository가
    호출마다 새 연결(WAL + busy_timeout)을 열어 처리하므로, 동시성 안전은
    SQLite 단일-writer 직렬화로 제공된다. 아래 테스트는 그 모델에서도 event_id와
    last_event_id 무결성이 유지되고 reader/writer가 교착하지 않음을 회귀로 지킨다.
    """

    def test_concurrent_writes_distinct_tools_integrity(self, db):
        # 스레드마다 자기 공구를 합법 fetch/return 순환으로 구동한다(같은 공구를
        # 여러 스레드가 동시에 전이시키면 상태머신상 경쟁이라, 공구를 분리한다).
        n_tools = 8
        steps = 12
        tool_ids = [f"tool_{i}" for i in range(n_tools)]
        for tid in tool_ids:
            db._conn.execute(
                "INSERT INTO tools (tool_id, display_name, current_status,"
                " home_slot_row, home_slot_col) VALUES (?, ?, 'in_slot', 0, 0)",
                (tid, tid),
            )
        db._conn.commit()

        errors: list[Exception] = []
        start = threading.Barrier(n_tools)

        def worker(tid: str) -> None:
            try:
                start.wait()
                for i in range(steps):
                    event_type, target = _LEGAL_CYCLE[i % 4]
                    db.log_event(
                        tool_id=tid,
                        event_type=event_type,
                        track="A",
                        status_before="",
                        status_after=target,
                        notes=f"{tid}-{i}",
                    )
            except Exception as e:  # noqa: BLE001 - re-raised via assertion below
                errors.append(e)

        threads = [threading.Thread(target=worker, args=(tid,)) for tid in tool_ids]
        for th in threads:
            th.start()
        for th in threads:
            th.join()

        assert not errors, f"concurrent log_event raised: {errors}"

        total = db._conn.execute("SELECT COUNT(*) FROM tool_events").fetchone()[0]
        assert total == n_tools * steps
        distinct = db._conn.execute("SELECT COUNT(DISTINCT event_id) FROM tool_events").fetchone()[0]
        assert distinct == total, "duplicate/overwritten event_ids — transaction interleaved"

        # 각 공구의 last_event_id는 실재하는 커밋된 이벤트를 가리켜야 한다.
        for tid in tool_ids:
            last_event_id = db._conn.execute(
                "SELECT last_event_id FROM tools WHERE tool_id=?", (tid,)
            ).fetchone()[0]
            exists = db._conn.execute(
                "SELECT COUNT(*) FROM tool_events WHERE event_id=?", (last_event_id,)
            ).fetchone()[0]
            assert exists == 1, f"last_event_id for {tid} points at a non-existent event"

    def test_concurrent_same_tool_writes_serialize_no_lock_error(self, db):
        # B2-1 BEGIN IMMEDIATE 회귀: 여러 스레드가 같은 공구에 같은 전이를
        # 동시에 시도하면, deferred BEGIN이었을 땐 SELECT 스냅샷 업그레이드가
        # SQLITE_BUSY_SNAPSHOT("database is locked")로 실패했다. IMMEDIATE는 쓰기
        # 잠금을 선점해 직렬화하므로: 정확히 하나가 성공(in_slot->out)하고 나머지는
        # 깨끗한 전이 검증 거부(DBError)를 받는다 — 절대 "database is locked"가 아니다.
        n_threads = 8
        results: list[str] = []
        errors: list[Exception] = []
        start = threading.Barrier(n_threads)

        def worker() -> None:
            try:
                start.wait()
                db.log_event(
                    tool_id="wrench_8mm",
                    event_type="fetch",
                    track="A",
                    status_before="in_slot",
                    status_after="out",
                )
                results.append("ok")
            except DBError as e:  # 기대: 전이 검증 거부
                errors.append(e)

        threads = [threading.Thread(target=worker) for _ in range(n_threads)]
        for th in threads:
            th.start()
        for th in threads:
            th.join()

        # 정확히 한 스레드만 성공, 나머지는 DBError.
        assert results.count("ok") == 1, f"expected exactly 1 success, got {results}"
        assert len(errors) == n_threads - 1
        # 어떤 실패도 SQLite 잠금/스냅샷 오류여서는 안 된다 — 전이 검증 거부여야 한다.
        for e in errors:
            msg = str(e).lower()
            assert "locked" not in msg and "snapshot" not in msg, f"lock/snapshot error leaked: {e}"
        assert db.get_tool_status("wrench_8mm").current_status == "out"

    def test_concurrent_reads_during_writes(self, db):
        """단일 writer가 합법 순환으로 wrench_8mm을 전이시키는 동안 reader들이
        crash/deadlock 없이 읽어야 한다. reader는 _conn(get_tool_status)과
        repository 연결(check_feasibility)을 동시에 사용한다."""
        stop = threading.Event()
        errors: list[Exception] = []

        def reader() -> None:
            try:
                while not stop.is_set():
                    db.get_tool_status("wrench_8mm")
                    db.check_feasibility("fetch", "wrench_8mm")
            except Exception as e:  # noqa: BLE001 - re-raised via assertion below
                errors.append(e)

        readers = [threading.Thread(target=reader) for _ in range(4)]
        for th in readers:
            th.start()
        try:
            for i in range(100):
                event_type, target = _LEGAL_CYCLE[i % 4]
                db.log_event(
                    tool_id="wrench_8mm",
                    event_type=event_type,
                    track="A",
                    status_before="",
                    status_after=target,
                    notes=str(i),
                )
        finally:
            stop.set()
            for th in readers:
                th.join(timeout=5)

        assert not any(th.is_alive() for th in readers), "reader thread deadlocked"
        assert not errors, f"concurrent read/write raised: {errors}"
