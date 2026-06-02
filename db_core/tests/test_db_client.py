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


class TestLogEvent:
    def test_happy_path(self, db):
        event_id = db.log_event(
            tool_id="wrench_8mm",
            event_type="fetch",
            track="A",
            status_before="in_slot",
            status_after="staged",
        )
        assert event_id is not None and event_id > 0
        status = db.get_tool_status("wrench_8mm")
        assert status.current_status == "staged"


class TestConcurrency:
    """Finding 8: a DBClient is shared across threads (check_same_thread=False).

    Without serialization the INSERT → UPDATE → commit sequence in log_event can
    interleave across threads — raising sqlite recursive-cursor/threading errors
    or leaving tools.last_event_id pointing at a stale event. The reentrant lock
    must make these transactions atomic. check_feasibility() also reacquires the
    lock via get_tool_status(), so a non-reentrant lock would deadlock these
    tests (a regression guard for RLock).
    """

    def test_concurrent_log_event_integrity(self, db):
        n_threads = 8
        per_thread = 25
        errors: list[Exception] = []
        # Maximize interleaving: every worker starts its loop at the same instant.
        start = threading.Barrier(n_threads)

        def worker(worker_id: int) -> None:
            track = "ABC"[worker_id % 3]  # schema CHECK: track IN ('A','B','C')
            try:
                start.wait()
                for i in range(per_thread):
                    db.log_event(
                        tool_id="wrench_8mm",
                        event_type="fetch",
                        track=track,
                        status_before="in_slot",
                        status_after="out",
                        notes=f"{worker_id}-{i}",
                    )
            except Exception as e:  # noqa: BLE001 - re-raised via assertion below
                errors.append(e)

        threads = [threading.Thread(target=worker, args=(t,)) for t in range(n_threads)]
        for th in threads:
            th.start()
        for th in threads:
            th.join()

        assert not errors, f"concurrent log_event raised: {errors}"

        total = db._conn.execute("SELECT COUNT(*) FROM tool_events").fetchone()[0]
        assert total == n_threads * per_thread
        distinct = db._conn.execute("SELECT COUNT(DISTINCT event_id) FROM tool_events").fetchone()[0]
        assert distinct == total, "duplicate/overwritten event_ids — transaction interleaved"

        # tools.last_event_id must reference a real, committed event row.
        last_event_id = db._conn.execute(
            "SELECT last_event_id FROM tools WHERE tool_id='wrench_8mm'"
        ).fetchone()[0]
        exists = db._conn.execute(
            "SELECT COUNT(*) FROM tool_events WHERE event_id=?", (last_event_id,)
        ).fetchone()[0]
        assert exists == 1, "last_event_id points at a non-existent event"

    def test_concurrent_reads_during_writes(self, db):
        """Readers must not crash (or deadlock) while a writer mutates the shared
        connection. check_feasibility nests get_tool_status under the lock."""
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
                db.log_event(
                    tool_id="wrench_8mm",
                    event_type="fetch",
                    track="A",
                    status_before="in_slot",
                    status_after="out",
                    notes=str(i),
                )
        finally:
            stop.set()
            for th in readers:
                th.join(timeout=5)

        assert not any(th.is_alive() for th in readers), "reader thread deadlocked"
        assert not errors, f"concurrent read/write raised: {errors}"
