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
