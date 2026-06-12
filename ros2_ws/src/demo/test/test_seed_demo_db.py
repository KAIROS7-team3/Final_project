"""scripts/seed_demo_db.seed_tool 단위 테스트 (ROS 비의존).

레포 루트의 db_core / scripts를 import 경로에 추가해 검증한다.
"""

from __future__ import annotations

import os
import sqlite3
import sys

import pytest


def _repo_root() -> str:
    """unit_actions/ 마커를 가진 조상을 레포 루트로 본다."""
    node = os.path.abspath(__file__)
    while True:
        if os.path.isdir(os.path.join(node, "unit_actions")):
            return node
        parent = os.path.dirname(node)
        if parent == node:
            raise RuntimeError("레포 루트(unit_actions 마커)를 찾지 못했습니다")
        node = parent


_ROOT = _repo_root()
for _p in (_ROOT, os.path.join(_ROOT, "scripts")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from db_core.repository import ToolRepository  # noqa: E402
from seed_demo_db import seed_tool  # noqa: E402


# ── happy path ────────────────────────────────────────────────────────────────

def test_seed_creates_tool_in_slot(tmp_path):
    db = tmp_path / "demo.db"
    seed_tool(db, "socket_19mm")

    # DB Gate가 fetch를 허용해야 한다 (in_slot 상태).
    repo = ToolRepository(db)
    result = repo.check_feasibility("fetch", "socket_19mm")
    assert result.feasible, result.reason


def test_seed_is_idempotent_and_resets_status(tmp_path):
    db = tmp_path / "demo.db"
    # 먼저 out 상태로 시드한 뒤 다시 in_slot으로 시드 → 상태가 복원돼야 한다.
    seed_tool(db, "socket_19mm", status="out")
    seed_tool(db, "socket_19mm", status="in_slot")

    with sqlite3.connect(str(db)) as conn:
        rows = conn.execute(
            "SELECT current_status FROM tools WHERE tool_id = ?", ("socket_19mm",)
        ).fetchall()
    assert rows == [("in_slot",)]  # 중복 행 없이 1건, 상태는 in_slot


def test_seed_custom_slot_and_display_name(tmp_path):
    db = tmp_path / "demo.db"
    seed_tool(db, "wrench_8mm", display_name="8mm 렌치", home_slot_row=1, home_slot_col=2)

    with sqlite3.connect(str(db)) as conn:
        row = conn.execute(
            "SELECT display_name, home_slot_row, home_slot_col "
            "FROM tools WHERE tool_id = ?",
            ("wrench_8mm",),
        ).fetchone()
    assert row == ("8mm 렌치", 1, 2)


# ── failure path ──────────────────────────────────────────────────────────────

def test_empty_tool_id_raises(tmp_path):
    with pytest.raises(ValueError, match="tool_id"):
        seed_tool(tmp_path / "demo.db", "  ")


def test_invalid_status_raises(tmp_path):
    with pytest.raises(ValueError, match="status"):
        seed_tool(tmp_path / "demo.db", "socket_19mm", status="broken")
