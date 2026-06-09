#!/usr/bin/env python3
"""데모용 DB 시드 스크립트.

demo_runner가 동작하려면 대상 공구가 DB에 `in_slot` 상태로 존재해야 한다.
이 스크립트는 SCHEMA_SQL로 스키마를 부트스트랩하고 공구 1건을 `in_slot`으로
등록한다 (idempotent — 이미 있으면 상태를 in_slot으로 되돌림).

`config/demo.yaml`의 tool_id / db_path를 기본값으로 읽고 CLI로 덮어쓸 수 있다.

사용:
    python3 scripts/seed_demo_db.py
    python3 scripts/seed_demo_db.py --tool-id socket_19mm --db ~/robot_tools.db
    python3 scripts/seed_demo_db.py --status staged          # 다른 초기 상태로 시드
"""

from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

# 레포 루트(scripts/의 부모)를 path에 추가해 db_core를 import한다.
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import yaml  # noqa: E402

from db_core.schema import SCHEMA_SQL  # noqa: E402

_VALID_STATUSES = ("in_slot", "out", "staged", "missing", "fod_alert")
_DEFAULT_CONFIG = _REPO_ROOT / "config" / "demo.yaml"


def load_demo_config(config_path: Path = _DEFAULT_CONFIG) -> dict:
    """config/demo.yaml의 demo 섹션을 반환한다 (없으면 빈 dict)."""
    if not config_path.exists():
        return {}
    with open(config_path) as f:
        return (yaml.safe_load(f) or {}).get("demo", {})


def seed_tool(
    db_path: str | Path,
    tool_id: str,
    *,
    display_name: str | None = None,
    status: str = "in_slot",
    home_slot_row: int = 0,
    home_slot_col: int = 0,
) -> None:
    """`db_path`에 스키마를 보장하고 공구 1건을 등록/갱신한다 (idempotent).

    Raises:
        ValueError: tool_id가 비었거나 status가 유효하지 않을 때.
    """
    tool_id = tool_id.strip()
    if not tool_id:
        raise ValueError("tool_id가 비어 있습니다")
    if status not in _VALID_STATUSES:
        raise ValueError(
            f"유효하지 않은 status: {status!r} (허용: {', '.join(_VALID_STATUSES)})"
        )

    db_path = Path(db_path).expanduser()
    db_path.parent.mkdir(parents=True, exist_ok=True)
    display_name = display_name or tool_id

    conn = sqlite3.connect(str(db_path))
    try:
        conn.executescript(SCHEMA_SQL)  # 스키마 + operator_01 부트스트랩 (idempotent)
        conn.execute(
            """
            INSERT INTO tools
                (tool_id, display_name, current_status, home_slot_row, home_slot_col)
            VALUES (?, ?, ?, ?, ?)
            ON CONFLICT(tool_id) DO UPDATE SET
                display_name   = excluded.display_name,
                current_status = excluded.current_status,
                home_slot_row  = excluded.home_slot_row,
                home_slot_col  = excluded.home_slot_col,
                last_updated   = strftime('%Y-%m-%dT%H:%M:%fZ', 'now')
            """,
            (tool_id, display_name, status, home_slot_row, home_slot_col),
        )
        conn.commit()
    finally:
        conn.close()


def _build_parser(defaults: dict) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="데모 DB 시드")
    parser.add_argument(
        "--tool-id", default=defaults.get("tool_id", "socket_19mm"),
        help="등록할 공구 ID (기본: config/demo.yaml)",
    )
    parser.add_argument(
        "--db", dest="db_path", default=defaults.get("db_path", "~/robot_tools.db"),
        help="SQLite 경로 (기본: config/demo.yaml)",
    )
    parser.add_argument(
        "--status", default="in_slot", choices=_VALID_STATUSES,
        help="초기 상태 (기본: in_slot)",
    )
    parser.add_argument("--display-name", default=None, help="표시 이름 (기본: tool_id)")
    parser.add_argument("--row", type=int, default=0, help="home_slot_row (기본: 0)")
    parser.add_argument("--col", type=int, default=0, help="home_slot_col (기본: 0)")
    return parser


def main(argv: list[str] | None = None) -> int:
    defaults = load_demo_config()
    args = _build_parser(defaults).parse_args(argv)
    seed_tool(
        args.db_path,
        args.tool_id,
        display_name=args.display_name,
        status=args.status,
        home_slot_row=args.row,
        home_slot_col=args.col,
    )
    resolved = Path(args.db_path).expanduser()
    print(f"[seed] {args.tool_id} → {args.status}  (db: {resolved})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
