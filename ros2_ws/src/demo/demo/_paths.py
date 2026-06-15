"""레포 루트 기준 파일 경로 탐색 헬퍼.

`colcon install` 이후 `__file__`은
`install/demo/lib/python3.10/site-packages/demo/`로 옮겨가므로 상대 경로
(`parents[N]`)로는 `config/demo.yaml`이나 `unit_actions/`를 찾을 수 없다.

탐색 우선순위:
  1. 환경변수 `FINAL_PROJECT_ROOT`
  2. 현재 작업 디렉터리에서 위로
  3. 이 모듈 위치에서 위로

`unit_actions/` 디렉터리를 포함한 첫 조상을 레포 루트로 본다
(`demo_runner.py`의 기존 unit_actions 탐색과 동일한 마커).
"""

from __future__ import annotations

import os
from pathlib import Path

_ROOT_MARKER = "unit_actions"


def find_repo_root() -> Path:
    """`unit_actions/`를 포함한 레포 루트를 반환한다."""
    candidates: list[Path] = []

    env_root = os.environ.get("FINAL_PROJECT_ROOT")
    if env_root:
        candidates.append(Path(env_root))

    for start in (Path.cwd(), Path(__file__).resolve()):
        node = start
        while True:
            candidates.append(node)
            if node.parent == node:
                break
            node = node.parent

    for root in candidates:
        if (root / _ROOT_MARKER).is_dir():
            return root

    raise RuntimeError(
        "레포 루트를 찾지 못했습니다 (unit_actions/ 마커 없음). "
        "FINAL_PROJECT_ROOT 환경변수를 레포 루트로 설정하세요."
    )


def find_repo_file(relative: str) -> Path:
    """레포 루트 기준 상대 경로를 절대 경로로 변환한다 (존재 확인 포함)."""
    path = find_repo_root() / relative
    if not path.exists():
        raise FileNotFoundError(f"레포 파일을 찾지 못했습니다: {relative} (→ {path})")
    return path
