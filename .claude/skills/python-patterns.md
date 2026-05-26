---
name: python-patterns
description: >
  현대 Python(3.10+) 작성 패턴 — 타입 힌트, dataclass/TypedDict, f-string,
  context manager, 제너레이터, async/await, 그리고 흔한 안티패턴 회피.
  Pythonic한 코드 작성, 타입 안전성, 모듈 설계 시 활성화.
when_to_use: >
  Python 모듈/클래스/함수 작성, 타입 힌트 추가, 데이터 구조 설계,
  레거시 코드 리팩토링 시.
  (코드 리뷰 체크리스트는 code-review-checklist 스킬 전담)
---

# Modern Python Patterns (3.10+)

> 프로젝트 표준: Python 3.10+ (Ubuntu 22.04 기본 3.10). 룰: [`.claude/rules/engineering.md`](../rules/engineering.md) E-7.

## 1. 타입 힌트 (필수)

모든 public 함수는 인자 + 반환값에 타입 힌트 작성.

```python
# ❌ 타입 정보 없음
def get_tool_status(tool_id):
    ...

# ✅ 명시적
def get_tool_status(tool_id: str) -> ToolStatus:
    ...

# ✅ Optional / Union — 3.10+ 표기
def find_tool(name: str) -> ToolInfo | None:
    ...

# ✅ Generic 컬렉션 — 3.9+ 표기 (typing.List 불필요)
def list_tools(category: str) -> list[ToolInfo]:
    ...

# ✅ Callable / Iterable
from collections.abc import Callable, Iterable

def apply(fn: Callable[[int], int], items: Iterable[int]) -> list[int]:
    return [fn(x) for x in items]
```

### 타입 검사 도구
```bash
pip install mypy ruff
mypy --strict <module>/
ruff check <module>/   # ruff는 lint + format 통합
```

## 2. dataclass — Value Objects

immutable value object는 `@dataclass(frozen=True)` 사용.

```python
from dataclasses import dataclass, field

@dataclass(frozen=True)
class GraspPose:
    position: tuple[float, float, float]      # (x, y, z) in m, robot_base_link
    quaternion: tuple[float, float, float, float]  # (x, y, z, w)
    width: float                              # 그리퍼 폭 (m)

@dataclass
class TrajectoryResult:
    success: bool
    duration_s: float
    waypoints: list[tuple[float, ...]] = field(default_factory=list)
    error_msg: str | None = None
```

- `frozen=True` → 해시 가능, 의도치 않은 수정 방지
- mutable default는 `field(default_factory=...)` 사용 (직접 `=[]` 금지)

## 3. TypedDict — 외부 API JSON 등

JSON 응답 형식을 타입화할 때 dataclass보다 가벼움.

```python
from typing import TypedDict

class IntentResult(TypedDict):
    action: str       # 'fetch' | 'return'
    tool_id: str
    feasible: bool
    reason: str

def parse_gemma_response(raw: dict) -> IntentResult:
    return IntentResult(
        action=raw["action"],
        tool_id=raw["tool_id"],
        feasible=raw["feasible"],
        reason=raw.get("reason", ""),
    )
```

- 더 엄격한 검증이 필요하면 `pydantic.BaseModel` 사용

## 4. Enum / Literal — 상수 집합

```python
from enum import StrEnum  # 3.11+
from typing import Literal

class ToolStatus(StrEnum):
    IN_SLOT = "in_slot"
    OUT = "out"
    STAGED = "staged"
    MISSING = "missing"
    FOD_ALERT = "fod_alert"

# 또는 Literal (간단한 경우)
Track = Literal["A", "B", "C"]

def log_event(tool_id: str, track: Track, status: ToolStatus) -> None:
    ...
```

## 5. f-string (3.6+)

문자열 포매팅은 f-string. `.format()`, `%`는 자제.

```python
# ✅
logger.info(f"[grasp] tool_id={tool_id} pose=({x:.3f}, {y:.3f}, {z:.3f})")

# ❌ 가독성 낮음
logger.info("[grasp] tool_id=%s pose=(%.3f, %.3f, %.3f)" % (tool_id, x, y, z))
```

> ⚠️ 예외: `logging` 사용 시 lazy 평가를 위해 `%` 스타일 권장 (logger가 메시지 포맷 비용을 지연시킴).
> ```python
> logger.debug("expensive=%s", expensive_repr())  # debug 비활성 시 호출 안 됨
> ```

## 6. Context Manager — 자원 관리

```python
# ✅ context manager로 자원 정리 보장
with open("config/toolbox.yaml") as f:
    tools = yaml.safe_load(f)

# 커스텀 context manager
from contextlib import contextmanager

@contextmanager
def doosan_session(host: str):
    arm = DooSanArm(host=host)
    arm.connect()
    try:
        yield arm
    finally:
        arm.disconnect()  # 예외 발생해도 disconnect 보장

# 사용
with doosan_session("192.168.137.100") as arm:
    arm.movej(home_pose)
```

## 7. 제너레이터 — Lazy iteration

대용량 데이터는 list 대신 generator.

```python
# ❌ 메모리 폭증 가능
def all_demos() -> list[Demo]:
    return [load_demo(p) for p in demo_paths]

# ✅ lazy
def all_demos() -> Iterator[Demo]:
    for p in demo_paths:
        yield load_demo(p)

# 사용
for demo in all_demos():
    process(demo)  # 한 번에 하나씩만 메모리에 로드
```

## 8. async/await — I/O 동시성

Track C에서 STT 대기 + 카메라 읽기 등 I/O 동시성에 유용.

```python
import asyncio

async def listen_audio() -> str:
    return await whisper.transcribe()

async def main():
    while True:
        text = await listen_audio()           # I/O 대기, CPU 점유 없음
        intent, tool_id = parse_intent(text)  # 동기 (빠름)
        await process_command(intent, tool_id)

asyncio.run(main())
```

- **CPU bound는 asyncio 효과 없음**. VLA 추론 같은 GPU 작업은 동기 호출 또는 별도 프로세스 사용
- 동시성 필요한 작업: `asyncio.gather(coro1, coro2)`

## 9. pathlib — 경로 처리

`os.path` 대신 `pathlib.Path`.

```python
from pathlib import Path

config_dir = Path(__file__).parent.parent / "config"
tools_file = config_dir / "toolbox.yaml"

if tools_file.exists():
    text = tools_file.read_text()

# glob
for yaml_file in config_dir.glob("*.yaml"):
    process(yaml_file)
```

## 10. structural pattern matching (3.10+)

if/elif/else 사슬보다 명확.

```python
def handle_event(event: dict) -> None:
    match event:
        case {"type": "fetch", "tool_id": str(tid)}:
            handle_fetch(tid)
        case {"type": "return", "tool_id": str(tid)}:
            handle_return(tid)
        case {"type": "error", "code": int(code)}:
            handle_error(code)
        case _:
            logger.warning(f"[handler] unknown event: {event}")
```

## 11. 안티패턴 — 피할 것

### ❌ Mutable default argument
```python
def add_item(item: str, items: list = []):  # 위험: 모든 호출이 같은 list 공유
    items.append(item)
    return items
```
✅ `None` 가드
```python
def add_item(item: str, items: list | None = None) -> list:
    if items is None:
        items = []
    items.append(item)
    return items
```

### ❌ bare except
```python
try:
    arm.execute(traj)
except:        # 위험: KeyboardInterrupt, SystemExit까지 잡음
    pass
```
✅ 명시적
```python
try:
    arm.execute(traj)
except (DoosanSDKError, TimeoutError) as e:
    logger.error("[motion] execute failed: %s", e)
    raise
```

### ❌ silent fallback
```python
def get_pose(tool_id: str) -> Pose:
    try:
        return db.lookup(tool_id)
    except Exception:
        return Pose()   # 빈 Pose 반환 — 호출자가 실패를 인지하지 못함
```
✅ 명시적 실패
```python
def get_pose(tool_id: str) -> Pose | None:
    try:
        return db.lookup(tool_id)
    except DBError:
        return None     # 호출자가 None 체크 강제됨
```

### ❌ god class
- 단일 클래스에 모든 책임 집중 — 단위 테스트 불가능
- ✅ Single Responsibility: 클래스 1개 = 책임 1개

### ❌ print 디버깅
- `print()`는 테스트 fixture에서만. 운영 코드는 `logging` 사용
- `logging.basicConfig(level=logging.INFO)`로 초기화

## 12. 프로젝트 패키지 구조 (권장)

```
package_name/
├── __init__.py
├── py.typed              # PEP 561 — 타입 정보 제공 마커
├── core.py               # 핵심 로직
├── types.py              # dataclass / TypedDict 정의
├── exceptions.py         # 커스텀 예외
└── tests/
    ├── __init__.py
    └── test_core.py
```

## 13. 참고

- PEP 8 (style): <https://peps.python.org/pep-0008/>
- PEP 484 (typing): <https://peps.python.org/pep-0484/>
- ruff: <https://docs.astral.sh/ruff/>
- mypy: <https://mypy.readthedocs.io/>
- 프로젝트 룰: [`.claude/rules/engineering.md`](../rules/engineering.md)
