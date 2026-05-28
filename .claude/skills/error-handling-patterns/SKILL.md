---
name: error-handling-patterns
description: >
  Python 에러 처리 패턴 — 커스텀 예외 계층, retry/backoff, circuit breaker,
  context manager 정리, 로깅 통합, silent fallback 안티패턴.
  에러 처리 설계, 재시도 로직, 장애 격리 구현 시 활성화.
when_to_use: >
  예외 계층 설계, retry/backoff 구현, circuit breaker 도입,
  에러 로깅 패턴, graceful degradation 구현 시.
---

# Python 에러 처리 패턴

> 프로젝트 룰: [`.claude/rules/engineering.md`](../rules/engineering.md) E-5 (에러 처리), [`.claude/rules/safety.md`](../rules/safety.md) S-3 (E-stop 경로 무결성).

## 1. 커스텀 예외 계층

### 기본 구조
```python
# exceptions.py

class RobotArmError(Exception):
    """이 시스템의 모든 예외의 기반 클래스."""

# 도메인별 분기
class HardwareError(RobotArmError):
    """하드웨어 통신·제어 실패."""

class PerceptionError(RobotArmError):
    """비전·센서 인식 실패."""

class PlanningError(RobotArmError):
    """경로·파지 계획 실패."""

class SafetyError(RobotArmError):
    """SafetyValidator 거부. 절대 무시 금지."""

# 구체적 예외
class JointLimitExceeded(HardwareError):
    def __init__(self, joint: int, value_rad: float, limit_rad: float):
        super().__init__(
            f"J{joint}: {value_rad:.3f} rad이 한계 {limit_rad:.3f} rad 초과"
        )
        self.joint = joint
        self.value_rad = value_rad
        self.limit_rad = limit_rad

class ToolNotFound(PerceptionError):
    def __init__(self, tool_id: str):
        super().__init__(f"공구 '{tool_id}'를 카메라에서 검출하지 못함")
        self.tool_id = tool_id

class DBGateBlocked(PlanningError):
    def __init__(self, tool_id: str, status: str):
        super().__init__(f"'{tool_id}' 상태={status}: DB gate 차단")
        self.tool_id = tool_id
        self.status = status
```

### 잡는 순서 — 구체 → 추상
```python
try:
    result = fetch_tool(tool_id)
except ToolNotFound as e:
    logger.warning("검출 실패: %s", e)
    notify_operator(str(e))
except PerceptionError as e:
    logger.error("인식 오류: %s", e)
    raise
except RobotArmError:
    raise
```

## 2. Retry + Exponential Backoff

### tenacity 사용 (권장)
```python
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type
import logging

logger = logging.getLogger(__name__)

@retry(
    retry=retry_if_exception_type(ToolNotFound),
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=0.5, min=0.5, max=4),
    before_sleep=lambda rs: logger.warning(
        "재시도 %d/3: %s", rs.attempt_number, rs.outcome.exception()
    ),
)
def localize_tool(tool_id: str) -> GraspPose:
    pose = perception_client.detect(tool_id)
    if pose is None:
        raise ToolNotFound(tool_id)
    return pose
```

### 수동 구현 (의존성 최소화 시)
```python
import time

def with_retry(fn, *, attempts: int = 3, base_delay: float = 0.5,
               retryable: type[Exception] = Exception):
    last_exc: Exception | None = None
    for attempt in range(attempts):
        try:
            return fn()
        except retryable as e:
            last_exc = e
            if attempt < attempts - 1:
                delay = base_delay * (2 ** attempt)
                logger.warning("시도 %d/%d 실패, %.1fs 후 재시도: %s",
                               attempt + 1, attempts, delay, e)
                time.sleep(delay)
    raise last_exc
```

> 하드웨어 에러(관절 한계 초과 등)는 retry 대상이 아니다. 재시도해도 해결되지 않는다.

## 3. Circuit Breaker

반복 실패 시 연속 호출을 차단해 시스템 보호.

```python
import time
from enum import Enum, auto
from dataclasses import dataclass, field

class CBState(Enum):
    CLOSED = auto()    # 정상
    OPEN = auto()      # 차단 중
    HALF_OPEN = auto() # 복구 시도 중

@dataclass
class CircuitBreaker:
    failure_threshold: int = 5
    recovery_timeout: float = 30.0
    _state: CBState = field(default=CBState.CLOSED, init=False)
    _failure_count: int = field(default=0, init=False)
    _opened_at: float = field(default=0.0, init=False)

    def call(self, fn, *args, **kwargs):
        if self._state == CBState.OPEN:
            if time.monotonic() - self._opened_at > self.recovery_timeout:
                self._state = CBState.HALF_OPEN
            else:
                raise HardwareError("Circuit open — 하드웨어 응답 없음")

        try:
            result = fn(*args, **kwargs)
            self._on_success()
            return result
        except Exception as e:
            self._on_failure()
            raise

    def _on_success(self):
        self._failure_count = 0
        self._state = CBState.CLOSED

    def _on_failure(self):
        self._failure_count += 1
        if self._failure_count >= self.failure_threshold:
            self._state = CBState.OPEN
            self._opened_at = time.monotonic()
            logger.error("Circuit OPEN: %d회 연속 실패", self._failure_count)

# 사용
arm_cb = CircuitBreaker(failure_threshold=3, recovery_timeout=10.0)

def move_joint_safe(pos):
    return arm_cb.call(arm.move_joint, pos)
```

## 4. Context Manager — 자원 정리 보장

```python
from contextlib import contextmanager

@contextmanager
def arm_session(arm):
    """예외 발생 시 홈 포즈로 복귀."""
    try:
        yield arm
    except SafetyError:
        raise   # 안전 에러는 복구 없이 전파
    except HardwareError:
        logger.warning("하드웨어 에러 — 홈으로 복귀 시도")
        try:
            arm.move_to_home()
        except Exception:
            logger.error("홈 복귀 실패", exc_info=True)
        raise
    finally:
        arm.gripper.open()   # 그리퍼는 항상 열린 상태로 종료

# 사용
with arm_session(arm) as a:
    a.move_to_pre_grasp(pose)
    a.close_gripper()
    a.place_at_staging()
```

## 5. 에러 로깅 패턴

```python
import logging
import traceback

logger = logging.getLogger(__name__)

def execute_fetch(tool_id: str) -> bool:
    try:
        pose = localize_tool(tool_id)
        arm.execute_grasp(pose)
        return True
    except ToolNotFound:
        # 예상 가능한 실패 — WARNING
        logger.warning("공구 미검출: %s", tool_id)
        return False
    except SafetyError as e:
        # 안전 관련 — ERROR, 재발생
        logger.error("안전 검증 실패: %s", e)
        raise
    except Exception:
        # 예상치 못한 실패 — ERROR + 스택 트레이스
        logger.error("fetch 실패: %s", tool_id, exc_info=True)
        raise
```

### 구조화 로깅 (structlog)
```python
import structlog

log = structlog.get_logger()

log.info("fetch_started", tool_id=tool_id, track="A")
log.error("fetch_failed", tool_id=tool_id, error=str(e), attempt=attempt)
```

## 6. 에러 코드 + 결과 타입

예외 대신 명시적 결과를 반환하는 패턴 (BT 조건 노드, DB gate 등에 유용).

```python
from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")

@dataclass
class Ok(Generic[T]):
    value: T

@dataclass
class Err:
    code: str
    message: str

Result = Ok[T] | Err

def check_feasibility(tool_id: str) -> Result[str]:
    status = db.get_tool_status(tool_id)
    if status == "available":
        return Ok(status)
    return Err(code="TOOL_UNAVAILABLE", message=f"{tool_id} 상태={status}")

# 사용 (match)
match check_feasibility("screwdriver"):
    case Ok(value):
        proceed_with_fetch(value)
    case Err(code="TOOL_UNAVAILABLE", message=msg):
        logger.warning(msg)
    case Err(message=msg):
        raise PlanningError(msg)
```

## 7. 흔한 함정

### ❌ Silent fallback — 에러 은폐
```python
try:
    pose = localize_tool(tool_id)
except Exception:
    pose = default_pose   # 에러를 삼켜버림 — 디버깅 불가
```
✅ 적어도 WARNING 로그
```python
except Exception as e:
    logger.warning("검출 실패, default 사용: %s", e)
    pose = default_pose
```

### ❌ bare `except:`
```python
try:
    ...
except:   # KeyboardInterrupt, SystemExit도 잡음
    pass
```
✅ `except Exception:` 또는 구체 타입

### ❌ SafetyError 복구 시도
```python
except SafetyError:
    logger.warning("안전 에러 무시")   # 절대 금지 — .claude/rules/engineering.md E-5 (silent fallback 금지)
    continue
```
✅ 항상 재발생 + E-stop

### ❌ 로그 없이 raise
```python
except HardwareError as e:
    raise RuntimeError("실패") from None   # 원인 소실
```
✅ `raise ... from e` 또는 `exc_info=True`

### ❌ 재시도 루프에 안전 에러 포함
```python
@retry(...)
def move():
    arm.move_joint(...)   # JointLimitExceeded도 retry — 위험
```
✅ `retry_if_exception_type(ToolNotFound)` 처럼 재시도 대상 명시

## 8. 참고

- tenacity: <https://tenacity.readthedocs.io/>
- structlog: <https://www.structlog.org/>
- 프로젝트 룰: [`.claude/rules/safety.md`](../rules/safety.md) S-3, [`.claude/rules/engineering.md`](../rules/engineering.md) E-5
- 관련 스킬: [`python-patterns`](python-patterns.md), [`pytest-patterns`](pytest-patterns.md)
