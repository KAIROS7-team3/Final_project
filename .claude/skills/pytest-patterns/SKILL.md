---
name: pytest-patterns
description: >
  pytest 기반 테스트 작성 패턴 — fixture, parametrize, 모킹, 테스트 조직,
  ROS2/HW 모킹, 커버리지 측정.
  단위 테스트, 통합 테스트, 골든 파일 회귀 테스트 작성 시 활성화.
when_to_use: >
  pytest 단위 테스트 작성, fixture 설계, parametrize 적용,
  ROS2 노드 / 하드웨어 mock 객체 작성, 안전 critical 경로 단위 테스트 시.
  (BT 골든 파일 회귀는 bt-py-trees 스킬 전담)
---

# pytest 테스트 패턴

> 프로젝트 표준: pytest 7+, pytest-mock. 룰: [`.claude/rules/process.md`](../rules/process.md) P-1.

## 1. 테스트 구조

```
package_name/
├── core.py
└── tests/
    ├── __init__.py
    ├── conftest.py           # 공유 fixture
    ├── test_core.py          # core.py 단위 테스트
    ├── test_integration.py   # 통합 테스트
    └── fixtures/             # 골든 파일, 샘플 데이터
        ├── sample_audio.wav
        └── expected_traj.json
```

### 테스트 디스커버리 규칙
- 파일: `test_*.py` 또는 `*_test.py`
- 클래스: `Test*`
- 함수: `test_*`

```python
# tests/test_safety_validator.py
def test_joint_limit_violation_rejected():
    validator = SafetyValidator()
    bad_traj = [(10.0, 0, 0, 0, 0, 0)]  # J1 > limit
    assert not validator.check(bad_traj)
```

## 2. Fixture — 재사용 가능한 setup

### 기본 fixture
```python
# conftest.py
import pytest
from db_core import DBClient

@pytest.fixture
def db_client(tmp_path):
    """임시 SQLite DB로 격리된 클라이언트 제공"""
    db_path = tmp_path / "test.db"
    client = DBClient(str(db_path))
    client.initialize_schema()
    yield client
    client.close()

# 사용
def test_log_event(db_client):
    db_client.log_event(tool_id="screwdriver", event_type="fetch", track="A")
    events = db_client.get_events()
    assert len(events) == 1
```

### Scope (lifetime)
| scope | 생명주기 |
|-------|----------|
| `function` (기본) | 매 테스트 함수마다 새로 생성 |
| `class` | 한 클래스 내 테스트 공유 |
| `module` | 한 파일 내 공유 |
| `session` | 전체 테스트 세션 공유 (느린 fixture에 유용) |

```python
@pytest.fixture(scope="session")
def vla_model():
    """VLA 모델 로딩 5초 → 세션 1회만 로드"""
    return VLAModel.load_from_checkpoint("test_model.ckpt")
```

### tmp_path / tmp_path_factory
- `tmp_path` — 함수 scope, 매 테스트 격리된 임시 디렉토리
- `tmp_path_factory` — session scope

## 3. Parametrize — 한 테스트로 여러 케이스

```python
import pytest

@pytest.mark.parametrize("status,intent,expected_feasible", [
    ("in_slot",   "fetch",  True),
    ("out",       "fetch",  False),
    ("staged",    "fetch",  False),
    ("missing",   "fetch",  False),
    ("fod_alert", "fetch",  False),
    ("staged",    "return", True),
    ("in_slot",   "return", False),
    ("out",       "return", False),
])
def test_check_feasibility(status, intent, expected_feasible, db_client):
    db_client.set_tool_status("test_tool", status)
    result, _ = check_feasibility(intent, "test_tool", db_client)
    assert result == expected_feasible
```

### Parametrize에 ID 부여 (테스트 출력에 표시)
```python
@pytest.mark.parametrize("status,expected", [
    pytest.param("in_slot",   True,  id="in_slot_allows_fetch"),
    pytest.param("out",       False, id="out_blocks_fetch"),
    pytest.param("fod_alert", False, id="fod_alert_blocks_fetch"),
])
def test_fetch_gate(status, expected, db_client):
    ...
```

## 4. Mocking — 외부 의존성 격리

### unittest.mock
```python
from unittest.mock import MagicMock, patch

def test_motion_execute_with_mock_arm():
    arm = MagicMock(spec=DooSanArm)
    arm.execute.return_value = MotionResult(success=True)

    controller = MotionController(arm=arm)
    controller.move_to_home()

    arm.execute.assert_called_once()
    call_args = arm.execute.call_args
    assert call_args.kwargs["vel"] <= 0.5  # 속도 한계 확인
```

### pytest-mock (mocker fixture)
```python
def test_db_failure_triggers_plc_warning(mocker):
    mock_plc = mocker.patch("orchestrator.plc_client")
    mocker.patch("orchestrator.db_client.get_tool_status",
                 side_effect=DBConnectionError("timeout"))

    result = handle_fetch("screwdriver")

    assert not result.success
    mock_plc.set_warning.assert_called_once()
```

### Mock 객체 동작 패턴
| 패턴 | 용도 |
|------|------|
| `MagicMock(spec=Class)` | Class와 동일한 attribute만 허용 (오타 방지) |
| `mock.return_value = X` | 호출 시 X 반환 |
| `mock.side_effect = Exception` | 호출 시 예외 발생 |
| `mock.side_effect = [v1, v2]` | 호출마다 다른 값 (순차) |
| `mock.assert_called_once_with(...)` | 정확히 1회 + 인자 검증 |
| `mock.call_count` | 호출 횟수 확인 |

## 5. 골든 파일 회귀 (Trajectory 검증)

BT/VLA 출력의 회귀를 막기 위해 기준 출력을 파일로 저장.

```python
import json
from pathlib import Path

GOLDEN_DIR = Path(__file__).parent / "fixtures" / "golden"

def test_fetch_trajectory_regression(bt_runner):
    result = bt_runner.run("FetchTool", tool_id="ratchet_wrench")
    actual = result.trajectory_summary()

    golden_path = GOLDEN_DIR / "fetch_ratchet_wrench.json"
    if not golden_path.exists():
        pytest.skip(f"Golden missing — write with UPDATE_GOLDENS=1")
    golden = json.loads(golden_path.read_text())

    # 허용 오차 안에서 비교 (joint 값은 float)
    assert_trajectory_close(actual, golden, joint_tol=0.01, time_tol=0.05)
```

### 골든 파일 갱신 패턴
```bash
UPDATE_GOLDENS=1 pytest tests/test_bt_regression.py
```
```python
def test_fetch_trajectory_regression(bt_runner):
    ...
    if os.environ.get("UPDATE_GOLDENS") == "1":
        golden_path.write_text(json.dumps(actual, indent=2))
        pytest.skip("Golden updated")
    ...
```

## 6. 예외 테스트

```python
def test_unknown_intent_raises():
    with pytest.raises(ValueError, match="unknown intent"):
        parse_intent("이상한 명령")

def test_db_disconnect_raises_after_ttl():
    with pytest.raises(DBConnectionError):
        with simulated_db_outage(duration_min=6):  # TTL 5분 초과
            db.get_tool_status("any")
```

## 7. Marker — 테스트 분류

```python
# pyproject.toml
[tool.pytest.ini_options]
markers = [
    "slow: VLA 추론 등 느린 테스트",
    "hardware: 실 하드웨어 필요",
    "safety: 안전 critical 경로",
]

# 사용
@pytest.mark.safety
def test_safety_validator_blocks_joint_violation():
    ...

@pytest.mark.hardware
@pytest.mark.slow
def test_full_fetch_cycle_with_real_arm():
    ...
```

### 실행 시 필터
```bash
pytest -m safety                # safety 마커만
pytest -m "not hardware"        # 하드웨어 제외
pytest -m "safety and not slow" # 조합
```

## 8. ROS2 노드 테스트

```python
import rclpy
from rclpy.node import Node

@pytest.fixture
def ros2_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_intent_node_publishes_correctly(ros2_context):
    node = Node("test_subscriber")
    received = []
    node.create_subscription(IntentMsg, "/voice/intent",
                             lambda m: received.append(m), 10)

    intent_node = GemmaIntentNode()  # SUT
    intent_node.process_text("드라이버 가져와")

    rclpy.spin_once(node, timeout_sec=1.0)
    assert len(received) == 1
    assert received[0].action == "fetch"
```

### launch_testing (통합 테스트)
```python
# tests/test_pipeline_launch.py
import launch
import launch_ros
import launch_testing

def generate_test_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(package='voice', executable='whisper_node'),
        launch_ros.actions.Node(package='voice', executable='gemma_intent_node'),
        launch_testing.actions.ReadyToTest(),
    ])

class TestPipeline(unittest.TestCase):
    def test_intent_received_within_2s(self):
        # ...
```

## 9. 커버리지

```bash
pip install pytest-cov
pytest --cov=db_core --cov-report=term-missing --cov-report=html
```

```toml
# pyproject.toml
[tool.coverage.report]
fail_under = 80     # 80% 미달 시 실패
exclude_lines = [
    "pragma: no cover",
    "raise NotImplementedError",
]
```

> 안전 모듈은 100% 커버리지 권장 ([`.claude/rules/process.md`](../rules/process.md) P-1).

## 10. CI 통합

```yaml
# .github/workflows/test.yml (예시)
name: tests
on: [push, pull_request]
jobs:
  unit:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-python@v5
        with: { python-version: "3.10" }
      - run: pip install -e ".[test]"
      - run: pytest -m "not hardware" --cov --cov-fail-under=80
```

## 11. 흔한 함정

### ❌ Fixture에서 mutable 공유
```python
@pytest.fixture
def shared_list():
    return [1, 2, 3]   # 함수 scope면 OK, session scope면 위험
```

### ❌ Production 코드를 import해서 mocking 실수
```python
# bad — 잘못된 경로 patch
mocker.patch("db_core.client.DBClient")  # 호출자가 import한 곳을 patch해야 함

# good — 사용처 patch
mocker.patch("orchestrator.handlers.DBClient")
```

### ❌ 시간 의존 테스트
```python
# bad
def test_fod_timeout():
    db.set_status("tool_1", "out")
    time.sleep(601)   # 10분 + 1초 — 테스트 느려짐
    assert db.get_status("tool_1") == "missing"

# good — 시간 추상화
def test_fod_timeout(mock_clock):
    db.set_status("tool_1", "out")
    mock_clock.advance(601)
    fod_monitor.tick()
    assert db.get_status("tool_1") == "missing"
```

### ❌ 실 네트워크 / 실 DB / 실 하드웨어 의존
- 단위 테스트에서는 mocking. 통합 테스트에서만 실제 자원
- 실 하드웨어 테스트는 `@pytest.mark.hardware` 표시 → CI에서 자동 스킵

## 12. 참고

- pytest docs: <https://docs.pytest.org/>
- pytest-mock: <https://pytest-mock.readthedocs.io/>
- 프로젝트 룰: [`.claude/rules/process.md`](../rules/process.md), [`.claude/rules/safety.md`](../rules/safety.md)
