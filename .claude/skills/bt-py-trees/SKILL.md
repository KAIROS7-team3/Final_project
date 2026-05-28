---
name: bt-py-trees
description: >
  py_trees / py_trees_ros 기반 Behavior Tree 작성 가이드 — Behavior·Composite·Decorator,
  Blackboard, ROS2 action server 통합, 골든 파일 회귀 테스트.
  Track A/B의 FetchTool/ReturnTool BT 작성 시 활성화.
when_to_use: >
  py_trees 노드 작성, Behavior Tree 설계, Blackboard 스키마,
  BT 통합 테스트, 골든 파일 회귀, BT 디버깅 시.
---

# py_trees Behavior Tree 가이드

> 이 프로젝트 Track A/B의 `orchestrator/` 패키지에서 BT 사용. 룰: [`.claude/rules/engineering.md`](../rules/engineering.md), [`.claude/rules/safety.md`](../rules/safety.md).

## 1. 핵심 개념

### Behavior Tree란
- **계층적 의사결정 구조**. 매 tick마다 root에서 시작해 트리를 순회하며 동작 선택
- 상태: `SUCCESS` / `FAILURE` / `RUNNING` / `INVALID`
- FSM 대비 장점: 모듈형 서브트리, 재사용 가능, 반응형 폴백 쉬움

### 노드 타입

| 타입 | 역할 |
|------|------|
| **Composite** | 자식 노드를 가진 컨테이너 |
| **Decorator** | 자식 1개의 동작 변형 |
| **Behavior (Leaf)** | 실제 동작 수행 |

### 주요 Composite
| 노드 | 동작 |
|------|------|
| `Sequence` | 자식을 순서대로 tick. 하나라도 FAILURE → 전체 FAILURE. 모두 SUCCESS → SUCCESS |
| `Selector` (Fallback) | 자식을 순서대로 tick. 하나라도 SUCCESS → 전체 SUCCESS. 모두 FAILURE → FAILURE |
| `Parallel` | 모든 자식을 동시에 tick. 정책에 따라 결과 결정 |

### 주요 Decorator
| 노드 | 동작 |
|------|------|
| `Inverter` | SUCCESS ↔ FAILURE 반전 |
| `Retry(n)` | FAILURE 시 n번까지 재시도 |
| `Timeout(t)` | t초 후 FAILURE 강제 |
| `OneShot` | 1회 성공/실패 후 같은 결과 반환 |
| `Repeat(n)` | n번 반복 |

## 2. 설치

```bash
# pip
pip install py_trees py_trees_ros

# ROS2 패키지 (Humble)
sudo apt install ros-humble-py-trees ros-humble-py-trees-ros
```

## 3. 이 프로젝트의 BT 구조

```
→ Sequence: Root
   ├── Condition: VoiceCommandReceived (feasible=true)
   └── → Fallback: DispatchByIntent
        ├── → Sequence: FetchTool                  ← intent==fetch
        │    ├── Condition: IsFetchIntent
        │    ├── Action: LocalizeTool              ← YOLOv8 + 6D Pose
        │    ├── Action: PlanGrasp                 ← grasp_planner
        │    ├── Action: MoveToPreGrasp
        │    ├── Action: CloseGripper
        │    ├── Action: PlaceAtStagingArea
        │    ├── Action: OpenGripper
        │    └── Action: MoveToHome
        └── → Sequence: ReturnTool                 ← intent==return
             ├── Condition: IsReturnIntent
             ├── Action: PickFromStagingArea
             ├── Action: MoveToToolSlot
             ├── Action: OpenGripper
             └── Action: MoveToHome
```

## 4. Leaf Behavior 작성 패턴

### 단순 동작
```python
import py_trees

class IsFetchIntent(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "IsFetchIntent"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="intent", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        if self.blackboard.intent == "fetch":
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
```

### 시간 걸리는 동작 (RUNNING 반환)
```python
class MoveToHome(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "MoveToHome"):
        super().__init__(name)
        self.future = None

    def initialise(self) -> None:
        """tick 시작 시 1회 호출 — 비동기 동작 시작"""
        self.future = motion_action_client.send_goal_async(home_pose)

    def update(self) -> py_trees.common.Status:
        if self.future.done():
            result = self.future.result()
            return (py_trees.common.Status.SUCCESS if result.success
                    else py_trees.common.Status.FAILURE)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """tick 종료 시 호출 — 자원 정리"""
        if new_status == py_trees.common.Status.INVALID and self.future:
            self.future.cancel()
```

### Lifecycle 메서드
| 메서드 | 호출 시점 |
|--------|-----------|
| `__init__` | 트리 생성 시 1회 |
| `setup(**kwargs)` | 초기 셋업 (ROS2 노드 등록 등) |
| `initialise()` | 매번 RUNNING 진입 시 |
| `update()` | 매 tick |
| `terminate(new_status)` | RUNNING 종료 시 (성공/실패/취소) |

## 5. Blackboard — 상태 공유

### 스키마 정의
```python
# blackboard_schema.py
from dataclasses import dataclass

@dataclass
class BlackboardSchema:
    intent: str            # 'fetch' | 'return'
    active_tool_id: str    # 예: 'screwdriver'
    tool_pose: GraspPose | None    # YOLOv8+6D pose 결과
    staging_state: str     # 'empty' | 'placed' | 'pickup_ready'
```

### Register + Access
```python
class LocalizeTool(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "LocalizeTool"):
        super().__init__(name)
        self.bb = self.attach_blackboard_client(name=name)
        # 읽기
        self.bb.register_key(key="active_tool_id", access=py_trees.common.Access.READ)
        # 쓰기
        self.bb.register_key(key="tool_pose", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        tool_id = self.bb.active_tool_id
        pose = perception_client.localize(tool_id)
        if pose is None:
            return py_trees.common.Status.FAILURE
        self.bb.tool_pose = pose
        return py_trees.common.Status.SUCCESS
```

## 6. 트리 조립 + 실행

```python
import py_trees

def build_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Root", memory=False)

    voice_check = VoiceCommandReceived()
    dispatch = py_trees.composites.Selector(name="DispatchByIntent", memory=False)

    fetch_seq = py_trees.composites.Sequence(name="FetchTool", memory=True)
    fetch_seq.add_children([
        IsFetchIntent(),
        LocalizeTool(),
        PlanGrasp(),
        MoveToPreGrasp(),
        CloseGripper(),
        PlaceAtStagingArea(),
        OpenGripper(),
        MoveToHome(),
    ])

    return_seq = py_trees.composites.Sequence(name="ReturnTool", memory=True)
    return_seq.add_children([
        IsReturnIntent(),
        PickFromStagingArea(),
        MoveToToolSlot(),
        OpenGripper(),
        MoveToHome(),
    ])

    dispatch.add_children([fetch_seq, return_seq])
    root.add_children([voice_check, dispatch])
    return root

# Tick loop
tree = py_trees.trees.BehaviourTree(root=build_root())
tree.setup(timeout=15)

while rclpy.ok():
    tree.tick()
    time.sleep(0.1)   # 10 Hz
```

> `memory=True` (Sequence): 한번 SUCCESS한 자식은 다시 tick 안 함 → 진행 중 작업 유지
> `memory=False`: 매번 첫 자식부터 tick (반응형 — Selector에 권장)

## 7. ROS2 통합 (py_trees_ros)

### Action Server 호출
```python
import py_trees_ros

class MoveToPreGrasp(py_trees_ros.actions.ActionClient):
    def __init__(self, name: str = "MoveToPreGrasp"):
        super().__init__(
            name=name,
            action_type=MoveJoint,
            action_name="/dsr/motion/move_joint_action",
            generate_feedback_message=False,
        )
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(key="tool_pose", access=py_trees.common.Access.READ)

    def initialise(self):
        pose = self.bb.tool_pose
        self.action_goal = MoveJoint.Goal()
        self.action_goal.pos = compute_pre_grasp_joints(pose)
        self.action_goal.vel = 0.3
        super().initialise()
```

### Subscribe 토픽 (조건 노드)
```python
class IsToolboxClear(py_trees_ros.subscribers.ToBlackboard):
    def __init__(self):
        super().__init__(
            name="IsToolboxClear",
            topic_name="/vision/obstacles",
            topic_type=ObstacleStatus,
            blackboard_variables={"obstacles_clear": "clear"},
        )
```

## 8. 에러 복구 패턴

### Retry 데코레이터
```python
fetch_with_retry = py_trees.decorators.Retry(
    name="FetchRetry",
    child=fetch_seq,
    num_failures=2,   # 2번까지 재시도
)
```

### Fallback으로 복구 동작
```python
# 정상 → 실패 시 복구 → 최후 수단
grasp_with_recovery = py_trees.composites.Selector(name="GraspWithRecovery", memory=False)
grasp_with_recovery.add_children([
    primary_grasp,           # 첫 번째 시도
    secondary_grasp,         # 다른 각도로 재시도
    abort_with_notification, # 모두 실패 시 운영자 안내
])
```

## 9. 테스트 — 골든 파일 회귀

```python
# tests/test_fetch_bt_regression.py
import json
from pathlib import Path
import pytest
import py_trees

GOLDEN_DIR = Path(__file__).parent / "fixtures" / "golden"

@pytest.fixture
def mock_perception(mocker):
    mocker.patch("perception_client.localize",
                 return_value=GraspPose(position=(0.5, 0.2, 0.1), ...))

@pytest.fixture
def mock_motion(mocker):
    mock = mocker.patch("motion_action_client.send_goal_async")
    mock.return_value.result.return_value = ActionResult(success=True)
    return mock

def test_fetch_screwdriver_trajectory(mock_perception, mock_motion):
    bb = py_trees.blackboard.Blackboard.set("intent", "fetch")
    bb.set("active_tool_id", "screwdriver")

    root = build_root()
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=5)

    # tick 끝까지 실행
    for _ in range(50):
        tree.tick()
        if tree.root.status != py_trees.common.Status.RUNNING:
            break

    assert tree.root.status == py_trees.common.Status.SUCCESS

    # Motion 호출 시퀀스 추출
    actual_calls = [c.args[0].pos for c in mock_motion.call_args_list]
    summary = {"calls": actual_calls}

    # 골든과 비교
    golden_path = GOLDEN_DIR / "fetch_phillips_small.json"
    if not golden_path.exists():
        pytest.skip("Golden missing — run with UPDATE_GOLDENS=1")
    expected = json.loads(golden_path.read_text())
    assert_trajectory_close(summary, expected, tol=0.01)
```

골든 갱신:
```bash
UPDATE_GOLDENS=1 pytest tests/test_fetch_bt_regression.py
```

## 10. 시각화 / 디버깅

### ASCII 트리 출력
```python
py_trees.display.print_ascii_tree(root, show_status=True)
```

### dot 그래프 생성
```python
py_trees.display.render_dot_tree(root, target_directory="./tmp")
# tmp/Root.dot 파일 생성 → graphviz로 PNG 변환
```

### Tick 로깅
```python
def print_tree_status(tree, time_stamp=None):
    print(py_trees.display.unicode_tree(root=tree.root, show_status=True))

tree.add_post_tick_handler(print_tree_status)
```

### py_trees-ros-viewer (실시간 GUI)
```bash
sudo apt install ros-humble-py-trees-ros-viewer
py-trees-tree-watcher
```

## 11. 흔한 함정

### ❌ Leaf에서 블로킹 동작
```python
def update(self):
    time.sleep(5.0)      # tick 루프 전체가 5초 멈춤
    return SUCCESS
```
✅ 비동기 패턴 + `RUNNING` 반환

### ❌ Blackboard에 mutable 객체 직접 저장 후 수정
```python
self.bb.detections.append(...)   # 다른 노드가 동시에 읽으면 race
```
✅ immutable로 교체
```python
self.bb.detections = self.bb.detections + [new_item]
```

### ❌ memory 설정 누락
- `Sequence(memory=False)`로 진행 중 작업이 매번 처음부터 다시 시작됨
- ✅ 액션 시퀀스에는 `memory=True`

### ❌ Action client cleanup 누락
- `terminate()`에서 `future.cancel()` 호출 안 하면 BT 종료 후에도 액션 계속 진행
- ✅ `terminate(new_status)`에서 항상 cleanup

### ❌ 너무 깊은 트리
- 깊이 5+ 트리는 디버깅 어려움
- ✅ 서브트리를 별도 파일로 분리 + 트리 깊이 4 이하 유지

### ❌ Tick 주기 너무 느림
- 100ms (10Hz) 미만은 반응성 ↓
- ✅ 50~100ms (10~20Hz) 권장

## 12. Best Practices

1. **서브트리 분리**: `bt_nodes/fetch_tool.py`, `bt_nodes/return_tool.py` 등 파일 분리
2. **Blackboard 스키마 정의**: dataclass로 키 + 타입 명시
3. **단위 테스트**: Leaf behavior는 mock으로 단독 테스트
4. **통합 테스트**: 전체 트리는 골든 파일 회귀
5. **로깅**: 각 tick에서 상태 변화 시 INFO 레벨로 기록
6. **타임아웃**: 모든 액션 노드는 `Timeout` 데코레이터로 감싸기

## 13. 참고

- py_trees: <https://py-trees.readthedocs.io/>
- py_trees_ros: <https://py-trees-ros.readthedocs.io/>
- 프로젝트 룰: [`.claude/rules/engineering.md`](../rules/engineering.md), [`.claude/rules/safety.md`](../rules/safety.md), [`.claude/rules/process.md`](../rules/process.md) P-1
