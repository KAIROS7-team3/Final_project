"""ReturnTool 서브트리 — Staging Area에서 공구를 집어 슬롯에 반납한다."""
import py_trees


def build_return_subtree() -> py_trees.behaviour.Behaviour:
    """ReturnTool 서브트리를 조립해 루트 노드를 반환한다.

    서브트리 구조 (Phase 5a에서 구현):
        Sequence("ReturnTool")
        ├── CheckFeasibility       ← staged 상태 확인 (S-2)
        ├── PickFromStaging
        ├── MoveToPose(slot)
        ├── Release
        └── MoveToHome

    Returns:
        py_trees.behaviour.Behaviour: 서브트리 루트.
    """
    # TODO(Phase 5a): 실제 서브트리 조립
    root = py_trees.composites.Sequence(name="ReturnTool", memory=True)
    root.add_child(
        py_trees.behaviours.Failure(name="TODO:ReturnTool(Phase5a)")
    )
    return root
