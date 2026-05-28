"""FetchTool 서브트리 — 공구함에서 공구를 꺼내 Staging Area에 거치한다."""
import py_trees


def build_fetch_subtree() -> py_trees.behaviour.Behaviour:
    """FetchTool 서브트리를 조립해 루트 노드를 반환한다.

    서브트리 구조 (Phase 5a에서 구현):
        Sequence("FetchTool")
        ├── CheckFeasibility
        ├── ScanWorkspace          ← vision에서 tool_pose 획득
        ├── MoveToPose(approach)
        ├── Grasp
        ├── PlaceAtStaging
        └── MoveToHome

    Returns:
        py_trees.behaviour.Behaviour: 서브트리 루트.
    """
    # TODO(Phase 5a): 실제 서브트리 조립
    root = py_trees.composites.Sequence(name="FetchTool", memory=False)
    root.add_child(
        py_trees.behaviours.Failure(name="TODO:FetchTool(Phase5a)")
    )
    return root
