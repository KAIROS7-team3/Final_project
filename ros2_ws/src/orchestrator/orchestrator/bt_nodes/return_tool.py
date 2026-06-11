"""ReturnTool 서브트리 — Staging Area에서 공구를 집어 슬롯에 반납한다."""
from __future__ import annotations

from typing import Any, Callable

import py_trees

from interfaces.action import ReturnToSlot
from orchestrator.bt_nodes.check_feasibility import CheckFeasibility
from orchestrator.bt_nodes.run_action import RunAction


def build_return_subtree(
    feasibility_client: Any,
    return_to_slot_client: Any,
    on_pick: Callable[[], None] | None = None,
    on_place: Callable[[], None] | None = None,
) -> py_trees.behaviour.Behaviour:
    """ReturnTool 서브트리를 조립해 루트 노드를 반환한다.

    서브트리 구조:
        Sequence("ReturnTool")
        ├── CheckFeasibility_return  ← /db/CheckToolFeasibility (S-2)
        └── RunAction_ReturnToSlot   ← motion tool_action_server

    Args:
        feasibility_client: /db/CheckToolFeasibility rclpy Client.
        return_to_slot_client: return_to_slot rclpy ActionClient.
        on_pick: Staging Area에서 공구를 집는 순간(action feedback phase="pick")
            호출 — DB 상태 staged -> out 전이 트리거.
        on_place: 슬롯에 반납하는 순간(phase="place") 호출
            — DB 상태 out -> in_slot 전이 트리거.
    """
    def _build_goal(tool_id: str) -> ReturnToSlot.Goal:
        goal = ReturnToSlot.Goal()
        goal.tool_id = tool_id
        goal.slot_row = 0
        goal.slot_col = 0
        return goal

    def _on_feedback(phase: str) -> None:
        if phase == "pick" and on_pick:
            on_pick()
        elif phase == "place" and on_place:
            on_place()

    root = py_trees.composites.Sequence(name="ReturnTool", memory=True)
    root.add_children([
        CheckFeasibility(
            name="CheckFeasibility_return",
            service_client=feasibility_client,
            intent_override="return",
        ),
        RunAction(
            name="RunAction_ReturnToSlot",
            action_client=return_to_slot_client,
            build_goal_fn=_build_goal,
            timeout_sec=180.0,
            feedback_callback=_on_feedback,
        ),
    ])
    return root
