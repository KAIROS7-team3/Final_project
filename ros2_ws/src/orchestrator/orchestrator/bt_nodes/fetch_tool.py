"""FetchTool 서브트리 — 공구함에서 공구를 꺼내 Staging Area에 거치한다."""
from __future__ import annotations

from typing import Any, Callable

import py_trees

from interfaces.action import PlaceAtStaging
from orchestrator.bt_nodes.check_feasibility import CheckFeasibility
from orchestrator.bt_nodes.run_action import RunAction


def build_fetch_subtree(
    feasibility_client: Any,
    place_at_staging_client: Any,
    on_pick: Callable[[], None] | None = None,
    on_place: Callable[[], None] | None = None,
) -> py_trees.behaviour.Behaviour:
    """FetchTool 서브트리를 조립해 루트 노드를 반환한다.

    서브트리 구조:
        Sequence("FetchTool")
        ├── CheckFeasibility_fetch   ← /db/CheckToolFeasibility (S-2)
        └── RunAction_PlaceAtStaging ← motion tool_action_server

    Args:
        feasibility_client: /db/CheckToolFeasibility rclpy Client.
        place_at_staging_client: place_at_staging rclpy ActionClient.
        on_pick: 슬롯에서 공구를 집는 순간(action feedback phase="pick") 호출
            — DB 상태 in_slot -> out 전이 트리거.
        on_place: Staging Area에 거치하는 순간(phase="place") 호출
            — DB 상태 out -> staged 전이 트리거.
    """
    def _build_goal(tool_id: str) -> PlaceAtStaging.Goal:
        goal = PlaceAtStaging.Goal()
        goal.tool_id = tool_id
        return goal

    def _on_feedback(phase: str) -> None:
        if phase == "pick" and on_pick:
            on_pick()
        elif phase == "place" and on_place:
            on_place()

    root = py_trees.composites.Sequence(name="FetchTool", memory=True)
    root.add_children([
        CheckFeasibility(
            name="CheckFeasibility_fetch",
            service_client=feasibility_client,
            intent_override="fetch",
        ),
        RunAction(
            name="RunAction_PlaceAtStaging",
            action_client=place_at_staging_client,
            build_goal_fn=_build_goal,
            timeout_sec=180.0,
            feedback_callback=_on_feedback,
        ),
    ])
    return root
