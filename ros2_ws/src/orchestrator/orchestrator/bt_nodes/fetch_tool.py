"""FetchTool 서브트리 — 공구함에서 공구를 꺼내 Staging Area에 거치한다."""
from __future__ import annotations

from typing import Any

import py_trees

from interfaces.action import PlaceAtStaging
from orchestrator.bt_nodes.check_feasibility import CheckFeasibility
from orchestrator.bt_nodes.run_action import RunAction


def build_fetch_subtree(
    feasibility_client: Any,
    place_at_staging_client: Any,
) -> py_trees.behaviour.Behaviour:
    """FetchTool 서브트리를 조립해 루트 노드를 반환한다.

    서브트리 구조:
        Sequence("FetchTool")
        ├── CheckFeasibility_fetch   ← /db/CheckToolFeasibility (S-2)
        └── RunAction_PlaceAtStaging ← motion tool_action_server

    Args:
        feasibility_client: /db/CheckToolFeasibility rclpy Client.
        place_at_staging_client: place_at_staging rclpy ActionClient.
    """
    def _build_goal(tool_id: str) -> PlaceAtStaging.Goal:
        goal = PlaceAtStaging.Goal()
        goal.tool_id = tool_id
        return goal

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
        ),
    ])
    return root
