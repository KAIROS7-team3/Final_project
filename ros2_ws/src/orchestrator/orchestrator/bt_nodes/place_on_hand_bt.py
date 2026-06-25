"""place_on_hand_bt.py — PlaceOnHand BT 서브트리.

/hand/ready=True 조건 확인 후 PlaceOnHand 액션 실행.
실패 시 PlaceAtStaging fallback.

서브트리 구조:
    Selector("HandoverSelector")
    ├── Sequence("TryHandover")
    │   ├── CheckHandReady      ← /hand/ready Bool 확인
    │   └── RunAction_PlaceOnHand ← motion tool_action_server
    └── RunAction_PlaceAtStaging  ← fallback
"""
from __future__ import annotations

from typing import Any, Callable

import py_trees
import py_trees_ros

from interfaces.action import PlaceAtStaging, PlaceOnHand
from orchestrator.bt_nodes.run_action import RunAction


class CheckHandReady(py_trees.behaviour.Behaviour):
    """/hand/ready 토픽이 True인지 확인."""

    def __init__(self, name: str, is_hand_ready: Callable[[], bool]) -> None:
        super().__init__(name)
        self._is_hand_ready = is_hand_ready

    def update(self) -> py_trees.common.Status:
        if self._is_hand_ready():
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


def build_handover_subtree(
    place_on_hand_client: Any,
    place_at_staging_client: Any,
    is_hand_ready: Callable[[], bool],
    on_pick: Callable[[], None] | None = None,
    on_deliver: Callable[[], None] | None = None,
) -> py_trees.behaviour.Behaviour:
    """HandoverSelector 서브트리 조립.

    Args:
        place_on_hand_client: PlaceOnHand rclpy ActionClient.
        place_at_staging_client: PlaceAtStaging rclpy ActionClient.
        is_hand_ready: /hand/ready 상태 콜백.
        on_pick: 공구 집은 순간 DB 상태 전이 콜백.
        on_deliver: 전달 완료 시 DB 상태 전이 콜백.
    """
    def _build_handover_goal(tool_id: str) -> PlaceOnHand.Goal:
        goal = PlaceOnHand.Goal()
        goal.tool_id = tool_id
        return goal

    def _build_staging_goal(tool_id: str) -> PlaceAtStaging.Goal:
        goal = PlaceAtStaging.Goal()
        goal.tool_id = tool_id
        return goal

    def _on_handover_feedback(phase: str) -> None:
        if phase == "pick" and on_pick:
            on_pick()
        elif phase == "place" and on_deliver:
            on_deliver()

    def _on_staging_feedback(phase: str) -> None:
        if phase == "pick" and on_pick:
            on_pick()

    try_handover = py_trees.composites.Sequence(name="TryHandover", memory=True)
    try_handover.add_children([
        CheckHandReady(name="CheckHandReady", is_hand_ready=is_hand_ready),
        RunAction(
            name="RunAction_PlaceOnHand",
            action_client=place_on_hand_client,
            build_goal_fn=_build_handover_goal,
            timeout_sec=60.0,
            feedback_callback=_on_handover_feedback,
        ),
    ])

    fallback_staging = RunAction(
        name="RunAction_PlaceAtStaging_fallback",
        action_client=place_at_staging_client,
        build_goal_fn=_build_staging_goal,
        timeout_sec=180.0,
        feedback_callback=_on_staging_feedback,
    )

    selector = py_trees.composites.Selector(name="HandoverSelector", memory=False)
    selector.add_children([try_handover, fallback_staging])
    return selector
