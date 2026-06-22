"""ReturnTool 서브트리 — ExecutePhase 기반 phase 오케스트레이션.

트리 구조:
    Sequence("ReturnTool_root")
    ├── CheckFeasibility_return       ← DB S-2 게이트
    └── Selector("ReturnTool_motion", memory=False)
        ├── Sequence("ReturnTool_main", memory=True)
        │   ├── SetMoving_true        ← /robot/status + PLC "moving"
        │   ├── RunAction_open_drawer ← ExecutePhase(open_drawer)
        │   ├── RunAction_return      ← ExecutePhase(return), max_attempts=2
        │   ├── RunAction_close_drawer← ExecutePhase(close_drawer)
        │   └── SetMoving_false       ← /robot/status + PLC "idle"
        └── FaultHandler              ← close_drawer+home 복구, 항상 FAILURE
"""
from __future__ import annotations

from typing import Any, Callable, Optional

import py_trees

from interfaces.action import ExecutePhase
from orchestrator.bt_nodes.check_feasibility import CheckFeasibility
from orchestrator.bt_nodes.fault_handler import FaultHandlerNode
from orchestrator.bt_nodes.run_action import RunAction
from orchestrator.bt_nodes.set_moving import SetMoving


def build_return_subtree(
    feasibility_client: Any,
    execute_phase_client: Any,
    publish_status_fn: Callable[[bool], None],
    set_plc_fn: Callable[[str], None],
    log_error_fn: Callable[[str, str], None],
    on_pick: Optional[Callable[[], None]] = None,
    on_place: Optional[Callable[[], None]] = None,
    layer_id: int = 1,
    max_return_attempts: int = 2,
) -> py_trees.behaviour.Behaviour:
    """ReturnTool 서브트리 조립.

    Args:
        feasibility_client: /db/CheckToolFeasibility rclpy Client.
        execute_phase_client: execute_phase rclpy ActionClient.
        publish_status_fn: is_moving(bool) → None — /robot/status 발행.
        set_plc_fn: PLC 상태 갱신 함수.
        log_error_fn: (tool_id, notes) → None — DB 에러 로그 (FaultHandler용).
        on_pick: return 파지 순간(feedback phase="pick") — DB staged→out.
        on_place: 슬롯 반납 순간(feedback phase="place") — DB out→in_slot.
        layer_id: 서랍 layer. 기본 1.
        max_return_attempts: return phase 최대 시도 횟수. 기본 2.
    """

    def _build_open_drawer_goal(tool_id: str) -> ExecutePhase.Goal:
        goal = ExecutePhase.Goal()
        goal.phase = "open_drawer"
        goal.tool_id = tool_id
        goal.layer_id = layer_id
        return goal

    def _build_return_goal(tool_id: str) -> ExecutePhase.Goal:
        goal = ExecutePhase.Goal()
        goal.phase = "return"
        goal.tool_id = tool_id
        goal.layer_id = 0
        return goal

    def _build_close_drawer_goal(tool_id: str) -> ExecutePhase.Goal:
        goal = ExecutePhase.Goal()
        goal.phase = "close_drawer"
        goal.tool_id = tool_id
        goal.layer_id = layer_id
        return goal

    def _on_return_feedback(phase: str) -> None:
        if phase == "pick" and on_pick:
            on_pick()
        elif phase == "place" and on_place:
            on_place()

    main_seq = py_trees.composites.Sequence("ReturnTool_main", memory=True)
    main_seq.add_children([
        SetMoving(
            "SetMoving_true",
            publish_fn=publish_status_fn,
            is_moving=True,
            set_plc_fn=set_plc_fn,
            plc_state="moving",
        ),
        RunAction(
            name="RunAction_open_drawer",
            action_client=execute_phase_client,
            build_goal_fn=_build_open_drawer_goal,
            timeout_sec=60.0,
        ),
        RunAction(
            name="RunAction_return",
            action_client=execute_phase_client,
            build_goal_fn=_build_return_goal,
            timeout_sec=180.0,
            max_attempts=max_return_attempts,
            feedback_callback=_on_return_feedback,
        ),
        RunAction(
            name="RunAction_close_drawer",
            action_client=execute_phase_client,
            build_goal_fn=_build_close_drawer_goal,
            timeout_sec=60.0,
        ),
        SetMoving(
            "SetMoving_false",
            publish_fn=publish_status_fn,
            is_moving=False,
            set_plc_fn=set_plc_fn,
            plc_state="idle",
        ),
    ])

    motion_selector = py_trees.composites.Selector("ReturnTool_motion", memory=False)
    motion_selector.add_children([
        main_seq,
        FaultHandlerNode(
            name="FaultHandler",
            execute_phase_client=execute_phase_client,
            publish_status_fn=publish_status_fn,
            set_plc_fn=set_plc_fn,
            log_error_fn=log_error_fn,
            layer_id=layer_id,
        ),
    ])

    root = py_trees.composites.Sequence("ReturnTool_root", memory=True)
    root.add_children([
        CheckFeasibility(
            name="CheckFeasibility_return",
            service_client=feasibility_client,
            intent_override="return",
        ),
        motion_selector,
    ])
    return root
