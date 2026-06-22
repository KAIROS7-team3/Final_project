"""FetchTool 서브트리 — ExecutePhase 기반 phase 오케스트레이션.

트리 구조:
    Sequence("FetchTool_root")
    ├── CheckFeasibility_fetch        ← DB S-2 게이트
    └── Selector("FetchTool_motion", memory=False)
        ├── Sequence("FetchTool_main", memory=True)
        │   ├── SetMoving_true        ← /robot/status + PLC "moving"
        │   ├── RunAction_open_drawer ← ExecutePhase(open_drawer)
        │   ├── RunAction_fetch       ← ExecutePhase(fetch), max_attempts=3
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


def build_fetch_subtree(
    feasibility_client: Any,
    execute_phase_client: Any,
    publish_status_fn: Callable[[bool], None],
    set_plc_fn: Callable[[str], None],
    log_error_fn: Callable[[str, str], None],
    on_pick: Optional[Callable[[], None]] = None,
    on_place: Optional[Callable[[], None]] = None,
    layer_id: int = 1,
    max_fetch_attempts: int = 3,
) -> py_trees.behaviour.Behaviour:
    """FetchTool 서브트리를 조립해 루트 노드를 반환한다.

    Args:
        feasibility_client: /db/CheckToolFeasibility rclpy Client.
        execute_phase_client: execute_phase rclpy ActionClient.
        publish_status_fn: is_moving(bool) → None — /robot/status 발행.
        set_plc_fn: PLC 상태 갱신 함수.
        log_error_fn: (tool_id, notes) → None — DB 에러 로그 (FaultHandler용).
        on_pick: fetch 파지 순간(feedback phase="pick") — DB in_slot→out.
        on_place: staging 거치 순간(feedback phase="place") — DB out→staged.
        layer_id: 서랍 layer. 기본 1.
        max_fetch_attempts: fetch phase 최대 시도 횟수. 기본 3.
    """

    def _build_open_drawer_goal(tool_id: str) -> ExecutePhase.Goal:
        goal = ExecutePhase.Goal()
        goal.phase = "open_drawer"
        goal.tool_id = tool_id
        goal.layer_id = layer_id
        return goal

    def _build_fetch_goal(tool_id: str) -> ExecutePhase.Goal:
        goal = ExecutePhase.Goal()
        goal.phase = "fetch"
        goal.tool_id = tool_id
        goal.layer_id = 0
        return goal

    def _build_close_drawer_goal(tool_id: str) -> ExecutePhase.Goal:
        goal = ExecutePhase.Goal()
        goal.phase = "close_drawer"
        goal.tool_id = tool_id
        goal.layer_id = layer_id
        return goal

    def _on_fetch_feedback(phase: str) -> None:
        if phase == "pick" and on_pick:
            on_pick()
        elif phase == "place" and on_place:
            on_place()

    # ── 메인 시퀀스 ───────────────────────────────────────────────────────
    main_seq = py_trees.composites.Sequence("FetchTool_main", memory=True)
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
            name="RunAction_fetch",
            action_client=execute_phase_client,
            build_goal_fn=_build_fetch_goal,
            timeout_sec=180.0,
            max_attempts=max_fetch_attempts,
            feedback_callback=_on_fetch_feedback,
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

    # ── Selector: 메인 성공 or FaultHandler ──────────────────────────────
    motion_selector = py_trees.composites.Selector("FetchTool_motion", memory=False)
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

    # ── 루트: CheckFeasibility → motion_selector ─────────────────────────
    root = py_trees.composites.Sequence("FetchTool_root", memory=True)
    root.add_children([
        CheckFeasibility(
            name="CheckFeasibility_fetch",
            service_client=feasibility_client,
            intent_override="fetch",
        ),
        motion_selector,
    ])
    return root
