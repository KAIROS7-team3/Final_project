"""CheckFeasibility BT 노드 — /db/CheckToolFeasibility 서비스로 DB Gate를 확인한다 (S-2)."""
from __future__ import annotations

import threading
from typing import Any

import py_trees

from orchestrator.blackboard import (
    KEY_ACTIVE_TOOL_ID,
    KEY_FEASIBILITY_REASON,
    KEY_INTENT,
)


class CheckFeasibility(py_trees.behaviour.Behaviour):
    """Blackboard의 intent + active_tool_id로 /db/CheckToolFeasibility를 호출한다.

    SUCCESS: 응답 feasible=True
    FAILURE: 응답 feasible=False 또는 서비스 실패 — feasibility_reason에 사유 기록

    서비스 클라이언트는 orchestrator_node에서 주입한다. update()는 blocking —
    BT tick 전용 스레드에서만 호출할 것.

    주의: fetch/return 서브트리에 각각 인스턴스를 추가할 때 name을 다르게 줄 것
    (Blackboard 클라이언트 이름 충돌 방지, E-9).
    """

    def __init__(
        self, name: str, service_client: Any, intent_override: str | None = None
    ) -> None:
        """
        Args:
            name: 고유 BT 노드 이름 (예: "CheckFeasibility_fetch").
            service_client: /db/CheckToolFeasibility rclpy Client.
            intent_override: None이면 blackboard.intent 사용, 지정하면 해당 값 고정.
        """
        super().__init__(name=name)
        self._cli = service_client
        self._intent_override = intent_override
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=KEY_INTENT, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=KEY_FEASIBILITY_REASON, access=py_trees.common.Access.WRITE
        )

    def update(self) -> py_trees.common.Status:
        tool_id = self.blackboard.active_tool_id or ""
        intent = self._intent_override or self.blackboard.intent or ""

        if not tool_id:
            self.blackboard.feasibility_reason = "tool_id 없음"
            return py_trees.common.Status.FAILURE

        if not self._cli.service_is_ready():
            self.logger.warn(f"[{self.name}] CheckToolFeasibility 서비스 미준비")
            self.blackboard.feasibility_reason = "DB 서비스 미준비"
            return py_trees.common.Status.FAILURE

        from interfaces.srv import CheckToolFeasibility
        req = CheckToolFeasibility.Request()
        req.intent = intent
        req.tool_id = tool_id

        done = threading.Event()
        result_holder: list[Any] = []

        def _cb(future):
            try:
                result_holder.append(future.result())
            except Exception as exc:
                self.logger.error(f"[{self.name}] CheckToolFeasibility 예외: {exc}")
                result_holder.append(None)
            done.set()

        self._cli.call_async(req).add_done_callback(_cb)

        if not done.wait(timeout=5.0):
            self.blackboard.feasibility_reason = "DB Gate 타임아웃"
            self.logger.error(f"[{self.name}] 타임아웃 — tool_id={tool_id}")
            return py_trees.common.Status.FAILURE

        res = result_holder[0] if result_holder else None
        if res is None:
            self.blackboard.feasibility_reason = "DB Gate 응답 없음"
            return py_trees.common.Status.FAILURE

        self.blackboard.feasibility_reason = res.reason
        if res.feasible:
            self.logger.info(f"[{self.name}] 가능: tool_id={tool_id}")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.warn(
                f"[{self.name}] 거부: tool_id={tool_id} reason={res.reason}"
            )
            return py_trees.common.Status.FAILURE
