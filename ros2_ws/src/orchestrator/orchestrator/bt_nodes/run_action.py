"""RunAction BT 노드 — PlaceAtStaging / ReturnToSlot 액션 클라이언트 리프.

액션 클라이언트는 orchestrator_node에서 생성해 주입한다 (BT 노드는 ROS2 의존 없음).
update()는 액션 완료까지 blocking — BT tick 전용 스레드에서만 호출할 것.
"""
from __future__ import annotations

import threading
from typing import Callable, Any

import py_trees

from orchestrator.blackboard import KEY_ACTIVE_TOOL_ID


class RunAction(py_trees.behaviour.Behaviour):
    """액션 서버로 goal을 보내고 완료를 대기하는 BT 리프.

    Args:
        name: BT 노드 이름 (고유하게).
        action_client: rclpy ActionClient 인스턴스.
        build_goal_fn: blackboard → goal 객체 생성 함수.
        timeout_sec: 액션 완료 대기 타임아웃 (초).
    """

    def __init__(
        self,
        name: str,
        action_client: Any,
        build_goal_fn: Callable,
        timeout_sec: float = 120.0,
    ) -> None:
        super().__init__(name=name)
        self._client = action_client
        self._build_goal_fn = build_goal_fn
        self._timeout = timeout_sec
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        tool_id = self.blackboard.active_tool_id or ""
        goal = self._build_goal_fn(tool_id)

        if not self._client.wait_for_server(timeout_sec=5.0):
            self.logger.error(f"[{self.name}] 액션 서버 없음")
            return py_trees.common.Status.FAILURE

        done = threading.Event()
        result_holder: list[Any] = []

        def _on_goal_response(future):
            gh = future.result()
            if not gh.accepted:
                self.logger.error(f"[{self.name}] goal 거부됨")
                result_holder.append(None)
                done.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            result_holder.append(future.result())
            done.set()

        self._client.send_goal_async(goal).add_done_callback(_on_goal_response)

        if not done.wait(timeout=self._timeout):
            self.logger.error(f"[{self.name}] 액션 타임아웃 ({self._timeout}s)")
            return py_trees.common.Status.FAILURE

        if not result_holder or result_holder[0] is None:
            return py_trees.common.Status.FAILURE

        result = result_holder[0].result
        if result.success:
            self.logger.info(f"[{self.name}] 액션 성공: {result.message}")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.error(f"[{self.name}] 액션 실패: {result.message}")
            return py_trees.common.Status.FAILURE
