"""RunAction BT 노드 — ExecutePhase / PlaceAtStaging / ReturnToSlot 액션 클라이언트 리프.

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
        build_goal_fn: tool_id(str) → goal 객체 생성 함수.
        timeout_sec: 액션 완료 대기 타임아웃 (초).
        max_attempts: 실패 시 재시도 횟수. 기본 1 (재시도 없음).
        feedback_callback: action feedback의 phase 문자열을 받는 콜백.
            tool_action_server는 step.marker가 설정된 step("pick"/"place")
            실행 직후 phase=marker로 feedback을 발행한다 (E-9: 콜백은 빠르게
            반환하거나 자체적으로 blocking 호출을 감당할 수 있어야 함).
    """

    def __init__(
        self,
        name: str,
        action_client: Any,
        build_goal_fn: Callable,
        timeout_sec: float = 120.0,
        max_attempts: int = 1,
        feedback_callback: Callable[[str], None] | None = None,
        success_callback: Callable[[], None] | None = None,
        no_retry_after_marker: str | None = None,
    ) -> None:
        super().__init__(name=name)
        self._client = action_client
        self._build_goal_fn = build_goal_fn
        self._timeout = timeout_sec
        self._max_attempts = max_attempts
        self._feedback_callback = feedback_callback
        self._success_callback = success_callback
        self._no_retry_after_marker = no_retry_after_marker
        self._marker_done = False  # no_retry_after_marker 수신 여부
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        tool_id = self.blackboard.active_tool_id or ""
        goal = self._build_goal_fn(tool_id)
        self._marker_done = False  # 매 tick 초기화

        for attempt in range(self._max_attempts):
            if attempt > 0:
                self.logger.warning(
                    f"[{self.name}] 재시도 {attempt}/{self._max_attempts - 1}"
                )
            status = self._send_and_wait(goal)
            if status == py_trees.common.Status.SUCCESS:
                if self._success_callback is not None:
                    try:
                        self._success_callback()
                    except Exception as exc:
                        self.logger.error(f"[{self.name}] success_callback 예외: {exc}")
                return py_trees.common.Status.SUCCESS

            # no_retry_after_marker를 이미 수신했으면 비가역적 동작(슬롯 거치)이
            # 완료된 것이므로 재시도 없이 SUCCESS로 처리한다.
            # (이후 HOME 타임아웃 등 후처리 실패는 무시)
            if self._marker_done:
                self.logger.warning(
                    f"[{self.name}] '{self._no_retry_after_marker}' 완료 후 실패 — "
                    "비가역 동작 성공으로 간주, retry 생략"
                )
                if self._success_callback is not None:
                    try:
                        self._success_callback()
                    except Exception as exc:
                        self.logger.error(f"[{self.name}] success_callback 예외: {exc}")
                return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.FAILURE

    def _send_and_wait(self, goal: Any) -> py_trees.common.Status:
        """goal을 발송하고 완료를 대기한다 (블로킹)."""
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

        def _on_feedback(feedback_msg):
            phase = feedback_msg.feedback.phase
            if (
                self._no_retry_after_marker is not None
                and phase == self._no_retry_after_marker
            ):
                self._marker_done = True
            if self._feedback_callback is None:
                return
            try:
                self._feedback_callback(phase)
            except Exception as exc:
                self.logger.error(f"[{self.name}] feedback 콜백 예외: {exc}")

        self._client.send_goal_async(
            goal, feedback_callback=_on_feedback
        ).add_done_callback(_on_goal_response)

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
