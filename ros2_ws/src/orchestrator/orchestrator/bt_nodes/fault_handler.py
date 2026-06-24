"""FaultHandlerNode — phase 실패 시 복구 정리 BT 리프.

서랍 닫기 → 홈 복귀 → DB 에러 로그. 항상 FAILURE 반환.
"""
from __future__ import annotations

import threading
from typing import Any, Callable

import py_trees

from interfaces.action import ExecutePhase
from orchestrator.blackboard import KEY_ACTIVE_TOOL_ID


class FaultHandlerNode(py_trees.behaviour.Behaviour):
    """phase 실패 후 정리 노드.

    1. is_moving=False 발행
    2. PLC "error" 설정
    3. close_drawer 복구 (블로킹, best-effort)
    4. home 복귀 (블로킹, best-effort)
    5. DB 에러 로그 (fire-and-forget)
    6. FAILURE 반환 — 항상

    Args:
        name: BT 노드 이름.
        execute_phase_client: ExecutePhase rclpy ActionClient.
        publish_status_fn: is_moving(bool) → None.
        set_plc_fn: PLC 상태 갱신 함수.
        log_error_fn: (tool_id: str, notes: str) → None — DB 에러 로그.
        layer_id: 복구 시 사용할 서랍 layer. 기본 1.
    """

    def __init__(
        self,
        name: str,
        execute_phase_client: Any,
        publish_status_fn: Callable[[bool], None],
        set_plc_fn: Callable[[str], None],
        log_error_fn: Callable[[str, str], None],
        layer_id: int = 1,
        close_phase: str = "close_drawer",
    ) -> None:
        super().__init__(name=name)
        self._client = execute_phase_client
        self._publish_status_fn = publish_status_fn
        self._set_plc_fn = set_plc_fn
        self._log_error_fn = log_error_fn
        self._layer_id = layer_id
        self._close_phase = close_phase
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.READ
        )

    def update(self) -> py_trees.common.Status:
        tool_id = getattr(self.blackboard, KEY_ACTIVE_TOOL_ID, "") or ""
        self.logger.error(f"[{self.name}] 복구 시작 — tool_id={tool_id!r}")

        # 1. is_moving 즉시 해제
        try:
            self._publish_status_fn(False)
        except Exception as exc:
            self.logger.error(f"[{self.name}] is_moving=False 발행 실패: {exc}")

        # 2. PLC 에러 표시
        try:
            self._set_plc_fn("error")
        except Exception as exc:
            self.logger.error(f"[{self.name}] PLC error 설정 실패: {exc}")

        # 3. close_drawer 복구 (best-effort)
        self._run_phase(self._close_phase, self._layer_id, timeout=30.0)

        # 4. home 복귀 (best-effort)
        self._run_phase("home", 0, timeout=30.0)

        # 5. DB 에러 로그 (fire-and-forget)
        threading.Thread(
            target=self._log_error_fn,
            args=(tool_id, f"FaultHandler: phase 실패 — layer_id={self._layer_id}, home 복귀 시도"),
            daemon=True,
        ).start()

        return py_trees.common.Status.FAILURE

    def _run_phase(self, phase: str, layer_id: int, timeout: float = 30.0) -> None:
        """ExecutePhase goal 발송 후 완료 대기 (블로킹, best-effort)."""
        if not self._client.wait_for_server(timeout_sec=3.0):
            self.logger.error(f"[{self.name}] {phase}: execute_phase 서버 없음")
            return

        goal = ExecutePhase.Goal()
        goal.phase = phase
        goal.tool_id = ""
        goal.layer_id = layer_id

        done = threading.Event()

        def _on_goal_response(future) -> None:
            gh = future.result()
            if not gh.accepted:
                self.logger.warning(f"[{self.name}] {phase} goal 거부됨")
                done.set()
                return
            gh.get_result_async().add_done_callback(lambda _: done.set())

        self._client.send_goal_async(goal).add_done_callback(_on_goal_response)

        if not done.wait(timeout=timeout):
            self.logger.error(f"[{self.name}] {phase} 타임아웃 ({timeout}s)")
        else:
            self.logger.info(f"[{self.name}] {phase} 복구 완료 또는 응답 수신")
