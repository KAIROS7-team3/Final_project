"""BT 노드 단위 테스트 — rclpy 불필요, plain pytest.

테스트 대상:
  - SetMoving (set_moving.py)
  - FaultHandlerNode (fault_handler.py)
  - RunAction — max_attempts 재시도 로직 (run_action.py)
  - build_fetch_subtree / build_return_subtree — 트리 구조 검증 (fetch_tool.py, return_tool.py)

설계 원칙:
  - interfaces 패키지는 MagicMock으로 대체 (ROS2 런타임 불필요)
  - py_trees Blackboard는 전역 싱글턴이므로 각 테스트는 고유한 client name 사용
  - action client는 MagicMock으로 주입; _send_and_wait는 monkeypatch로 대체
"""
from __future__ import annotations

import sys
import threading
import time
import unittest.mock as mock
from typing import Any
from unittest.mock import MagicMock, call, patch

import py_trees
import pytest

# ── interfaces 패키지를 ROS2 없이 mock 처리 ──────────────────────────────────
_interfaces_mock = MagicMock()
_interfaces_mock.action.ExecutePhase.Goal = MagicMock
sys.modules.setdefault("interfaces", _interfaces_mock)
sys.modules.setdefault("interfaces.action", _interfaces_mock.action)

# ── 테스트 대상 import (interfaces mock 이후) ─────────────────────────────────
# sys.path에 orchestrator 패키지 루트 추가
import os
_PKG_ROOT = os.path.join(os.path.dirname(__file__), "..")
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from orchestrator.blackboard import KEY_ACTIVE_TOOL_ID
from orchestrator.bt_nodes.fault_handler import FaultHandlerNode
from orchestrator.bt_nodes.fetch_tool import build_fetch_subtree
from orchestrator.bt_nodes.return_tool import build_return_subtree
from orchestrator.bt_nodes.run_action import RunAction
from orchestrator.bt_nodes.set_moving import SetMoving

# ── 편의 상수 ─────────────────────────────────────────────────────────────────
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE

# ── blackboard write helper ───────────────────────────────────────────────────

def _write_bb(client_name: str, tool_id: str) -> None:
    """blackboard에 active_tool_id를 기록한다."""
    bb = py_trees.blackboard.Client(name=client_name)
    bb.register_key(key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.WRITE)
    bb.active_tool_id = tool_id


def _make_mock_clients() -> tuple[MagicMock, MagicMock]:
    """(feasibility_client, execute_phase_client) 쌍을 반환한다."""
    return MagicMock(), MagicMock()


# ═══════════════════════════════════════════════════════════════════════════════
# SetMoving 테스트
# ═══════════════════════════════════════════════════════════════════════════════

class TestSetMoving:

    def test_set_moving_true_publishes_true_and_returns_success(self):
        pub = MagicMock()
        node = SetMoving("sm_true_pub", publish_fn=pub, is_moving=True)
        status = node.update()
        pub.assert_called_once_with(True)
        assert status == SUCCESS

    def test_set_moving_false_publishes_false(self):
        pub = MagicMock()
        node = SetMoving("sm_false_pub", publish_fn=pub, is_moving=False)
        status = node.update()
        pub.assert_called_once_with(False)
        assert status == SUCCESS

    def test_set_moving_calls_plc_when_provided(self):
        pub = MagicMock()
        plc = MagicMock()
        node = SetMoving(
            "sm_plc_call",
            publish_fn=pub,
            is_moving=True,
            set_plc_fn=plc,
            plc_state="moving",
        )
        node.update()
        plc.assert_called_once_with("moving")

    def test_set_moving_no_plc_call_when_fn_not_provided(self):
        pub = MagicMock()
        plc = MagicMock()
        node = SetMoving("sm_no_plc", publish_fn=pub, is_moving=False, set_plc_fn=None)
        node.update()
        plc.assert_not_called()

    def test_set_moving_publish_exception_returns_failure(self):
        pub = MagicMock(side_effect=RuntimeError("pub error"))
        node = SetMoving("sm_exc", publish_fn=pub, is_moving=True)
        status = node.update()
        assert status == FAILURE

    def test_set_moving_plc_exception_still_returns_success(self):
        """PLC 갱신 실패는 WARNING 수준 — SUCCESS 반환은 유지된다."""
        pub = MagicMock()
        plc = MagicMock(side_effect=RuntimeError("plc error"))
        node = SetMoving(
            "sm_plc_exc",
            publish_fn=pub,
            is_moving=False,
            set_plc_fn=plc,
            plc_state="idle",
        )
        status = node.update()
        assert status == SUCCESS


# ═══════════════════════════════════════════════════════════════════════════════
# FaultHandlerNode 테스트
# ═══════════════════════════════════════════════════════════════════════════════

class TestFaultHandlerNode:

    def _make_node(self, name: str, **kwargs) -> FaultHandlerNode:
        """공통 생성 헬퍼 — execute_phase_client.wait_for_server → False (서버 없음)."""
        exec_client = MagicMock()
        exec_client.wait_for_server.return_value = False  # 서버 없음 시뮬
        defaults = dict(
            publish_status_fn=MagicMock(),
            set_plc_fn=MagicMock(),
            log_error_fn=MagicMock(),
        )
        defaults.update(kwargs)
        node = FaultHandlerNode(
            name=name,
            execute_phase_client=exec_client,
            **defaults,
        )
        # blackboard에 tool_id 기록
        _write_bb(f"setup_{name}", "wrench_8mm")
        return node

    def test_fault_handler_always_returns_failure(self):
        node = self._make_node("fh_always_fail")
        status = node.update()
        assert status == FAILURE

    def test_fault_handler_calls_publish_status_false(self):
        pub = MagicMock()
        node = self._make_node("fh_pub_false", publish_status_fn=pub)
        node.update()
        pub.assert_called_once_with(False)

    def test_fault_handler_calls_set_plc_error(self):
        plc = MagicMock()
        node = self._make_node("fh_plc_err", set_plc_fn=plc)
        node.update()
        plc.assert_called_once_with("error")

    def test_fault_handler_calls_log_error_fn(self):
        """fire-and-forget 스레드가 log_error_fn을 호출하는지 확인한다."""
        log_called = threading.Event()
        log_fn = MagicMock(side_effect=lambda *_: log_called.set())
        node = self._make_node("fh_log_err", log_error_fn=log_fn)
        node.update()
        # 스레드 완료 대기 (최대 2초)
        log_called.wait(timeout=2.0)
        log_fn.assert_called_once()

    def test_fault_handler_returns_failure_even_if_publish_raises(self):
        pub = MagicMock(side_effect=RuntimeError("pub crash"))
        node = self._make_node("fh_pub_raise", publish_status_fn=pub)
        status = node.update()
        assert status == FAILURE


# ═══════════════════════════════════════════════════════════════════════════════
# RunAction — max_attempts 재시도 로직 테스트
# ═══════════════════════════════════════════════════════════════════════════════

class TestRunActionMaxAttempts:
    """_send_and_wait를 monkeypatch해 retry 로직만 검증한다."""

    def _make_node(self, name: str, max_attempts: int) -> RunAction:
        client = MagicMock()
        node = RunAction(
            name=name,
            action_client=client,
            build_goal_fn=lambda tool_id: MagicMock(),
            timeout_sec=5.0,
            max_attempts=max_attempts,
        )
        _write_bb(f"setup_{name}", "socket_19mm")
        return node

    def test_run_action_single_attempt_success(self, monkeypatch):
        node = self._make_node("ra_single_ok", max_attempts=1)
        monkeypatch.setattr(node, "_send_and_wait", lambda _: SUCCESS)
        assert node.update() == SUCCESS

    def test_run_action_single_attempt_failure(self, monkeypatch):
        node = self._make_node("ra_single_fail", max_attempts=1)
        monkeypatch.setattr(node, "_send_and_wait", lambda _: FAILURE)
        assert node.update() == FAILURE

    def test_run_action_retries_until_success(self, monkeypatch):
        """처음 2회 FAILURE, 3번째 SUCCESS → SUCCESS 반환, 총 3회 호출."""
        call_count = 0

        def fake_send(goal):
            nonlocal call_count
            call_count += 1
            return SUCCESS if call_count >= 3 else FAILURE

        node = self._make_node("ra_retry_ok", max_attempts=3)
        monkeypatch.setattr(node, "_send_and_wait", fake_send)
        result = node.update()
        assert result == SUCCESS
        assert call_count == 3

    def test_run_action_all_retries_exhausted(self, monkeypatch):
        """max_attempts=3, 모두 FAILURE → FAILURE, 총 3회 호출."""
        call_count = 0

        def fake_send(goal):
            nonlocal call_count
            call_count += 1
            return FAILURE

        node = self._make_node("ra_all_fail", max_attempts=3)
        monkeypatch.setattr(node, "_send_and_wait", fake_send)
        result = node.update()
        assert result == FAILURE
        assert call_count == 3

    def test_run_action_success_on_first_no_extra_calls(self, monkeypatch):
        """첫 시도 SUCCESS → 불필요한 재시도 없음 (호출 횟수 1)."""
        call_count = 0

        def fake_send(goal):
            nonlocal call_count
            call_count += 1
            return SUCCESS

        node = self._make_node("ra_first_ok", max_attempts=3)
        monkeypatch.setattr(node, "_send_and_wait", fake_send)
        result = node.update()
        assert result == SUCCESS
        assert call_count == 1


# ═══════════════════════════════════════════════════════════════════════════════
# 서브트리 구조 검증 — build_fetch_subtree / build_return_subtree
# ═══════════════════════════════════════════════════════════════════════════════

class TestFetchSubtreeStructure:
    """build_fetch_subtree가 올바른 트리 구조를 조립하는지 검증한다."""

    def _build(self, **kwargs) -> py_trees.behaviour.Behaviour:
        feasibility_cli, exec_cli = _make_mock_clients()
        defaults = dict(
            feasibility_client=feasibility_cli,
            execute_phase_client=exec_cli,
            publish_status_fn=MagicMock(),
            set_plc_fn=MagicMock(),
            log_error_fn=MagicMock(),
        )
        defaults.update(kwargs)
        return build_fetch_subtree(**defaults)

    def test_fetch_subtree_root_is_sequence(self):
        root = self._build()
        assert isinstance(root, py_trees.composites.Sequence)
        assert root.name == "FetchTool_root"

    def test_fetch_subtree_has_check_feasibility_and_selector(self):
        root = self._build()
        children = root.children
        assert len(children) == 2
        from orchestrator.bt_nodes.check_feasibility import CheckFeasibility
        assert isinstance(children[0], CheckFeasibility)
        assert isinstance(children[1], py_trees.composites.Selector)

    def test_fetch_subtree_main_seq_has_five_nodes(self):
        """Selector의 첫 번째 자식(main_seq)이 5개 자식을 가진다:
        SetMoving_true, RunAction×3, SetMoving_false."""
        root = self._build()
        selector = root.children[1]
        main_seq = selector.children[0]
        assert isinstance(main_seq, py_trees.composites.Sequence)
        assert len(main_seq.children) == 5

    def test_fetch_subtree_main_seq_starts_and_ends_with_set_moving(self):
        root = self._build()
        main_seq = root.children[1].children[0]
        assert isinstance(main_seq.children[0], SetMoving)
        assert isinstance(main_seq.children[-1], SetMoving)

    def test_fetch_subtree_fault_handler_is_second_selector_child(self):
        root = self._build()
        selector = root.children[1]
        assert isinstance(selector.children[1], FaultHandlerNode)

    def test_fetch_subtree_run_action_nodes_in_main_seq(self):
        """main_seq의 중간 3개 자식이 RunAction 인스턴스다."""
        root = self._build()
        main_seq = root.children[1].children[0]
        middle = main_seq.children[1:4]
        assert all(isinstance(n, RunAction) for n in middle)

    def test_fetch_subtree_run_action_fetch_has_max_attempts_3(self):
        """RunAction_fetch의 max_attempts는 기본값 3이다."""
        root = self._build()
        main_seq = root.children[1].children[0]
        fetch_node = main_seq.children[2]  # open_drawer(1), fetch(2), close_drawer(3)
        assert fetch_node.name == "RunAction_fetch"
        assert fetch_node._max_attempts == 3


class TestReturnSubtreeStructure:

    def _build(self, **kwargs) -> py_trees.behaviour.Behaviour:
        feasibility_cli, exec_cli = _make_mock_clients()
        defaults = dict(
            feasibility_client=feasibility_cli,
            execute_phase_client=exec_cli,
            publish_status_fn=MagicMock(),
            set_plc_fn=MagicMock(),
            log_error_fn=MagicMock(),
        )
        defaults.update(kwargs)
        return build_return_subtree(**defaults)

    def test_return_subtree_root_is_sequence(self):
        root = self._build()
        assert isinstance(root, py_trees.composites.Sequence)
        assert root.name == "ReturnTool_root"

    def test_return_subtree_has_check_feasibility_and_selector(self):
        root = self._build()
        children = root.children
        assert len(children) == 2
        from orchestrator.bt_nodes.check_feasibility import CheckFeasibility
        assert isinstance(children[0], CheckFeasibility)
        assert isinstance(children[1], py_trees.composites.Selector)

    def test_return_subtree_main_seq_has_five_nodes(self):
        root = self._build()
        selector = root.children[1]
        main_seq = selector.children[0]
        assert isinstance(main_seq, py_trees.composites.Sequence)
        assert len(main_seq.children) == 5

    def test_return_subtree_fault_handler_is_second_selector_child(self):
        root = self._build()
        selector = root.children[1]
        assert isinstance(selector.children[1], FaultHandlerNode)

    def test_return_subtree_run_action_return_has_max_attempts_2(self):
        """RunAction_return의 max_attempts는 기본값 2다."""
        root = self._build()
        main_seq = root.children[1].children[0]
        return_node = main_seq.children[2]
        assert return_node.name == "RunAction_return"
        assert return_node._max_attempts == 2


# ═══════════════════════════════════════════════════════════════════════════════
# RunAction — pick/place 마커 feedback 콜백 회귀 테스트
# ═══════════════════════════════════════════════════════════════════════════════

class TestRunActionFeedbackMarker:
    """sequence_engine이 step.marker 실행 후 발행한 feedback("pick"/"place")이
    RunAction.feedback_callback까지 정확히 전달됨을 보장하는 회귀 테스트.

    _on_feedback 클로저를 send_goal_async 인자 캡처 방식으로 직접 검증한다.
    timeout_sec=0.05로 done.wait을 빠르게 만료시켜 update() 블로킹을 최소화한다.
    """

    def _make_node(self, name: str, feedback_cb=None) -> RunAction:
        client = MagicMock()
        client.wait_for_server.return_value = True
        node = RunAction(
            name=name,
            action_client=client,
            build_goal_fn=lambda tool_id: MagicMock(),
            timeout_sec=0.05,
            max_attempts=1,
            feedback_callback=feedback_cb,
        )
        _write_bb(f"setup_{name}", "screwdriver_phillips_small")
        return node

    def _capture_on_feedback(self, node: RunAction) -> dict:
        """send_goal_async에 전달된 feedback_callback 클로저를 캡처한다."""
        captured: dict = {}

        def fake_sga(goal, feedback_callback=None):
            captured["fn"] = feedback_callback
            fut = MagicMock()
            fut.add_done_callback = MagicMock()  # done.set 미호출 → 타임아웃
            return fut

        node._client.send_goal_async = fake_sga
        node.update()  # 0.05s 타임아웃 후 FAILURE — 정상
        return captured

    def test_pick_marker_forwarded_to_feedback_callback(self):
        """phase='pick' feedback → feedback_callback('pick') 호출."""
        received: list = []
        node = self._make_node("fb_pick", feedback_cb=received.append)
        captured = self._capture_on_feedback(node)

        fb_msg = MagicMock()
        fb_msg.feedback.phase = "pick"
        captured["fn"](fb_msg)

        assert received == ["pick"]

    def test_place_marker_forwarded_to_feedback_callback(self):
        """phase='place' feedback → feedback_callback('place') 호출."""
        received: list = []
        node = self._make_node("fb_place", feedback_cb=received.append)
        captured = self._capture_on_feedback(node)

        fb_msg = MagicMock()
        fb_msg.feedback.phase = "place"
        captured["fn"](fb_msg)

        assert received == ["place"]

    def test_multiple_feedbacks_all_forwarded_in_order(self):
        """pick 후 place 순서로 복수 feedback 모두 전달된다."""
        received: list = []
        node = self._make_node("fb_multi", feedback_cb=received.append)
        captured = self._capture_on_feedback(node)

        for phase in ("pick", "place"):
            fb = MagicMock()
            fb.feedback.phase = phase
            captured["fn"](fb)

        assert received == ["pick", "place"]

    def test_feedback_callback_none_does_not_crash(self):
        """feedback_callback=None일 때 _on_feedback은 예외 없이 종료된다."""
        node = self._make_node("fb_none_safe", feedback_cb=None)
        captured = self._capture_on_feedback(node)

        fb_msg = MagicMock()
        fb_msg.feedback.phase = "pick"
        captured["fn"](fb_msg)  # 예외 없으면 통과

    def test_feedback_callback_exception_is_swallowed(self):
        """feedback_callback이 예외를 던져도 _on_feedback이 삼킨다 (BT 흐름 보호)."""
        crashing_cb = MagicMock(side_effect=RuntimeError("cb crash"))
        node = self._make_node("fb_exc_safe", feedback_cb=crashing_cb)
        captured = self._capture_on_feedback(node)

        fb_msg = MagicMock()
        fb_msg.feedback.phase = "pick"
        captured["fn"](fb_msg)  # 예외 없으면 통과
        crashing_cb.assert_called_once_with("pick")
