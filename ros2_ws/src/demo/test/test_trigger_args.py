"""demo._trigger_args 단위 테스트 (ROS 비의존)."""

from __future__ import annotations

import os
import sys

import pytest

# demo 패키지를 import 경로에 추가 (colcon test 외 plain pytest도 동작).
_DEMO_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _DEMO_PKG_ROOT not in sys.path:
    sys.path.insert(0, _DEMO_PKG_ROOT)

from demo._trigger_args import TriggerArgs, parse_args, strip_ros_args  # noqa: E402


# ── happy path ────────────────────────────────────────────────────────────────

def test_parse_fetch_with_explicit_tool():
    result = parse_args(["fetch", "socket_19mm"])
    assert result == TriggerArgs(intent_type="fetch", tool_id="socket_19mm")


def test_parse_return_uses_default_tool_when_omitted():
    result = parse_args(["return"], default_tool_id="socket_19mm")
    assert result == TriggerArgs(intent_type="return", tool_id="socket_19mm")


def test_parse_normalizes_case_and_whitespace():
    result = parse_args(["  FETCH  ", "  socket_19mm  "])
    assert result.intent_type == "fetch"
    assert result.tool_id == "socket_19mm"


def test_blank_tool_falls_back_to_default():
    result = parse_args(["fetch", "   "], default_tool_id="wrench_8mm")
    assert result.tool_id == "wrench_8mm"


# ── failure path ──────────────────────────────────────────────────────────────

def test_empty_argv_raises():
    with pytest.raises(ValueError, match="intent_type"):
        parse_args([])


def test_unsupported_intent_raises():
    with pytest.raises(ValueError, match="지원하지 않는"):
        parse_args(["open", "socket_19mm"])


def test_cancel_intent_rejected():
    # cancel은 데모 트리거 대상이 아니다 (fetch/return만).
    with pytest.raises(ValueError):
        parse_args(["cancel"])


# ── strip_ros_args ────────────────────────────────────────────────────────────

def test_strip_ros_args_removes_trailing_ros_args():
    argv = ["fetch", "socket_19mm", "--ros-args", "--log-level", "debug"]
    assert strip_ros_args(argv) == ["fetch", "socket_19mm"]


def test_strip_ros_args_noop_without_marker():
    argv = ["fetch", "socket_19mm"]
    assert strip_ros_args(argv) == argv
