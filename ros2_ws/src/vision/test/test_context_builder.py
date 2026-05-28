"""unit tests — context_builder 순수 Python 헬퍼 (_assign_slot, _build_summary)."""
import numpy as np
import pytest

import conftest  # noqa: F401 — activates ROS2 stubs

from vision.context_builder import ContextBuilder, _SlotInfo, _assign_slot

_build_summary = ContextBuilder._build_summary


# ---------- _assign_slot fixtures ----------

def _make_slots() -> list[_SlotInfo]:
    """3×3 그리드: origin=(0.3,-0.2,0.05), slot_size=0.06×0.06."""
    ox, oy, oz = 0.300, -0.200, 0.050
    sw, sh = 0.060, 0.060
    slots = []
    for r in range(3):
        for c in range(3):
            cx = ox + c * sw + sw / 2
            cy = oy + r * sh + sh / 2
            slots.append(_SlotInfo(row=r, col=c, center=np.array([cx, cy, oz]),
                                   half_w=sw / 2, half_h=sh / 2))
    return slots


# ---------- _assign_slot tests ----------

def test_assign_slot_center_of_slot_00() -> None:
    slots = _make_slots()
    # slot (0,0) 중심
    cx = 0.300 + 0 * 0.060 + 0.030   # 0.330
    cy = -0.200 + 0 * 0.060 + 0.030  # -0.170
    result = _assign_slot(np.array([cx, cy, 0.05]), slots)
    assert result == [0, 0]


def test_assign_slot_center_of_slot_12() -> None:
    slots = _make_slots()
    # slot (1,2): row=1, col=2
    cx = 0.300 + 2 * 0.060 + 0.030   # 0.450
    cy = -0.200 + 1 * 0.060 + 0.030  # -0.110
    result = _assign_slot(np.array([cx, cy, 0.05]), slots)
    assert result == [1, 2]


def test_assign_slot_within_margin() -> None:
    """슬롯 중심에서 margin 이내 점도 할당되어야 한다."""
    slots = _make_slots()
    # slot (0,0) 중심 + 작은 오프셋
    cx = 0.330 + 0.010
    cy = -0.170 + 0.010
    result = _assign_slot(np.array([cx, cy, 0.05]), slots)
    assert result == [0, 0]


def test_assign_slot_outside_all_slots() -> None:
    """슬롯 그리드 밖 (멀리 떨어진) 점은 None 반환."""
    slots = _make_slots()
    # 그리드 중심(0.39, -0.11)에서 1m 이상 떨어진 점
    result = _assign_slot(np.array([2.0, 2.0, 0.05]), slots)
    assert result is None


def test_assign_slot_empty_list() -> None:
    result = _assign_slot(np.array([0.0, 0.0, 0.0]), [])
    assert result is None


# ---------- _build_summary tests ----------

def test_build_summary_no_tools() -> None:
    summary = _build_summary([], [[0, 0], [0, 1], [1, 0]])
    assert "비어 있음" in summary
    assert "3" in summary


def test_build_summary_no_tools_zero_empty() -> None:
    summary = _build_summary([], [])
    assert "비어 있음" in summary
    assert "0" in summary


def test_build_summary_with_tools_in_slot() -> None:
    tools = [
        {"tool_id": "wrench_8mm", "confidence": 0.92,
         "position": {"x": 0.33, "y": -0.17, "z": 0.05}, "slot": [0, 0]},
    ]
    summary = _build_summary(tools, [[0, 1]])
    assert "wrench_8mm" in summary
    assert "슬롯(0,0)" in summary


def test_build_summary_tool_without_slot() -> None:
    tools = [
        {"tool_id": "screwdriver_phillips_small", "confidence": 0.75,
         "position": {"x": 2.0, "y": 2.0, "z": 0.05}, "slot": None},
    ]
    summary = _build_summary(tools, [[0, 0]])
    assert "screwdriver_phillips_small" in summary
    assert "미할당" in summary


def test_build_summary_multiple_tools() -> None:
    tools = [
        {"tool_id": "wrench_8mm", "confidence": 0.91,
         "position": {}, "slot": [0, 0]},
        {"tool_id": "screwdriver_flat_small", "confidence": 0.85,
         "position": {}, "slot": [0, 1]},
    ]
    summary = _build_summary(tools, [])
    assert "wrench_8mm" in summary
    assert "screwdriver_flat_small" in summary
