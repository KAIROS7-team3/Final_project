"""unit tests — tracker_node 순수 Python 헬퍼 (_Track, _find_closest)."""
import numpy as np
import pytest

import conftest  # noqa: F401 — activates ROS2 stubs

from vision.tracker_node import _Track, TrackerNode


# ---------- _Track tests ----------

def test_track_initial_state() -> None:
    pos = np.array([0.33, -0.17, 0.05])
    t = _Track(tool_id="wrench_8mm", position=pos.copy(), score=0.9)
    assert t.hits == 1
    assert t.misses == 0
    assert t.confirmed is False


def test_track_update_ema() -> None:
    """EMA: new_pos = alpha*incoming + (1-alpha)*old."""
    alpha = 0.3
    old_pos = np.array([0.0, 0.0, 0.0])
    new_pos = np.array([1.0, 0.0, 0.0])
    t = _Track(tool_id="t", position=old_pos.copy(), score=0.8)

    t.update(new_pos, score=0.9, alpha=alpha, det=None)

    expected = alpha * new_pos + (1 - alpha) * old_pos
    np.testing.assert_allclose(t.position, expected, atol=1e-9)


def test_track_update_increments_hits() -> None:
    t = _Track(tool_id="t", position=np.zeros(3), score=0.8)
    assert t.hits == 1
    t.update(np.zeros(3), 0.8, 0.3, None)
    assert t.hits == 2
    t.update(np.zeros(3), 0.8, 0.3, None)
    assert t.hits == 3


def test_track_update_resets_misses() -> None:
    t = _Track(tool_id="t", position=np.zeros(3), score=0.8)
    t.misses = 3
    t.update(np.zeros(3), 0.8, 0.3, None)
    assert t.misses == 0


def test_track_mark_missed_increments_misses() -> None:
    t = _Track(tool_id="t", position=np.zeros(3), score=0.8)
    t.mark_missed()
    assert t.misses == 1
    t.mark_missed()
    assert t.misses == 2


def test_track_mark_missed_resets_hits() -> None:
    t = _Track(tool_id="t", position=np.zeros(3), score=0.8, hits=5)
    t.mark_missed()
    assert t.hits == 0


def test_track_update_stores_det() -> None:
    t = _Track(tool_id="t", position=np.zeros(3), score=0.8)
    sentinel = object()
    t.update(np.zeros(3), 0.8, 0.3, sentinel)
    assert t._last_det is sentinel


# ---------- _find_closest tests ----------

def test_find_closest_empty_list() -> None:
    idx, dist = TrackerNode._find_closest([], np.array([1.0, 0.0, 0.0]))
    assert idx is None
    assert dist == float("inf")


def test_find_closest_single_track() -> None:
    t = _Track(tool_id="t", position=np.array([0.0, 0.0, 0.0]), score=0.9)
    idx, dist = TrackerNode._find_closest([t], np.array([0.1, 0.0, 0.0]))
    assert idx == 0
    assert pytest.approx(dist, abs=1e-9) == 0.1


def test_find_closest_picks_nearest() -> None:
    t0 = _Track(tool_id="t", position=np.array([0.0, 0.0, 0.0]), score=0.9)
    t1 = _Track(tool_id="t", position=np.array([1.0, 0.0, 0.0]), score=0.9)
    t2 = _Track(tool_id="t", position=np.array([0.5, 0.0, 0.0]), score=0.9)

    query = np.array([0.45, 0.0, 0.0])
    idx, dist = TrackerNode._find_closest([t0, t1, t2], query)
    assert idx == 2   # t2 가장 가까움
    assert dist < 0.1


def test_find_closest_equal_distance_returns_first() -> None:
    """거리가 같으면 argmin이 첫 번째를 반환한다."""
    t0 = _Track(tool_id="t", position=np.array([-1.0, 0.0, 0.0]), score=0.9)
    t1 = _Track(tool_id="t", position=np.array([1.0, 0.0, 0.0]), score=0.9)
    idx, dist = TrackerNode._find_closest([t0, t1], np.array([0.0, 0.0, 0.0]))
    assert idx == 0
    assert pytest.approx(dist) == 1.0
