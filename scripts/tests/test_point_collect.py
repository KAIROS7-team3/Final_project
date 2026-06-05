"""
get_median_depth() 단위 테스트 — point_collect.py (P-1)

point_collect.py는 모듈 레벨에서 RealSense pipeline과 cv2 윈도우를
초기화하므로 직접 import 대신 함수 로직을 독립 재정의해서 테스트한다.
"""
import numpy as np
import pytest
from unittest.mock import MagicMock

WIDTH = 640
HEIGHT = 480
FILTER_SIZE = 3


def get_median_depth(depth_frame, x: int, y: int, size: int = FILTER_SIZE) -> float:
    """point_collect.py의 get_median_depth와 동일한 로직."""
    depths = []
    for dx in range(-size, size + 1):
        for dy in range(-size, size + 1):
            px, py = x + dx, y + dy
            if px < 0 or py < 0 or px >= WIDTH or py >= HEIGHT:
                continue
            d = depth_frame.get_distance(px, py)
            if d > 0:
                depths.append(d)
    return float(np.median(depths)) if depths else 0.0


def _make_frame(data: dict[tuple[int, int], float]) -> MagicMock:
    frame = MagicMock()
    frame.get_distance.side_effect = lambda x, y: data.get((x, y), 0.0)
    return frame


class TestGetMedianDepth:
    def test_uniform_depth_returns_correct_median(self):
        depth = 0.95
        data = {(x, y): depth for x in range(317, 324) for y in range(237, 244)}
        result = get_median_depth(_make_frame(data), 320, 240)
        assert abs(result - depth) < 1e-6

    def test_outlier_rejected_by_median(self):
        data = {(x, y): 0.95 for x in range(317, 324) for y in range(237, 244)}
        data[(320, 240)] = 2.0
        result = get_median_depth(_make_frame(data), 320, 240)
        assert result < 1.0

    def test_zero_depth_pixels_excluded(self):
        data = {(320, 240): 0.90, (321, 240): 0.91}
        result = get_median_depth(_make_frame(data), 320, 240)
        assert 0.89 < result < 0.92

    def test_all_zero_returns_zero(self):
        result = get_median_depth(_make_frame({}), 320, 240)
        assert result == 0.0

    def test_boundary_pixel_no_index_error(self):
        data = {(0, 0): 0.80}
        result = get_median_depth(_make_frame(data), 0, 0)
        assert result > 0.0

    def test_filter_size_respected(self):
        call_coords: list[tuple[int, int]] = []
        frame = MagicMock()
        def get_dist(x, y):
            call_coords.append((x, y))
            return 0.95
        frame.get_distance.side_effect = get_dist
        get_median_depth(frame, 10, 10, size=1)
        assert len(call_coords) == 9
