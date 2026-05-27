"""Phase 1 bring-up: D455f RGB + aligned depth 스트림 수신 검증."""
import time

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image

_TIMEOUT_SEC = 15.0
_MIN_FRAMES = 5
_DEPTH_SCALE = 0.001


class _StreamCollector(Node):
    def __init__(self) -> None:
        super().__init__("_stream_collector")
        self.frames: list[tuple[np.ndarray, np.ndarray]] = []
        self._bridge = CvBridge()

        rgb_sub = Subscriber(self, Image, "/d455f/color/image_raw")
        depth_sub = Subscriber(self, Image, "/d455f/aligned_depth_to_color/image_raw")
        sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.05)
        sync.registerCallback(self._cb)

    def _cb(self, rgb_msg: Image, depth_msg: Image) -> None:
        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32) * _DEPTH_SCALE
        self.frames.append((rgb, depth))


@pytest.fixture(scope="module")
def collected_frames():
    rclpy.init()
    node = _StreamCollector()
    deadline = time.monotonic() + _TIMEOUT_SEC
    while time.monotonic() < deadline and len(node.frames) < _MIN_FRAMES:
        rclpy.spin_once(node, timeout_sec=0.1)
    frames = list(node.frames)
    node.destroy_node()
    rclpy.shutdown()
    return frames


def test_enough_frames_received(collected_frames):
    assert len(collected_frames) >= _MIN_FRAMES, (
        f"D455f 스트림 미수신 — {_TIMEOUT_SEC}초 내 {_MIN_FRAMES}프레임 이상 필요. "
        "카메라 USB 3.x 연결 및 realsense_bringup.launch.py 실행 여부 확인."
    )


def test_rgb_resolution(collected_frames):
    rgb, _ = collected_frames[0]
    h, w = rgb.shape[:2]
    assert (w, h) == (1280, 720), f"RGB 해상도 불일치: {w}x{h} (expected 1280x720)"


def test_depth_aligned_resolution(collected_frames):
    rgb, depth = collected_frames[0]
    assert rgb.shape[:2] == depth.shape[:2], (
        f"RGB {rgb.shape[:2]} ≠ depth {depth.shape[:2]} — align_depth.enable 확인"
    )


def test_depth_has_valid_values(collected_frames):
    _, depth = collected_frames[0]
    valid_ratio = (depth > 0).mean()
    assert valid_ratio > 0.5, (
        f"유효 depth 비율 {valid_ratio:.2%} < 50% — 조명·거리·USB 연결 확인"
    )


def test_depth_range_plausible(collected_frames):
    _, depth = collected_frames[0]
    valid = depth[depth > 0]
    mean_m = float(valid.mean())
    assert 0.3 <= mean_m <= 3.0, (
        f"평균 depth {mean_m:.3f}m 가 작업 범위(0.3–3.0m) 밖 — 카메라 위치 확인"
    )
