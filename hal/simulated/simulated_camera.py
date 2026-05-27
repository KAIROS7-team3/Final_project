import logging

import numpy as np

from hal.camera_interface import CameraInterface, CameraIntrinsics

logger = logging.getLogger(__name__)

# D455f approximate intrinsics for 640x480
_DEFAULT_INTRINSICS = CameraIntrinsics(
    fx=385.0, fy=385.0, cx=320.0, cy=240.0,
    width=640, height=480,
    dist_coeffs=[0.0, 0.0, 0.0, 0.0, 0.0],
)


class SimulatedCamera(CameraInterface):
    """Mock camera that returns synthetic frames. No hardware access."""

    def __init__(self) -> None:
        self._streaming = False

    def start(self) -> None:
        self._streaming = True
        logger.info("[SimulatedCamera] stream started")

    def stop(self) -> None:
        self._streaming = False
        logger.info("[SimulatedCamera] stream stopped")

    def is_streaming(self) -> bool:
        return self._streaming

    def get_rgb_frame(self) -> np.ndarray:
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def get_depth_frame(self) -> np.ndarray:
        # Uniform depth at 1.0 m
        return np.ones((480, 640), dtype=np.float32)

    def get_aligned_frames(self) -> tuple[np.ndarray, np.ndarray]:
        return self.get_rgb_frame(), self.get_depth_frame()

    def get_intrinsics(self) -> CameraIntrinsics:
        return _DEFAULT_INTRINSICS
