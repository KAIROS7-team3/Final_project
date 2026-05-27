from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    dist_coeffs: list[float]  # k1, k2, p1, p2, k3


class CameraInterface(ABC):
    @abstractmethod
    def get_rgb_frame(self) -> np.ndarray:
        """Return latest RGB frame as HxWx3 uint8 numpy array."""

    @abstractmethod
    def get_depth_frame(self) -> np.ndarray:
        """Return latest depth frame as HxW float32 array (meters)."""

    @abstractmethod
    def get_aligned_frames(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (rgb, depth) frames aligned to same pixel grid."""

    @abstractmethod
    def get_intrinsics(self) -> CameraIntrinsics:
        """Return camera intrinsic parameters."""

    @abstractmethod
    def is_streaming(self) -> bool:
        """Return True if camera is actively streaming."""

    @abstractmethod
    def start(self) -> None:
        """Start camera stream."""

    @abstractmethod
    def stop(self) -> None:
        """Stop camera stream."""
