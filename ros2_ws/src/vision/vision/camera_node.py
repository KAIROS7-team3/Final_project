import logging
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

logger = logging.getLogger(__name__)

_LOG_INTERVAL_FRAMES = 30
_SYNC_SLOP_SEC = 0.05
_DEPTH_SCALE = 0.001  # uint16 → meters (D455f default)
_DEPTH_WARN_ZERO_RATIO = 0.3  # 30% 이상 zero depth면 경고


class CameraNode(Node):
    """D455f RGB + aligned depth 스트림 검증 노드 (Phase 1 bring-up용)."""

    def __init__(self) -> None:
        super().__init__("camera_node")

        self._bridge = CvBridge()
        self._frame_count = 0
        self._t_start = time.monotonic()

        rgb_sub = Subscriber(self, Image, "/d455f/color/image_raw")
        depth_sub = Subscriber(self, Image, "/d455f/aligned_depth_to_color/image_raw")

        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=_SYNC_SLOP_SEC
        )
        self._sync.registerCallback(self._on_rgbd)

        self.create_subscription(
            CameraInfo, "/d455f/color/camera_info", self._on_camera_info, 1
        )

        self.get_logger().info("[camera_node] waiting for /d455f streams...")

    def _on_camera_info(self, msg: CameraInfo) -> None:
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        self.get_logger().info(
            f"[camera_node] intrinsics received - fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}",
        )
        self.destroy_subscription(self.get_subscriptions()[0])

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image) -> None:
        self._frame_count += 1

        if self._frame_count % _LOG_INTERVAL_FRAMES != 0:
            return

        elapsed = time.monotonic() - self._t_start
        fps = self._frame_count / elapsed if elapsed > 0 else 0.0

        rgb = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_m = depth_raw.astype(np.float32) * _DEPTH_SCALE

        valid_mask = depth_m > 0
        zero_ratio = 1.0 - valid_mask.mean()
        depth_min = float(depth_m[valid_mask].min()) if valid_mask.any() else 0.0
        depth_max = float(depth_m[valid_mask].max()) if valid_mask.any() else 0.0
        depth_mean = float(depth_m[valid_mask].mean()) if valid_mask.any() else 0.0

        stamp_skew_ms = abs(
            (rgb_msg.header.stamp.sec - depth_msg.header.stamp.sec) * 1e3
            + (rgb_msg.header.stamp.nanosec - depth_msg.header.stamp.nanosec) * 1e-6
        )

        self.get_logger().info(
            f"[camera_node] frames={self._frame_count} fps={fps:.1f} "
            f"rgb={rgb.shape[1]}x{rgb.shape[0]} "
            f"depth_range=[{depth_min:.3f},{depth_max:.3f}]m mean={depth_mean:.3f}m "
            f"zero_ratio={zero_ratio:.2%} skew={stamp_skew_ms:.1f}ms"
        )

        if zero_ratio > _DEPTH_WARN_ZERO_RATIO:
            self.get_logger().warn(
                f"[camera_node] high zero-depth ratio={zero_ratio:.2%} "
                "— check USB 3.x connection and lighting"
            )

        if stamp_skew_ms > _SYNC_SLOP_SEC * 1000:
            self.get_logger().warn(
                f"[camera_node] timestamp skew={stamp_skew_ms:.1f}ms exceeds slop "
                f"({_SYNC_SLOP_SEC * 1000:.0f}ms)"
            )


def main() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
