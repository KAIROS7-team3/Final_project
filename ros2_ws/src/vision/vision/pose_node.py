"""6D 포즈 추정 노드 (Track A/B).

Subscribe : /vision/detections                      (vision_msgs/Detection2DArray)
            /d455f/aligned_depth_to_color/image_raw (sensor_msgs/Image)
Publish   : /vision/tool_poses                      (vision_msgs/Detection3DArray)

처리 흐름:
  1. YOLOv8 bbox 중심 + aligned depth → 3D 점 (카메라 좌표계)
  2. hand_eye.yaml T_cam_to_base 적용 → base_link 좌표계
  3. Detection3D로 발행 (position 확정, orientation 탑뷰 기본값)

hand_eye.yaml 미캘리브레이션 시: 카메라 좌표계 그대로 발행 + 경고.
Phase 2에서 ICP/FoundationPose 도입 시 orientation 정밀화 예정.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from vision.hand_eye_loader import HandEyeNotCalibratedError, camera_to_base, load_transform

_QOS_BEST_EFFORT_10 = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)
_QOS_BEST_EFFORT_5 = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

_CAMERA_INFO_PATH = Path("config/camera_info.yaml")
_HAND_EYE_PATH = Path("config/hand_eye.yaml")
_DEPTH_SCALE = 0.001        # uint16 → meters
_BBOX_INNER_RATIO = 0.5     # depth 샘플링 시 bbox 내부 비율 (가장자리 배경 제거)
_MIN_VALID_DEPTH_PX = 10    # 유효 depth 픽셀 최소 개수


@dataclass(frozen=True)
class _Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


def _load_intrinsics() -> _Intrinsics:
    with _CAMERA_INFO_PATH.open() as f:
        cfg = yaml.safe_load(f)["intrinsics"]
    return _Intrinsics(fx=cfg["fx"], fy=cfg["fy"], cx=cfg["cx"], cy=cfg["cy"])


def _deproject(u: float, v: float, depth_m: float, K: _Intrinsics) -> np.ndarray:
    """픽셀 (u, v) + depth → 카메라 좌표계 3D 점 [m]."""
    x = (u - K.cx) * depth_m / K.fx
    y = (v - K.cy) * depth_m / K.fy
    return np.array([x, y, depth_m], dtype=np.float64)


def _sample_depth(depth_img: np.ndarray, cx: float, cy: float, w: float, h: float) -> float | None:
    """bbox 내부 중심 영역의 median depth [m]. 유효 픽셀 부족 시 None."""
    iw = w * _BBOX_INNER_RATIO / 2
    ih = h * _BBOX_INNER_RATIO / 2
    x0 = max(0, int(cx - iw))
    x1 = min(depth_img.shape[1], int(cx + iw))
    y0 = max(0, int(cy - ih))
    y1 = min(depth_img.shape[0], int(cy + ih))

    roi = depth_img[y0:y1, x0:x1]
    valid = roi[roi > 0]
    if len(valid) < _MIN_VALID_DEPTH_PX:
        return None
    return float(np.median(valid)) * _DEPTH_SCALE


class PoseNode(Node):
    """YOLOv8 bbox + aligned depth → 3D 공구 포즈 추정 노드."""

    def __init__(self) -> None:
        super().__init__("pose_node")

        self._K = _load_intrinsics()
        self._T: np.ndarray | None = self._try_load_hand_eye()
        self._bridge = CvBridge()

        det_sub = Subscriber(self, Detection2DArray, "/vision/detections",
                             qos_profile=_QOS_BEST_EFFORT_10)
        depth_sub = Subscriber(self, Image, "/d455f/aligned_depth_to_color/image_raw",
                               qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [det_sub, depth_sub], queue_size=10, slop=0.05
        )
        self._sync.registerCallback(self._on_detections_depth)

        self._pose_pub = self.create_publisher(
            Detection3DArray, "/vision/tool_poses", _QOS_BEST_EFFORT_5
        )

        self.get_logger().info(
            f"[pose_node] ready - calibrated={self._T is not None}"
        )

    def _try_load_hand_eye(self) -> np.ndarray | None:
        try:
            T = load_transform(_HAND_EYE_PATH)
            self.get_logger().info("[pose_node] hand-eye transform loaded")
            return T
        except HandEyeNotCalibratedError:
            self.get_logger().warn(
                "[pose_node] hand-eye 미캘리브레이션 — 포즈를 카메라 좌표계로 발행. "
                "Phase 1 캘리브레이션 완료 후 노드 재기동 필요."
            )
            return None
        except FileNotFoundError:
            self.get_logger().warn(
                "[pose_node] config/hand_eye.yaml 없음 — "
                "scripts/calibrate_hand_eye.sh 실행 후 노드 재기동."
            )
            return None

    def _on_detections_depth(self, det_msg: Detection2DArray, depth_msg: Image) -> None:
        depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        pose_array = Detection3DArray()
        pose_array.header = det_msg.header
        # interfaces.md §5: calibrated → base_link, 미캘리브 → camera_optical_frame
        pose_array.header.frame_id = "base_link" if self._T is not None else "camera_optical_frame"

        for det2d in det_msg.detections:
            tool_id = det2d.results[0].hypothesis.class_id
            score = det2d.results[0].hypothesis.score
            cx = det2d.bbox.center.position.x
            cy = det2d.bbox.center.position.y
            bw = det2d.bbox.size_x
            bh = det2d.bbox.size_y

            depth_m = _sample_depth(depth_raw, cx, cy, bw, bh)
            if depth_m is None:
                self.get_logger().warn(
                    f"[pose_node] depth 샘플링 실패 - tool_id={tool_id} "
                    f"bbox=({cx:.0f},{cy:.0f},{bw:.0f}x{bh:.0f}) "
                    "— 금속 반사 또는 bbox 영역 확인"
                )
                continue

            point_cam = _deproject(cx, cy, depth_m, self._K)
            point_base = camera_to_base(point_cam, self._T) if self._T is not None else point_cam

            det3d = self._build_detection3d(
                det_msg.header, tool_id, score, point_base, bw, bh, depth_m
            )
            pose_array.detections.append(det3d)

        self._pose_pub.publish(pose_array)

        if pose_array.detections:
            self.get_logger().debug(
                f"[pose_node] published {len(pose_array.detections)} poses"
            )

    def _build_detection3d(
        self,
        header,
        tool_id: str,
        score: float,
        pos: np.ndarray,
        bbox_w_px: float,
        bbox_h_px: float,
        depth_m: float,
    ) -> Detection3D:
        det3d = Detection3D()
        det3d.header = header

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = tool_id
        hyp.hypothesis.score = score
        hyp.pose.pose.position.x = float(pos[0])
        hyp.pose.pose.position.y = float(pos[1])
        hyp.pose.pose.position.z = float(pos[2])
        # 탑뷰 기본 orientation: z축 아래방향, in-plane 회전 미결정 (Phase 2 FoundationPose 도입 시 갱신)
        hyp.pose.pose.orientation.w = 1.0
        det3d.results.append(hyp)

        # 3D bbox 크기 추정: px → m 변환 (depth 기반 선형 근사)
        size_x = bbox_w_px * depth_m / self._K.fx
        size_y = bbox_h_px * depth_m / self._K.fy
        size_z = 0.05  # 공구 두께 기본값 [m], Phase 2에서 모델별 실측 교체
        det3d.bbox.center.position.x = float(pos[0])
        det3d.bbox.center.position.y = float(pos[1])
        det3d.bbox.center.position.z = float(pos[2])
        det3d.bbox.center.orientation.w = 1.0
        det3d.bbox.size.x = size_x
        det3d.bbox.size.y = size_y
        det3d.bbox.size.z = size_z

        return det3d


def main() -> None:
    rclpy.init()
    node = PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
