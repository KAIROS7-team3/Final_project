"""탑뷰 D455f ArUco 다중 마커 스캔 노드.

감지된 모든 마커의 ID와 3D 위치를 robot base_link 좌표로 변환 후
/vision/marker/map (MarkerMap) 토픽으로 발행한다.

동작 흐름:
  RGB + aligned-depth 동기화 수신
  → ArUco 마커 전체 감지
  → depth 기반 3D 위치 보정
  → hand_eye.yaml 변환 행렬로 robot base_link 좌표 변환
  → MarkerMap 발행 (place_zone_radius 포함)

핵심 설정: config/vision.yaml aruco 섹션
좌표 단위: m (position), quaternion [x,y,z,w] (rotation) — E-1 준수
DSR SDK(mm·deg) 변환은 motion 노드에서 수행
"""
from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from interfaces.msg import MarkerMap
from vision.hand_eye_loader import HandEyeNotCalibratedError, camera_to_base, load_transform

_ARUCO_DICT_MAP: dict[str, int] = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
}

_DEPTH_SCALE: float = 0.001          # D455f uint16 → m
_SYNC_SLOP_SEC: float = 0.05
_DEPTH_PATCH_HALF: int = 2           # 마커 중심 5×5 패치로 depth median

# config/vision.yaml 은 프로젝트 루트 기준 상대경로 (hand_eye_loader 관례 동일)
_CONFIG_DIR = Path("config")


class MarkerScanNode(Node):
    """탑뷰 D455f로 ArUco 전체 마커 스캔 → MarkerMap 발행."""

    def __init__(self) -> None:
        super().__init__("marker_scan_node")
        self._bridge = CvBridge()

        self._load_camera_intrinsics()
        self._load_aruco_config()
        self._load_hand_eye()
        self._build_detector()

        # 마지막 감지 시각 (stale 판정용)
        self._last_detected_t: float = 0.0

        # 토픽 구독: RGB + aligned depth 동기화
        rgb_sub = Subscriber(self, Image, "/d455f/color/image_raw")
        depth_sub = Subscriber(self, Image, "/d455f/aligned_depth_to_color/image_raw")
        self._sync = ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=_SYNC_SLOP_SEC
        )
        self._sync.registerCallback(self._on_rgbd)

        # 발행
        self._map_pub = self.create_publisher(MarkerMap, "/vision/marker/map", 10)
        self._debug_pub = (
            self.create_publisher(Image, "/vision/marker/debug/image", 10)
            if self._publish_debug
            else None
        )

        self.get_logger().info(
            f"[marker_scan_node] ready — "
            f"dict={self._dict_name} size={self._marker_size_m}m "
            f"zone={self._place_zone_radius_mm}mm "
            f"calibrated={self._calibrated}"
        )

    # ------------------------------------------------------------------
    # 초기화 헬퍼
    # ------------------------------------------------------------------

    def _load_camera_intrinsics(self) -> None:
        path = _CONFIG_DIR / "camera_info.yaml"
        with path.open() as f:
            cfg = yaml.safe_load(f)
        intr = cfg["intrinsics"]
        self._fx = float(intr["fx"])
        self._fy = float(intr["fy"])
        self._cx_intr = float(intr["cx"])
        self._cy_intr = float(intr["cy"])
        coeffs = intr.get("coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])
        self._K = np.array(
            [[self._fx, 0.0, self._cx_intr],
             [0.0, self._fy, self._cy_intr],
             [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self._D = np.array(coeffs, dtype=np.float64)

    def _load_aruco_config(self) -> None:
        path = _CONFIG_DIR / "vision.yaml"
        with path.open() as f:
            cfg = yaml.safe_load(f)
        aruco = cfg.get("aruco", {})
        self._dict_name: str = aruco.get("dictionary", "DICT_4X4_50")
        self._marker_size_m: float = float(aruco.get("marker_size_m", 0.10))
        self._place_zone_radius_mm: float = float(aruco.get("place_zone_radius_mm", 150.0))
        self._stale_timeout_sec: float = float(aruco.get("stale_timeout_sec", 2.0))
        self._publish_debug: bool = bool(aruco.get("publish_debug_image", True))

    def _load_hand_eye(self) -> None:
        try:
            self._T: np.ndarray | None = load_transform(_CONFIG_DIR / "hand_eye.yaml")
            self._calibrated = True
        except HandEyeNotCalibratedError:
            self._T = None
            self._calibrated = False
            self.get_logger().warn(
                "[marker_scan_node] hand_eye.yaml 미캘리브레이션 — "
                "camera_color_optical_frame 기준으로 발행 (robot 변환 없음)"
            )

    def _build_detector(self) -> None:
        dict_id = _ARUCO_DICT_MAP.get(self._dict_name)
        if dict_id is None:
            raise ValueError(
                f"[marker_scan_node] 지원하지 않는 ArUco dictionary: {self._dict_name}"
            )
        aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # ------------------------------------------------------------------
    # 메인 콜백
    # ------------------------------------------------------------------

    def _on_rgbd(self, rgb_msg: Image, depth_msg: Image) -> None:
        bgr = self._bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_raw = self._bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        depth_m = depth_raw.astype(np.float32) * _DEPTH_SCALE

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        msg = self._build_marker_map_msg(rgb_msg.header.stamp, corners, ids, depth_m)
        self._map_pub.publish(msg)

        if self._debug_pub is not None:
            debug = self._annotate(bgr, corners, ids)
            self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug, "bgr8"))

    # ------------------------------------------------------------------
    # MarkerMap 조립
    # ------------------------------------------------------------------

    def _build_marker_map_msg(
        self,
        stamp,
        corners: list,
        ids: np.ndarray | None,
        depth_m: np.ndarray,
    ) -> MarkerMap:
        msg = MarkerMap()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link" if self._calibrated else "camera_color_optical_frame"
        msg.place_zone_radius = self._place_zone_radius_mm * 0.001  # mm → m
        msg.calibrated = self._calibrated

        if ids is None or len(corners) == 0:
            # stale 체크: 일정 시간 감지 없으면 warn
            if time.monotonic() - self._last_detected_t > self._stale_timeout_sec:
                self.get_logger().debug(
                    "[marker_scan_node] 마커 미감지 (stale)"
                )
            return msg

        self._last_detected_t = time.monotonic()

        for i, marker_id in enumerate(ids.flatten()):
            pose = self._estimate_pose(corners[i], depth_m, int(marker_id))
            if pose is None:
                continue
            msg.marker_ids.append(int(marker_id))
            msg.poses_robot.append(pose)

        if msg.marker_ids:
            self.get_logger().debug(
                f"[marker_scan_node] 감지 ID: {msg.marker_ids}"
            )

        return msg

    # ------------------------------------------------------------------
    # 포즈 추정: depth 기반 위치 + ArUco rvec 방향
    # ------------------------------------------------------------------

    def _estimate_pose(
        self,
        corner: np.ndarray,   # shape (1, 4, 2)
        depth_m: np.ndarray,
        marker_id: int,
    ) -> Pose | None:
        """마커 포즈 반환 (position: m, orientation: quaternion). depth=0이면 None."""
        pts = corner[0]  # (4, 2)
        cx_px = float(pts[:, 0].mean())
        cy_px = float(pts[:, 1].mean())

        # depth median (5×5 patch) — 단일 픽셀보다 노이즈에 강건
        z_m = self._sample_depth(depth_m, cx_px, cy_px)
        if z_m is None:
            self.get_logger().warn(
                f"[marker_scan_node] ID={marker_id} depth=0 — 프레임 skip"
            )
            return None

        # depth 기반 3D 역투영 (카메라 좌표, 단위 m)
        x_cam = (cx_px - self._cx_intr) * z_m / self._fx
        y_cam = (cy_px - self._cy_intr) * z_m / self._fy
        pos_cam_m = np.array([x_cam, y_cam, z_m], dtype=np.float64)

        # ArUco rvec → 마커 방향 (카메라 좌표)
        rvec, _, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corner], self._marker_size_m, self._K, self._D
        )
        R_cam, _ = cv2.Rodrigues(rvec[0, 0])

        if self._calibrated and self._T is not None:
            pos_out_m = camera_to_base(pos_cam_m, self._T)
            R_out = self._T[:3, :3] @ R_cam
        else:
            pos_out_m = pos_cam_m
            R_out = R_cam

        # 회전 행렬 → quaternion [x, y, z, w] — E-1
        quat = Rotation.from_matrix(R_out).as_quat()  # scipy: [x, y, z, w]

        pose = Pose()
        pose.position = Point(
            x=float(pos_out_m[0]),
            y=float(pos_out_m[1]),
            z=float(pos_out_m[2]),
        )
        pose.orientation = Quaternion(
            x=float(quat[0]),
            y=float(quat[1]),
            z=float(quat[2]),
            w=float(quat[3]),
        )
        return pose

    def _sample_depth(
        self, depth_m: np.ndarray, cx_px: float, cy_px: float
    ) -> float | None:
        h, w = depth_m.shape
        x0 = max(0, int(cx_px) - _DEPTH_PATCH_HALF)
        y0 = max(0, int(cy_px) - _DEPTH_PATCH_HALF)
        x1 = min(w, int(cx_px) + _DEPTH_PATCH_HALF + 1)
        y1 = min(h, int(cy_px) + _DEPTH_PATCH_HALF + 1)
        valid = depth_m[y0:y1, x0:x1]
        valid = valid[valid > 0]
        if len(valid) == 0:
            return None
        return float(np.median(valid))

    # ------------------------------------------------------------------
    # 디버그 이미지 생성
    # ------------------------------------------------------------------

    def _annotate(
        self,
        bgr: np.ndarray,
        corners: list,
        ids: np.ndarray | None,
    ) -> np.ndarray:
        out = bgr.copy()
        if ids is None or len(corners) == 0:
            return out

        cv2.aruco.drawDetectedMarkers(out, corners, ids)

        for i, corner in enumerate(corners):
            pts = corner[0]
            cx_px = int(pts[:, 0].mean())
            cy_px = int(pts[:, 1].mean())

            # place zone 원 — 마커 한 변 픽셀 길이로 m/px 스케일 추정
            side_px = float(np.linalg.norm(pts[0] - pts[1]))
            if side_px > 0:
                px_per_m = side_px / self._marker_size_m
                zone_px = max(1, int(self._place_zone_radius_mm * 0.001 * px_per_m))
            else:
                zone_px = 30

            cv2.circle(out, (cx_px, cy_px), zone_px, (0, 165, 255), 2)
            zone_m = self._place_zone_radius_mm * 0.001
            label = f"ID:{ids[i][0]}  zone={zone_m:.3f}m"
            cv2.putText(
                out, label,
                (cx_px - 50, cy_px - zone_px - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2,
            )

        return out


def main() -> None:
    rclpy.init()
    node = MarkerScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
