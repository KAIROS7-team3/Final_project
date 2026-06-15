"""C270 그리퍼캠 ArUco 마커 기반 공구 3D 좌표 추출 노드 (Track A/B).

서랍 내부 ArUco 마커를 기준점으로 공구의 3D 좌표를 추출한다.
  X, Y : 마커 중심 기준 공구 마스크 무게중심 오프셋 (m)
  Z    : solvePnP로 추정한 카메라~서랍 바닥 거리 − 공구 두께/2

마커 ID 규칙 (config/toolbox.yaml aruco_front):
  ID 0 → 아랫층 (layer_0, 1층 서랍)
  ID 1 → 윗층   (layer_1, 2층 서랍)

Subscribe:
  /c270/image_raw                  sensor_msgs/Image
  /vision/detections/gripper       vision_msgs/Detection2DArray  ← yolo_node(gripper) 마스크 무게중심

Publish:
  /vision/tool_gripper_pose      geometry_msgs/PointStamped  (TCP frame, m)
  /vision/debug/gripper_marker   sensor_msgs/Image           (debug 시에만)
"""
from __future__ import annotations

import logging
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

_CONFIG_CAM = Path("config/c270_camera_info.yaml")
_CONFIG_HE  = Path("config/c270_hand_eye.yaml")
_CONFIG_TB  = Path("config/toolbox.yaml")
_CONFIG_RT  = Path("config/runtime.yaml")

_ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters()
_DETECTOR     = cv2.aruco.ArucoDetector(_ARUCO_DICT, _ARUCO_PARAMS)

_MARKER_ID_TO_LAYER: dict[int, str] = {0: "layer_0(아랫층)", 1: "layer_1(윗층)"}

_QOS_BE10 = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """쿼터니언 → 3×3 회전행렬."""
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


class MarkerScanNode(Node):
    """ArUco 마커 기반 공구 3D 좌표 추출 노드."""

    def __init__(self) -> None:
        super().__init__("marker_scan_node")

        self._bridge = CvBridge()
        self._cam_matrix, self._dist_coeffs = self._load_camera()
        self._hand_eye_R, self._hand_eye_t  = self._load_hand_eye()
        self._tool_heights                  = self._load_tool_heights()
        self._marker_size_m: float          = self._load_marker_size()
        self._gripper_frame: str            = self._load_gripper_frame()

        debug_cfg = self._load_debug_flag()

        # 퍼블리셔
        self._pose_pub = self.create_publisher(
            PointStamped, "/vision/tool_gripper_pose", _QOS_BE10
        )
        self._debug_pub = (
            self.create_publisher(Image, "/vision/debug/gripper_marker", 1)
            if debug_cfg else None
        )

        # 이미지 + 검출 결과 동기화 구독
        img_sub = Subscriber(self, Image, "/c270/image_raw", qos_profile=qos_profile_sensor_data)
        det_sub = Subscriber(self, Detection2DArray, "/vision/detections/gripper", qos_profile=_QOS_BE10)
        self._sync = ApproximateTimeSynchronizer([img_sub, det_sub], queue_size=10, slop=0.1)
        self._sync.registerCallback(self._on_sync)

        self.get_logger().info(
            f"[marker_scan_node] ready — marker_size={self._marker_size_m*100:.1f}cm "
            f"tool_heights={self._tool_heights}"
        )

    # ─── 설정 로드 ────────────────────────────────────────────────────────────

    def _load_camera(self) -> tuple[np.ndarray, np.ndarray]:
        with _CONFIG_CAM.open() as f:
            cfg = yaml.safe_load(f)
        K = np.array(cfg["camera_matrix_row_major"], dtype=np.float64)
        D = np.array(cfg["intrinsics"]["coeffs"], dtype=np.float64)
        return K, D

    def _load_hand_eye(self) -> tuple[np.ndarray, np.ndarray]:
        with _CONFIG_HE.open() as f:
            cfg = yaml.safe_load(f)
        rot = cfg["transformation"]["rotation"]
        tra = cfg["transformation"]["translation"]
        R = _quat_to_rot(rot["x"], rot["y"], rot["z"], rot["w"])
        t = np.array([tra["x"], tra["y"], tra["z"]], dtype=np.float64)
        return R, t

    def _load_tool_heights(self) -> dict[str, float]:
        with _CONFIG_TB.open() as f:
            cfg = yaml.safe_load(f)
        heights: dict[str, float] = {}
        for tool in cfg.get("tools", []):
            dims = tool["dimensions"]
            h = float(dims.get("height", dims.get("diameter", 0.020)))
            heights[tool["tool_id"]] = h
        return heights

    def _load_marker_size(self) -> float:
        with _CONFIG_TB.open() as f:
            cfg = yaml.safe_load(f)
        return float(cfg["aruco_front"]["marker_size_m"])

    def _load_gripper_frame(self) -> str:
        with _CONFIG_RT.open() as f:
            cfg = yaml.safe_load(f)
        return str(cfg.get("gripper_frame", "link_6"))

    def _load_debug_flag(self) -> bool:
        with Path("config/vision.yaml").open() as f:
            cfg = yaml.safe_load(f)
        return bool(cfg.get("debug", {}).get("publish_annotated_image", True))

    # ─── 핵심 계산 ───────────────────────────────────────────────────────────

    def _marker_pose(
        self, corners: np.ndarray
    ) -> tuple[np.ndarray, float] | tuple[None, None]:
        """solvePnP로 마커 포즈 추정. (rvec, tvec) 반환. tvec[2] = Z (m)."""
        half = self._marker_size_m / 2.0
        obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)
        img_pts = corners[0].astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, self._cam_matrix, self._dist_coeffs
        )
        if not ok:
            return None, None
        return rvec, tvec.flatten()

    def _pixel_to_cam(self, u: float, v: float, Z: float) -> np.ndarray:
        """픽셀 좌표 (u, v) + 깊이 Z → 카메라 프레임 3D 좌표."""
        fx = self._cam_matrix[0, 0]
        fy = self._cam_matrix[1, 1]
        cx = self._cam_matrix[0, 2]
        cy = self._cam_matrix[1, 2]
        return np.array([(u - cx) * Z / fx, (v - cy) * Z / fy, Z])

    def _cam_to_tcp(self, p_cam: np.ndarray) -> np.ndarray:
        """카메라 프레임 → TCP 프레임 변환 (c270_hand_eye.yaml)."""
        return self._hand_eye_R @ p_cam + self._hand_eye_t

    # ─── 콜백 ────────────────────────────────────────────────────────────────

    def _on_sync(self, img_msg: Image, det_msg: Detection2DArray) -> None:
        if not det_msg.detections:
            return

        bgr = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        corners_list, ids, _ = _DETECTOR.detectMarkers(gray)
        if ids is None:
            return

        # 가장 신뢰도 높은 검출 하나 선택
        best = max(det_msg.detections, key=lambda d: d.results[0].hypothesis.score)
        tool_id = best.results[0].hypothesis.class_id
        tool_cx = best.bbox.center.position.x
        tool_cy = best.bbox.center.position.y
        tool_h  = self._tool_heights.get(tool_id, 0.020)

        # 유효한 서랍 마커(ID 0 또는 1) 중 첫 번째 사용
        rvec, tvec, marker_id = None, None, None
        for corners, mid in zip(corners_list, ids.flatten()):
            mid = int(mid)
            if mid in _MARKER_ID_TO_LAYER:
                rvec, tvec = self._marker_pose(corners)
                marker_id = mid
                break
        if tvec is None:
            return
        Z_floor     = float(tvec[2])            # 카메라 ~ 서랍 바닥 거리 (m)
        Z_grasp     = Z_floor - tool_h / 2.0   # 공구 두께 절반만큼 올림

        # 공구 마스크 무게중심 → 카메라 프레임 3D
        P_cam = self._pixel_to_cam(tool_cx, tool_cy, Z_grasp)

        # 카메라 → TCP 프레임
        P_tcp = self._cam_to_tcp(P_cam)

        # 발행
        msg = PointStamped()
        msg.header = img_msg.header
        msg.header.frame_id = self._gripper_frame
        msg.point.x = float(P_tcp[0])
        msg.point.y = float(P_tcp[1])
        msg.point.z = float(P_tcp[2])
        self._pose_pub.publish(msg)

        self.get_logger().info(
            f"[marker_scan] {tool_id} | marker_id={marker_id} "
            f"({_MARKER_ID_TO_LAYER.get(marker_id, '?')}) | "
            f"Z_floor={Z_floor*1000:.1f}mm | "
            f"TCP=({P_tcp[0]*1000:.1f}, {P_tcp[1]*1000:.1f}, {P_tcp[2]*1000:.1f})mm"
        )

        if self._debug_pub is not None:
            vis = self._draw_debug(bgr, corners_list, ids, tool_cx, tool_cy,
                                   tool_id, P_tcp, Z_floor)
            self._debug_pub.publish(
                self._bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            )

    def _draw_debug(
        self,
        img: np.ndarray,
        corners_list: list,
        ids: np.ndarray,
        tool_cx: float,
        tool_cy: float,
        tool_id: str,
        P_tcp: np.ndarray,
        Z_floor: float,
    ) -> np.ndarray:
        vis = img.copy()
        cv2.aruco.drawDetectedMarkers(vis, corners_list, ids)

        # 마커 중심
        c = corners_list[0][0]
        mx, my = int(c[:, 0].mean()), int(c[:, 1].mean())
        cv2.circle(vis, (mx, my), 6, (0, 255, 255), -1)

        # 공구 중심
        tx, ty = int(tool_cx), int(tool_cy)
        cv2.circle(vis, (tx, ty), 8, (0, 165, 255), -1)
        cv2.line(vis, (mx, my), (tx, ty), (255, 255, 0), 1)

        # 오프셋 벡터 표시
        cv2.putText(vis, tool_id, (tx + 8, ty - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
        cv2.putText(
            vis,
            f"TCP ({P_tcp[0]*1000:.0f},{P_tcp[1]*1000:.0f},{P_tcp[2]*1000:.0f})mm",
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
        )
        cv2.putText(
            vis,
            f"Z_floor={Z_floor*1000:.0f}mm  marker_id={int(ids[0][0])}",
            (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1,
        )
        return vis


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
