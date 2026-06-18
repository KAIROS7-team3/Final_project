"""C270 그리퍼캠 ArUco 마커 기반 공구 3D 좌표 + theta 추출 노드 (Track A/B).

서랍 내부 ArUco 마커를 기준점으로 공구의 3D 좌표와 방향각을 추출한다.
  X, Y : 마커 중심 기준 공구 마스크 무게중심 오프셋 (m)
  Z    : solvePnP로 추정한 카메라~서랍 바닥 거리 − 공구 두께/2
  rz   : PCA 주축 방향각 (deg) → quaternion으로 변환해 orientation에 담음

마커 ID 규칙 (config/toolbox.yaml aruco_front):
  ID 0 → 아랫층 (layer_0, 1층 서랍)
  ID 1 → 윗층   (layer_1, 2층 서랍)

Subscribe:
  /c270/image_raw                  sensor_msgs/Image
  /vision/detections/gripper       vision_msgs/Detection2DArray  ← yolo_node(gripper) 마스크 무게중심

Publish:
  /vision/tool_gripper_pose      geometry_msgs/PoseStamped  (base_link frame, m + rz)
  /vision/debug/gripper_marker   sensor_msgs/Image          (debug 시에만)
"""
from __future__ import annotations

import logging
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rclpy.time
import yaml
from cv_bridge import CvBridge
import math

from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from vision_msgs.msg import Detection2DArray

# 설정 경로는 패키지 파일 위치 기준 레포 루트로 해석한다 (CWD 의존 금지).
# ros2_ws/src/vision/vision/marker_scan_node.py → repo_root (parents[4])
_REPO_ROOT  = Path(__file__).resolve().parents[4]
_CONFIG_CAM = _REPO_ROOT / "config/c270_camera_info.yaml"
_CONFIG_HE  = _REPO_ROOT / "config/c270_hand_eye.yaml"
_CONFIG_TB  = _REPO_ROOT / "config/toolbox.yaml"
_CONFIG_RT  = _REPO_ROOT / "config/runtime.yaml"
_CONFIG_VISION = _REPO_ROOT / "config/vision.yaml"

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


def _select_drawer_marker(
    corners_list: list[np.ndarray], ids: np.ndarray
) -> tuple[np.ndarray, int] | None:
    """검출된 마커 중 유효한 서랍 마커(ID 0/1)를 단일 선택한다.

    PR #49 재검토 Medium-2: 두 층 마커가 동시에 보이면 어느 층의 Z인지
    판단할 근거가 없으므로(현재 열려 있는 서랍을 알려주는 입력이 없음)
    모호한 경우는 처리하지 않고 None을 반환해 호출측이 프레임을 스킵하게 한다.
    """
    valid = [
        (corners, int(mid))
        for corners, mid in zip(corners_list, ids.flatten())
        if int(mid) in _MARKER_ID_TO_LAYER
    ]
    if len(valid) != 1:
        return None
    return valid[0]


class MarkerScanNode(Node):
    """ArUco 마커 기반 공구 3D 좌표 추출 노드."""

    def __init__(self) -> None:
        super().__init__("marker_scan_node")

        self._bridge = CvBridge()
        self._cam_matrix, self._dist_coeffs = self._load_camera()
        self._hand_eye_R, self._hand_eye_t  = self._load_hand_eye()
        self._tool_heights                  = self._load_tool_heights()
        self._marker_size_m: float          = self._load_marker_size()
        self._base_frame, self._gripper_frame = self._load_frames()

        # PR #49 재검토 High: marker_scan_node가 발행하던 좌표가 link_6(TCP)
        # 프레임에서 멈춰 있었음. base_link←link_6 EE 포즈를 TF로 조회해
        # 합성한 뒤에야 인터페이스 계약(base_link)을 만족한다.
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # theta EMA 스무딩 (sin/cos 분리로 각도 wraparound 안전)
        self._theta_ema_s: float | None = None  # sin 누적
        self._theta_ema_c: float | None = None  # cos 누적
        self._THETA_ALPHA = 0.25

        debug_cfg = self._load_debug_flag()

        # 퍼블리셔
        self._pose_pub = self.create_publisher(
            PoseStamped, "/vision/tool_gripper_pose", _QOS_BE10
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

    def _load_frames(self) -> tuple[str, str]:
        """runtime.yaml calibration 섹션에서 TF 프레임 이름 로드.

        기존 코드는 top-level cfg.get("gripper_frame", ...)로 읽어
        실제로는 항상 fallback("link_6")만 반환하고 있었다 (실값은
        `calibration.gripper_frame`에 있음) — base_frame 추가하며 같이 수정.
        """
        with _CONFIG_RT.open() as f:
            cfg = yaml.safe_load(f)
        calib = cfg.get("calibration", {})
        base_frame    = str(calib.get("base_frame", "base_link"))
        gripper_frame = str(calib.get("gripper_frame", "link_6"))
        return base_frame, gripper_frame

    def _load_debug_flag(self) -> bool:
        with _CONFIG_VISION.open() as f:
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

    def _ray_plane_cam(self, u: float, v: float,
                       rvec: np.ndarray, tvec: np.ndarray,
                       tool_h: float) -> np.ndarray | None:
        """픽셀 ray와 기울어진 마커 평면의 교점 → 카메라 프레임 3D 좌표.

        rvec/tvec은 solvePnP 결과. 마커 Z축(법선)을 BASE 변환 없이
        카메라 프레임 안에서 직접 계산하므로 TF 불필요.
        """
        fx = self._cam_matrix[0, 0]
        fy = self._cam_matrix[1, 1]
        cx = self._cam_matrix[0, 2]
        cy = self._cam_matrix[1, 2]

        # 마커 법선 → 카메라 프레임 (rvec Z축)
        import cv2 as _cv2
        R_m2cam, _ = _cv2.Rodrigues(rvec.flatten())
        n_cam = R_m2cam @ np.array([0.0, 0.0, 1.0])
        if n_cam[2] > 0:  # 법선이 카메라 방향 반대면 반전
            n_cam = -n_cam

        # 파지점 평면: 마커 중심에서 법선 반대 방향으로 tool_h/2 (공구 두께)
        P_m = tvec.flatten()
        P_plane = P_m - (tool_h / 2.0) * n_cam

        # ray: 원점(카메라) → 픽셀 방향
        d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])

        denom = np.dot(n_cam, d)
        if abs(denom) < 1e-6:
            return None
        t = np.dot(n_cam, P_plane) / denom
        if t < 0:
            return None
        return d * t

    def _cam_to_tcp(self, p_cam: np.ndarray) -> np.ndarray:
        """카메라 프레임 → TCP(link_6) 프레임 변환 (c270_hand_eye.yaml)."""
        return self._hand_eye_R @ p_cam + self._hand_eye_t

    def _lookup_base_from_gripper(self) -> np.ndarray | None:
        """TF base_frame ← gripper_frame 4×4 변환 조회.

        EE 포즈를 구해야 link_6(TCP) 좌표를 base_link로 합성할 수 있다.
        조회 실패 시 잘못된 프레임으로 좌표를 발행하느니 None을 반환해
        호출측이 이번 프레임을 스킵하도록 한다 (E-5 silent fallback 금지
        — 예외를 삼키되 결과를 무시하지 않고 호출측에 실패를 알린다).
        """
        try:
            tf = self._tf_buf.lookup_transform(
                self._base_frame, self._gripper_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning(
                f"[marker_scan] TF {self._base_frame}<-{self._gripper_frame} "
                f"조회 실패 — 이번 프레임 스킵: {e}"
            )
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = _quat_to_rot(q.x, q.y, q.z, q.w)
        T[:3, 3]  = [t.x, t.y, t.z]
        return T

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

        # 유효한 서랍 마커(ID 0 또는 1) 단일 선택 — 동시에 여러 개 보이면
        # 어느 층 Z인지 판단 근거가 없으므로 이번 프레임은 스킵한다.
        selected = _select_drawer_marker(corners_list, ids)
        if selected is None:
            return
        corners, marker_id = selected
        rvec, tvec = self._marker_pose(corners)
        if tvec is None:
            return
        Z_floor = float(tvec[2])   # 로그용

        # 공구 마스크 무게중심 → 카메라 프레임 3D (서랍 기울기 반영)
        # rvec으로 마커 실제 평면 법선을 구해 ray-plane 교점 계산
        P_cam = self._ray_plane_cam(tool_cx, tool_cy, rvec, tvec, tool_h)
        if P_cam is None:
            return

        # 카메라 → TCP(link_6) 프레임
        P_tcp = self._cam_to_tcp(P_cam)

        # TCP(link_6) → base_link 프레임 — 현재 EE 포즈 합성 (PR #49 재검토 High)
        T_base_gripper = self._lookup_base_from_gripper()
        if T_base_gripper is None:
            return
        P_base = T_base_gripper[:3, :3] @ P_tcp + T_base_gripper[:3, 3]

        # PCA로 공구 마스크 주축 방향각(rz) 계산 + EMA 스무딩
        rz_raw = _compute_pca_rz(gray, tool_cx, tool_cy)
        s = math.sin(math.radians(rz_raw))
        c = math.cos(math.radians(rz_raw))
        if self._theta_ema_s is None:
            self._theta_ema_s, self._theta_ema_c = s, c
        else:
            self._theta_ema_s = self._THETA_ALPHA * s + (1.0 - self._THETA_ALPHA) * self._theta_ema_s
            self._theta_ema_c = self._THETA_ALPHA * c + (1.0 - self._THETA_ALPHA) * self._theta_ema_c
        rz_deg = math.degrees(math.atan2(self._theta_ema_s, self._theta_ema_c))

        # quaternion 변환 (yaw only: roll=0, pitch=0)
        half_yaw = math.radians(rz_deg) / 2.0
        qw = math.cos(half_yaw)
        qz = math.sin(half_yaw)

        # 발행 — img_msg.header를 그대로 대입하면 frame_id 변경이 원본
        # 메시지(동기화 큐에서 다른 콜백과 공유될 수 있음)까지 변형시키므로
        # stamp만 복사하고 frame_id는 새로 지정한다.
        msg = PoseStamped()
        msg.header.stamp = img_msg.header.stamp
        msg.header.frame_id = self._base_frame
        msg.pose.position.x = float(P_base[0])
        msg.pose.position.y = float(P_base[1])
        msg.pose.position.z = float(P_base[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self._pose_pub.publish(msg)

        self.get_logger().info(
            f"[marker_scan] {tool_id} | marker_id={marker_id} "
            f"({_MARKER_ID_TO_LAYER.get(marker_id, '?')}) | "
            f"Z_floor={Z_floor*1000:.1f}mm | "
            f"BASE=({P_base[0]*1000:.1f}, {P_base[1]*1000:.1f}, {P_base[2]*1000:.1f})mm | "
            f"rz={rz_deg:.1f}°"
        )

        if self._debug_pub is not None:
            vis = self._draw_debug(bgr, corners_list, ids, tool_cx, tool_cy,
                                   tool_id, P_base, Z_floor)
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
        P_base: np.ndarray,
        Z_floor: float,
    ) -> np.ndarray:
        vis = img.copy()
        cv2.aruco.drawDetectedMarkers(vis, corners_list, ids)

        # 마커 중심 — 실제 사용된 서랍 마커(ID 0/1) 기준, 다른 마커가 같이
        # 보여도(layer 모호 시 _on_sync가 이미 스킵하므로 여기 도달 시엔
        # 단일 후보) 잘못된 마커를 표시하지 않도록 동일 선택 로직 재사용
        selected = _select_drawer_marker(corners_list, ids)
        c = selected[0][0] if selected is not None else corners_list[0][0]
        marker_id = selected[1] if selected is not None else int(ids[0][0])
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
            f"BASE ({P_base[0]*1000:.0f},{P_base[1]*1000:.0f},{P_base[2]*1000:.0f})mm",
            (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2,
        )
        cv2.putText(
            vis,
            f"Z_floor={Z_floor*1000:.0f}mm  marker_id={marker_id}",
            (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1,
        )
        return vis


def _compute_pca_rz(gray: np.ndarray, cx: float, cy: float, radius: int = 40) -> float:
    """공구 마스크 근사 영역에서 PCA 주축 방향각(deg) 계산.

    Detection2DArray는 마스크 픽셀을 제공하지 않으므로 bbox 중심 주변
    원형 ROI의 엣지 픽셀을 마스크 대용으로 사용한다.
    PCA가 불안정하면 0.0 반환.
    """
    h, w = gray.shape
    y0, y1 = max(0, int(cy) - radius), min(h, int(cy) + radius)
    x0, x1 = max(0, int(cx) - radius), min(w, int(cx) + radius)
    roi = gray[y0:y1, x0:x1]
    edges = cv2.Canny(roi, 50, 150)
    pts = np.argwhere(edges > 0).astype(np.float32)
    if len(pts) < 5:
        return 0.0
    mean = pts.mean(axis=0)
    centered = pts - mean
    x, y = centered[:, 1], centered[:, 0]
    n = len(pts)
    cov = np.array([[np.sum(x**2)/n, np.sum(x*y)/n],
                    [np.sum(x*y)/n,  np.sum(y**2)/n]], dtype=np.float64)
    eigenvalues, eigenvectors = np.linalg.eigh(cov)
    v1 = eigenvectors[:, np.argmax(eigenvalues)]
    # 180° 모호성 해소: 엣지 포인트를 v1에 투영해 더 긴 쪽을 +v1로 고정
    # centered shape: (N, 2) — row=y, col=x 순서이므로 v1도 (col, row) 매핑
    proj = centered[:, 1] * v1[0] + centered[:, 0] * v1[1]
    if abs(proj.min()) > abs(proj.max()):
        v1 = -v1
    return float(math.degrees(math.atan2(float(v1[1]), float(v1[0]))))


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
