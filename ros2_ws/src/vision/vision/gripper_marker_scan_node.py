"""C270 그리퍼캠 ArUco 마커 기반 공구 3D 좌표 + theta 추출 노드 (Track A/B).

서랍 내부 ArUco 마커를 기준점으로 공구의 3D 좌표와 방향각을 추출한다.
  X, Y : 마커 중심 기준 공구 마스크 무게중심 오프셋 (m)
  Z    : solvePnP로 추정한 카메라~서랍 바닥 거리 − 공구 두께/2
  rz   : PCA 주축 방향각 (deg) → quaternion으로 변환해 orientation에 담음

마커 ID 규칙 (config/toolbox.yaml aruco_front):
  ID 0 → 아랫층 (layer_0, 1층 서랍)
  ID 1 → 윗층   (layer_1, 2층 서랍)
  ID 3 → 바닥   (floor, 작업 테이블 바닥면)

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

import time
import cv2
import numpy as np
import rclpy
import rclpy.time
import yaml
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

# 설정 경로: source/install 어느 쪽에서 실행해도 c270_camera_info.yaml이 있는
# 디렉토리를 레포 루트로 탐색한다 (parents[4] 하드코딩은 install 경로에서 깨짐).
def _find_repo_root() -> Path:
    for p in Path(__file__).resolve().parents:
        if (p / "config" / "c270_camera_info.yaml").exists():
            return p
    raise RuntimeError(
        "[gripper_marker_scan_node] repo root 탐색 실패 — "
        "config/c270_camera_info.yaml 가 없습니다. "
        f"(탐색 기준: {Path(__file__).resolve()})"
    )

_REPO_ROOT = _find_repo_root()
_CONFIG_CAM = _REPO_ROOT / "config/c270_camera_info.yaml"
_CONFIG_HE  = _REPO_ROOT / "config/c270_hand_eye.yaml"
_CONFIG_TB  = _REPO_ROOT / "config/toolbox.yaml"
_CONFIG_RT  = _REPO_ROOT / "config/runtime.yaml"
_CONFIG_VISION = _REPO_ROOT / "config/vision.yaml"

_ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters()
_DETECTOR     = cv2.aruco.ArucoDetector(_ARUCO_DICT, _ARUCO_PARAMS)

_MARKER_ID_TO_LAYER: dict[int, str] = {0: "layer_0(아랫층)", 1: "layer_1(윗층)", 3: "floor(바닥)"}
_DRAWER_MARKER_IDS: frozenset[int] = frozenset({0, 1})  # 서랍 층 마커
_FLOOR_MARKER_IDS:  frozenset[int] = frozenset({3})     # staging 바닥 마커

_QOS_BE10 = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)


# ─── cv_bridge 없이 sensor_msgs/Image ↔ numpy 변환 ──────────────────────────
# cv_bridge의 C++ boost 모듈이 NumPy 2.x와 호환되지 않아 대체 구현.

def _imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → BGR uint8 (3-channel) numpy array."""
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("bgr8", "8uc3"):
        return arr.reshape(msg.height, msg.width, 3).copy()
    if enc in ("rgb8",):
        return arr.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
    if enc in ("bgra8", "8uc4"):
        return arr.reshape(msg.height, msg.width, 4)[:, :, :3].copy()
    if enc in ("rgba8",):
        t = arr.reshape(msg.height, msg.width, 4)
        return t[:, :, [2, 1, 0]].copy()
    if enc in ("mono8", "8uc1"):
        gray = arr.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"[imgmsg_to_bgr] 지원하지 않는 encoding: {enc}")


def _imgmsg_to_mono8(msg: Image) -> np.ndarray:
    """sensor_msgs/Image → mono8 uint8 (2D) numpy array."""
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    enc = msg.encoding.lower()
    if enc in ("mono8", "8uc1"):
        return arr.reshape(msg.height, msg.width).copy()
    if enc in ("bgr8", "8uc3"):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width, 3), cv2.COLOR_BGR2GRAY)
    if enc in ("rgb8",):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2GRAY)
    raise ValueError(f"[imgmsg_to_mono8] 지원하지 않는 encoding: {enc}")


def _bgr_to_imgmsg(arr: np.ndarray) -> Image:
    """BGR uint8 numpy array → sensor_msgs/Image (bgr8)."""
    msg = Image()
    msg.height = arr.shape[0]
    msg.width  = arr.shape[1]
    msg.encoding = "bgr8"
    msg.step     = arr.shape[1] * 3
    msg.data     = arr.tobytes()
    return msg


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
    """검출된 마커 중 사용할 기준 마커를 우선순위에 따라 단일 선택한다.

    우선순위:
      1. 서랍 마커(ID 0/1) 1개 → 사용 (바닥 마커 동시 검출 시에도 서랍 마커 우선)
      2. 서랍 마커 0개 + 바닥 마커(ID 3) 1개 → 사용 (staging return 상황)
      3. 서랍 마커 2개 이상 → 층 모호 → None (PR #49 재검토 Medium-2)
      4. 그 외 → None

    이 로직 덕분에 drawer 스캔 중 바닥 마커가 동시에 보여도 깜빡이지 않는다.
    """
    all_valid = [
        (corners, int(mid))
        for corners, mid in zip(corners_list, ids.flatten())
        if int(mid) in _MARKER_ID_TO_LAYER
    ]
    drawer = [(c, m) for c, m in all_valid if m in _DRAWER_MARKER_IDS]
    if len(drawer) == 1:
        return drawer[0]          # 서랍 마커 단일 → 바닥 마커 무시하고 사용
    if len(drawer) > 1:
        return None               # 두 층 동시 → 모호, 스킵
    floor = [(c, m) for c, m in all_valid if m in _FLOOR_MARKER_IDS]
    if len(floor) == 1:
        return floor[0]           # 서랍 마커 없음 + 바닥 마커 1개 → staging 상황
    return None


class MarkerScanNode(Node):
    """ArUco 마커 기반 공구 3D 좌표 추출 노드."""

    def __init__(self) -> None:
        super().__init__("marker_scan_node")

        self._cam_matrix, self._dist_coeffs = self._load_camera()
        self._hand_eye_R, self._hand_eye_t  = self._load_hand_eye()
        self._tool_heights                  = self._load_tool_heights()
        self._marker_size_m: float          = self._load_marker_size()
        self._base_frame, self._gripper_frame = self._load_frames()
        self._grasp_offset_cfg: dict[str, dict] = self._load_grasp_offset()

        # PR #49 재검토 High: marker_scan_node가 발행하던 좌표가 link_6(TCP)
        # 프레임에서 멈춰 있었음. base_link←link_6 EE 포즈를 TF로 조회해
        # 합성한 뒤에야 인터페이스 계약(base_link)을 만족한다.
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # theta EMA 스무딩 — 공구별 (sin, cos) 쌍으로 저장 (각도 wraparound 안전)
        self._theta_emas: dict[str, tuple[float, float]] = {}
        self._theta_ema_ts: dict[str, float] = {}   # 공구별 마지막 EMA 갱신 시각 (monotonic)
        self._THETA_ALPHA = 0.25
        self._THETA_OUTLIER_DEG = 30.0   # 이 이상 순간 점프하면 해당 프레임 무시
        self._THETA_EMA_RESET_SEC = 3.0  # 이 시간 이상 미검출 시 EMA 리셋 (공구 재배치 대응)

        # 마스크 캐시 — /vision/masks/gripper 구독, _on_sync에서 마스크 PCA 우선 사용
        self._latest_mask: np.ndarray | None = None
        # header.stamp 대신 monotonic 수신 시각 사용 — YOLO가 stamp를 0으로 두거나
        # camera timestamp와 다르게 설정하면 header 기반 비교가 항상 stale 판정됨
        self._latest_mask_rcv_mono: float = 0.0
        self._MASK_STALE_SEC = 1.0
        # 마스크가 어떤 class에 대해 발행됐는지 (yolo_node가 frame_id에 인코딩)
        self._latest_mask_class_id: str = ""
        # per-class 마스크 캐시 — {class_id: (mask_ndarray, rcv_monotonic)}
        self._mask_cache: dict[str, tuple[np.ndarray, float]] = {}

        # _on_sync 조기 return 경고 rate limiting (3s 간격)
        self._last_no_det_warn: float = 0.0
        self._last_no_aruco_warn: float = 0.0
        self._last_ambig_warn: float = 0.0

        # 파지점 debug 발행용 캐시 (ArUco 없이 마스크만으로 시각화)
        self._latest_bgr: np.ndarray | None = None
        self._latest_detections: list = []  # 신뢰도 내림차순 정렬된 전체 검출 목록

        debug_cfg = self._load_debug_flag()

        # 퍼블리셔
        self._pose_pub = self.create_publisher(
            PoseStamped, "/vision/tool_gripper_pose", _QOS_BE10
        )
        self._debug_pub = (
            self.create_publisher(Image, "/vision/debug/gripper_marker", 1)
            if debug_cfg else None
        )

        # 마스크 토픽 별도 구독 (캐싱 방식 — 3-way sync 없이 최신 마스크 재사용)
        self.create_subscription(Image, "/vision/masks/gripper", self._on_mask, _QOS_BE10)

        # debug 발행용 캐시 구독 (ArUco 없이 파지점 시각화)
        self.create_subscription(Image, "/c270/image_raw", self._on_raw_image, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/vision/detections/gripper", self._on_det_cache, _QOS_BE10)

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

    def _load_grasp_offset(self) -> dict[str, dict]:
        with _CONFIG_VISION.open() as f:
            cfg = yaml.safe_load(f)
        return {
            k: {"ratio": float(v["ratio"]), "toward_narrow": bool(v["toward_narrow"])}
            for k, v in cfg.get("grasp_offset", {}).items()
        }

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

    def _on_raw_image(self, msg: Image) -> None:
        """raw 이미지 캐싱 — 파지점 debug 시각화용."""
        self._latest_bgr = _imgmsg_to_bgr(msg)

    def _on_det_cache(self, msg: Detection2DArray) -> None:
        """전체 검출 목록 캐싱 (신뢰도 내림차순) — 파지점 debug 시각화용."""
        self._latest_detections = sorted(
            msg.detections,
            key=lambda d: d.results[0].hypothesis.score,
            reverse=True,
        )

    def _on_mask(self, msg: Image) -> None:
        """yolo_node가 발행하는 class별 이진 마스크 캐싱."""
        mono = _imgmsg_to_mono8(msg)
        rcv = time.monotonic()
        self._latest_mask = mono
        self._latest_mask_rcv_mono = rcv
        self._latest_mask_class_id = msg.header.frame_id
        class_id = msg.header.frame_id
        if class_id:
            self._mask_cache[class_id] = (mono, rcv)
        self._publish_grasp_debug()

    def _publish_grasp_debug(self) -> None:
        """ArUco 없이 마스크 + 원본 이미지만으로 파지점 시각화 debug 이미지 발행.

        모든 검출 공구의 파지점을 표시한다. 마스크는 최고 신뢰도 공구 기준이므로
        PCA offset은 첫 번째 공구에만 적용하고 나머지는 bbox 무게중심으로 표시한다.
        """
        if self._debug_pub is None or self._latest_mask is None or self._latest_bgr is None:
            return

        vis = self._latest_bgr.copy()
        H, W = vis.shape[:2]
        mH, mW = self._latest_mask.shape[:2]
        mask_resized = (
            self._latest_mask if (mH == H and mW == W)
            else cv2.resize(self._latest_mask, (W, H), interpolation=cv2.INTER_NEAREST)
        )

        # 마스크 반투명 오버레이 (주황)
        overlay = np.zeros_like(vis)
        overlay[mask_resized > 0] = (0, 140, 255)
        vis = cv2.addWeighted(vis, 0.65, overlay, 0.35, 0)

        # PCA는 이미지 해상도로 정규화된 마스크로 계산
        pca = _compute_pca_full(mask_resized)

        _COLORS = [(0, 255, 255), (0, 165, 255), (0, 255, 128), (255, 128, 0)]
        detections = self._latest_detections if self._latest_detections else []

        if not detections and pca is not None:
            cx_pca, cy_pca, v1 = pca
            cv2.circle(vis, (int(cx_pca), int(cy_pca)), 7, (160, 160, 160), -1)
            cv2.putText(vis, "no detection", (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 2)
            self._debug_pub.publish(_bgr_to_imgmsg(vis))
            return

        # 마스크가 어느 공구에 해당하는지 centroid 근접성으로 추정
        mask_det_idx: int | None = None
        if pca is not None and detections:
            cx_pca_g, cy_pca_g, _ = pca
            mask_det_idx = min(
                range(len(detections)),
                key=lambda j: (
                    (detections[j].bbox.center.position.x - cx_pca_g) ** 2
                    + (detections[j].bbox.center.position.y - cy_pca_g) ** 2
                ),
            )

        for i, det in enumerate(detections):
            tool_id = det.results[0].hypothesis.class_id
            color = _COLORS[i % len(_COLORS)]

            if i == mask_det_idx and pca is not None:
                cx_pca, cy_pca, v1 = pca
                ax_len = 50
                cv2.arrowedLine(
                    vis,
                    (int(cx_pca), int(cy_pca)),
                    (int(cx_pca + v1[0] * ax_len), int(cy_pca + v1[1] * ax_len)),
                    (255, 255, 255), 2, tipLength=0.25,
                )
                cv2.circle(vis, (int(cx_pca), int(cy_pca)), 5, (160, 160, 160), -1)

                if tool_id in self._grasp_offset_cfg:
                    off = self._grasp_offset_cfg[tool_id]
                    gx, gy = _grasp_point_px(
                        cx_pca, cy_pca, v1, mask_resized,
                        ratio=off["ratio"],
                        toward_narrow=off["toward_narrow"],
                    )
                else:
                    gx, gy = cx_pca, cy_pca
            else:
                gx = det.bbox.center.position.x
                gy = det.bbox.center.position.y

            cv2.circle(vis, (int(gx), int(gy)), 9, color, -1)
            cv2.circle(vis, (int(gx), int(gy)), 11, (0, 0, 0), 2)
            cv2.putText(
                vis,
                f"{tool_id} ({int(gx)},{int(gy)})",
                (10, 26 + i * 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
            )

        self._debug_pub.publish(_bgr_to_imgmsg(vis))

    # ─── 콜백 ────────────────────────────────────────────────────────────────

    def _on_sync(self, img_msg: Image, det_msg: Detection2DArray) -> None:
        if not det_msg.detections:
            now = time.monotonic()
            if now - self._last_no_det_warn > 3.0:
                self.get_logger().warn("[marker_scan] _on_sync: YOLO 검출 없음 — 공구가 스캔 영역에 있는지 확인")
                self._last_no_det_warn = now
            return

        bgr = _imgmsg_to_bgr(img_msg)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        corners_list, ids, _ = _DETECTOR.detectMarkers(gray)
        if ids is None:
            now = time.monotonic()
            if now - self._last_no_aruco_warn > 3.0:
                self.get_logger().warn("[marker_scan] _on_sync: ArUco 미검출 — 마커가 카메라 시야에 있는지 확인")
                self._last_no_aruco_warn = now
            return

        # 유효한 서랍 마커(ID 0/1) 또는 바닥 마커(ID 3) 단일 선택.
        # 동시에 여러 층 마커가 보이면 어느 층 Z인지 판단 근거가 없으므로 스킵.
        selected = _select_drawer_marker(corners_list, ids)
        if selected is None:
            now = time.monotonic()
            if now - self._last_ambig_warn > 3.0:
                detected_ids = ids.flatten().tolist()
                self.get_logger().warn(f"[marker_scan] _on_sync: 마커 선택 불가 — 검출 IDs={detected_ids} (서랍 2개 이상 또는 미지원 ID)")
                self._last_ambig_warn = now
            return
        corners, marker_id = selected
        rvec, tvec = self._marker_pose(corners)
        if tvec is None:
            return
        Z_floor = float(tvec[2])

        # TF 조회는 루프 밖에서 한 번만
        T_base_gripper = self._lookup_base_from_gripper()
        if T_base_gripper is None:
            return

        bgr_h, bgr_w = bgr.shape[:2]
        now_mono = time.monotonic()

        # 신뢰도 내림차순 정렬 후 전체 처리
        detections = sorted(
            det_msg.detections,
            key=lambda d: d.results[0].hypothesis.score,
            reverse=True,
        )

        debug_results: list[tuple[str, float, float, np.ndarray]] = []

        for i, det in enumerate(detections):
            tool_id = det.results[0].hypothesis.class_id
            tool_cx = det.bbox.center.position.x
            tool_cy = det.bbox.center.position.y
            tool_h  = self._tool_heights.get(tool_id, 0.020)

            # 공구별 마스크 캐시 조회 → 개별 PCA + grasp offset 적용
            tool_pca: tuple | None = None
            tool_mask_cam: np.ndarray | None = None
            mask_entry = self._mask_cache.get(tool_id)
            if mask_entry is not None:
                mask_data, rcv_mono = mask_entry
                if (now_mono - rcv_mono) < self._MASK_STALE_SEC:
                    mH, mW = mask_data.shape[:2]
                    tool_mask_cam = (
                        mask_data if (mH == bgr_h and mW == bgr_w)
                        else cv2.resize(mask_data, (bgr_w, bgr_h),
                                        interpolation=cv2.INTER_NEAREST)
                    )
                    tool_pca = _compute_pca_full(tool_mask_cam)

            if tool_pca is not None:
                _cx_pca, _cy_pca, _v1 = tool_pca
                rz_raw = float(math.degrees(math.atan2(float(_v1[1]), float(_v1[0]))))
                if tool_id in self._grasp_offset_cfg:
                    _off = self._grasp_offset_cfg[tool_id]
                    tool_cx, tool_cy = _grasp_point_px(
                        _cx_pca, _cy_pca, _v1, tool_mask_cam,
                        ratio=_off["ratio"],
                        toward_narrow=_off["toward_narrow"],
                    )
                    self.get_logger().debug(
                        f"[marker_scan] {tool_id} grasp_offset 적용: "
                        f"centroid=({_cx_pca:.0f},{_cy_pca:.0f}) → "
                        f"grasp=({tool_cx:.0f},{tool_cy:.0f})"
                    )
            else:
                rz_raw = _compute_pca_rz(gray, tool_cx, tool_cy, radius=60)

            P_cam = self._ray_plane_cam(tool_cx, tool_cy, rvec, tvec, tool_h)
            if P_cam is None:
                continue

            P_tcp = self._cam_to_tcp(P_cam)
            P_base = T_base_gripper[:3, :3] @ P_tcp + T_base_gripper[:3, 3]

            # 공구별 EMA 스무딩 + 이상치 게이팅
            s = math.sin(math.radians(rz_raw))
            c = math.cos(math.radians(rz_raw))
            # 미검출 공백 감지 → EMA 리셋 (공구 재배치 시 이전 각도에 고착되는 현상 방지)
            _last_ts = self._theta_ema_ts.get(tool_id, 0.0)
            if now_mono - _last_ts > self._THETA_EMA_RESET_SEC:
                self._theta_emas.pop(tool_id, None)
            self._theta_ema_ts[tool_id] = now_mono

            ema = self._theta_emas.get(tool_id)
            if ema is None:
                ema_s, ema_c = s, c
                rz_deg = rz_raw
            else:
                ema_s_prev, ema_c_prev = ema
                cur_deg = math.degrees(math.atan2(ema_s_prev, ema_c_prev))
                diff = abs((rz_raw - cur_deg + 180.0) % 360.0 - 180.0)
                if diff > self._THETA_OUTLIER_DEG:
                    ema_s, ema_c = ema_s_prev, ema_c_prev
                    rz_deg = cur_deg
                else:
                    ema_s = self._THETA_ALPHA * s + (1.0 - self._THETA_ALPHA) * ema_s_prev
                    ema_c = self._THETA_ALPHA * c + (1.0 - self._THETA_ALPHA) * ema_c_prev
                    rz_deg = math.degrees(math.atan2(ema_s, ema_c))
            self._theta_emas[tool_id] = (ema_s, ema_c)

            half_yaw = math.radians(rz_deg) / 2.0
            qw = math.cos(half_yaw)
            qz = math.sin(half_yaw)

            # frame_id에 tool_id 임베딩 → orchestrator가 그룹핑에 사용
            pose_msg = PoseStamped()
            pose_msg.header.stamp = img_msg.header.stamp
            pose_msg.header.frame_id = f"tool:{tool_id}"
            pose_msg.pose.position.x = float(P_base[0])
            pose_msg.pose.position.y = float(P_base[1])
            pose_msg.pose.position.z = float(P_base[2])
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw
            self._pose_pub.publish(pose_msg)

            self.get_logger().info(
                f"[marker_scan] {tool_id} | marker_id={marker_id} "
                f"({_MARKER_ID_TO_LAYER.get(marker_id, '?')}) | "
                f"Z_floor={Z_floor*1000:.1f}mm | "
                f"BASE=({P_base[0]*1000:.1f}, {P_base[1]*1000:.1f}, {P_base[2]*1000:.1f})mm | "
                f"rz={rz_deg:.1f}°"
            )

            debug_results.append((tool_id, tool_cx, tool_cy, P_base))

        if self._debug_pub is not None and debug_results:
            vis = self._draw_debug(
                bgr, corners_list, ids, debug_results, Z_floor, marker_id
            )
            self._debug_pub.publish(_bgr_to_imgmsg(vis))

    def _draw_debug(
        self,
        img: np.ndarray,
        corners_list: list,
        ids: np.ndarray,
        results: list[tuple[str, float, float, np.ndarray]],
        Z_floor: float,
        marker_id: int,
    ) -> np.ndarray:
        """모든 검출 공구의 파지점과 BASE 좌표를 오버레이한 debug 이미지 반환."""
        vis = img.copy()
        cv2.aruco.drawDetectedMarkers(vis, corners_list, ids)

        selected = _select_drawer_marker(corners_list, ids)
        c = selected[0][0] if selected is not None else corners_list[0][0]
        mx, my = int(c[:, 0].mean()), int(c[:, 1].mean())
        cv2.circle(vis, (mx, my), 6, (0, 255, 255), -1)

        _COLORS = [(0, 165, 255), (0, 255, 128), (255, 128, 0), (180, 0, 255)]
        for i, (tool_id, gx, gy, P_base) in enumerate(results):
            color = _COLORS[i % len(_COLORS)]
            tx, ty = int(gx), int(gy)
            cv2.circle(vis, (tx, ty), 8, color, -1)
            cv2.line(vis, (mx, my), (tx, ty), (255, 255, 0), 1)
            cv2.putText(vis, tool_id, (tx + 8, ty - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(
                vis,
                f"BASE ({P_base[0]*1000:.0f},{P_base[1]*1000:.0f},{P_base[2]*1000:.0f})mm",
                (10, 26 + i * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
            )

        cv2.putText(
            vis,
            f"Z_floor={Z_floor*1000:.0f}mm  marker_id={marker_id}",
            (10, 26 + len(results) * 22 + 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
        )
        return vis


def _compute_pca_full(
    mask: np.ndarray,
) -> tuple[float, float, np.ndarray] | None:
    """이진 마스크에서 centroid (cx_col, cy_row) + 주축 벡터 v1 반환.

    infer_c270_centroid.py compute_pca()와 동일한 로직.
    Returns:
        (cx, cy, v1) — cx/cy는 이미지 픽셀(col, row),
        v1은 주성분 방향 [vx(col방향), vy(row방향)].
        점이 5개 미만이면 None.
    """
    pts = np.argwhere(mask > 0).astype(np.float32)  # shape (N, 2): [row, col]
    if len(pts) < 5:
        return None
    cy_img = float(pts[:, 0].mean())
    cx_img = float(pts[:, 1].mean())
    centered = pts - np.array([cy_img, cx_img])
    x = centered[:, 1]  # col delta
    y = centered[:, 0]  # row delta
    N = len(pts)
    C = np.array([
        [float(np.sum(x**2) / N), float(np.sum(x * y) / N)],
        [float(np.sum(x * y) / N), float(np.sum(y**2) / N)],
    ], dtype=np.float64)
    eigenvalues, eigenvectors = np.linalg.eig(C)
    order = np.argsort(eigenvalues)[::-1]
    v1 = eigenvectors[:, order[0]]   # [vx, vy]
    # 부호 고정: 투영 양수 쪽이 더 긴 방향
    proj = centered[:, 1] * v1[0] + centered[:, 0] * v1[1]
    if abs(proj.min()) > abs(proj.max()):
        v1 = -v1
    return (cx_img, cy_img, v1)


def _grasp_point_px(
    cx: float, cy: float,
    v1: np.ndarray,
    mask: np.ndarray,
    ratio: float,
    toward_narrow: bool,
) -> tuple[float, float]:
    """PCA 주축 방향으로 centroid를 offset해 그립 포인트 픽셀 좌표 반환.

    infer_c270_centroid.py _grasp_point()와 동일한 로직.
    Args:
        cx, cy: PCA centroid (col, row).
        v1: 주축 단위벡터 [vx(col), vy(row)].
        mask: 이진 마스크 (mono8).
        ratio: 공구 전체 길이 대비 offset 비율.
        toward_narrow: True → 마스크가 좁은 쪽(손잡이)으로 이동.
    Returns:
        (gx_col, gy_row) — 클리핑된 픽셀 좌표.
    """
    pts = np.argwhere(mask > 0).astype(np.float32)
    centered = pts - np.array([cy, cx])
    proj = centered[:, 1] * v1[0] + centered[:, 0] * v1[1]

    length = float(proj.max() - proj.min())
    offset_px = length * ratio

    pos_pts = pts[proj > 0]
    neg_pts = pts[proj < 0]

    def _perp_width(cluster: np.ndarray) -> float:
        if len(cluster) < 3:
            return 0.0
        perp = cluster[:, 1] * (-v1[1]) + cluster[:, 0] * v1[0]
        return float(perp.max() - perp.min())

    pos_narrow = _perp_width(pos_pts) < _perp_width(neg_pts)
    if toward_narrow:
        sign = 1.0 if pos_narrow else -1.0
    else:
        sign = -1.0 if pos_narrow else 1.0

    gx = cx + sign * offset_px * v1[0]
    gy = cy + sign * offset_px * v1[1]
    H, W = mask.shape
    return (float(np.clip(gx, 0, W - 1)), float(np.clip(gy, 0, H - 1)))


def _compute_pca_rz(gray: np.ndarray, cx: float, cy: float, radius: int = 60) -> float:
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


def _compute_pca_rz_from_mask(mask: np.ndarray) -> float:
    """이진 마스크(mono8) 픽셀 전체로 PCA 주축 방향각(deg) 계산.

    test_marker_scan.py와 동일한 입력을 사용하므로 Canny ROI 근사보다 안정적.
    PCA가 불안정하면 0.0 반환.
    """
    pts = np.argwhere(mask > 0).astype(np.float32)
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
