"""손 감지 + 핸드오버 준비 상태 판단 노드 (Track A).

D455F 탑뷰 aligned depth + MediaPipe 랜드마크를 결합해
손바닥 3D 위치·자세를 base_link 기준으로 발행한다.

조건 충족 시 위치를 lock → /hand/ready = True.
로봇이 접근 중 손이 가려져도 locked pose를 유지한다 (occlusion 대응).

Subscribe:
  /hands/detections                   (handpose_interfaces/HandLandmarks)
  /d455f/aligned_depth_to_color/image_raw (sensor_msgs/Image, uint16 mm)

Publish:
  /hand/pose   (geometry_msgs/PoseStamped) — frame_id: base_link
  /hand/ready  (std_msgs/Bool)

참조: docs/hand_delivery.md, config/handover.yaml, .claude/rules/safety.md S-6
"""
from __future__ import annotations

import os
import time
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from vision.cv_bridge_compat import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from handpose_interfaces.msg import Hands, HandLandmarks
from vision.hand_eye_loader import HandEyeNotCalibratedError, camera_to_base, load_transform

def _find_cfg_dir() -> Path:
    if env_root := os.environ.get("FINAL_PROJECT_ROOT"):
        return Path(env_root) / "config"
    # 설치 경로(install/.../site-packages/vision/)와 소스 경로(src/vision/vision/) 모두 대응:
    # config/hand_eye.yaml이 있는 디렉터리를 부모를 따라 올라가며 탐색
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "config" / "hand_eye.yaml"
        if candidate.exists():
            return parent / "config"
    return Path(__file__).parents[4] / "config"  # 최후 fallback

_CFG_DIR = _find_cfg_dir()
_HAND_EYE_PATH = _CFG_DIR / "hand_eye.yaml"
_CAMERA_INFO_PATH = _CFG_DIR / "camera_info.yaml"
_HANDOVER_CFG_PATH = _CFG_DIR / "handover.yaml"
_DEPTH_SCALE = 0.001  # uint16 mm → float m

_QOS_SENSOR = qos_profile_sensor_data
_QOS_RELIABLE_10 = QoSProfile(depth=10)
_QOS_BEST_EFFORT_10 = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)
# RealSense 이미지 토픽은 BEST_EFFORT(sensor_data) QoS로 발행
_QOS_DEPTH = qos_profile_sensor_data

# MediaPipe 랜드마크 인덱스
_WRIST = 0
_INDEX_MCP = 5
_MIDDLE_MCP = 9
_PINKY_MCP = 17
_FINGER_PAIRS = [(8, 5), (12, 9), (16, 13), (20, 17)]  # (TIP, MCP) 검지~새끼


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


def _load_handover_cfg() -> dict:
    with _HANDOVER_CFG_PATH.open() as f:
        return yaml.safe_load(f)["handover"]


def _reshape_landmarks(flat: list[float]) -> np.ndarray:
    """63-float flat list → (21, 3) float64 array."""
    return np.array(flat, dtype=np.float64).reshape(21, 3)


def _sample_depth(
    depth_img: np.ndarray,
    u: int,
    v: int,
    radius: int,
    min_valid: int,
) -> float | None:
    """랜드마크 주변 원형 영역 median depth [m]. 유효 픽셀 부족 시 None."""
    h, w = depth_img.shape[:2]
    u0 = max(0, u - radius)
    u1 = min(w, u + radius + 1)
    v0 = max(0, v - radius)
    v1 = min(h, v + radius + 1)
    roi = depth_img[v0:v1, u0:u1]
    valid = roi[roi > 0]
    if len(valid) < min_valid:
        return None
    return float(np.median(valid)) * _DEPTH_SCALE


def _deproject(u: float, v: float, depth_m: float, K: _Intrinsics) -> np.ndarray:
    """픽셀 (u, v) + depth → 카메라 좌표계 3D 점 [m]."""
    x = (u - K.cx) * depth_m / K.fx
    y = (v - K.cy) * depth_m / K.fy
    return np.array([x, y, depth_m], dtype=np.float64)


def _palm_normal_to_quat(
    normal_base: np.ndarray,
    finger_dir_base: np.ndarray,
) -> np.ndarray:
    """손바닥 법선 + 손가락 방향(base_link) → 쿼터니언 (x, y, z, w).

    Z축 = palm normal (위)
    Y축 = 손가락 방향 (손목→중지MCP)
    X축 = 손잡이 방향 (새끼→검지, 손가락과 수직) — 로봇이 이 축으로 손잡이를 내밀어야 함
    """
    z = normal_base / np.linalg.norm(normal_base)

    # 손가락 방향을 손바닥 평면에 투영 → Y축
    y_raw = finger_dir_base / (np.linalg.norm(finger_dir_base) + 1e-9)
    y = y_raw - np.dot(y_raw, z) * z
    norm_y = np.linalg.norm(y)
    if norm_y < 1e-6:
        ref = np.array([0.0, 1.0, 0.0])
        if abs(np.dot(z, ref)) > 0.99:
            ref = np.array([1.0, 0.0, 0.0])
        y = np.cross(z, ref)
        y /= np.linalg.norm(y)
    else:
        y /= norm_y

    # X축 = 손잡이 방향 (Y×Z 아니라 Z×Y — 오른손 좌표계 유지)
    x = np.cross(y, z)
    x /= np.linalg.norm(x)
    R = np.column_stack([x, y, z])
    return Rotation.from_matrix(R).as_quat()  # (x, y, z, w)


class HandNode(Node):
    """MediaPipe 손 랜드마크 + D455F aligned depth → /hand/pose + /hand/ready."""

    def __init__(self) -> None:
        super().__init__("hand_node")

        # hand_eye 변환 로드 (필수 — 없으면 기동 실패)
        try:
            self._T = load_transform(_HAND_EYE_PATH)
        except (HandEyeNotCalibratedError, FileNotFoundError) as e:
            self.get_logger().error(f"[hand_node] hand_eye 로드 실패: {e}")
            raise

        self._K = _load_intrinsics()
        cfg = _load_handover_cfg()

        self._score_min: float = cfg["min_detection_score"]
        self._palm_up_thresh: float = cfg["palm_up_threshold"]
        self._stable_frames: int = cfg["stable_frames"]
        self._stable_thresh_m: float = cfg["stable_threshold_m"]
        self._finger_extend_ratio: float = cfg["finger_extend_ratio"]
        self._min_fingers_open: int = cfg["min_fingers_open"]
        self._lock_keep_s: float = cfg.get("lock_keep_s", 5.0)
        self._lock_update_dist: float = cfg.get("lock_update_distance_m", 0.03)
        self._lock_break_dist: float = cfg.get("lock_break_distance_m", 0.15)
        self._lock_break_z: float = cfg.get("lock_break_z_m", 0.08)
        self._depth_radius: int = cfg["depth_sample_radius_px"]
        self._min_depth_px: int = cfg["min_valid_depth_px"]

        self._roi_enabled: bool = cfg.get("roi_enabled", False)
        self._roi = (
            int(cfg.get("roi_x_min", 0)),
            int(cfg.get("roi_x_max", 1280)),
            int(cfg.get("roi_y_min", 0)),
            int(cfg.get("roi_y_max", 720)),
        )  # (x_min, x_max, y_min, y_max)
        self.get_logger().info(
            f"[hand_node] ROI enabled={self._roi_enabled} "
            f"x={self._roi[0]}-{self._roi[1]} y={self._roi[2]}-{self._roi[3]}"
        )

        self._bridge = CvBridge()
        self._mutex = threading.Lock()

        # 안정성 추적
        self._pose_history: deque[np.ndarray] = deque(maxlen=self._stable_frames)

        # lock 상태
        self._locked: bool = False
        self._locked_pos: np.ndarray | None = None   # (3,) base_link [m]
        self._locked_quat: np.ndarray | None = None  # (4,) (x,y,z,w)
        self._no_detect_start: float | None = None   # 소실 시작 시각 (time.monotonic)

        # 최신 depth 캐시 (ApproximateTimeSynchronizer 대체)
        self._latest_depth: np.ndarray | None = None
        self._depth_count: int = 0
        self._hand_count: int = 0

        # depth 토픽은 realsense launch의 camera_namespace에 따라 달라진다:
        #   vision_pipeline.launch (namespace="")     → /d455f/aligned_depth_to_color/image_raw  (기본)
        #   realsense_bringup.launch (namespace=d455f) → /d455f/d455f/aligned_depth_to_color/image_raw
        # 통합 운용은 vision_pipeline 기준이므로 single 네임스페이스를 기본값으로 둔다.
        self._depth_topic: str = (
            self.declare_parameter(
                "depth_topic", "/d455f/aligned_depth_to_color/image_raw"
            ).get_parameter_value().string_value
        )
        self.get_logger().info(f"[hand_node] depth 토픽 구독: {self._depth_topic}")
        self.create_subscription(
            Image, self._depth_topic,
            self._on_depth, _QOS_DEPTH,
        )
        self.create_subscription(
            Hands, "/hands/detections",
            self._on_hands_msg, _QOS_BEST_EFFORT_10,
        )

        self._pose_pub = self.create_publisher(PoseStamped, "/hand/pose", _QOS_BEST_EFFORT_10)
        self._ready_pub = self.create_publisher(Bool, "/hand/ready", _QOS_BEST_EFFORT_10)
        self._debug_pub = self.create_publisher(String, "/hand/debug", _QOS_BEST_EFFORT_10)

        # 1초마다 현재 상태 터미널 출력
        self.create_timer(1.0, self._status_log)

        self.get_logger().info("[hand_node] 시작 — /hand/pose, /hand/ready 발행 대기")

    # ─── 개별 콜백 ───────────────────────────────────────────────────────────

    def _on_depth(self, depth_msg: Image) -> None:
        """depth 이미지 캐시 업데이트."""
        with self._mutex:
            self._latest_depth = self._bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
            self._depth_count += 1

    def _on_hands_msg(self, hands_msg: Hands) -> None:
        """hands 감지 메시지 수신 → 최신 depth와 결합해 처리."""
        self._hand_count += 1
        with self._mutex:
            depth_img = self._latest_depth
        if depth_img is None:
            self._publish_debug(
                f"hand_rx={self._hand_count} depth_rx={self._depth_count} [NO_DEPTH]"
            )
            return
        self._on_hand_depth(hands_msg, depth_img)

    # ─── 메인 처리 ────────────────────────────────────────────────────────────

    def _on_hand_depth(self, hands_msg: Hands, depth_img: np.ndarray) -> None:
        with self._mutex:

            # ── Phase 1: 손 선택 / 신뢰도 검사 ──────────────────────────────
            # ROI 내 2개 이상 → 신뢰도 최고, 그 외 → base_link Z 최고(depth 최소) 손 선택
            hand_msg = self._select_hand(hands_msg.hands, depth_img)
            if hand_msg is None:
                self._handle_no_detection()
                return

            if hand_msg.score < self._score_min:
                self._handle_no_detection()
                return

            canon = _reshape_landmarks(hand_msg.landmarks_canon)   # (21,3) pixel
            world = _reshape_landmarks(hand_msg.landmarks_world)    # (21,3) MediaPipe world [m]

            # ── Phase 2: 손바닥 위 방향 판단 ───────────────────────────────
            palm_normal_base, palm_pos_base, finger_dir_base = self._compute_palm(
                canon, depth_img, hand_label=hand_msg.label
            )
            if palm_normal_base is None:
                self._handle_no_detection()
                return

            # label 기반 flip → normal_z > 0 = 손바닥 위, < 0 = 손바닥 아래
            palm_up = palm_normal_base[2] > self._palm_up_thresh

            # ── Phase 2: 손 펼침 판단 ──────────────────────────────────────
            hand_open = self._is_hand_open(world)

            if not (palm_up and hand_open):
                # 손이 감지됐지만 방향/모양 조건 미충족 → 즉시 lock 해제
                if self._locked:
                    self.get_logger().info(
                        f"[hand_node] lock 해제 — 조건 미충족 "
                        f"(palm_up={palm_up} normal_z={palm_normal_base[2]:+.3f} "
                        f"hand_open={hand_open})"
                    )
                    self._reset_state()
                self._publish_debug(
                    f"normal_z={palm_normal_base[2]:+.3f} "
                    f"label={hand_msg.label} "
                    f"palm_up={'O' if palm_up else 'X'}({self._palm_up_thresh:.1f}) "
                    f"hand_open={'O' if hand_open else 'X'} "
                    f"score={hand_msg.score:.2f} "
                    f"stable={len(self._pose_history)}/{self._stable_frames} "
                    f"[WAITING] cond_FAIL"
                )
                self._publish_ready(False)
                return

            # ── Phase 3: 안정성 추적 ───────────────────────────────────────
            self._pose_history.append(palm_pos_base)
            self._no_detect_start = None

            stable_cur = len(self._pose_history)

            if stable_cur < self._stable_frames:
                self._publish_debug(
                    f"normal_z={palm_normal_base[2]:+.3f} "
                    f"palm_up=O hand_open=O "
                    f"score={hand_msg.score:.2f} "
                    f"stable={stable_cur}/{self._stable_frames} "
                    f"[ACCUMULATING]"
                )
                self._publish_ready(False)
                return

            positions = np.array(self._pose_history)
            spread = float(np.max(np.std(positions, axis=0)))
            if spread > self._stable_thresh_m:
                self._publish_debug(
                    f"normal_z={palm_normal_base[2]:+.3f} "
                    f"palm_up=O hand_open=O "
                    f"score={hand_msg.score:.2f} "
                    f"stable={stable_cur}/{self._stable_frames} "
                    f"spread={spread:.4f}m>{self._stable_thresh_m}m [UNSTABLE]"
                )
                self._publish_ready(False)
                return

            # ── Phase 3: lock ──────────────────────────────────────────────
            if not self._locked:
                self._locked = True
                # 현재 프레임 hand가 바뀌어도 history 평균으로 고정 (ROI 밖 손 오염 방지)
                self._locked_pos = np.mean(positions, axis=0)
                self._locked_quat = _palm_normal_to_quat(palm_normal_base, finger_dir_base)
                self.get_logger().info(
                    f"[hand_node] 위치 lock: "
                    f"x={self._locked_pos[0]:.3f} "
                    f"y={self._locked_pos[1]:.3f} "
                    f"z={self._locked_pos[2]:.3f} m (base_link)"
                )
            else:
                # ── lock 후 손 이동 체크 ───────────────────────────────────
                diff = palm_pos_base - self._locked_pos
                dist_3d = float(np.linalg.norm(diff))
                dist_z = abs(float(diff[2]))

                # full reset: 15cm 이상 이동 또는 Z 8cm 이상 단독 변화
                if dist_3d >= self._lock_break_dist or dist_z >= self._lock_break_z:
                    reason = (f"3D {dist_3d:.3f}m>={self._lock_break_dist}m"
                              if dist_3d >= self._lock_break_dist
                              else f"Z {dist_z:.3f}m>={self._lock_break_z}m")
                    self.get_logger().info(
                        f"[hand_node] 작업 정지 — full reset ({reason})"
                    )
                    self._reset_state()
                    self._publish_ready(False)
                    return

                # 위치 업데이트 구간: 3~15cm → lock 좌표 즉시 갱신 (재안정화 없음)
                if dist_3d >= self._lock_update_dist:
                    self._locked_pos = palm_pos_base.copy()
                    self._locked_quat = _palm_normal_to_quat(palm_normal_base, finger_dir_base)
                    self.get_logger().info(
                        f"[hand_node] 손 이동 → 좌표 갱신 "
                        f"(이동 {dist_3d:.3f}m) "
                        f"→ ({self._locked_pos[0]:.3f},{self._locked_pos[1]:.3f},{self._locked_pos[2]:.3f})"
                    )

            yaw_deg = float(
                Rotation.from_quat(self._locked_quat).as_euler("xyz", degrees=True)[2]
            )
            self.get_logger().info(
                f"[STATUS] LOCKED  "
                f"xyz=({self._locked_pos[0]:.3f},{self._locked_pos[1]:.3f},{self._locked_pos[2]:.3f})m  "
                f"yaw={yaw_deg:.1f}deg  "
                f"no_detect={(time.monotonic() - self._no_detect_start) if self._no_detect_start else 0.0:.1f}s/{self._lock_keep_s:.1f}s  "
                f"stable_buf={stable_cur}"
            )
            self._publish_debug(
                f"normal_z={palm_normal_base[2]:+.3f} "
                f"palm_up=O hand_open=O "
                f"score={hand_msg.score:.2f} "
                f"stable={stable_cur}/{self._stable_frames} "
                f"spread={spread:.4f}m "
                f"yaw={yaw_deg:.1f}deg "
                f"[LOCKED] xyz=({self._locked_pos[0]:.3f},{self._locked_pos[1]:.3f},{self._locked_pos[2]:.3f})"
            )

            self._publish_pose(hands_msg.header.stamp)
            self._publish_ready(True)

    # ─── 내부 계산 ────────────────────────────────────────────────────────────

    def _compute_palm(
        self,
        canon: np.ndarray,
        depth_img: np.ndarray,
        hand_label: str = "right",
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        """손바닥 법선 + 중심 위치 + 손가락 방향을 base_link 기준으로 반환.

        depth 획득 실패 시 (None, None, None) 반환.
        반환: (palm_normal_base, palm_pos_base, finger_dir_base)
          - palm_normal_base : 손바닥 법선 단위벡터 (위=+Z 기대)
          - palm_pos_base    : 손바닥 중심 3D 위치 [m]
          - finger_dir_base  : 손목→중지MCP 방향 단위벡터 (손가락 방향, yaw 정보)
        """
        pts_px = {
            "wrist":     (int(canon[_WRIST][0]),      int(canon[_WRIST][1])),
            "index_mcp": (int(canon[_INDEX_MCP][0]),  int(canon[_INDEX_MCP][1])),
            "pinky_mcp": (int(canon[_PINKY_MCP][0]),  int(canon[_PINKY_MCP][1])),
            "middle_mcp":(int(canon[_MIDDLE_MCP][0]), int(canon[_MIDDLE_MCP][1])),
        }

        pts_3d: dict[str, np.ndarray] = {}
        for name, (u, v) in pts_px.items():
            d = _sample_depth(depth_img, u, v, self._depth_radius, self._min_depth_px)
            if d is None or d <= 0.0:
                self.get_logger().debug(f"[hand_node] depth 획득 실패: {name}")
                return None, None, None
            pts_3d[name] = _deproject(u, v, d, self._K)

        p0  = pts_3d["wrist"]
        p5  = pts_3d["index_mcp"]
        p17 = pts_3d["pinky_mcp"]

        # 손바닥 법선 (카메라 좌표계)
        v1 = p5 - p0
        v2 = p17 - p0
        cross = np.cross(v1, v2)
        norm_len = np.linalg.norm(cross)
        if norm_len < 1e-6:
            return None, None, None
        # MediaPipe는 비미러 카메라 기준 — 사용자 오른손 = 카메라 "Left"
        # 카메라 "Left"(=사용자 오른손)는 flip 불필요, "Right"(=사용자 왼손)만 flip
        flip = 1.0 if hand_label.lower() == "left" else -1.0
        palm_normal_cam = cross / norm_len * flip

        # 손가락 방향: 손목 → 중지MCP (카메라 좌표계)
        finger_vec_cam = pts_3d["middle_mcp"] - p0
        finger_len = np.linalg.norm(finger_vec_cam)
        if finger_len < 1e-6:
            return None, None, None
        finger_dir_cam = finger_vec_cam / finger_len

        # 손바닥 중심 = 4점 평균 (카메라 좌표계)
        palm_center_cam = np.mean(list(pts_3d.values()), axis=0)

        # base_link 변환
        Rot = self._T[:3, :3]
        palm_normal_base = Rot @ palm_normal_cam
        palm_pos_base    = camera_to_base(palm_center_cam, self._T)
        finger_dir_base  = Rot @ finger_dir_cam

        return palm_normal_base, palm_pos_base, finger_dir_base

    def _is_hand_open(self, world: np.ndarray) -> bool:
        """손 펼침 여부: TIP-wrist 거리 > MCP-wrist 거리 × ratio."""
        wrist = world[_WRIST]
        extended = sum(
            1 for tip_i, mcp_i in _FINGER_PAIRS
            if np.linalg.norm(world[tip_i] - wrist)
            > np.linalg.norm(world[mcp_i] - wrist) * self._finger_extend_ratio
        )
        return extended >= self._min_fingers_open

    def _select_hand(
        self,
        hands: list,
        depth_img: np.ndarray,
    ) -> "HandLandmarks | None":
        """손 선택.

        Step 1 — ROI 필터링 (roi_enabled 시 바깥 손 완전 제거, 항상 먼저 실행)
        Step 2 — 후보 선택:
          - 1개 → 해당 손
          - 2개 이상 + ROI 활성 → 신뢰도(score) 최고 손
          - 2개 이상 + ROI 비활성 → base_link Z 최고(wrist depth 최소) 손
        """
        if not hands:
            return None

        # Step 1: ROI 필터링 — roi_enabled면 바깥 손은 무조건 제거
        # 손목(0) 단독이 아닌 손바닥 중심(0,5,9,13,17 평균)으로 판정
        _PALM_CENTER_IDX = [0, 5, 9, 13, 17]
        candidates = list(hands)
        if self._roi_enabled:
            x_min, x_max, y_min, y_max = self._roi
            filtered = []
            for h in candidates:
                canon = _reshape_landmarks(h.landmarks_canon)
                px = float(np.mean(canon[_PALM_CENTER_IDX, 0]))
                py = float(np.mean(canon[_PALM_CENTER_IDX, 1]))
                inside = x_min <= px <= x_max and y_min <= py <= y_max
                self.get_logger().debug(
                    f"[select] label={h.label} score={h.score:.2f} "
                    f"palm_center=({px:.0f},{py:.0f}) roi=({x_min}-{x_max},{y_min}-{y_max}) "
                    f"{'IN' if inside else 'OUT'}"
                )
                if inside:
                    filtered.append(h)
            if not filtered:
                self.get_logger().debug("[select] ROI 내 손 없음 → None")
                return None
            candidates = filtered

        # Step 2: 후보 선택
        if len(candidates) == 1:
            return candidates[0]

        if self._roi_enabled:
            # ROI 내 2개 이상 → 신뢰도 최고
            return max(candidates, key=lambda h: h.score)

        # ROI 비활성, 2개 이상 → base_link Z 최고(depth 최소)
        return self._pick_highest_z(candidates, depth_img)

    def _pick_highest_z(
        self,
        hands: list,
        depth_img: np.ndarray,
    ) -> "HandLandmarks | None":
        """wrist depth 최소(= 탑뷰 기준 base_link Z 최고) 손 반환.

        depth 획득 실패 시 score 최고 손으로 fallback.
        """
        best_hand = None
        best_depth = float("inf")
        for h in hands:
            canon = _reshape_landmarks(h.landmarks_canon)
            wu, wv = int(canon[_WRIST][0]), int(canon[_WRIST][1])
            d = _sample_depth(depth_img, wu, wv, self._depth_radius, self._min_depth_px)
            if d is not None and 0.0 < d < best_depth:
                best_depth = d
                best_hand = h
        return best_hand if best_hand is not None else max(hands, key=lambda h: h.score)

    def _handle_no_detection(self) -> None:
        """감지 소실 처리 — lock 상태면 lock_keep_s 동안 유지 (occlusion 대응)."""
        if self._locked:
            now = time.monotonic()
            if self._no_detect_start is None:
                self._no_detect_start = now
            elapsed = now - self._no_detect_start
            if elapsed <= self._lock_keep_s:
                self._publish_pose(stamp=None)
                self._publish_ready(True)
                return
            self.get_logger().info(
                f"[hand_node] lock 해제 (손 소실 {elapsed:.1f}s > {self._lock_keep_s}s)"
            )
        self._reset_state()
        self._publish_ready(False)

    def _reset_state(self) -> None:
        self._pose_history.clear()
        self._locked = False
        self._locked_pos = None
        self._locked_quat = None
        self._no_detect_start = None

    # ─── 발행 ─────────────────────────────────────────────────────────────────

    def _publish_pose(self, stamp) -> None:
        if self._locked_pos is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = stamp if stamp is not None else self.get_clock().now().to_msg()
        msg.pose.position.x = float(self._locked_pos[0])
        msg.pose.position.y = float(self._locked_pos[1])
        msg.pose.position.z = float(self._locked_pos[2])
        q = self._locked_quat
        msg.pose.orientation.x = float(q[0])
        msg.pose.orientation.y = float(q[1])
        msg.pose.orientation.z = float(q[2])
        msg.pose.orientation.w = float(q[3])
        self._pose_pub.publish(msg)

    def _publish_ready(self, ready: bool) -> None:
        self._ready_pub.publish(Bool(data=ready))

    def _publish_debug(self, text: str) -> None:
        self._debug_pub.publish(String(data=text))

    def _status_log(self) -> None:
        """1초마다 터미널에 현재 lock 상태 출력."""
        if self._locked and self._locked_pos is not None:
            p = self._locked_pos
            elapsed = (time.monotonic() - self._no_detect_start) if self._no_detect_start else 0.0
            self.get_logger().info(
                f"[STATUS] LOCKED  xyz=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})m  "
                f"no_detect={elapsed:.1f}s/{self._lock_keep_s}s  "
                f"stable_buf={len(self._pose_history)}"
            )
        else:
            self.get_logger().info(
                f"[STATUS] WAITING  stable={len(self._pose_history)}/{self._stable_frames}"
                f"  depth_rx={self._depth_count}  hand_rx={self._hand_count}"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
