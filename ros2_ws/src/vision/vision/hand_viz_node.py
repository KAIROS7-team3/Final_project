"""카메라 화면에 /hand/pose xyz 좌표 + ROI 박스 + 랜드마크 오버레이 (cv2 직접 표시)."""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from handpose_interfaces.msg import Hands
from vision.hand_eye_loader import load_transform

_QOS_BE = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
_REPO_ROOT = next(
    p for p in Path(__file__).resolve().parents if (p / "config" / "handover.yaml").exists()
)
_HANDOVER_CFG = _REPO_ROOT / "config" / "handover.yaml"
_CAMERA_INFO = _REPO_ROOT / "config" / "camera_info.yaml"
_HAND_EYE = _REPO_ROOT / "config" / "hand_eye.yaml"
_TOOLBOX_CFG = _REPO_ROOT / "config" / "toolbox.yaml"
_WIN = "Hand Monitor"

# MediaPipe 손 스켈레톤 연결선 (21개 랜드마크)
_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),           # 엄지
    (0,5),(5,6),(6,7),(7,8),           # 검지
    (0,9),(9,10),(10,11),(11,12),      # 중지
    (0,13),(13,14),(14,15),(15,16),    # 약지
    (0,17),(17,18),(18,19),(19,20),    # 새끼
    (5,9),(9,13),(13,17),              # 손바닥 가로
]


def _load_intrinsics() -> tuple[float, float, float, float] | None:
    """(fx, fy, cx, cy) 반환. 실패 시 None."""
    try:
        cfg = yaml.safe_load(_CAMERA_INFO.read_text())["intrinsics"]
        return cfg["fx"], cfg["fy"], cfg["cx"], cfg["cy"]
    except Exception:
        return None


def _load_T() -> np.ndarray | None:
    """hand_eye 변환 행렬 (camera→base). 실패 시 None."""
    try:
        return load_transform(_HAND_EYE)
    except Exception:
        return None


def _project_base_to_pixel(
    pos_base: np.ndarray,
    T: np.ndarray,
    fx: float, fy: float, cx: float, cy: float,
) -> tuple[int, int] | None:
    """base_link 3D 좌표 → 이미지 픽셀 (u, v). 카메라 뒤면 None."""
    T_inv = np.linalg.inv(T)
    p_cam = (T_inv @ np.append(pos_base, 1.0))[:3]
    if p_cam[2] <= 0:
        return None
    u = int(fx * p_cam[0] / p_cam[2] + cx)
    v = int(fy * p_cam[1] / p_cam[2] + cy)
    return u, v


def _load_roi() -> tuple[bool, tuple[int, int, int, int]]:
    try:
        cfg = yaml.safe_load(_HANDOVER_CFG.read_text())["handover"]
        enabled = cfg.get("roi_enabled", False)
        roi = (
            int(cfg.get("roi_x_min", 441)),
            int(cfg.get("roi_x_max", 585)),
            int(cfg.get("roi_y_min", 248)),
            int(cfg.get("roi_y_max", 449)),
        )
        return enabled, roi
    except Exception as e:
        print(f"[hand_viz] ROI yaml 로드 실패: {e} — 기본값 사용")
        return True, (695, 839, 248, 449)


def _load_handover_params() -> dict:
    try:
        cfg = yaml.safe_load(_HANDOVER_CFG.read_text())["handover"]
        return {
            "approach_height_m": cfg.get("approach_height_m", 0.05),
            "place_z_offset_m": cfg.get("place_z_offset_m", -0.03),
            "rz_cal_hand_yaw": cfg.get("rz_cal_hand_yaw", 77.0),
            "rz_cal_robot_rz": cfg.get("rz_cal_robot_rz", 8.39),
            "rz_sign": cfg.get("rz_sign", -1.0),
        }
    except Exception as e:
        print(f"[hand_viz] handover.yaml 로드 실패: {e}")
        return {"approach_height_m": 0.05, "place_z_offset_m": -0.03,
                "rz_cal_hand_yaw": 77.0, "rz_cal_robot_rz": 8.39, "rz_sign": -1.0}


def _load_tool_lengths() -> dict[str, float]:
    """toolbox.yaml에서 handle_first 공구별 길이(m) 반환."""
    try:
        tools = yaml.safe_load(_TOOLBOX_CFG.read_text()).get("tools", [])
        return {
            t["tool_id"]: t.get("dimensions", {}).get("length", 0.15)
            for t in tools
            if t.get("handover_type") == "handle_first"
        }
    except Exception:
        return {"screwdriver": 0.18, "utility_knife": 0.16, "ratchet_wrench": 0.20}


def _imgmsg_to_bgr(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return arr.reshape(h, w, 3).copy()
    if msg.encoding == "mono8":
        return cv2.cvtColor(arr.reshape(h, w), cv2.COLOR_GRAY2BGR)
    return arr.reshape(h, w, 3).copy()


def _draw_landmarks(img: np.ndarray, landmarks_flat: list, ready: bool) -> None:
    """21개 랜드마크 점 + 스켈레톤 선 + 손가락 방향 화살표 그리기."""
    pts = np.array(landmarks_flat, dtype=np.float32).reshape(21, 3)
    line_color = (0, 255, 0) if ready else (0, 200, 255)
    dot_color = (255, 255, 255)
    tip_color = (0, 100, 255)
    tip_indices = {4, 8, 12, 16, 20}  # 손가락 끝

    for a, b in _CONNECTIONS:
        x1, y1 = int(pts[a][0]), int(pts[a][1])
        x2, y2 = int(pts[b][0]), int(pts[b][1])
        cv2.line(img, (x1, y1), (x2, y2), line_color, 1)

    for i, (x, y, _) in enumerate(pts):
        px, py = int(x), int(y)
        color = tip_color if i in tip_indices else dot_color
        radius = 5 if i in tip_indices else 3
        cv2.circle(img, (px, py), radius, color, -1)

    # ── 손잡이 방향 화살표: 검지MCP(5) → 새끼MCP(17) 수직축 ──────────────
    # 손가락(wrist→middle)과 수직인 방향 = 로봇이 손잡이를 내밀어야 하는 축
    palm_cx = int(np.mean(pts[[0, 5, 9, 13, 17], 0]))
    palm_cy = int(np.mean(pts[[0, 5, 9, 13, 17], 1]))

    ix, iy = int(pts[5][0]), int(pts[5][1])   # 검지MCP
    px, py = int(pts[17][0]), int(pts[17][1])  # 새끼MCP
    hdx, hdy = ix - px, iy - py               # 새끼→검지 방향 (엄지 쪽)
    hlen = max(1, int(np.hypot(hdx, hdy)))
    # 손바닥 너비 크기로 정규화해 화살표 길이 고정 (50px)
    scale = 50.0 / hlen
    arrow_tip = (int(palm_cx + hdx * scale), int(palm_cy + hdy * scale))
    arrow_tail = (int(palm_cx - hdx * scale), int(palm_cy - hdy * scale))
    cv2.arrowedLine(img, arrow_tail, arrow_tip, (0, 255, 255), 2, tipLength=0.25)
    cv2.putText(img, "HANDLE", (arrow_tip[0] + 4, arrow_tip[1] - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)


class HandVizNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_viz_node")

        self._pose: PoseStamped | None = None
        self._locked_pose: PoseStamped | None = None
        self._ready: bool = False
        loaded_enabled, loaded_roi = _load_roi()
        self._roi_enabled: bool = loaded_enabled
        self._roi: tuple[int, int, int, int] = loaded_roi
        print(f"[hand_viz] ROI enabled={self._roi_enabled} roi={self._roi}")
        self._frame: np.ndarray | None = None
        self._debug_str: str = ""
        self._all_landmarks: list[list] = []  # 감지된 모든 손 랜드마크

        self._K = _load_intrinsics()
        self._T = _load_T()
        self._lock_px: tuple[int, int] | None = None  # lock 시점 픽셀 좌표
        self._h_params = _load_handover_params()
        self._tool_lengths = _load_tool_lengths()
        # ROS2 파라미터: 현재 검증할 tool_id (기본: screwdriver)
        self.declare_parameter("tool_id", "screwdriver")
        print(f"[hand_viz] approach_height={self._h_params['approach_height_m']*100:.0f}cm "
              f"tool_lengths={self._tool_lengths}")

        self.create_subscription(PoseStamped, "/hand/pose", self._on_pose, _QOS_BE)
        self.create_subscription(Bool, "/hand/ready", self._on_ready, _QOS_BE)
        self.create_subscription(String, "/hand/debug", self._on_debug, _QOS_BE)
        self.create_subscription(Hands, "/hands/detections", self._on_hands, _QOS_BE)
        self.create_subscription(
            Image, "/d455f/d455f/color/image_raw",
            self._on_image, qos_profile_sensor_data,
        )

        self.create_timer(1.0 / 30.0, self._display)

        cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_WIN, 960, 540)
        self.get_logger().info("[hand_viz_node] 시작")

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose = msg

    def _on_ready(self, msg: Bool) -> None:
        if msg.data and not self._ready:
            self._locked_pose = self._pose
        elif not msg.data:
            self._locked_pose = None
        self._ready = msg.data

    def _on_debug(self, msg: String) -> None:
        self._debug_str = msg.data

    def _on_hands(self, msg: Hands) -> None:
        self._all_landmarks = [list(h.landmarks_canon) for h in msg.hands]

    def _on_image(self, msg: Image) -> None:
        self._frame = _imgmsg_to_bgr(msg)

    def _draw_approach_overlay(self, img: np.ndarray) -> None:
        """locked pose 기반으로 approach / place / handle_first 위치를 역투영해 오버레이."""
        pose = self._locked_pose
        p = pose.pose.position
        o = pose.pose.orientation
        palm_base = np.array([p.x, p.y, p.z])
        hp = self._h_params
        ah = hp["approach_height_m"]
        pz = hp["place_z_offset_m"]

        fx, fy, cx, cy = self._K
        T = self._T

        def proj(pt: np.ndarray) -> tuple[int, int] | None:
            return _project_base_to_pixel(pt, T, fx, fy, cx, cy)

        palm_px = proj(palm_base)
        approach_px = proj(palm_base + np.array([0.0, 0.0, ah]))
        place_px = proj(palm_base + np.array([0.0, 0.0, pz]))

        R_mat = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        y_col = R_mat[:, 1]  # finger direction (손가락 방향)

        tool_id = self.get_parameter("tool_id").get_parameter_value().string_value
        tool_len = self._tool_lengths.get(tool_id, 0.18)
        # handle_first: 그리퍼가 헤드 파지 → 손잡이는 -Y 방향으로 offset
        handle_offset = -y_col * (tool_len / 2.0)
        handle_approach_px = proj(palm_base + handle_offset + np.array([0.0, 0.0, ah]))
        handle_place_px = proj(palm_base + handle_offset + np.array([0.0, 0.0, pz]))

        # palm 중심 (파란 원)
        if palm_px:
            cv2.circle(img, palm_px, 8, (255, 100, 0), 2)

        # approach 위치 (빨간 십자 + 원)
        if approach_px:
            cv2.circle(img, approach_px, 14, (0, 0, 255), 2)
            cv2.drawMarker(img, approach_px, (0, 0, 255), cv2.MARKER_CROSS, 28, 2)
            cv2.putText(img, f"APPROACH +{ah*100:.0f}cm",
                        (approach_px[0] + 8, approach_px[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 80, 255), 1)
            if palm_px:
                cv2.arrowedLine(img, approach_px, palm_px, (0, 0, 255), 1, tipLength=0.12)

        # place 위치 (주황 원)
        if place_px:
            cv2.circle(img, place_px, 10, (0, 140, 255), 2)
            cv2.putText(img, f"PLACE {pz*100:+.0f}cm",
                        (place_px[0] + 8, place_px[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 140, 255), 1)

        # handle_first approach / place (노란색)
        if handle_approach_px:
            cv2.circle(img, handle_approach_px, 12, (0, 220, 255), 2)
            cv2.drawMarker(img, handle_approach_px, (0, 220, 255),
                           cv2.MARKER_TILTED_CROSS, 24, 2)
            cv2.putText(img, f"H+APPROACH ({tool_id})",
                        (handle_approach_px[0] + 8, handle_approach_px[1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 220, 255), 1)
        if handle_place_px:
            cv2.circle(img, handle_place_px, 8, (0, 200, 255), 2)
            cv2.putText(img, "H+PLACE",
                        (handle_place_px[0] + 8, handle_place_px[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 255), 1)

        # handle_first 오프셋 방향 화살표 (palm → handle approach)
        if palm_px and handle_approach_px:
            cv2.arrowedLine(img, palm_px, handle_approach_px, (0, 220, 255), 1, tipLength=0.12)

    def _display(self) -> None:
        if self._frame is None:
            return

        img = self._frame.copy()
        h, w = img.shape[:2]

        # ── 모든 손 랜드마크 오버레이 ────────────────────────────────────
        for lm in self._all_landmarks:
            _draw_landmarks(img, lm, self._ready)

        # ── Lock 위치 픽셀 표시 (손바닥 중심 = 랜드마크 9번 MIDDLE_MCP) ──
        if self._ready and self._all_landmarks:
            pts = np.array(self._all_landmarks[0], dtype=np.float32).reshape(21, 3)
            # 손바닥 중심: 0(wrist), 5, 9, 13, 17 평균
            palm_idx = [0, 5, 9, 13, 17]
            cx_f = float(np.mean(pts[palm_idx, 0]))
            cy_f = float(np.mean(pts[palm_idx, 1]))
            if self._lock_px is None:
                self._lock_px = (int(cx_f), int(cy_f))
            u, v = self._lock_px
            cv2.circle(img, (u, v), 16, (0, 255, 0), 3)
            cv2.drawMarker(img, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 32, 2)
            cv2.putText(img, "LOCK", (u + 18, v - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        elif not self._ready:
            self._lock_px = None


        # ── ROI 박스 (항상 표시 — enabled: 색상, disabled: 회색) ──────────
        x_min, x_max, y_min, y_max = self._roi
        if self._roi_enabled:
            roi_color = (0, 255, 0) if self._ready else (0, 165, 255)
            label = "READY" if self._ready else "WAITING"
            thickness = 2
        else:
            roi_color = (120, 120, 120)
            label = "ROI(OFF)"
            thickness = 1
        cv2.rectangle(img, (x_min, y_min), (x_max, y_max), roi_color, thickness)
        cv2.putText(img, label, (x_min + 4, y_min - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, roi_color, 1)

        # ── LOCK / WAITING 텍스트 (좌상단 한 줄) ────────────────────────
        color = (0, 255, 0) if self._ready else (0, 165, 255)
        label = "LOCK" if self._ready else "WAITING"
        cv2.putText(img, label, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        cv2.imshow(_WIN, img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandVizNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
