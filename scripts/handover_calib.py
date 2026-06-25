#!/usr/bin/env python3
"""핸드오버 파라미터 캘리브레이션 도구.

카메라 오버레이로 APPROACH / PLACE / handle_first 위치를 실시간 확인하고
키보드/마우스로 파라미터를 조정한 뒤 handover.yaml에 저장한다.

사용법:
  ros2 run vision hand_viz_node  (별도 터미널에서 실행 안 해도 됨 — 이 스크립트가 독립 실행)
  python3 scripts/handover_calib.py

키 조작:
  f         handle 방향 반전 (rz_sign 토글 ±1)
  r         현재 hand_yaw를 rz_cal_hand_yaw 기준으로 재설정
  + / =     approach_height +1cm
  -         approach_height -1cm
  [ / ]     place_z_offset ±1cm
  1         공구: screwdriver
  2         공구: ratchet_wrench
  3         공구: utility_knife
  4         공구: multi_tool (direct)
  s         현재 값을 handover.yaml에 저장 (원본 백업 후 덮어쓰기)
  q         저장 없이 종료

마우스 클릭:
  이미지 위 원하는 위치 클릭 → depth 역투영 → approach_height 자동 계산
  (손 lock 상태에서만 동작)
"""
from __future__ import annotations

import shutil
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

# ── ROS2 환경 체크 ──────────────────────────────────────────────────────────
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Bool
    from handpose_interfaces.msg import Hands
except ImportError:
    print("[calib] ROS2 환경이 없습니다. source /opt/ros/humble/setup.bash 후 재실행")
    sys.exit(1)

# ── 경로 ─────────────────────────────────────────────────────────────────────
_REPO = next(
    p for p in Path(__file__).resolve().parents
    if (p / "config" / "handover.yaml").exists()
)
_HANDOVER_YAML = _REPO / "config" / "handover.yaml"
_CAMERA_INFO   = _REPO / "config" / "camera_info.yaml"
_HAND_EYE      = _REPO / "config" / "hand_eye.yaml"
_TOOLBOX_YAML  = _REPO / "config" / "toolbox.yaml"

_WIN = "Handover Calibration"
_DEPTH_SCALE = 0.001  # uint16 mm → m

# ── 기본 공구 목록 ────────────────────────────────────────────────────────────
_TOOL_KEYS = {
    ord("1"): "screwdriver",
    ord("2"): "ratchet_wrench",
    ord("3"): "utility_knife",
    ord("4"): "multi_tool",
}

_CONNECTIONS = [
    (0,1),(1,2),(2,3),(3,4),
    (0,5),(5,6),(6,7),(7,8),
    (0,9),(9,10),(10,11),(11,12),
    (0,13),(13,14),(14,15),(15,16),
    (0,17),(17,18),(18,19),(19,20),
    (5,9),(9,13),(13,17),
]


# ── 유틸 ─────────────────────────────────────────────────────────────────────

def _load_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text())


def _save_yaml(path: Path, data: dict) -> None:
    backup = path.with_suffix(".yaml.bak")
    shutil.copy2(path, backup)
    with path.open("w") as f:
        yaml.dump(data, f, allow_unicode=True, sort_keys=False)
    print(f"[calib] 저장 완료: {path}  (백업: {backup})")


def _build_T(hand_eye_path: Path) -> np.ndarray:
    """hand_eye.yaml → 4×4 T_cam2base."""
    cfg = _load_yaml(hand_eye_path)["transformation"]
    q = cfg["rotation"]
    t = cfg["translation"]
    R = Rotation.from_quat([q["x"], q["y"], q["z"], q["w"]]).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    return T


def _project(pt_base: np.ndarray, T: np.ndarray,
             fx: float, fy: float, cx: float, cy: float) -> tuple[int, int] | None:
    T_inv = np.linalg.inv(T)
    p = (T_inv @ np.append(pt_base, 1.0))[:3]
    if p[2] <= 0:
        return None
    return int(fx * p[0] / p[2] + cx), int(fy * p[1] / p[2] + cy)


def _deproject_pixel(u: int, v: int, depth_m: float,
                     fx: float, fy: float, cx: float, cy: float,
                     T: np.ndarray) -> np.ndarray:
    """픽셀 + depth → base_link 3D [m]."""
    p_cam = np.array([(u - cx) * depth_m / fx,
                      (v - cy) * depth_m / fy,
                      depth_m])
    return (T @ np.append(p_cam, 1.0))[:3]


def _imgmsg_to_bgr(msg: Image) -> np.ndarray:
    h, w = msg.height, msg.width
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return arr.reshape(h, w, 3).copy()
    return arr.reshape(h, w, 3).copy()


def _depth_median(depth_img: np.ndarray, u: int, v: int, r: int = 5) -> float | None:
    h, w = depth_img.shape[:2]
    roi = depth_img[max(0, v-r):min(h, v+r+1), max(0, u-r):min(w, u+r+1)]
    valid = roi[roi > 0]
    return float(np.median(valid)) * _DEPTH_SCALE if len(valid) >= 3 else None


# ── 캘리브레이션 상태 ──────────────────────────────────────────────────────────

class CalibState:
    def __init__(self, h_cfg: dict, tool_lengths: dict[str, float]) -> None:
        self.approach_height_m:   float = h_cfg.get("approach_height_m", 0.05)
        self.place_z_offset_m:    float = h_cfg.get("place_z_offset_m", -0.03)
        self.rz_cal_hand_yaw:     float = h_cfg.get("rz_cal_hand_yaw", 77.0)
        self.rz_cal_robot_rz:     float = h_cfg.get("rz_cal_robot_rz", 8.39)
        self.rz_sign:             float = h_cfg.get("rz_sign", -1.0)
        self.approach_x_offset_m: float = h_cfg.get("approach_x_offset_m", 0.0)
        self.approach_y_offset_m: float = h_cfg.get("approach_y_offset_m", 0.0)
        self.tool_lengths         = tool_lengths
        self.tool_id:             str   = "screwdriver"
        self.message:             str   = ""
        self.message_time:        float = 0.0

    def rz_robot(self, yaw_deg: float) -> float:
        return (yaw_deg - self.rz_cal_hand_yaw) * self.rz_sign + self.rz_cal_robot_rz

    def tool_len(self) -> float:
        return self.tool_lengths.get(self.tool_id, 0.18)

    def set_message(self, msg: str) -> None:
        self.message = msg
        self.message_time = time.monotonic()
        print(f"[calib] {msg}")

    def apply_to_yaml(self, data: dict) -> None:
        hc = data["handover"]
        hc["approach_height_m"]   = round(self.approach_height_m, 4)
        hc["place_z_offset_m"]    = round(self.place_z_offset_m, 4)
        hc["rz_cal_hand_yaw"]     = round(self.rz_cal_hand_yaw, 2)
        hc["rz_cal_robot_rz"]     = round(self.rz_cal_robot_rz, 2)
        hc["rz_sign"]             = float(self.rz_sign)
        hc["approach_x_offset_m"] = round(self.approach_x_offset_m, 4)
        hc["approach_y_offset_m"] = round(self.approach_y_offset_m, 4)


# ── ROS2 노드 ─────────────────────────────────────────────────────────────────

class CalibNode(Node):
    def __init__(self, state: CalibState, T: np.ndarray,
                 fx: float, fy: float, cx: float, cy: float) -> None:
        super().__init__("handover_calib")
        self._state = state
        self._T = T
        self._fx, self._fy, self._cx, self._cy = fx, fy, cx, cy

        self._pose:    PoseStamped | None = None
        self._ready:   bool = False
        self._frame:   np.ndarray | None = None
        self._depth:   np.ndarray | None = None
        self._landmarks: list[list] = []
        self._mouse_pt: tuple[int, int] | None = None
        self._lock = threading.Lock()

        _be = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, "/hand/pose",   self._on_pose,  _be)
        self.create_subscription(Bool,        "/hand/ready",  self._on_ready, _be)
        self.create_subscription(Hands, "/hands/detections",  self._on_hands, _be)
        self.create_subscription(Image, "/d455f/d455f/color/image_raw",
                                 self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/d455f/d455f/aligned_depth_to_color/image_raw",
                                 self._on_depth, qos_profile_sensor_data)

        self.create_timer(1.0 / 30.0, self._display)
        cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_WIN, 1280, 720)
        cv2.setMouseCallback(_WIN, self._on_mouse)
        self.get_logger().info("[calib] 시작. 키 조작: f/r/+/-/[/]/1-4/s/q")

    # ── 구독 콜백 ──────────────────────────────────────────────────────────────

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._pose = msg

    def _on_ready(self, msg: Bool) -> None:
        with self._lock:
            self._ready = msg.data

    def _on_hands(self, msg: Hands) -> None:
        with self._lock:
            self._landmarks = [list(h.landmarks_canon) for h in msg.hands]

    def _on_color(self, msg: Image) -> None:
        with self._lock:
            self._frame = _imgmsg_to_bgr(msg)

    def _on_depth(self, msg: Image) -> None:
        arr = np.frombuffer(msg.data, dtype=np.uint16)
        with self._lock:
            self._depth = arr.reshape(msg.height, msg.width)

    # ── 마우스 클릭 → approach_height 자동 계산 ────────────────────────────────

    def _on_mouse(self, event: int, x: int, y: int, *_) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        with self._lock:
            pose  = self._pose
            ready = self._ready
            depth = self._depth
        if not ready or pose is None or depth is None:
            self._state.set_message("클릭 무시 — 손이 lock되지 않았음")
            return

        d = _depth_median(depth, x, y)
        if d is None:
            self._state.set_message(f"depth 획득 실패 ({x},{y}) — 다시 클릭")
            return

        pt_base = _deproject_pixel(x, y, d,
                                   self._fx, self._fy, self._cx, self._cy, self._T)
        palm_z = pose.pose.position.z
        new_ah = round(pt_base[2] - palm_z, 3)

        if new_ah < 0.01 or new_ah > 0.30:
            self._state.set_message(
                f"클릭 위치 이상 (approach_height={new_ah*100:.1f}cm) — 무시")
            return

        old = self._state.approach_height_m
        self._state.approach_height_m = new_ah
        self._state.set_message(
            f"[CLICK] approach_height: {old*100:.1f}cm → {new_ah*100:.1f}cm  "
            f"(클릭 3D: {pt_base[0]:+.3f},{pt_base[1]:+.3f},{pt_base[2]:+.3f})")

    # ── 표시 ──────────────────────────────────────────────────────────────────

    def _display(self) -> None:
        with self._lock:
            frame  = self._frame
            pose   = self._pose
            ready  = self._ready
            lms    = list(self._landmarks)

        if frame is None:
            return

        img = frame.copy()
        s = self._state

        # 랜드마크
        for lm in lms:
            pts = np.array(lm, dtype=np.float32).reshape(21, 3)
            color = (0, 255, 0) if ready else (0, 165, 255)
            for a, b in _CONNECTIONS:
                cv2.line(img, (int(pts[a,0]), int(pts[a,1])),
                              (int(pts[b,0]), int(pts[b,1])), color, 1)
            for i, (x, y, _) in enumerate(pts):
                c = (0, 100, 255) if i in {4,8,12,16,20} else (255,255,255)
                cv2.circle(img, (int(x), int(y)), 4 if i in {4,8,12,16,20} else 2, c, -1)

        # approach / place / handle_first 역투영
        if ready and pose is not None:
            self._draw_targets(img, pose, s)

        # 상단 정보 바
        self._draw_header(img, pose, ready, s)

        # 하단 메시지
        if s.message and (time.monotonic() - s.message_time) < 4.0:
            h = img.shape[0]
            cv2.rectangle(img, (0, h - 34), (img.shape[1], h), (30, 30, 30), -1)
            cv2.putText(img, s.message, (8, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        # 키 도움말
        self._draw_help(img)

        cv2.imshow(_WIN, img)
        key = cv2.waitKey(1) & 0xFF
        self._handle_key(key, pose, ready)

    def _draw_targets(self, img: np.ndarray, pose: PoseStamped, s: CalibState) -> None:
        p = pose.pose.position
        o = pose.pose.orientation
        palm = np.array([p.x, p.y, p.z])

        def proj(pt):
            return _project(pt, self._T, self._fx, self._fy, self._cx, self._cy)

        ah = s.approach_height_m
        pz = s.place_z_offset_m

        xy_off = np.array([s.approach_x_offset_m, s.approach_y_offset_m, 0.0])
        approach_center = palm + xy_off

        palm_px     = proj(palm)
        approach_px = proj(approach_center + np.array([0, 0, ah]))
        place_px    = proj(approach_center + np.array([0, 0, pz]))

        R = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        x_col = R[:, 0]
        offset = x_col * (s.tool_len() / 2.0)
        h_approach_px = proj(approach_center + offset + np.array([0, 0, ah]))
        h_place_px    = proj(approach_center + offset + np.array([0, 0, pz]))

        # palm 중심
        if palm_px:
            cv2.circle(img, palm_px, 8, (200, 120, 0), 2)
            cv2.putText(img, "PALM", (palm_px[0]+6, palm_px[1]-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200,120,0), 1)

        # APPROACH (빨강)
        if approach_px:
            cv2.circle(img, approach_px, 16, (0, 0, 255), 2)
            cv2.drawMarker(img, approach_px, (0, 0, 255), cv2.MARKER_CROSS, 32, 2)
            cv2.putText(img, f"APPROACH +{ah*100:.1f}cm",
                        (approach_px[0]+10, approach_px[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 80, 255), 1)
            if palm_px:
                cv2.arrowedLine(img, approach_px, palm_px, (0, 0, 200), 1, tipLength=0.1)

        # PLACE (주황)
        if place_px:
            cv2.circle(img, place_px, 10, (0, 140, 255), 2)
            cv2.putText(img, f"PLACE {pz*100:+.1f}cm",
                        (place_px[0]+8, place_px[1]+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 140, 255), 1)

        # handle_first APPROACH (노랑)
        if h_approach_px:
            cv2.circle(img, h_approach_px, 14, (0, 220, 255), 2)
            cv2.drawMarker(img, h_approach_px, (0, 220, 255),
                           cv2.MARKER_TILTED_CROSS, 28, 2)
            cv2.putText(img, f"H+APPROACH ({s.tool_id})",
                        (h_approach_px[0]+10, h_approach_px[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1)

        # handle_first PLACE (연노랑)
        if h_place_px:
            cv2.circle(img, h_place_px, 8, (0, 190, 200), 2)
            cv2.putText(img, "H+PLACE",
                        (h_place_px[0]+8, h_place_px[1]+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 190, 200), 1)

        # handle_first 방향 화살표
        if palm_px and h_approach_px:
            cv2.arrowedLine(img, palm_px, h_approach_px, (0, 220, 255), 2, tipLength=0.1)

    def _draw_header(self, img: np.ndarray, pose, ready: bool, s: CalibState) -> None:
        color = (0, 255, 0) if ready else (0, 165, 255)
        cv2.rectangle(img, (0, 0), (img.shape[1], 90), (20, 20, 20), -1)

        if pose:
            p = pose.pose.position
            o = pose.pose.orientation
            yaw = float(Rotation.from_quat([o.x,o.y,o.z,o.w]).as_euler("xyz",degrees=True)[2])
            rz  = s.rz_robot(yaw)
            cv2.putText(img,
                f"{'LOCKED' if ready else 'WAITING'}  "
                f"X:{p.x:+.3f}  Y:{p.y:+.3f}  Z:{p.z:+.3f} m",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.60, color, 2)
            cv2.putText(img,
                f"Yaw:{yaw:+.1f}deg  rz_robot:{rz:+.1f}deg  "
                f"rz_cal_yaw:{s.rz_cal_hand_yaw:.1f}  sign:{s.rz_sign:+.0f}  "
                f"tool:{s.tool_id}({s.tool_len()*100:.0f}cm)",
                (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 255), 1)
            cv2.putText(img,
                f"approach_height:{s.approach_height_m*100:.1f}cm  "
                f"place_z_offset:{s.place_z_offset_m*100:+.1f}cm  "
                f"XY_offset: x={s.approach_x_offset_m*1000:+.1f}mm y={s.approach_y_offset_m*1000:+.1f}mm  "
                f"[RED=APPROACH  ORANGE=PLACE  YELLOW=H+]",
                (10, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1)
        else:
            cv2.putText(img, "WAITING — /hand/pose 없음",
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    def _draw_help(self, img: np.ndarray) -> None:
        help_lines = [
            "w/s:Y offset±1mm  a/d:X offset±1mm  (TCP XY 보정 — 오버레이 맞는데 로봇이 틀릴때)",
            "f:flip handle  r:rz_cal_yaw재설정  +/-:approach±1cm  [/]:place±1cm",
            "1:screwdriver  2:ratchet_wrench  3:utility_knife  4:multi_tool  S(Shift+s):저장  q:종료",
            "마우스 클릭 → 클릭 위치로 approach_height 자동 설정",
        ]
        w = img.shape[1]
        y0 = img.shape[0] - 8 - 18 * len(help_lines)
        cv2.rectangle(img, (0, y0 - 6), (w, img.shape[0] - 36), (25, 25, 25), -1)
        for i, line in enumerate(help_lines):
            cv2.putText(img, line, (8, y0 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (160, 160, 160), 1)

    # ── 키 처리 ────────────────────────────────────────────────────────────────

    def _handle_key(self, key: int, pose, ready: bool) -> None:
        s = self._state
        if key == ord("q"):
            rclpy.shutdown()
            return

        if key == ord("f"):
            s.rz_sign *= -1.0
            s.set_message(f"handle 방향 반전 → rz_sign={s.rz_sign:+.0f}")

        elif key == ord("r"):
            if pose is None or not ready:
                s.set_message("r: 손 lock 후 실행 필요")
            else:
                o = pose.pose.orientation
                yaw = float(Rotation.from_quat([o.x,o.y,o.z,o.w]).as_euler("xyz",degrees=True)[2])
                old = s.rz_cal_hand_yaw
                s.rz_cal_hand_yaw = round(yaw, 2)
                s.set_message(
                    f"rz_cal_hand_yaw 재설정: {old:.1f} → {yaw:.2f}deg  "
                    f"(손이 홈 방향일 때만 사용)")

        elif key in (ord("+"), ord("=")):
            s.approach_height_m = round(s.approach_height_m + 0.01, 3)
            s.set_message(f"approach_height → {s.approach_height_m*100:.1f}cm")

        elif key == ord("-"):
            s.approach_height_m = max(0.01, round(s.approach_height_m - 0.01, 3))
            s.set_message(f"approach_height → {s.approach_height_m*100:.1f}cm")

        elif key == ord("]"):
            s.place_z_offset_m = round(s.place_z_offset_m + 0.01, 3)
            s.set_message(f"place_z_offset → {s.place_z_offset_m*100:+.1f}cm")

        elif key == ord("["):
            s.place_z_offset_m = round(s.place_z_offset_m - 0.01, 3)
            s.set_message(f"place_z_offset → {s.place_z_offset_m*100:+.1f}cm")

        elif key == ord("w"):
            s.approach_y_offset_m = round(s.approach_y_offset_m + 0.001, 4)
            s.set_message(f"Y offset → {s.approach_y_offset_m*1000:+.1f}mm")

        elif key == ord("s"):
            s.approach_y_offset_m = round(s.approach_y_offset_m - 0.001, 4)
            s.set_message(f"Y offset → {s.approach_y_offset_m*1000:+.1f}mm")

        elif key == ord("d"):
            s.approach_x_offset_m = round(s.approach_x_offset_m + 0.001, 4)
            s.set_message(f"X offset → {s.approach_x_offset_m*1000:+.1f}mm")

        elif key == ord("a"):
            s.approach_x_offset_m = round(s.approach_x_offset_m - 0.001, 4)
            s.set_message(f"X offset → {s.approach_x_offset_m*1000:+.1f}mm")

        elif key in _TOOL_KEYS:
            s.tool_id = _TOOL_KEYS[key]
            s.set_message(f"공구 선택: {s.tool_id}  (길이 {s.tool_len()*100:.0f}cm)")

        elif key == ord("S"):  # Shift+s = 저장
            data = _load_yaml(_HANDOVER_YAML)
            s.apply_to_yaml(data)
            _save_yaml(_HANDOVER_YAML, data)
            s.set_message(
                f"저장 완료  approach={s.approach_height_m*100:.1f}cm  "
                f"xy_offset=({s.approach_x_offset_m*1000:+.1f},{s.approach_y_offset_m*1000:+.1f})mm  "
                f"rz_sign={s.rz_sign:+.0f}")


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    h_cfg = _load_yaml(_HANDOVER_YAML).get("handover", {})

    # 공구 길이 로드
    try:
        tools = _load_yaml(_TOOLBOX_YAML).get("tools", [])
        tool_lengths = {t["tool_id"]: t.get("dimensions", {}).get("length", 0.15)
                        for t in tools}
    except Exception:
        tool_lengths = {"screwdriver": 0.18, "ratchet_wrench": 0.20, "utility_knife": 0.16}

    # 카메라 intrinsics
    cam = _load_yaml(_CAMERA_INFO)["intrinsics"]
    fx, fy, cx, cy = cam["fx"], cam["fy"], cam["cx"], cam["cy"]

    # hand_eye 변환
    T = _build_T(_HAND_EYE)

    state = CalibState(h_cfg, tool_lengths)

    rclpy.init()
    node = CalibNode(state, T, fx, fy, cx, cy)
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
