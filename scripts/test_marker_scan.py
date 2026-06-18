"""marker_scan_node 실기 검증 스크립트 (ROS2 없이 단독 실행).

C270 카메라로 ArUco 마커 + 그리퍼 모델 세그멘테이션을 동시에 실행하여
TCP 프레임 3D 좌표를 화면에 실시간 표시한다.
PCA 기반 grasp offset 적용 (vision.yaml grasp_offset 섹션).

사용법:
    python3 scripts/test_marker_scan.py

종료: q  / 객체 클릭: PCA 화살표 + 파지점 강제 표시 (재클릭 시 해제)
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

_ROOT = Path(__file__).resolve().parents[1]


# ── 설정 로드 ─────────────────────────────────────────────────────────────────

def _load(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f)

_cam_cfg   = _load(_ROOT / "config/c270_camera_info.yaml")
_he_cfg    = _load(_ROOT / "config/c270_hand_eye.yaml")
_tb_cfg    = _load(_ROOT / "config/toolbox.yaml")
_rt_cfg    = _load(_ROOT / "config/runtime.yaml")
_vis_cfg   = _load(_ROOT / "config/vision.yaml")
_pose_cfg  = _load(_ROOT / "config/robot_poses.yaml")

# 카메라 내부 파라미터
_intr = _cam_cfg["intrinsics"]
CAM_K = np.array([
    [_intr["fx"], 0.0,         _intr["cx"]],
    [0.0,         _intr["fy"], _intr["cy"]],
    [0.0,         0.0,         1.0],
], dtype=np.float64)
DIST = np.array(_intr["coeffs"], dtype=np.float64)

# 핸드-아이 (카메라 → TCP)
def _quat_to_rot(x, y, z, w):
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])

_rot = _he_cfg["transformation"]["rotation"]
_tra = _he_cfg["transformation"]["translation"]
HE_R = _quat_to_rot(_rot["x"], _rot["y"], _rot["z"], _rot["w"])
HE_T = np.array([_tra["x"], _tra["y"], _tra["z"]])

# 마커 크기 (서랍 내부 5cm)
MARKER_SIZE = float(_tb_cfg["aruco_front"]["marker_size_m"])

# 공구 두께
TOOL_H: dict[str, float] = {}
for _tool in _tb_cfg.get("tools", []):
    _dims = _tool["dimensions"]
    TOOL_H[_tool["tool_id"]] = float(_dims.get("height", _dims.get("diameter", 0.020)))

# PCA grasp offset (vision.yaml)
GRASP_OFFSET: dict[str, dict] = {
    k: {"ratio": float(v["ratio"]), "toward_narrow": bool(v["toward_narrow"])}
    for k, v in _vis_cfg.get("grasp_offset", {}).items()
}

# link_6(flange) → TCP 오프셋 (E-1: base_link frame 출력 보장)
# 핸드아이 캘리브레이션이 DART TCP 기준으로 수집됐으므로 TF link_6과 기준 맞춤
TCP_OFFSET_M = np.array(_pose_cfg["tcp"]["offset_mm"][:3]) / 1000.0

LAYER_LABEL = {0: "layer_0(bottom)", 1: "layer_1(top)"}
COLORS = [(0,255,0),(0,128,255),(255,0,128),(255,255,0),(0,255,255),(255,0,255)]


def _ray_plane(gcx: float, gcy: float, tvec_marker: np.ndarray,
               tool_h: float, T: np.ndarray) -> np.ndarray | None:
    """ray-plane intersection 역투영 (카메라 기울기 보정).

    tool_h: 공구 전체 높이(m). 파지면은 tool_h/2 높이로 계산.
            마커 자체 좌표 계산 시 0.0 전달.
    반환: BASE 프레임 좌표 (m) 또는 None.
    """
    fx, fy = CAM_K[0, 0], CAM_K[1, 1]
    cx0, cy0 = CAM_K[0, 2], CAM_K[1, 2]

    P_m_cam  = tvec_marker.flatten()
    P_m_tcp  = HE_R @ P_m_cam + HE_T
    P_m_base = (T @ np.array([*P_m_tcp, 1.0]))[:3]

    plane_z = P_m_base[2] + tool_h / 2.0

    d_cam  = np.array([(gcx - cx0) / fx, (gcy - cy0) / fy, 1.0])
    d_base = T[:3, :3] @ HE_R @ d_cam
    O_base = (T @ np.array([*HE_T, 1.0]))[:3]

    if abs(d_base[2]) < 1e-6:
        return None
    t = (plane_z - O_base[2]) / d_base[2]
    if t < 0:
        return None
    return O_base + t * d_base


# ── TF 리스너 (base_link ← link_6) ───────────────────────────────────────────

class _TFListener:
    """백그라운드 스레드에서 rclpy spin → base_link←link_6 변환 행렬 캐시."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener
        from sensor_msgs.msg import JointState

        rclpy.init()
        self._node = Node("test_marker_scan_tf")
        self._buf  = Buffer()
        self._tfl  = TransformListener(self._buf, self._node)
        self._T: np.ndarray | None = None
        self._lock = threading.Lock()

        # rviz_joint_state_merger 미실행 시 fallback relay:
        # /dsr01/joint_states → /dsr01/joint_states_rviz
        # → robot_state_publisher가 TF 체인(base_link~link_6)을 발행하게 됨
        _js_pub = self._node.create_publisher(JointState, "/dsr01/joint_states_rviz", 10)
        self._node.create_subscription(
            JointState, "/dsr01/joint_states",
            lambda msg: _js_pub.publish(msg), 10,
        )

        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self) -> None:
        import rclpy
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def get_T(self) -> np.ndarray | None:
        import rclpy
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        try:
            tf = self._buf.lookup_transform("base_link", "link_6", rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            n = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            qx, qy, qz, qw = q.x/n, q.y/n, q.z/n, q.w/n
            R = np.array([
                [1-2*(qy*qy+qz*qz),   2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
                [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz),   2*(qy*qz-qx*qw)],
                [  2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
            ])
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3]  = [t.x, t.y, t.z]
            T[:3, 3]  = T[:3, 3] + R @ TCP_OFFSET_M
            with self._lock:
                self._T = T
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        with self._lock:
            return self._T.copy() if self._T is not None else None

    def shutdown(self) -> None:
        # rclpy.shutdown()은 호출하지 않음 — 로봇 bringup 노드들과 공유하는
        # rclpy 컨텍스트를 종료시키지 않기 위해 노드만 제거
        try:
            self._node.destroy_node()
        except Exception:
            pass

# 마우스 클릭 선택 상태
_state: dict = {"selected": -1, "polys": []}

# theta EMA 스무딩 (클래스별, sin/cos 분리로 각도 wraparound 안전)
_THETA_ALPHA = 0.25   # 낮을수록 더 부드럽고 느림, 높을수록 빠르고 노이즈 많음
_theta_ema: dict[str, tuple[float, float]] = {}  # name -> (sin_avg, cos_avg)

def _ema_theta(name: str, theta_deg: float) -> float:
    import math as _m
    s = _m.sin(_m.radians(theta_deg))
    c = _m.cos(_m.radians(theta_deg))
    if name not in _theta_ema:
        _theta_ema[name] = (s, c)
    else:
        ps, pc = _theta_ema[name]
        s = _THETA_ALPHA * s + (1.0 - _THETA_ALPHA) * ps
        c = _THETA_ALPHA * c + (1.0 - _THETA_ALPHA) * pc
        _theta_ema[name] = (s, c)
    return float(_m.degrees(_m.atan2(_theta_ema[name][0], _theta_ema[name][1])))

def _on_mouse(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN:
        return
    for i, poly in enumerate(_state["polys"]):
        if cv2.pointPolygonTest(poly, (x, y), False) >= 0:
            _state["selected"] = i if _state["selected"] != i else -1
            return
    _state["selected"] = -1

# ── ArUco ─────────────────────────────────────────────────────────────────────

_DICT     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_PARAMS   = cv2.aruco.DetectorParameters()
_DETECTOR = cv2.aruco.ArucoDetector(_DICT, _PARAMS)

OBJ_PTS = np.array([
    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0],
    [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
], dtype=np.float64)


# ── PCA grasp point ───────────────────────────────────────────────────────────

def _compute_pca(mask_2d: np.ndarray):
    pts = np.argwhere(mask_2d > 0)
    if len(pts) < 5:
        return None
    cy = float(pts[:, 0].mean())
    cx = float(pts[:, 1].mean())
    centered = pts.astype(np.float32) - np.array([cy, cx])
    x, y = centered[:, 1], centered[:, 0]
    N = len(pts)
    C = np.array([[np.sum(x**2)/N, np.sum(x*y)/N],
                  [np.sum(x*y)/N,  np.sum(y**2)/N]], dtype=np.float64)
    eigenvalues, eigenvectors = np.linalg.eig(C)
    order = np.argsort(eigenvalues)[::-1]
    lam1, lam2 = float(eigenvalues[order[0]]), float(eigenvalues[order[1]])
    v1 = eigenvectors[:, order[0]]
    reliability = lam1 / lam2 if lam2 > 1e-6 else 0.0
    return cx, cy, v1, reliability


def _grasp_point(cx, cy, v1, mask_2d, ratio, toward_narrow) -> tuple[float, float]:
    pts = np.argwhere(mask_2d > 0).astype(np.float32)
    centered = pts - np.array([cy, cx])
    proj = centered[:, 1] * v1[0] + centered[:, 0] * v1[1]
    length = float(proj.max() - proj.min())
    offset_px = length * ratio

    pos_pts = pts[proj > 0]
    neg_pts = pts[proj < 0]

    def _width(cluster):
        if len(cluster) < 3:
            return 0.0
        perp = cluster[:, 0] * v1[0] - cluster[:, 1] * v1[1]
        return float(perp.max() - perp.min())

    pos_narrow = _width(pos_pts) < _width(neg_pts)
    sign = (1.0 if pos_narrow else -1.0) if toward_narrow else (-1.0 if pos_narrow else 1.0)

    H, W = mask_2d.shape
    gx = float(np.clip(cx + sign * offset_px * v1[0], 0, W - 1))
    gy = float(np.clip(cy + sign * offset_px * v1[1], 0, H - 1))
    return gx, gy


# ── 메인 루프 ─────────────────────────────────────────────────────────────────

def main() -> None:
    model_path = _ROOT / "ros2_ws/src/vision/model_library/gripper_model/v1/weights/best.pt"
    device_idx = int(
        _rt_cfg.get("calibration", {}).get("c270_device", "/dev/video8")
        .replace("/dev/video", "")
    )

    print(f"[test_marker_scan] 모델: {model_path}")
    print(f"[test_marker_scan] 카메라: /dev/video{device_idx}")
    print(f"[test_marker_scan] 마커: {MARKER_SIZE*100:.1f}cm  PCA offset: {list(GRASP_OFFSET.keys())}")

    # TF 리스너 초기화 (ROS2 없으면 skip)
    tf_listener: _TFListener | None = None
    try:
        tf_listener = _TFListener()
        print("[test_marker_scan] TF 리스너 시작 (base_link ← link_6)")
    except Exception as e:
        print(f"[test_marker_scan] TF 없음 — link_6 기준 좌표 표시 ({e})")

    model = YOLO(str(model_path))

    cap = cv2.VideoCapture(device_idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(1)
    for _ in range(10):
        cap.grab()

    if not cap.isOpened():
        print("[FAIL] 카메라 열기 실패")
        sys.exit(1)

    cv2.namedWindow("marker_scan [click=select PCA | q=quit]", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("marker_scan [click=select PCA | q=quit]", _on_mouse)

    while True:
        for _ in range(2):
            cap.grab()
        ret, frame = cap.retrieve()
        if not ret:
            ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = _DETECTOR.detectMarkers(gray)

        results = model(frame, conf=0.50, verbose=False)
        r = results[0]

        vis = frame.copy()
        best_tool = None  # (name, grasp_cx, grasp_cy, score)

        # ── YOLO 마스크 + PCA grasp point ────────────────────────────────
        if r.masks is not None:
            h_img, w_img = frame.shape[:2]
            _state["polys"] = []
            for i, xy in enumerate(r.masks.xy):
                if len(xy) == 0:
                    _state["polys"].append(None)
                    continue
                color = COLORS[i % len(COLORS)]
                pts = xy.astype(np.int32).reshape((-1, 1, 2))
                _state["polys"].append(pts)

                selected = (i == _state["selected"])

                overlay = vis.copy()
                cv2.fillPoly(overlay, [pts], color)
                cv2.addWeighted(overlay, 0.35 if selected else 0.25, vis, 0.65 if selected else 0.75, 0, vis)
                cv2.polylines(vis, [pts], True, (255,255,255) if selected else color, 3 if selected else 2)

                cls_idx = int(r.boxes.cls[i])
                score   = float(r.boxes.conf[i])
                name    = model.names[cls_idx]

                # PCA grasp point 계산
                mask_2d = np.zeros((h_img, w_img), dtype=np.uint8)
                cv2.fillPoly(mask_2d, [pts], 255)
                pca = _compute_pca(mask_2d)

                if pca is not None:
                    cx_pca, cy_pca, v1, reliability = pca

                    # 180° 모호성 해소: 더 긴 쪽을 +v1 방향으로 고정
                    pts_f = np.argwhere(mask_2d > 0).astype(np.float32)
                    centered_f = pts_f - np.array([cy_pca, cx_pca])
                    proj_f = centered_f[:, 1] * v1[0] + centered_f[:, 0] * v1[1]
                    if len(proj_f) > 0 and abs(proj_f.min()) > abs(proj_f.max()):
                        v1 = -v1
                    import math as _math
                    theta_raw = _math.degrees(_math.atan2(float(v1[1]), float(v1[0])))
                    theta_deg = _ema_theta(name, theta_raw)

                    # 선택된 객체이거나 조건 충족 시 PCA 화살표 + 파지점 표시
                    show_pca = selected or (name in GRASP_OFFSET and reliability > 1.5)
                    if show_pca:
                        cfg = GRASP_OFFSET.get(name, {"ratio": 0.0, "toward_narrow": True})
                        if cfg["ratio"] > 0:
                            gx, gy = _grasp_point(cx_pca, cy_pca, v1, mask_2d,
                                                  cfg["ratio"], cfg["toward_narrow"])
                        else:
                            gx, gy = cx_pca, cy_pca
                        draw_color = (255, 255, 255) if selected else color
                        ax = int(cx_pca + v1[0] * 40)
                        ay = int(cy_pca + v1[1] * 40)
                        cv2.arrowedLine(vis, (int(cx_pca), int(cy_pca)),
                                        (ax, ay), draw_color, 2, tipLength=0.3)
                        grasp_cx, grasp_cy = int(gx), int(gy)
                        cv2.drawMarker(vis, (grasp_cx, grasp_cy), draw_color,
                                       cv2.MARKER_CROSS, 24, 2)
                        label_suffix = f" [PCA r={reliability:.1f} th={theta_deg:.1f}deg]"
                    else:
                        grasp_cx, grasp_cy = int(cx_pca), int(cy_pca)
                        cv2.circle(vis, (grasp_cx, grasp_cy), 6, color, -1)
                        label_suffix = f" [centroid r={reliability:.1f} th={theta_deg:.1f}deg]"
                else:
                    grasp_cx = int(xy[:, 0].mean())
                    grasp_cy = int(xy[:, 1].mean())
                    cv2.circle(vis, (grasp_cx, grasp_cy), 6, color, -1)
                    label_suffix = ""

                cv2.putText(vis, f"{name} {score:.2f}{label_suffix}",
                            (grasp_cx+8, grasp_cy-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)

                # 선택된 객체를 best_tool로 우선 사용, 없으면 최고 신뢰도
                if selected or best_tool is None or score > best_tool[3]:
                    best_tool = (name, grasp_cx, grasp_cy, score)
        else:
            _state["polys"] = []

        # ── ArUco + 3D 좌표 계산 ─────────────────────────────────────────
        P_tcp = None
        P_base = None
        P_marker_base = None  # 마커 중심점 BASE 좌표
        marker_info = "NOT FOUND"
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners_list, ids)
            for ci, mid in enumerate(ids.flatten()):
                if mid not in (0, 1):
                    continue
                img_pts = corners_list[ci][0].astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, CAM_K, DIST)
                if not ok:
                    continue
                tvec = tvec.flatten()
                Z_floor = float(tvec[2])

                mc = corners_list[ci][0]
                mx, my = int(mc[:, 0].mean()), int(mc[:, 1].mean())
                cv2.circle(vis, (mx, my), 8, (0,255,255), -1)
                cv2.putText(vis, f"ID{mid} Z={Z_floor*1000:.0f}mm",
                            (mx+10, my-10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)

                # 마커 중심점 BASE 좌표 (ray-plane, plane_h=0 → 마커 평면 자체)
                T = tf_listener.get_T() if tf_listener is not None else None
                if T is not None:
                    P_marker_base = _ray_plane(mx, my, tvec, 0.0, T)

                if best_tool is not None and T is not None:
                    name, gcx, gcy, _ = best_tool
                    tool_h = TOOL_H.get(name, 0.020)
                    P_base = _ray_plane(gcx, gcy, tvec, tool_h, T)
                    cv2.line(vis, (mx, my), (gcx, gcy), (255,255,0), 1)
                    P_tcp = P_base  # HUD 호환용 (link6 표시는 생략)
                else:
                    P_base = None

                marker_info = LAYER_LABEL.get(int(mid), f"ID{mid}")
                break

        # ── HUD ──────────────────────────────────────────────────────────
        sel_name = model.names[int(r.boxes.cls[_state["selected"]])] if (
            r.masks is not None and 0 <= _state["selected"] < len(r.boxes)
        ) else "none"
        hud = [
            f"gripper_model/v1  objs:{len(r.boxes)}  marker:{marker_info}  selected:{sel_name}",
        ]
        if P_marker_base is not None:
            hud.append(
                f"MARKER_BASE X={P_marker_base[0]*1000:+.1f}  Y={P_marker_base[1]*1000:+.1f}  Z={P_marker_base[2]*1000:+.1f} mm"
            )
        if P_tcp is not None:
            if P_base is not None:
                hud.append(
                    f"TOOL_BASE  X={P_base[0]*1000:+.1f}  Y={P_base[1]*1000:+.1f}  Z={P_base[2]*1000:+.1f} mm"
                )
                if P_marker_base is not None:
                    dx = (P_base[0] - P_marker_base[0]) * 1000
                    dy = (P_base[1] - P_marker_base[1]) * 1000
                    hud.append(f"OFFSET(tool-marker) dX={dx:+.1f}  dY={dy:+.1f} mm")
            hud.append(f"tool={best_tool[0]}  grasp_h={TOOL_H.get(best_tool[0],0.020)*1000:.0f}mm")
        else:
            hud.append("좌표: 마커 또는 공구 미검출")

        for li, txt in enumerate(hud):
            cv2.putText(vis, txt, (10, 28 + li*26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255,255,255), 2)

        cv2.imshow("marker_scan [click=select PCA | q=quit]", vis)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    if tf_listener is not None:
        tf_listener.shutdown()


if __name__ == "__main__":
    main()
