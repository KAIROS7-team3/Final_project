"""verify_grasp_coords.py
카메라(C270) 추정 좌표 vs TW 직접교시 기준값 오차 검증 스크립트.

흐름:
  ① 카메라 UI에서 객체 클릭 선택
  ② 'g' 키 → 추정 BASE (X,Y) 확보
  ③ MoveL → 안전 높이(400mm) + 수직 정렬
  ④ MoveL → 서랍 천장 높이(242.7mm)
  ⑤ GetCurrentPosx → 실제 TCP (X,Y) 읽기
  ⑥ TW 기준값 비교 → 오차 출력
  'r': 현재 위치에서 안전 높이로 복귀
  'q': 종료

대상: layer_1 공구 3종 (ratchet_wrench, utility_knife, socket_19mm)

실행:
  python3 scripts/verify_grasp_coords.py
"""
from __future__ import annotations

import logging
import queue
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

_ROOT = Path(__file__).resolve().parents[1]

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(message)s')
log = logging.getLogger('verify_grasp')


# ── 설정 로드 ─────────────────────────────────────────────────────────────────

def _load(p):
    with open(p) as f:
        return yaml.safe_load(f)

_cam = _load(_ROOT / "config/c270_camera_info.yaml")
_he  = _load(_ROOT / "config/c270_hand_eye.yaml")
_tb  = _load(_ROOT / "config/toolbox.yaml")
_rt  = _load(_ROOT / "config/runtime.yaml")
_vis = _load(_ROOT / "config/vision.yaml")

_i = _cam["intrinsics"]
CAM_K = np.array([[_i["fx"], 0, _i["cx"]], [0, _i["fy"], _i["cy"]], [0, 0, 1]], dtype=np.float64)
DIST  = np.array(_i["coeffs"], dtype=np.float64)

def _q2r(x, y, z, w):
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])

_r = _he["transformation"]["rotation"]
_t = _he["transformation"]["translation"]
HE_R = _q2r(_r["x"], _r["y"], _r["z"], _r["w"])
HE_T = np.array([_t["x"], _t["y"], _t["z"]])

# TCP 오프셋 (flange/link_6 → tool tip, config/robot_poses.yaml 기준).
# 핸드아이 캘리브레이션은 DART TCP 좌표(이 오프셋 포함)로 수집했으므로,
# ROS2 TF의 link_6(flange, 오프셋 미포함)를 그대로 쓰면 그만큼 누락된다.
_robot_poses = _load(_ROOT / "config/robot_poses.yaml")
TCP_OFFSET_M = np.array(_robot_poses["tcp"]["offset_mm"][:3]) / 1000.0

MARKER_SIZE = float(_tb["aruco_front"]["marker_size_m"])
def _tool_height(dims: dict) -> float:
    return float(dims.get("height", dims.get("diameter", 0.020)))

TOOL_H = {t["tool_id"]: _tool_height(t["dimensions"]) for t in _tb.get("tools", [])}
GRASP_OFFSET = {k: {"ratio": float(v["ratio"]), "toward_narrow": bool(v["toward_narrow"])}
                for k, v in _vis.get("grasp_offset", {}).items()}


# ── TW 기준값 (layer_1, X/Y만, mm) ──────────────────────────────────────────

TW_LAYER1: dict[str, tuple[float, float]] = {
    "ratchet_wrench": (392.97, 262.79),
    "utility_knife":  (447.11, 351.54),
    "socket_19mm":    (252.22, 328.52),
}

# 아랫층(layer_0) — 마커 직접교시가 프레임에 막혀 -Y 임의 오프셋이 섞여 있었으므로,
# 마커 X,Y는 윗층과 동일하다는 전제(같은 X,Y에 부착)로 보정한 값. 공구 좌표 자체는
# 직접교시 원본 그대로(가림 없이 도달 가능했음).
TW_LAYER0: dict[str, tuple[float, float]] = {
    "multi_tool":    (272.97, 335.26),
    "spanner_16mm":  (425.08, 354.26),
    "screwdriver":   (404.38, 267.27),
}
TW_REF: dict[str, tuple[float, float]] = {**TW_LAYER1, **TW_LAYER0}

# 마커 중심 직접교시 ground truth (사물함내부마커TCP좌표.tw)
# 윗층/아랫층 마커는 같은 X,Y에 부착됨(가정) — Z만 층별로 다름.
MARKER_GT_XY: tuple[float, float] = (202.82, 388.03)
MARKER_GT_Z_LAYER1: float = 92.7
MARKER_GT_Z_LAYER0: float = 37.13


# ── 모션 파라미터 ─────────────────────────────────────────────────────────────

TARGET_Z          = 92.7 + 150.0                         # 242.7mm (마커 바닥 + 서랍 내부 높이)
APPROACH_Z        = 350.0                                # MoveL 경유 높이
DESCENT_ORI       = [38.26, -180.0, 128.26]              # [rx, ry, rz] TW 교시값 기반 수직 정렬
GRIPPER_HOME_J    = [26.2, 18.04, 35.57, 25.49, 111.91, 42.14]   # gripper home 관절각도 (.tw)
GRIPPER_VIEW_POSE = [373.06, 331.52, 482.72,
                     89.99, 153.58, 93.26]               # gripper view TCP 포즈 (.tw)
VEL_L, ACC_L = 20.0, 15.0                               # 검증용 저속
DR_BASE, DR_ABS = 0, 0


# ── ArUco ─────────────────────────────────────────────────────────────────────

_DICT     = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
_DETECTOR = cv2.aruco.ArucoDetector(_DICT, cv2.aruco.DetectorParameters())
OBJ_PTS   = np.array([
    [-MARKER_SIZE/2,  MARKER_SIZE/2, 0], [ MARKER_SIZE/2,  MARKER_SIZE/2, 0],
    [ MARKER_SIZE/2, -MARKER_SIZE/2, 0], [-MARKER_SIZE/2, -MARKER_SIZE/2, 0],
], dtype=np.float64)
COLORS = [(0,255,0),(0,128,255),(255,0,128),(255,255,0),(0,255,255),(255,0,255)]

_state: dict = {"selected": -1, "polys": [], "status": "IDLE", "last_result": ""}
_cmd_q: queue.Queue = queue.Queue(maxsize=1)


def _on_mouse(event, x, y, flags, param):
    if event != cv2.EVENT_LBUTTONDOWN or _state["status"] == "MOVING":
        return
    for i, poly in enumerate(_state["polys"]):
        if poly is not None and cv2.pointPolygonTest(poly, (x, y), False) >= 0:
            _state["selected"] = i if _state["selected"] != i else -1
            return
    _state["selected"] = -1


# ── PCA grasp point ───────────────────────────────────────────────────────────

def _pca(mask):
    pts = np.argwhere(mask > 0)
    if len(pts) < 5:
        return None
    cy, cx = float(pts[:, 0].mean()), float(pts[:, 1].mean())
    c = pts.astype(np.float32) - np.array([cy, cx])
    x, y = c[:, 1], c[:, 0]
    N = len(pts)
    C = np.array([[np.sum(x*x)/N, np.sum(x*y)/N], [np.sum(x*y)/N, np.sum(y*y)/N]])
    ev, evec = np.linalg.eig(C)
    o = np.argsort(ev)[::-1]
    lam1, lam2 = float(ev[o[0]]), float(ev[o[1]])
    return cx, cy, evec[:, o[0]], (lam1 / lam2 if lam2 > 1e-6 else 0.0)


def _grasp_pt(cx, cy, v1, mask, ratio, toward_narrow):
    pts = np.argwhere(mask > 0).astype(np.float32)
    c = pts - np.array([cy, cx])
    proj = c[:, 1] * v1[0] + c[:, 0] * v1[1]
    off = float(proj.max() - proj.min()) * ratio
    pos, neg = pts[proj > 0], pts[proj < 0]
    def w(cl):
        if len(cl) < 3: return 0.0
        p = cl[:, 0] * v1[0] - cl[:, 1] * v1[1]
        return float(p.max() - p.min())
    pn = w(pos) < w(neg)
    sign = (1.0 if pn else -1.0) if toward_narrow else (-1.0 if pn else 1.0)
    H, W = mask.shape
    return (float(np.clip(cx + sign * off * v1[0], 0, W-1)),
            float(np.clip(cy + sign * off * v1[1], 0, H-1)))


def _marker_base_xyz(tvec_marker: np.ndarray, T: np.ndarray) -> tuple[float, float, float]:
    """마커 중심의 BASE 프레임 좌표 (ray-plane reprojection 없이 직접 변환).

    공구별 그립포인트 계산(PCA, ray-plane)을 거치지 않는 가장 단순한 경로라서,
    여기서 오차가 크게 나오면 문제가 핸드아이/마커 자체에 있다는 뜻이고,
    여기서는 괜찮은데 공구 좌표에서만 크면 그립포인트 계산 쪽이 원인이라는 뜻.
    """
    P_m_cam  = tvec_marker.flatten()
    P_m_tcp  = HE_R @ P_m_cam + HE_T
    P_m_base = (T @ np.array([*P_m_tcp, 1.0]))[:3]
    return float(P_m_base[0]*1000), float(P_m_base[1]*1000), float(P_m_base[2]*1000)


def _ray_plane(gcx: float, gcy: float,
               rvec_marker: np.ndarray, tvec_marker: np.ndarray,
               tool_h: float, T: np.ndarray) -> tuple[float, float] | None:
    """ray-plane intersection 역투영 — 서랍 기울기 보정 포함.

    1. 픽셀 (gcx, gcy) → 카메라 프레임 ray 방향
    2. ray 방향 → BASE 프레임 변환
    3. 카메라 광학 중심 → BASE 프레임
    4. 마커 rvec으로 서랍 실제 평면 법선 계산 (기울기 반영)
    5. 기울어진 평면과 ray 교점 + 법선 방향 tool_h/2 오프셋
    """
    fx, fy = CAM_K[0, 0], CAM_K[1, 1]
    cx0, cy0 = CAM_K[0, 2], CAM_K[1, 2]

    # 마커 중심 → BASE 프레임
    P_m_cam  = tvec_marker.flatten()
    P_m_tcp  = HE_R @ P_m_cam + HE_T
    P_m_base = (T @ np.array([*P_m_tcp, 1.0]))[:3]

    # 마커 평면 법선 → BASE 프레임 (rvec Z축 = 마커 법선, 서랍 기울기 반영)
    R_m2cam, _ = cv2.Rodrigues(rvec_marker.flatten())
    n_cam  = R_m2cam @ np.array([0.0, 0.0, 1.0])
    n_base = T[:3, :3] @ HE_R @ n_cam
    n_base = n_base / np.linalg.norm(n_base)
    if n_base[2] < 0:  # 법선이 아래를 향하면 반전
        n_base = -n_base

    # 공구 파지점 평면: 마커 평면에서 법선 방향으로 tool_h/2 올림
    P_plane = P_m_base + (tool_h / 2.0) * n_base

    # 픽셀 → 카메라 프레임 ray 방향
    d_cam  = np.array([(gcx - cx0) / fx, (gcy - cy0) / fy, 1.0])

    # ray → BASE 프레임
    d_base = T[:3, :3] @ HE_R @ d_cam

    # 카메라 광학 중심 → BASE 프레임
    O_base = (T @ np.array([*HE_T, 1.0]))[:3]

    # 기울어진 평면과 ray 교점
    denom = np.dot(n_base, d_base)
    if abs(denom) < 1e-6:
        return None
    t = np.dot(n_base, P_plane - O_base) / denom
    if t < 0:
        return None
    P = O_base + t * d_base
    return float(P[0] * 1000), float(P[1] * 1000)


# ── ROS 브리지 ────────────────────────────────────────────────────────────────

class _ROSBridge:
    """백그라운드에서 TF 수신 + MoveL + GetCurrentPosx 처리."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor
        from tf2_ros import Buffer, TransformListener
        from sensor_msgs.msg import JointState
        from dsr_msgs2.srv import MoveLine, MoveJoint, MoveJointx, GetCurrentPosx

        rclpy.init()
        self._node = Node("verify_grasp_coords")
        self._buf  = Buffer()
        self._tfl  = TransformListener(self._buf, self._node)
        self._T: np.ndarray | None = None
        self._T_lock = threading.Lock()

        # joint_states relay — rviz_joint_state_merger 미실행 시에만 활성화
        # bringup에서 이미 발행 중이면 relay 생략 (중복 publisher → TF 충돌 방지)
        _existing = self._node.get_publishers_info_by_topic("/dsr01/joint_states_rviz")
        if not _existing:
            _jspub = self._node.create_publisher(JointState, "/dsr01/joint_states_rviz", 10)
            self._node.create_subscription(JointState, "/dsr01/joint_states",
                                           lambda m: _jspub.publish(m), 10)
            log.info("[ROS] joint_states relay 활성화")
        else:
            log.info("[ROS] joint_states_rviz 이미 발행 중 — relay 생략")
        self._movel_cli   = self._node.create_client(MoveLine,    "/dsr01/motion/move_line")
        self._movej_cli   = self._node.create_client(MoveJoint,  "/dsr01/motion/move_joint")
        self._movejx_cli  = self._node.create_client(MoveJointx, "/dsr01/motion/move_jointx")
        self._getpos_cli  = self._node.create_client(
            GetCurrentPosx, "/dsr01/aux_control/get_current_posx")

        self._exec = MultiThreadedExecutor()
        self._exec.add_node(self._node)
        threading.Thread(target=self._exec.spin, daemon=True).start()
        threading.Thread(target=self._motion_loop, daemon=True).start()

        log.info("[ROS] 서비스 대기 중 (최대 10s)...")
        for cli, name in [(self._movel_cli,  "move_line"),
                          (self._movej_cli,  "move_joint"),
                          (self._movejx_cli, "move_jointx"),
                          (self._getpos_cli, "get_current_posx")]:
            if not cli.wait_for_service(timeout_sec=10.0):
                log.info(f"[ROS] {name} 서비스 없음 — bringup 먼저 실행")
    def get_T(self) -> np.ndarray | None:
        import rclpy
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
        try:
            tf = self._buf.lookup_transform("base_link", "link_6", rclpy.time.Time())
            t = tf.transform.translation; q = tf.transform.rotation
            n = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            qx, qy, qz, qw = q.x/n, q.y/n, q.z/n, q.w/n
            R = np.array([
                [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                [  2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                [  2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
            ])
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [t.x, t.y, t.z]
            # link_6(flange) → TCP 보정: 핸드아이 캘리브레이션이 TCP 기준으로
            # 수집됐으므로, 여기서도 TCP 기준 좌표로 맞춰야 HE_R/HE_T와 정합됨.
            T[:3, 3] = T[:3, 3] + R @ TCP_OFFSET_M
            with self._T_lock:
                self._T = T
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        with self._T_lock:
            return self._T.copy() if self._T is not None else None

    def _movej(self, pos: list, timeout: float = 30.0) -> bool:
        from dsr_msgs2.srv import MoveJoint
        req = MoveJoint.Request()
        req.pos = [float(v) for v in pos]
        req.vel = VEL_L; req.acc = ACC_L
        req.time = 0.0; req.radius = 0.0
        req.mode = DR_ABS; req.blend_type = 0; req.sync_type = 0
        fut = self._movej_cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            log.error("  [ABORT] MoveJ 타임아웃")
            return False
        res = fut.result()
        if res is None or not res.success:
            log.error("  [ABORT] MoveJ 실패 — E-stop 또는 컨트롤러 오류")
            return False
        return True

    def _movejx(self, pos: list, timeout: float = 30.0) -> bool:
        from dsr_msgs2.srv import MoveJointx
        req = MoveJointx.Request()
        req.pos = [float(v) for v in pos]
        req.sol = 0
        req.vel = VEL_L; req.acc = ACC_L
        req.time = 0.0; req.ref = DR_BASE
        req.mode = DR_ABS; req.blend_type = 0; req.sync_type = 0
        fut = self._movejx_cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            log.error("  [ABORT] MoveJx 타임아웃")
            return False
        res = fut.result()
        if res is None or not res.success:
            log.error("  [ABORT] MoveJx 실패 — E-stop 또는 컨트롤러 오류")
            return False
        return True

    def _movel(self, pos: list, timeout: float = 30.0) -> bool:
        from dsr_msgs2.srv import MoveLine
        req = MoveLine.Request()
        req.pos = [float(v) for v in pos]
        req.vel = [VEL_L, VEL_L]; req.acc = [ACC_L, ACC_L]
        req.time = 0.0; req.radius = 0.0
        req.ref = DR_BASE; req.mode = DR_ABS
        req.blend_type = 0; req.sync_type = 0
        fut = self._movel_cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            log.error("  [ABORT] MoveL 타임아웃")
            return False
        res = fut.result()
        if res is None or not res.success:
            log.error("  [ABORT] MoveL 실패 — E-stop 또는 컨트롤러 오류")
            return False
        return True

    def _get_tcp_xyz(self) -> tuple[float, float, float] | None:
        from dsr_msgs2.srv import GetCurrentPosx
        req = GetCurrentPosx.Request(); req.ref = DR_BASE
        fut = self._getpos_cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        if not done.wait(3.0):
            return None
        res = fut.result()
        if res is None or not res.success:
            return None
        p = res.task_pos_info[0].data
        return (p[0], p[1], p[2])

    def _motion_loop(self):
        while True:
            try:
                cmd = _cmd_q.get(timeout=0.5)
            except queue.Empty:
                continue
            t = cmd.get("type")
            if t == "return":
                self._do_return()
            elif t == "gripper_view":
                self._do_gripper_view()
            elif t == "verify_marker":
                self._do_verify_marker(cmd)
            else:
                self._do_verify(cmd)

    def _do_verify(self, cmd: dict):
        name = cmd["name"]; cam_x, cam_y = cmd["cam_x"], cmd["cam_y"]
        _state["status"] = "MOVING"
        log.info(f"\n[검증 시작] {name}  추정 X={cam_x:.1f}  Y={cam_y:.1f} mm")
        # ① MoveJ — gripper home으로 IK 분기 고정
        log.info(f"  ① MoveJ  → gripper home (IK 분기 고정)")
        if not self._movej(GRIPPER_HOME_J):
            _state["status"] = "ERROR"; return

        # ② MoveL — 경유 높이(350mm)에서 추정 X,Y + 수직 정렬
        log.info(f"  ② MoveL  → X={cam_x:.1f} Y={cam_y:.1f} Z={APPROACH_Z:.0f}mm (수직 정렬)")
        if not self._movel([cam_x, cam_y, APPROACH_Z] + DESCENT_ORI):
            _state["status"] = "ERROR"; return

        # ※ 안전을 위해 실제 깊이(TARGET_Z)까지는 하강하지 않음 — X,Y는 approach
        #   높이에서도 동일하므로 X,Y 정확도 검증에는 하강이 필요 없음. 아랫층처럼
        #   깊이 가정값이 아직 검증되지 않은 경우 특히 중요.
        log.info(f"  ※ 실제 깊이({TARGET_Z:.1f}mm)까지는 하강하지 않음 — X,Y는 이 높이에서도 동일")
        # ④ 실제 TCP 읽기
        tcp = self._get_tcp_xyz()
        if tcp is None:
            log.error("  [ABORT] TCP 좌표 읽기 실패")
            _state["status"] = "ERROR"; return
        tx, ty, _ = tcp

        # ⑤ TW 기준값 비교
        lines = [
            f"\n{'='*54}",
            f"  공구:        {name}",
            f"  카메라 추정  X={cam_x:+7.1f}  Y={cam_y:+7.1f} mm",
            f"  실제 TCP     X={tx:+7.1f}  Y={ty:+7.1f} mm",
        ]
        tw = TW_REF.get(name)
        if tw:
            wx, wy = tw
            ex, ey = tx - wx, ty - wy
            lines += [
                f"  TW 기준값    X={wx:+7.1f}  Y={wy:+7.1f} mm",
                f"  오차         dX={ex:+7.1f}  dY={ey:+7.1f} mm",
                f"  거리 오차    {np.sqrt(ex*ex + ey*ey):.1f} mm",
            ]
        else:
            lines.append("  TW 기준값 없음 (미등록 공구)")
        lines.append('='*54)
        result = "\n".join(lines)
        log.info(result)
        _state["last_result"] = result
        _state["status"] = "DONE"

    def _do_verify_marker(self, cmd: dict):
        """마커 자체의 BASE 좌표 검증 — ray-plane/그립포인트 계산을 거치지 않는 직접 경로."""
        cam_x, cam_y, cam_z = cmd["cam_x"], cmd["cam_y"], cmd["cam_z"]
        _state["status"] = "MOVING"
        log.info(f"\n[마커 검증 시작] 추정 X={cam_x:.1f}  Y={cam_y:.1f}  Z={cam_z:.1f} mm")
        if not self._movej(GRIPPER_HOME_J):
            _state["status"] = "ERROR"; return
        log.info(f"  ① MoveL  → X={cam_x:.1f} Y={cam_y:.1f} Z={APPROACH_Z:.0f}mm (수직 정렬, 하강 없음)")
        if not self._movel([cam_x, cam_y, APPROACH_Z] + DESCENT_ORI):
            _state["status"] = "ERROR"; return
        log.info(f"  ※ 마커 높이까지는 하강하지 않음(층별 92.7mm/37.13mm) — X,Y는 이 높이에서도 동일")
        tcp = self._get_tcp_xyz()
        if tcp is None:
            log.error("  [ABORT] TCP 좌표 읽기 실패")
            _state["status"] = "ERROR"; return
        tx, ty, _ = tcp
        wx, wy = MARKER_GT_XY
        ex, ey = tx - wx, ty - wy
        lines = [
            f"\n{'='*54}",
            f"  대상:        마커 중심 (ground truth)",
            f"  카메라 추정  X={cam_x:+7.1f}  Y={cam_y:+7.1f} mm",
            f"  실제 TCP     X={tx:+7.1f}  Y={ty:+7.1f} mm",
            f"  GT 기준값    X={wx:+7.1f}  Y={wy:+7.1f} mm",
            f"  오차         dX={ex:+7.1f}  dY={ey:+7.1f} mm",
            f"  거리 오차    {np.sqrt(ex*ex + ey*ey):.1f} mm",
            '='*54,
        ]
        result = "\n".join(lines)
        log.info(result)
        _state["last_result"] = result
        _state["status"] = "DONE"

    def _do_gripper_view(self):
        _state["status"] = "MOVING"
        log.info("[이동] gripper home 관절각도로 이동 중...")
        if not self._movej(GRIPPER_HOME_J):
            _state["status"] = "ERROR"; return
        log.info("[이동] gripper view TCP 포즈로 이동 중...")
        if not self._movel(GRIPPER_VIEW_POSE):
            _state["status"] = "ERROR"; return
        _state["status"] = "IDLE"

    def _do_return(self):
        _state["status"] = "MOVING"
        log.info("[복귀] gripper home → gripper view 복귀 중...")
        if not self._movej(GRIPPER_HOME_J):
            _state["status"] = "ERROR"; return
        if not self._movel(GRIPPER_VIEW_POSE):
            _state["status"] = "ERROR"; return
        _state["status"] = "IDLE"

    def request_verify(self, name: str, cam_x: float, cam_y: float):
        try:
            _cmd_q.put_nowait({"type": "verify", "name": name, "cam_x": cam_x, "cam_y": cam_y})
        except queue.Full:
            log.warning("[경고] 이전 명령 처리 중 — 잠시 후 다시 시도")
    def request_verify_marker(self, cam_x: float, cam_y: float, cam_z: float):
        try:
            _cmd_q.put_nowait(
                {"type": "verify_marker", "cam_x": cam_x, "cam_y": cam_y, "cam_z": cam_z})
        except queue.Full:
            log.warning("[경고] 이전 명령 처리 중 — 잠시 후 다시 시도")
    def request_gripper_view(self):
        try:
            _cmd_q.put_nowait({"type": "gripper_view"})
        except queue.Full:
            log.warning("[경고] 이전 명령 처리 중")
    def request_return(self):
        try:
            _cmd_q.put_nowait({"type": "return"})
        except queue.Full:
            pass

    def shutdown(self):
        try:
            self._node.destroy_node()
        except Exception:
            pass


# ── 메인 루프 ─────────────────────────────────────────────────────────────────

def main():
    device_idx = int(
        _rt.get("calibration", {}).get("c270_device", "/dev/video8").replace("/dev/video", "")
    )
    model_path = _ROOT / "ros2_ws/src/vision/model_library/gripper_model/v1/weights/best.pt"

    log.info(f"[verify] 카메라: /dev/video{device_idx}")
    log.info(f"[verify] 모델: {model_path}")
    log.info("[verify] 키: 클릭=객체선택  h=gripper_view이동  g=이동+검증  m=마커자체검증  r=검증후복귀  q=종료")
    ros: _ROSBridge | None = None
    try:
        ros = _ROSBridge()
    except Exception as e:
        log.warning(f"[경고] ROS2 초기화 실패 ({e}) — 카메라 전용 모드 (좌표만 표시)")
    model = YOLO(str(model_path))
    cap = cv2.VideoCapture(device_idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    time.sleep(1)
    for _ in range(10):
        cap.grab()
    if not cap.isOpened():
        log.error("[FAIL] 카메라 열기 실패")
        sys.exit(1)

    WIN = "verify_grasp [click=select | h=gripper_view | g=go | r=return | q=quit]"
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WIN, _on_mouse)

    fx, fy = CAM_K[0, 0], CAM_K[1, 1]
    cx0, cy0 = CAM_K[0, 2], CAM_K[1, 2]
    best_tool: tuple | None = None   # (name, base_x_mm, base_y_mm, score)

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
        best_tool = None

        T = ros.get_T() if ros else None

        # ArUco 마커 → rvec/tvec 확보 (ray-plane intersection용)
        rvec_marker: np.ndarray | None = None
        tvec_marker: np.ndarray | None = None
        marker_base: tuple[float, float, float] | None = None
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, corners_list, ids)
            for ci, mid in enumerate(ids.flatten()):
                if mid not in (0, 1):
                    continue
                img_pts = corners_list[ci][0].astype(np.float64)
                ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, CAM_K, DIST)
                if not ok:
                    continue
                rvec_marker = rvec.flatten()
                tvec_marker = tvec.flatten()
                mc = corners_list[ci][0]
                mx, my = int(mc[:, 0].mean()), int(mc[:, 1].mean())
                cv2.circle(vis, (mx, my), 8, (0, 255, 255), -1)
                cv2.putText(vis, f"ID{mid} Z={tvec_marker[2]*1000:.0f}mm",
                            (mx+8, my-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                if T is not None:
                    marker_base = _marker_base_xyz(tvec_marker, T)
                    mb_txt = f"marker BASE X={marker_base[0]:+.0f} Y={marker_base[1]:+.0f}"
                    cv2.putText(vis, mb_txt, (mx+8, my+14),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
                break

        # YOLO 마스크 + BASE 좌표 계산
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
                sel = (i == _state["selected"])

                ov = vis.copy()
                cv2.fillPoly(ov, [pts], color)
                cv2.addWeighted(ov, 0.35 if sel else 0.25, vis, 0.65 if sel else 0.75, 0, vis)
                cv2.polylines(vis, [pts], True, (255, 255, 255) if sel else color, 3 if sel else 2)

                cls_idx = int(r.boxes.cls[i])
                score   = float(r.boxes.conf[i])
                name    = model.names[cls_idx]

                mask2d = np.zeros((h_img, w_img), dtype=np.uint8)
                cv2.fillPoly(mask2d, [pts], 255)
                pca = _pca(mask2d)

                if pca:
                    gcx, gcy, v1, rel = pca
                    cfg = GRASP_OFFSET.get(name, {"ratio": 0.0, "toward_narrow": True})
                    if cfg["ratio"] > 0:
                        gcx, gcy = _grasp_pt(
                            gcx, gcy, v1, mask2d, cfg["ratio"], cfg["toward_narrow"])
                else:
                    gcx, gcy = float(xy[:, 0].mean()), float(xy[:, 1].mean())

                cv2.drawMarker(vis, (int(gcx), int(gcy)),
                               (255, 255, 255) if sel else color, cv2.MARKER_CROSS, 20, 2)
                cv2.putText(vis, f"{name} {score:.2f}",
                            (int(gcx)+8, int(gcy)-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # BASE 좌표 계산 — ray-plane intersection (카메라 기울기 보정)
                base_x_mm = base_y_mm = None
                if rvec_marker is not None and tvec_marker is not None and T is not None:
                    tool_h = TOOL_H.get(name, 0.020)
                    result = _ray_plane(gcx, gcy, rvec_marker, tvec_marker, tool_h, T)
                    if result is not None:
                        base_x_mm, base_y_mm = result
                    if sel and base_x_mm is not None:
                        cv2.putText(vis, f"BASE X={base_x_mm:+.0f} Y={base_y_mm:+.0f} mm",
                                    (int(gcx)+8, int(gcy)+18),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 2)

                if sel:
                    best_tool = (name, base_x_mm, base_y_mm, score)
                elif best_tool is None or score > best_tool[3]:
                    best_tool = (name, base_x_mm, base_y_mm, score)
        else:
            _state["polys"] = []

        # HUD
        status = _state["status"]
        sc = {"IDLE": (200,200,200), "MOVING": (0,200,255),
              "DONE": (0,255,0),    "ERROR":  (0,0,255)}.get(status, (200,200,200))
        sel_name = (model.names[int(r.boxes.cls[_state["selected"]])]
                    if r.masks is not None and 0 <= _state["selected"] < len(r.boxes) else "none")
        hud = [
            f"status:{status}  selected:{sel_name}",
            f"target Z={TARGET_Z:.1f}mm  return→gripper_view",
        ]
        if best_tool and best_tool[1] is not None:
            hud.append(f"추정 BASE  X={best_tool[1]:+.1f}  Y={best_tool[2]:+.1f} mm  → 'g' 이동")
        elif tvec_marker is None:
            hud.append("마커 미검출 — BASE 좌표 계산 불가")
        for li, txt in enumerate(hud):
            cv2.putText(vis, txt, (10, 28 + li*26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, sc, 2)

        cv2.imshow(WIN, vis)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('h'):
            if ros:
                if _state["status"] == "MOVING":
                    log.warning("[경고] 이동 중 — 완료 후 'h' 다시 누르세요")
                else:
                    ros.request_gripper_view()
        elif key == ord('g'):
            if _state["status"] == "MOVING":
                log.warning("[경고] 이미 이동 중")
            elif not ros:
                log.warning("[경고] ROS2 미연결 — 좌표만 표시 가능")
            elif best_tool is None or best_tool[1] is None:
                log.warning("[경고] 객체 미선택 또는 BASE 좌표 없음 (마커 검출 확인)")
            else:
                name, bx, by, _ = best_tool
                ros.request_verify(name, bx, by)
        elif key == ord('m'):
            if _state["status"] == "MOVING":
                log.warning("[경고] 이미 이동 중")
            elif not ros:
                log.warning("[경고] ROS2 미연결")
            elif marker_base is None:
                log.warning("[경고] 마커 미검출 — BASE 좌표 없음")
            else:
                ros.request_verify_marker(marker_base[0], marker_base[1], marker_base[2])
        elif key == ord('r'):
            if ros:
                if _state["status"] == "MOVING":
                    log.warning("[경고] 이동 중에는 복귀 불가 — 완료 후 'r' 다시 누르세요")
                else:
                    ros.request_return()

    cap.release()
    cv2.destroyAllWindows()
    if ros:
        ros.shutdown()


if __name__ == "__main__":
    main()
