"""C270 그리퍼 카메라 segmentation centroid → link_6 / base_link 실제 좌표 변환.

파이프라인:
  YOLO seg → 픽셀 centroid (cx, cy)
  → 왜곡 보정 (c270_camera_info.yaml)
  → deproject: (u-cx)*Z/fx, (v-cy)*Z/fy, Z  (카메라 좌표계)
  → T_cam2ee (c270_hand_eye.yaml)  → link_6 좌표
  → T_ee2base (ROS2 TF 실시간)     → base_link 좌표

ROS2 드라이버가 없으면 link_6 좌표만 표시.

사용법:
    python3 scripts/examples/infer_c270_world_coords.py
    python3 scripts/examples/infer_c270_world_coords.py --device /dev/video2
    python3 scripts/examples/infer_c270_world_coords.py --images ~/gripper_cam_0
    python3 scripts/examples/infer_c270_world_coords.py --depth 0.20
    python3 scripts/examples/infer_c270_world_coords.py --no-ros  # TF 없이 link_6만
"""
from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT    = Path(__file__).resolve().parents[2]
_VISION_CFG      = _PROJECT_ROOT / "config" / "vision.yaml"
_CAM_INFO_PATH   = _PROJECT_ROOT / "config" / "c270_camera_info.yaml"
_HAND_EYE_PATH   = _PROJECT_ROOT / "config" / "c270_hand_eye.yaml"
_SNAPSHOT_DIR    = _PROJECT_ROOT / "scripts" / "infer_snapshots"
_DEFAULT_DEPTH_M = 0.15   # 기본 카메라~공구 거리 [m]


def _load_default_model() -> Path:
    """config/vision.yaml gripper_model_path에서 기본 모델 경로 로드.
    null이거나 미설정이면 model_library 기본 경로 반환.
    """
    try:
        with _VISION_CFG.open() as f:
            p = yaml.safe_load(f).get("yolo", {}).get("gripper_model_path")
        if p:
            return _PROJECT_ROOT / p
    except Exception:
        pass
    return _PROJECT_ROOT / "ros2_ws/src/vision/model_library/gripper_model/v1/weights/best.pt"


_DEFAULT_MODEL = _load_default_model()
_WIN_NAME        = "C270 World Coords"

_COLORS = [
    (255,  80,  80),
    ( 80, 200,  80),
    ( 80,  80, 255),
    (255, 200,  50),
    (200,  80, 255),
    ( 50, 220, 220),
]


# ── ROS2 TF 조회 (옵션) ───────────────────────────────────────────────────────

class TFListener:
    """백그라운드 스레드에서 rclpy spin → base_link←link_6 변환 캐시."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener

        rclpy.init()
        self._node = Node("c270_world_coords_tf")
        self._buf  = Buffer()
        self._tfl  = TransformListener(self._buf, self._node)
        self._T: np.ndarray | None = None
        self._lock = threading.Lock()

        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        logger.info("TF 리스너 시작 (base_link ← link_6)")

    def _spin(self) -> None:
        import rclpy
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def get_T_ee2base(self) -> np.ndarray | None:
        """최신 T_ee2base (4×4) 반환. 아직 없으면 None."""
        import rclpy
        from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

        try:
            tf = self._buf.lookup_transform(
                "base_link", "link_6", rclpy.time.Time()
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3]  = [t.x, t.y, t.z]
            with self._lock:
                self._T = T
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

        with self._lock:
            return self._T.copy() if self._T is not None else None

    def shutdown(self) -> None:
        import rclpy
        self._node.destroy_node()
        rclpy.shutdown()


# ── 캘리브레이션 로더 ─────────────────────────────────────────────────────────

def _load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """(K 3×3, dist 1×14) 반환."""
    with path.open() as f:
        cfg = yaml.safe_load(f)
    intr = cfg["intrinsics"]
    K = np.array([
        [intr["fx"], 0.0,        intr["cx"]],
        [0.0,        intr["fy"], intr["cy"]],
        [0.0,        0.0,        1.0       ],
    ], dtype=np.float64)
    dist = np.array(intr["coeffs"], dtype=np.float64).reshape(1, -1)
    return K, dist


def _load_hand_eye(path: Path) -> np.ndarray:
    """c270_hand_eye.yaml → T_cam2ee (4×4, c270_optical_frame → link_6)."""
    with path.open() as f:
        cfg = yaml.safe_load(f)
    tr = cfg["transformation"]
    rot = tr["rotation"]
    t   = tr["translation"]
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat(
        [rot["x"], rot["y"], rot["z"], rot["w"]]
    ).as_matrix()
    T[:3, 3] = [t["x"], t["y"], t["z"]]
    return T


# ── 좌표 변환 ────────────────────────────────────────────────────────────────

def pixel_to_link6(
    cx_px: float,
    cy_px: float,
    depth_m: float,
    K: np.ndarray,
    dist: np.ndarray,
    T_cam2ee: np.ndarray,
) -> np.ndarray:
    """픽셀 centroid → link_6 좌표 [m].

    1. 왜곡 보정 (undistortPoints)
    2. deproject: Z 곱해 카메라 3D 좌표
    3. T_cam2ee 적용 → link_6 좌표
    """
    # 왜곡 보정
    pts = np.array([[[cx_px, cy_px]]], dtype=np.float64)
    undist = cv2.undistortPoints(pts, K, dist, P=K)
    u, v = undist[0, 0]

    # deproject
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    X_cam = (u - cx) * depth_m / fx
    Y_cam = (v - cy) * depth_m / fy
    Z_cam = depth_m
    p_cam = np.array([X_cam, Y_cam, Z_cam, 1.0])

    # T_cam2ee
    p_ee = T_cam2ee @ p_cam
    return p_ee[:3]


# ── 시각화 ───────────────────────────────────────────────────────────────────

def _draw_centroid(
    img: np.ndarray,
    cx: float,
    cy: float,
    label: str,
    color: tuple[int, int, int],
    link6_xyz: np.ndarray | None = None,
    base_xyz: np.ndarray | None = None,
) -> None:
    ix, iy = int(cx), int(cy)
    arm = 14
    cv2.line(img, (ix - arm, iy), (ix + arm, iy), color, 2)
    cv2.line(img, (ix, iy - arm), (ix, iy + arm), color, 2)
    cv2.circle(img, (ix, iy), 5, color, -1)

    lines = [label]
    if link6_xyz is not None:
        x, y, z = link6_xyz
        lines.append(f"link6: ({x*1000:+.1f},{y*1000:+.1f},{z*1000:+.1f})mm")
    if base_xyz is not None:
        x, y, z = base_xyz
        lines.append(f"base: ({x*1000:+.1f},{y*1000:+.1f},{z*1000:+.1f})mm")

    for i, line in enumerate(lines):
        cv2.putText(img, line, (ix + 8, iy - 10 + i * 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)


def _draw_hud(img: np.ndarray, depth_m: float, n_det: int) -> None:
    h = img.shape[0]
    cv2.putText(img, f"depth={depth_m*100:.1f}cm (+/-) | det={n_det}",
                (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
    cv2.putText(img, "coords: link_6 frame [mm]  |  s:snap  q:quit",
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)


# ── 추론 ─────────────────────────────────────────────────────────────────────

def process_frame(
    frame: np.ndarray,
    model: YOLO,
    conf: float,
    iou: float,
    depth_m: float,
    K: np.ndarray,
    dist: np.ndarray,
    T_cam2ee: np.ndarray,
    T_ee2base: np.ndarray | None = None,
) -> tuple[np.ndarray, list[dict]]:
    """프레임 추론 → annotated 이미지 + 검출 결과 반환."""
    results = model(frame, conf=conf, iou=iou, verbose=False)
    result  = results[0]
    names   = model.names

    annotated = frame.copy()
    detections: list[dict] = []

    if result.masks is None or len(result.boxes) == 0:
        cv2.putText(annotated, "No detection", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
        _draw_hud(annotated, depth_m, 0)
        return annotated, detections

    masks = result.masks.data.cpu().numpy()
    boxes = result.boxes
    H, W  = frame.shape[:2]

    for i, mask in enumerate(masks):
        cls_id = int(boxes.cls[i].item())
        score  = float(boxes.conf[i].item())
        label  = names.get(cls_id, str(cls_id))
        color  = _COLORS[cls_id % len(_COLORS)]

        if mask.shape != (H, W):
            mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)
        binary = (mask > 0.5).astype(np.uint8)

        # 마스크 오버레이
        layer = np.zeros_like(annotated)
        layer[binary == 1] = color
        annotated = cv2.addWeighted(annotated, 0.65, layer, 0.35, 0)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(annotated, contours, -1, color, 2)

        # centroid
        pts = np.argwhere(binary > 0)
        if len(pts) == 0:
            continue
        cy_px = float(pts[:, 0].mean())
        cx_px = float(pts[:, 1].mean())

        # link_6 좌표
        link6_xyz = pixel_to_link6(cx_px, cy_px, depth_m, K, dist, T_cam2ee)

        # base_link 좌표 (TF 있을 때만)
        base_xyz: np.ndarray | None = None
        if T_ee2base is not None:
            p_hom = np.append(link6_xyz, 1.0)
            base_xyz = (T_ee2base @ p_hom)[:3]

        _draw_centroid(annotated, cx_px, cy_px,
                       f"{label} {score:.2f}", color, link6_xyz, base_xyz)

        det = {
            "label": label,
            "score": score,
            "centroid_px": (cx_px, cy_px),
            "link6_mm": link6_xyz * 1000.0,
            "base_mm": base_xyz * 1000.0 if base_xyz is not None else None,
        }
        detections.append(det)
        if base_xyz is not None:
            logger.info(
                "%-16s  link6=(%.1f,%.1f,%.1f)mm  base=(%.1f,%.1f,%.1f)mm",
                label,
                link6_xyz[0]*1000, link6_xyz[1]*1000, link6_xyz[2]*1000,
                base_xyz[0]*1000,  base_xyz[1]*1000,  base_xyz[2]*1000,
            )
        else:
            logger.info(
                "%-16s  link6=(%.1f,%.1f,%.1f)mm",
                label, link6_xyz[0]*1000, link6_xyz[1]*1000, link6_xyz[2]*1000,
            )

    _draw_hud(annotated, depth_m, len(detections))
    return annotated, detections


# ── 실행 모드 ────────────────────────────────────────────────────────────────

_DEPTH_STEP = 0.01   # +/- 키 한 번에 1cm


def run_camera(device: str, model: YOLO, conf: float, iou: float,
               K: np.ndarray, dist: np.ndarray, T: np.ndarray,
               init_depth: float, tf_listener: "TFListener | None") -> None:
    arg = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(arg)
    if not cap.isOpened():
        logger.error("카메라 열기 실패: %s", device)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    _SNAPSHOT_DIR.mkdir(exist_ok=True)
    cv2.namedWindow(_WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WIN_NAME, 800, 600)

    mode = "link_6 + base_link (TF)" if tf_listener else "link_6 only"
    logger.info("C270 스트림 시작 | 모드: %s | +/-: depth  s: snap  q: quit", mode)
    depth_m = init_depth
    frame_count = 0
    fps_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("프레임 수신 실패")
            continue

        T_ee2base = tf_listener.get_T_ee2base() if tf_listener else None
        annotated, _ = process_frame(frame, model, conf, iou, depth_m, K, dist, T, T_ee2base)
        cv2.imshow(_WIN_NAME, annotated)

        frame_count += 1
        if frame_count % 30 == 0:
            fps = 30 / (time.time() - fps_time)
            fps_time = time.time()
            logger.info("FPS: %.1f", fps)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            ts = time.strftime("%Y%m%d_%H%M%S")
            p  = _SNAPSHOT_DIR / f"world_{ts}.jpg"
            cv2.imwrite(str(p), annotated)
            logger.info("스냅샷: %s", p.name)
        elif key == ord("+") or key == ord("="):
            depth_m = min(depth_m + _DEPTH_STEP, 0.50)
            logger.info("depth → %.2fm", depth_m)
        elif key == ord("-"):
            depth_m = max(depth_m - _DEPTH_STEP, 0.01)
            logger.info("depth → %.2fm", depth_m)

    cap.release()
    cv2.destroyAllWindows()
    if tf_listener:
        tf_listener.shutdown()


def run_images(image_paths: list[Path], model: YOLO, conf: float, iou: float,
               K: np.ndarray, dist: np.ndarray, T: np.ndarray,
               init_depth: float, tf_listener: "TFListener | None") -> None:
    _SNAPSHOT_DIR.mkdir(exist_ok=True)
    cv2.namedWindow(_WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WIN_NAME, 800, 600)

    depth_m = init_depth
    idx = 0
    while True:
        path  = image_paths[idx]
        frame = cv2.imread(str(path))
        if frame is None:
            logger.warning("로드 실패: %s", path)
            idx = (idx + 1) % len(image_paths)
            continue

        T_ee2base = tf_listener.get_T_ee2base() if tf_listener else None
        annotated, _ = process_frame(frame, model, conf, iou, depth_m, K, dist, T, T_ee2base)

        info = f"[{idx+1}/{len(image_paths)}] {path.name}"
        cv2.putText(annotated, info, (8, annotated.shape[0] - 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        cv2.imshow(_WIN_NAME, annotated)
        cv2.waitKey(1)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key in (ord("n"), ord(" ")):
            idx = (idx + 1) % len(image_paths)
        elif key == ord("p"):
            idx = (idx - 1) % len(image_paths)
        elif key == ord("s"):
            p = _SNAPSHOT_DIR / f"world_{path.stem}.jpg"
            cv2.imwrite(str(p), annotated)
            logger.info("저장: %s", p.name)
        elif key == ord("+") or key == ord("="):
            depth_m = min(depth_m + _DEPTH_STEP, 0.50)
            logger.info("depth → %.2fm", depth_m)
        elif key == ord("-"):
            depth_m = max(depth_m - _DEPTH_STEP, 0.01)
            logger.info("depth → %.2fm", depth_m)

    cv2.destroyAllWindows()


# ── 메인 ─────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="C270 centroid → link_6 / base_link 실제 좌표")
    parser.add_argument("--model",  type=Path, default=_DEFAULT_MODEL)
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--images", type=Path, default=None)
    parser.add_argument("--depth",  type=float, default=_DEFAULT_DEPTH_M,
                        help=f"초기 카메라~공구 거리 [m] (기본 {_DEFAULT_DEPTH_M})")
    parser.add_argument("--no-ros", action="store_true",
                        help="ROS2 TF 없이 link_6 좌표만 표시")
    args = parser.parse_args()

    if not args.model.exists():
        logger.error("모델 파일 없음: %s", args.model)
        sys.exit(1)

    # 캘리브레이션 로드
    K, dist = _load_intrinsics(_CAM_INFO_PATH)
    T_cam2ee = _load_hand_eye(_HAND_EYE_PATH)
    logger.info("intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                K[0,0], K[1,1], K[0,2], K[1,2])
    logger.info("T_cam2ee t=[%.1f, %.1f, %.1f] mm",
                T_cam2ee[0,3]*1000, T_cam2ee[1,3]*1000, T_cam2ee[2,3]*1000)

    with _VISION_CFG.open() as f:
        yolo_cfg = yaml.safe_load(f)["yolo"]
    conf = yolo_cfg["confidence_threshold"]
    iou  = yolo_cfg["iou_threshold"]

    logger.info("모델 로드: %s", args.model)
    model = YOLO(str(args.model))
    logger.info("task=%s  classes=%s", model.task, list(model.names.values()))

    # TF 리스너 (ROS2 드라이버 필요)
    tf_listener: TFListener | None = None
    if not args.no_ros:
        try:
            tf_listener = TFListener()
            logger.info("ROS2 TF 활성 — base_link 좌표도 표시")
        except Exception as e:
            logger.warning("TF 초기화 실패 (%s) — link_6 모드로 전환", e)

    if args.images is not None:
        exts = {".jpg", ".jpeg", ".png", ".bmp"}
        p = args.images
        paths = [p] if p.is_file() else sorted(x for x in p.iterdir() if x.suffix.lower() in exts)
        if not paths:
            logger.error("이미지 없음: %s", p)
            sys.exit(1)
        run_images(paths, model, conf, iou, K, dist, T_cam2ee, args.depth, tf_listener)
    else:
        run_camera(args.device, model, conf, iou, K, dist, T_cam2ee, args.depth, tf_listener)


if __name__ == "__main__":
    main()
