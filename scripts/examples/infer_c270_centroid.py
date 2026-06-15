"""C270 카메라 또는 이미지 폴더로 공구 무게중심(Centroid) 추출.

[파이프라인]
  YOLO seg 마스크 → np.argwhere로 픽셀 좌표 전체 추출
  → x,y 평균 → Centroid → 화면에 표시

사용법:
    # 카메라 실시간
    python3 scripts/examples/infer_c270_centroid.py

    # 이미지 폴더 (n/p: 다음/이전, s: 저장, q: 종료)
    python3 scripts/examples/infer_c270_centroid.py --images ~/gripper_cam_0
    python3 scripts/examples/infer_c270_centroid.py --images ~/gripper_cam_1

    # 단일 이미지
    python3 scripts/examples/infer_c270_centroid.py --images ~/gripper_cam_0/gripper_xxx.jpg

키 조작 (이미지 모드):
    n 또는 스페이스 — 다음 이미지
    p             — 이전 이미지
    s             — 스냅샷 저장
    q             — 종료
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_PATH  = _PROJECT_ROOT / "config" / "vision.yaml"
_SNAPSHOT_DIR = _PROJECT_ROOT / "scripts" / "infer_snapshots"
_DEFAULT_MODEL = _PROJECT_ROOT / "ros2_ws/src/vision/model_library/gripper_model/v1/weights/best.pt"

# 클래스별 시각화 색상 (BGR)
_COLORS = [
    (255,  80,  80),   # multi_tool     — 파랑
    ( 80, 200,  80),   # ratchet_wrench — 초록
    ( 80,  80, 255),   # screwdriver    — 빨강
    (255, 200,  50),   # socket_19mm    — 하늘
    (200,  80, 255),   # spanner_16mm   — 보라
    ( 50, 220, 220),   # utility_knife  — 노랑
]

def _load_offset_tools(config_path: Path) -> dict[str, dict]:
    with config_path.open() as f:
        cfg = yaml.safe_load(f)
    return {
        k: {"ratio": float(v["ratio"]), "toward_narrow": bool(v["toward_narrow"])}
        for k, v in cfg.get("grasp_offset", {}).items()
    }


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="C270 centroid extraction")
    p.add_argument("--model",  type=Path, default=_DEFAULT_MODEL)
    p.add_argument("--device", default="/dev/video8",
                   help="카메라 장치 (--images 미지정 시 사용)")
    p.add_argument("--images", type=Path, default=None,
                   help="이미지 파일 또는 폴더 경로 (지정 시 카메라 대신 사용)")
    return p.parse_args()


def _collect_images(path: Path) -> list[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp"}
    if path.is_file():
        return [path]
    files = sorted(p for p in path.iterdir() if p.suffix.lower() in exts)
    if not files:
        logger.error("이미지 없음: %s", path)
        sys.exit(1)
    logger.info("이미지 %d장 발견: %s", len(files), path)
    return files


def _open_camera(device: str) -> cv2.VideoCapture:
    arg = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(arg)
    if not cap.isOpened():
        logger.error("카메라 열기 실패: %s", device)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


def compute_pca(mask_2d: np.ndarray) -> tuple[float, float, float, float, np.ndarray] | None:
    """이진 마스크에서 무게중심 + 장축 정보 추출.

    Returns:
        (cx, cy, theta_deg, reliability, eigvec1)
        - eigvec1: 주축 단위벡터 [vx, vy] (이미지 x,y 좌표계)
    """
    pts = np.argwhere(mask_2d > 0)
    if len(pts) < 5:
        return None

    cy = float(pts[:, 0].mean())
    cx = float(pts[:, 1].mean())

    centered = pts.astype(np.float32) - np.array([cy, cx])
    x = centered[:, 1]
    y = centered[:, 0]

    N = len(pts)
    var_x  = float(np.sum(x ** 2) / N)
    var_y  = float(np.sum(y ** 2) / N)
    cov_xy = float(np.sum(x * y) / N)

    C = np.array([[var_x, cov_xy],
                  [cov_xy, var_y]], dtype=np.float64)

    eigenvalues, eigenvectors = np.linalg.eig(C)
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues  = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    lam1, lam2 = float(eigenvalues[0]), float(eigenvalues[1])
    v1 = eigenvectors[:, 0]   # [vx, vy]

    theta_rad = np.arctan2(float(v1[1]), float(v1[0]))
    theta_deg = float(np.degrees(theta_rad))
    if theta_deg > 90.0:
        theta_deg -= 180.0
    elif theta_deg < -90.0:
        theta_deg += 180.0

    reliability = lam1 / lam2 if lam2 > 1e-6 else 0.0

    return cx, cy, theta_deg, reliability, v1


def _grasp_point(
    cx: float, cy: float,
    v1: np.ndarray,
    mask_2d: np.ndarray,
    ratio: float,
    toward_narrow: bool,
) -> tuple[float, float]:
    """PCA 주축 방향으로 centroid를 offset해 그립 포인트 계산.

    공구 전체 길이(픽셀) * ratio 만큼 주축 방향으로 이동.
    toward_narrow=True: 마스크가 더 좁은 쪽(손잡이)으로 이동.
    """
    pts = np.argwhere(mask_2d > 0).astype(np.float32)
    # 주축 투영값 계산
    centered = pts - np.array([cy, cx])
    proj = centered[:, 1] * v1[0] + centered[:, 0] * v1[1]
    length = float(proj.max() - proj.min())
    offset_px = length * ratio

    # 주축 양 끝 중 어느 쪽이 더 좁은지 판별
    pos_pts = pts[proj > 0]
    neg_pts = pts[proj < 0]

    def _width(cluster: np.ndarray) -> float:
        if len(cluster) < 3:
            return 0.0
        perp = cluster[:, 0] * v1[0] - cluster[:, 1] * v1[1]
        return float(perp.max() - perp.min())

    pos_narrow = _width(pos_pts) < _width(neg_pts)

    # toward_narrow=True 이면 좁은 쪽으로, False면 넓은 쪽으로
    if toward_narrow:
        sign = 1.0 if pos_narrow else -1.0
    else:
        sign = -1.0 if pos_narrow else 1.0

    gx = cx + sign * offset_px * v1[0]
    gy = cy + sign * offset_px * v1[1]
    H, W = mask_2d.shape
    gx = float(np.clip(gx, 0, W - 1))
    gy = float(np.clip(gy, 0, H - 1))
    return gx, gy


def draw_result(
    image: np.ndarray,
    cx: float, cy: float,
    label: str,
    color: tuple[int, int, int],
    theta_deg: float | None = None,
    reliability: float | None = None,
    grasp_pt: tuple[float, float] | None = None,
    axis_len: int = 60,
) -> None:
    """무게중심 + 장축 방향 + 그립 포인트 시각화."""
    ix, iy = int(cx), int(cy)
    arm = 12

    # centroid 십자선
    cv2.line(image, (ix - arm, iy), (ix + arm, iy), color, 2)
    cv2.line(image, (ix, iy - arm), (ix, iy + arm), color, 2)
    cv2.circle(image, (ix, iy), 4, color, -1)

    # 장축 방향 화살표
    if theta_deg is not None:
        theta_rad = np.radians(theta_deg)
        dx = int(axis_len * np.cos(theta_rad))
        dy = int(axis_len * np.sin(theta_rad))
        cv2.arrowedLine(image, (ix, iy), (ix + dx, iy + dy),
                        (255, 255, 255), 2, tipLength=0.2)
        cv2.arrowedLine(image, (ix, iy), (ix - dx, iy - dy),
                        color, 1, tipLength=0.15)

    # 그립 포인트 (별도 표시)
    if grasp_pt is not None:
        gx, gy = int(grasp_pt[0]), int(grasp_pt[1])
        cv2.circle(image, (gx, gy), 8, (0, 255, 255), -1)
        cv2.circle(image, (gx, gy), 10, (0, 0, 0), 2)
        cv2.line(image, (ix, iy), (gx, gy), (0, 255, 255), 1)
        cv2.putText(image, f"GRASP({gx},{gy})", (gx + 8, gy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)

    # 레이블
    if theta_deg is not None and reliability is not None:
        stable = "stable" if reliability > 2.0 else "unstable"
        text = f"{label} ({ix},{iy}) θ={theta_deg:.1f}° [{stable}]"
    else:
        text = f"{label} ({ix},{iy})"
    cv2.putText(image, text, (ix + 8, iy + 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def process_frame(
    frame: np.ndarray,
    model: YOLO,
    conf: float,
    iou: float,
    names: dict[int, str],
    offset_tools: dict[str, dict],
) -> np.ndarray:
    results = model(frame, conf=conf, iou=iou, verbose=False)
    result  = results[0]
    annotated = frame.copy()

    if result.masks is None or len(result.boxes) == 0:
        cv2.putText(annotated, "No detection", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
        return annotated

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
        color_layer = np.zeros_like(annotated)
        color_layer[binary == 1] = color
        annotated = cv2.addWeighted(annotated, 0.6, color_layer, 0.4, 0)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(annotated, contours, -1, color, 2)

        pca = compute_pca(binary)
        if pca is None:
            continue
        cx, cy, theta_deg, reliability, v1 = pca

        # offset 공구면 그립 포인트 계산
        grasp_pt: tuple[float, float] | None = None
        if label in offset_tools:
            cfg = offset_tools[label]
            grasp_pt = _grasp_point(cx, cy, v1, binary,
                                    cfg["ratio"], cfg["toward_narrow"])
            logger.info("%-15s  centroid=(%.1f,%.1f)  grasp=(%.1f,%.1f)  θ=%+.1f°",
                        label, cx, cy, grasp_pt[0], grasp_pt[1], theta_deg)
        else:
            logger.info("%-15s  centroid=(%.1f,%.1f)  θ=%+.1f°  rel=%.1f",
                        label, cx, cy, theta_deg, reliability)

        draw_result(annotated, cx, cy, f"{label} {score:.2f}",
                    color, theta_deg, reliability, grasp_pt)

    return annotated


def run_images(
    image_paths: list[Path],
    model: YOLO,
    conf: float,
    iou: float,
    names: dict[int, str],
    offset_tools: dict[str, dict],
) -> None:
    _SNAPSHOT_DIR.mkdir(exist_ok=True)
    idx = 0
    win = "Centroid Extraction  [n/space: 다음  p: 이전  s: 저장  q: 종료]"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 640, 480)

    while True:
        path  = image_paths[idx]
        frame = cv2.imread(str(path))
        if frame is None:
            logger.warning("이미지 로드 실패: %s", path)
            idx = (idx + 1) % len(image_paths)
            continue

        annotated = process_frame(frame, model, conf, iou, names, offset_tools)
        info = f"[{idx+1}/{len(image_paths)}] {path.name}"
        cv2.putText(annotated, info, (8, annotated.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        cv2.imshow(win, annotated)
        cv2.waitKey(1)
        logger.info("[%d/%d] %s", idx + 1, len(image_paths), path.name)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("q"):
            break
        elif key in (ord("n"), ord(" ")):
            idx = (idx + 1) % len(image_paths)
        elif key == ord("p"):
            idx = (idx - 1) % len(image_paths)
        elif key == ord("s"):
            snap_path = _SNAPSHOT_DIR / f"centroid_{path.stem}.jpg"
            cv2.imwrite(str(snap_path), annotated)
            logger.info("저장: %s", snap_path.name)

    cv2.destroyAllWindows()


def run_camera(
    device: str,
    model: YOLO,
    conf: float,
    iou: float,
    names: dict[int, str],
    offset_tools: dict[str, dict],
) -> None:
    _SNAPSHOT_DIR.mkdir(exist_ok=True)
    cap = _open_camera(device)
    logger.info("C270 스트림 시작 | 's': 스냅샷  'q': 종료")

    frame_count = 0
    fps_time    = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("프레임 수신 실패")
            continue

        annotated = process_frame(frame, model, conf, iou, names, offset_tools)
        cv2.imshow("Centroid + Grasp  [s: snap  q: quit]", annotated)

        frame_count += 1
        if frame_count % 30 == 0:
            fps = 30 / (time.time() - fps_time)
            fps_time = time.time()
            logger.info("FPS: %.1f", fps)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            ts        = time.strftime("%Y%m%d_%H%M%S")
            snap_path = _SNAPSHOT_DIR / f"centroid_{ts}.jpg"
            cv2.imwrite(str(snap_path), annotated)
            logger.info("스냅샷 저장: %s", snap_path.name)

    cap.release()
    cv2.destroyAllWindows()
    logger.info("종료 | %d프레임", frame_count)


def main() -> None:
    args = _parse_args()

    if not args.model.exists():
        logger.error("모델 파일 없음: %s", args.model)
        sys.exit(1)

    with _CONFIG_PATH.open() as f:
        yolo_cfg = yaml.safe_load(f)["yolo"]
    conf = yolo_cfg["confidence_threshold"]
    iou  = yolo_cfg["iou_threshold"]

    offset_tools = _load_offset_tools(_CONFIG_PATH)
    logger.info("grasp_offset 로드: %s", list(offset_tools.keys()))

    logger.info("모델 로드: %s", args.model)
    model = YOLO(str(args.model))
    names = model.names
    logger.info("task=%s  classes=%s", model.task, list(names.values()))

    if args.images is not None:
        image_paths = _collect_images(args.images)
        run_images(image_paths, model, conf, iou, names, offset_tools)
    else:
        run_camera(args.device, model, conf, iou, names, offset_tools)


if __name__ == "__main__":
    main()
