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
_DEFAULT_MODEL = Path.home() / "Downloads" / "cocoham" / "best.pt"

# 클래스별 시각화 색상 (BGR)
_COLORS = [
    (255,  80,  80),   # multi_tool     — 파랑
    ( 80, 200,  80),   # ratchet_wrench — 초록
    ( 80,  80, 255),   # screwdriver    — 빨강
    (255, 200,  50),   # socket_19mm    — 하늘
    (200,  80, 255),   # spanner_16mm   — 보라
    ( 50, 220, 220),   # utility_knife  — 노랑
]


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


def compute_centroid(mask_2d: np.ndarray) -> tuple[float, float] | None:
    """이진 마스크에서 픽셀 무게중심 계산.

    Args:
        mask_2d: (H, W) bool/uint8 배열, 공구 픽셀=True(1)

    Returns:
        (cx, cy) 픽셀 좌표 또는 픽셀이 없으면 None
    """
    pts = np.argwhere(mask_2d > 0)
    if len(pts) == 0:
        return None

    # 1.1: 평균 계산 (argwhere는 [row(y), col(x)] 순서)
    cy = float(pts[:, 0].mean())
    cx = float(pts[:, 1].mean())
    return cx, cy


def compute_pca(mask_2d: np.ndarray) -> tuple[float, float, float, float] | None:
    """이진 마스크에서 무게중심 + 장축 각도(θ) 추출.

    [파이프라인]
      argwhere → 중심화 → 공분산 행렬(2×2) → 고유값 분해 → arctan2

    Args:
        mask_2d: (H, W) bool/uint8 배열

    Returns:
        (cx, cy, theta_deg, reliability)
        - cx, cy    : 픽셀 무게중심
        - theta_deg : 장축 각도 [-90°, +90°]
        - reliability: λ1/λ2 비율 (1에 가까울수록 각도 불안정)
        또는 픽셀이 부족하면 None
    """
    pts = np.argwhere(mask_2d > 0)
    if len(pts) < 5:
        return None

    # 1.1: 평균 → 무게중심
    cy = float(pts[:, 0].mean())
    cx = float(pts[:, 1].mean())

    # 1.1: 중심화 (원점으로 이동)
    centered = pts.astype(np.float32) - np.array([cy, cx])
    # centered 열 순서: [dy, dx]  →  x=col(1), y=row(0)
    x = centered[:, 1]   # col 방향
    y = centered[:, 0]   # row 방향

    # 1.2: 분산·공분산 계산
    N = len(pts)
    var_x  = float(np.sum(x ** 2) / N)
    var_y  = float(np.sum(y ** 2) / N)
    cov_xy = float(np.sum(x * y) / N)

    # 1.3: 공분산 행렬 구성
    C = np.array([[var_x, cov_xy],
                  [cov_xy, var_y]], dtype=np.float64)

    # 2.1: 고유값 분해
    eigenvalues, eigenvectors = np.linalg.eig(C)

    # 2.2: 고유값 내림차순 정렬 → PC1(장축) 선택
    order = np.argsort(eigenvalues)[::-1]
    eigenvalues  = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]

    lam1, lam2 = float(eigenvalues[0]), float(eigenvalues[1])
    v1 = eigenvectors[:, 0]   # PC1 고유벡터 [vx, vy]

    # 2.3: arctan2로 각도 추출 → [-90°, +90°] 정규화
    theta_rad = np.arctan2(float(v1[1]), float(v1[0]))
    theta_deg = float(np.degrees(theta_rad))
    if theta_deg > 90.0:
        theta_deg -= 180.0
    elif theta_deg < -90.0:
        theta_deg += 180.0

    # λ1/λ2 신뢰도 (클수록 각도 안정, 1에 가까우면 불안정)
    reliability = lam1 / lam2 if lam2 > 1e-6 else 0.0

    return cx, cy, theta_deg, reliability


def draw_result(
    image: np.ndarray,
    cx: float,
    cy: float,
    label: str,
    color: tuple[int, int, int],
    theta_deg: float | None = None,
    reliability: float | None = None,
    axis_len: int = 60,
) -> None:
    """무게중심 + 장축 방향 시각화."""
    ix, iy = int(cx), int(cy)
    arm = 12

    # 십자선
    cv2.line(image, (ix - arm, iy), (ix + arm, iy), color, 2)
    cv2.line(image, (ix, iy - arm), (ix, iy + arm), color, 2)
    cv2.circle(image, (ix, iy), 4, color, -1)

    # 장축 방향 화살표 (θ가 있을 때만)
    if theta_deg is not None:
        theta_rad = np.radians(theta_deg)
        dx = int(axis_len * np.cos(theta_rad))
        dy = int(axis_len * np.sin(theta_rad))
        cv2.arrowedLine(image, (ix, iy), (ix + dx, iy + dy),
                        (255, 255, 255), 2, tipLength=0.2)
        cv2.arrowedLine(image, (ix, iy), (ix - dx, iy - dy),
                        color, 1, tipLength=0.15)

    # 레이블 텍스트
    if theta_deg is not None and reliability is not None:
        stable = "stable" if reliability > 2.0 else "unstable"
        text = f"{label} ({ix},{iy}) θ={theta_deg:.1f}° [{stable}]"
    else:
        text = f"{label} ({ix},{iy})"

    cv2.putText(image, text, (ix + 8, iy - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)


def process_frame(
    frame: np.ndarray,
    model: YOLO,
    conf: float,
    iou: float,
    names: dict[int, str],
) -> np.ndarray:
    """한 프레임에서 추론 → 마스크 → 무게중심 → 시각화."""
    results = model(frame, conf=conf, iou=iou, verbose=False)
    result  = results[0]

    # 원본 프레임을 베이스로 사용 (result.plot() 대신)
    annotated = frame.copy()

    # 마스크가 없으면 원본 + 안내 텍스트 반환
    if result.masks is None or len(result.boxes) == 0:
        cv2.putText(annotated, "No detection", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
        return annotated

    masks = result.masks.data.cpu().numpy()   # (N, H, W) float32, 0~1
    boxes = result.boxes
    H, W  = frame.shape[:2]

    for i, mask in enumerate(masks):
        cls_id = int(boxes.cls[i].item())
        score  = float(boxes.conf[i].item())
        label  = names.get(cls_id, str(cls_id))
        color  = _COLORS[cls_id % len(_COLORS)]

        # 마스크를 원본 해상도로 리사이즈
        if mask.shape != (H, W):
            mask = cv2.resize(mask, (W, H), interpolation=cv2.INTER_NEAREST)

        # 마스크 반투명 오버레이
        binary = (mask > 0.5).astype(np.uint8)
        color_layer = np.zeros_like(annotated)
        color_layer[binary == 1] = color
        annotated = cv2.addWeighted(annotated, 0.6, color_layer, 0.4, 0)

        # 외곽선
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(annotated, contours, -1, color, 2)

        # PCA: 무게중심 + 장축 각도
        pca = compute_pca(binary)
        if pca is None:
            continue

        cx, cy, theta_deg, reliability = pca
        draw_result(annotated, cx, cy, f"{label} {score:.2f}",
                    color, theta_deg, reliability)

        logger.info("%-15s  centroid=(%.1f, %.1f)  θ=%+.1f°  "
                    "reliability=%.1f  px=%d",
                    label, cx, cy, theta_deg, reliability,
                    int(binary.sum()))

    return annotated


def run_images(
    image_paths: list[Path],
    model: YOLO,
    conf: float,
    iou: float,
    names: dict[int, str],
) -> None:
    """이미지 폴더 모드: n/p로 탐색, s로 저장, q로 종료."""
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

        annotated = process_frame(frame, model, conf, iou, names)

        # 파일명 + 인덱스 오버레이
        info = f"[{idx+1}/{len(image_paths)}] {path.name}"
        cv2.putText(annotated, info, (8, annotated.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        # 화면 표시 + 첫 장은 파일로도 저장 (디스플레이 문제 디버그용)
        if idx == 0:
            cv2.imwrite(str(_SNAPSHOT_DIR / "_debug_first.jpg"), annotated)
            logger.info("첫 프레임 저장: scripts/infer_snapshots/_debug_first.jpg")

        cv2.imshow(win, annotated)
        cv2.waitKey(1)   # 렌더링 강제 flush
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
) -> None:
    """카메라 실시간 모드."""
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

        annotated = process_frame(frame, model, conf, iou, names)
        cv2.imshow("Centroid Extraction  [s: snap  q: quit]", annotated)

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

    logger.info("모델 로드: %s", args.model)
    model = YOLO(str(args.model))
    names = model.names
    logger.info("task=%s  classes=%s", model.task, list(names.values()))
    logger.info("conf=%.2f  iou=%.2f", conf, iou)

    if args.images is not None:
        image_paths = _collect_images(args.images)
        run_images(image_paths, model, conf, iou, names)
    else:
        run_camera(args.device, model, conf, iou, names)


if __name__ == "__main__":
    main()
