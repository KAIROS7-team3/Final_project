"""C270 그리퍼 카메라로 통합 모델 테스트.

모델: Downloads/cocoham/best.pt
classes: multi_tool, ratchet_wrench, screwdriver, socket_19mm, spanner_16mm, utility_knife

사용법:
    python3 scripts/examples/infer_c270_dual_layer_test.py

    # 모델/카메라 직접 지정
    python3 scripts/examples/infer_c270_dual_layer_test.py \
        --model /path/to/best.pt --device /dev/video8

키 조작:
    s — 스냅샷 저장 (scripts/infer_snapshots/)
    q — 종료
"""
from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import cv2
import yaml
from ultralytics import YOLO

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_PATH  = _PROJECT_ROOT / "config" / "vision.yaml"
_SNAPSHOT_DIR = _PROJECT_ROOT / "scripts" / "infer_snapshots"
_DEFAULT_MODEL = Path.home() / "Downloads" / "cocoham" / "best.pt"


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="C270 gripper cam YOLO test")
    p.add_argument("--model", type=Path, default=_DEFAULT_MODEL)
    p.add_argument("--device", default="/dev/video8",
                   help="카메라 장치 경로 또는 인덱스 (예: /dev/video8, 0)")
    return p.parse_args()


def _open_camera(device: str) -> cv2.VideoCapture:
    arg = int(device) if device.isdigit() else device
    cap = cv2.VideoCapture(arg)
    if not cap.isOpened():
        logger.error("카메라 열기 실패: %s", device)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    return cap


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
    logger.info("classes(%d): %s", len(names), list(names.values()))
    logger.info("conf=%.2f  iou=%.2f", conf, iou)

    _SNAPSHOT_DIR.mkdir(exist_ok=True)
    cap = _open_camera(args.device)
    logger.info("C270 스트림 시작 | 's': 스냅샷  'q': 종료")

    frame_count = 0
    fps_time    = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("프레임 수신 실패")
            continue

        results   = model(frame, conf=conf, iou=iou, verbose=False)
        annotated = results[0].plot()

        cv2.putText(annotated, "cocoham gripper model", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imshow("C270 YOLO Test  [s: snap  q: quit]", annotated)

        frame_count += 1
        if frame_count % 30 == 0:
            fps = 30 / (time.time() - fps_time)
            fps_time = time.time()
            logger.info("FPS: %.1f", fps)

        boxes = results[0].boxes
        if boxes is not None and len(boxes):
            dets = [
                f"{names[int(c)]}({s:.2f})"
                for c, s in zip(boxes.cls.tolist(), boxes.conf.tolist())
            ]
            logger.info("DETECT: %s", ", ".join(dets))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            ts        = time.strftime("%Y%m%d_%H%M%S")
            snap_path = _SNAPSHOT_DIR / f"c270_snap_{ts}.jpg"
            cv2.imwrite(str(snap_path), annotated)
            logger.info("스냅샷 저장: %s", snap_path.name)

    cap.release()
    cv2.destroyAllWindows()
    logger.info("종료 | %d프레임", frame_count)


if __name__ == "__main__":
    main()
