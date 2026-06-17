"""C270 카메라로 탑뷰 모델 임시 테스트 (가능성 확인용).

탑뷰 학습 모델을 그리퍼 시점에서 돌려보는 테스트.
시점이 달라 정확도는 낮을 수 있음 — 동작 여부 확인 목적.

사용법:
    python3 scripts/examples/infer_c270_test.py

키 조작:
    s — 스냅샷 저장 (scripts/infer_snapshots/)
    q — 종료
"""
from __future__ import annotations

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


def main() -> None:
    with _CONFIG_PATH.open() as f:
        cfg = yaml.safe_load(f)["yolo"]

    model_path  = _PROJECT_ROOT / cfg["top_view_model_path"]
    class_names = cfg["class_names"]
    conf        = cfg["confidence_threshold"]
    iou         = cfg["iou_threshold"]

    if not model_path.exists():
        logger.error("모델 파일 없음: %s", model_path)
        sys.exit(1)

    _SNAPSHOT_DIR.mkdir(exist_ok=True)

    logger.info("모델 로드: %s", model_path)
    model = YOLO(str(model_path))
    logger.info("로드 완료 | conf=%.2f iou=%.2f", conf, iou)

    cap = cv2.VideoCapture('/dev/video9')
    if not cap.isOpened():
        logger.error("C270 카메라 열기 실패 (/dev/video9)")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
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

        cv2.imshow("C270 YOLO Test (topview model)", annotated)

        frame_count += 1
        if frame_count % 30 == 0:
            fps = 30 / (time.time() - fps_time)
            fps_time = time.time()
            logger.info("FPS: %.1f", fps)

        boxes = results[0].boxes
        if boxes is not None and len(boxes):
            detections = [
                f"{class_names[int(c)] if int(c) < len(class_names) else int(c)}({s:.2f})"
                for c, s in zip(boxes.cls.tolist(), boxes.conf.tolist())
            ]
            logger.info("DETECT: %s", ", ".join(detections))

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
