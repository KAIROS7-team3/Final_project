"""C270 그리퍼 카메라 실시간 객체 탐지."""
import logging
import sys
import time
import cv2
import yaml
import numpy as np
from pathlib import Path
from ultralytics import YOLO

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_PATH  = _PROJECT_ROOT / "config" / "vision.yaml"
_SNAPSHOT_DIR = _PROJECT_ROOT / "scripts" / "infer_snapshots"
MODEL_PATH    = _PROJECT_ROOT / "ros2_ws/src/vision/model_library/gripper_model/v1/weights/best.pt"
DEVICE        = "/dev/video8"

with _CONFIG_PATH.open() as f:
    _yolo_cfg = yaml.safe_load(f)["yolo"]
CONF = _yolo_cfg["confidence_threshold"]
IOU  = _yolo_cfg["iou_threshold"]

try:
    model = YOLO(str(MODEL_PATH))
    logger.info("모델 로드 완료 | task=%s | classes=%s", model.task, list(model.names.values()))
except Exception as e:
    logger.error("모델 로드 실패: %s", e)
    sys.exit(1)

cap = cv2.VideoCapture(DEVICE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    logger.error("카메라 열기 실패: %s", DEVICE)
    sys.exit(1)

cv2.namedWindow("C270 Gripper Cam", cv2.WINDOW_NORMAL)
cv2.resizeWindow("C270 Gripper Cam", 800, 600)
logger.info("스트림 시작 | conf=%.2f  s: 스냅샷  q: 종료", CONF)
_SNAPSHOT_DIR.mkdir(exist_ok=True)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        annotated = model(frame, conf=CONF, iou=IOU, verbose=False)[0].plot()
        cv2.imshow("C270 Gripper Cam", annotated)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            p = _SNAPSHOT_DIR / f"gripper_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(str(p), annotated)
            logger.info("저장: %s", p.name)
finally:
    cap.release()
    cv2.destroyAllWindows()
