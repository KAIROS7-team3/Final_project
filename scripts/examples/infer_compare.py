"""v3-2 (bbox) vs v3-seg (segmentation) 실시간 비교."""
import logging
import sys
import cv2
import yaml
import numpy as np
import pyrealsense2 as rs
from pathlib import Path
from ultralytics import YOLO

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_PATH  = _PROJECT_ROOT / "config" / "vision.yaml"

with _CONFIG_PATH.open() as f:
    _yolo_cfg = yaml.safe_load(f)["yolo"]
CONF = _yolo_cfg["confidence_threshold"]
IOU  = _yolo_cfg["iou_threshold"]

MODEL_BBOX = _PROJECT_ROOT / "ros2_ws/src/vision/model_library/top_view_model/v3-2/weights/best.pt"
MODEL_SEG  = _PROJECT_ROOT / "ros2_ws/src/vision/model_library/top_view_model/v3-seg/weights/best.pt"

try:
    model_bbox = YOLO(str(MODEL_BBOX))
    model_seg  = YOLO(str(MODEL_SEG))
    logger.info("모델 로드 완료 | conf=%.2f", CONF)
except Exception as e:
    logger.error("모델 로드 실패: %s", e)
    sys.exit(1)

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)
logger.info("스트림 시작 | q: 종료")

cv2.namedWindow('v3-2(bbox) | v3-seg(seg)', cv2.WINDOW_NORMAL)
cv2.resizeWindow('v3-2(bbox) | v3-seg(seg)', 1280, 480)

try:
    while True:
        frame = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data())

        left  = model_bbox(frame, conf=CONF, iou=IOU, verbose=False)[0].plot()
        right = model_seg (frame, conf=CONF, iou=IOU, verbose=False)[0].plot()

        cv2.putText(left,  'v3-2  (bbox)', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.putText(right, 'v3-seg (seg)', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        combined = np.hstack([left, right])
        cv2.imshow('v3-2(bbox) | v3-seg(seg)', combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipe.stop()
    cv2.destroyAllWindows()
