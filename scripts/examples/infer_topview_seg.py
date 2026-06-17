import cv2
import yaml
import numpy as np
import pyrealsense2 as rs
from pathlib import Path
from ultralytics import YOLO

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_cfg = yaml.safe_load((_PROJECT_ROOT / "config" / "vision.yaml").open())["yolo"]
CONF = _cfg["confidence_threshold"]
IOU  = _cfg["iou_threshold"]

model = YOLO(str(_PROJECT_ROOT / "ros2_ws/src/vision/model_library/top_view_model/v3-seg/weights/best.pt"))

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipe.start(cfg)

cv2.namedWindow('v3-seg', cv2.WINDOW_NORMAL)
cv2.resizeWindow('v3-seg', 800, 600)

try:
    while True:
        frame = np.asanyarray(pipe.wait_for_frames().get_color_frame().get_data())
        annotated = model(frame, conf=CONF, iou=IOU, verbose=False)[0].plot()
        cv2.imshow('v3-seg', annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipe.stop()
    cv2.destroyAllWindows()
