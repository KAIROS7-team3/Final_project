"""C270 그리퍼 카메라 사진 촬영 (YOLO 없음)."""
import logging
import sys
import time
import cv2
from pathlib import Path

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

DEVICE        = "/dev/video2"
_SNAPSHOT_DIR = Path.home() / "bottomspanner+"

cap = cv2.VideoCapture(DEVICE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    logger.error("카메라 열기 실패: %s", DEVICE)
    sys.exit(1)

_SNAPSHOT_DIR.mkdir(exist_ok=True)
cv2.namedWindow("C270 Gripper Cam", cv2.WINDOW_NORMAL)
cv2.resizeWindow("C270 Gripper Cam", 800, 600)
logger.info("스트림 시작 | s: 촬영  q: 종료 | 저장 경로: %s", _SNAPSHOT_DIR)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        cv2.imshow("C270 Gripper Cam", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            p = _SNAPSHOT_DIR / f"gripper_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(str(p), frame)
            logger.info("저장: %s", p)
finally:
    cap.release()
    cv2.destroyAllWindows()
