"""탑뷰 D455f 실시간 추론 데모 스크립트 (ROS2 불필요).

사용법:
    python3 scripts/examples/infer_topview_demo.py
    python3 scripts/examples/infer_topview_demo.py 10   # 10초 실행 후 자동 종료

출력: scripts/infer_snapshots/ 에 's' 키로 annotated 이미지 저장
종료: 'q' 또는 Ctrl+C
"""
from __future__ import annotations

import logging
import select
import sys
import termios
import time
import tty
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from ultralytics import YOLO

logging.basicConfig(
    format="[%(levelname)s] %(message)s",
    level=logging.INFO,
)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONFIG_PATH  = _PROJECT_ROOT / "config" / "vision.yaml"
_SNAPSHOT_DIR = _PROJECT_ROOT / "scripts" / "infer_snapshots"


def _keypress() -> str | None:
    """non-blocking 단일 키 감지 — 눌린 키 반환, 없으면 None."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main() -> None:
    run_sec: float | None = float(sys.argv[1]) if len(sys.argv) > 1 else None

    with _CONFIG_PATH.open() as f:
        cfg = yaml.safe_load(f)["yolo"]

    model_path  = _PROJECT_ROOT / cfg["top_view_model_path"]
    class_names = cfg["class_names"]
    conf        = cfg["confidence_threshold"]
    iou         = cfg["iou_threshold"]

    if not model_path.exists():
        logger.error("모델 파일 없음: %s", model_path)
        logger.error("config/vision.yaml top_view_model_path 경로 확인 또는 Drive에서 weights 다운로드")
        sys.exit(1)

    _SNAPSHOT_DIR.mkdir(exist_ok=True)

    logger.info("모델 로드: %s", model_path)
    model = YOLO(str(model_path))
    logger.info("로드 완료 | conf=%.2f iou=%.2f classes=%s", conf, iou, class_names)

    pipeline = rs.pipeline()
    pipeline_cfg = rs.config()
    pipeline_cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    logger.info("D455f 스트림 시작...")
    pipeline.start(pipeline_cfg)
    logger.info("스트림 시작 완료 | 's': 스냅샷  'q'/Ctrl+C: 종료")

    frame_count   = 0
    fps_time      = time.time()
    last_annotated: np.ndarray | None = None
    t_start       = time.monotonic()
    fd            = sys.stdin.fileno()
    old_tty       = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            if run_sec is not None and time.monotonic() - t_start >= run_sec:
                break

            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            img       = np.asanyarray(color_frame.get_data())
            results   = model(img, conf=conf, iou=iou, verbose=False)
            annotated = results[0].plot()
            last_annotated = annotated

            frame_count += 1
            if frame_count % 30 == 0:
                fps = 30 / (time.time() - fps_time)
                fps_time = time.time()
                sys.stdout.write(f"\r[FPS] {fps:.1f}  ")
                sys.stdout.flush()

            boxes = results[0].boxes
            if boxes is not None and len(boxes):
                detections = [
                    f"{class_names[int(c)] if int(c) < len(class_names) else int(c)}"
                    f"({s:.2f})"
                    for c, s in zip(boxes.cls.tolist(), boxes.conf.tolist())
                ]
                sys.stdout.write(f"\r[DETECT] {', '.join(detections)}          \n")
                sys.stdout.flush()

            key = _keypress()
            if key == "s" and last_annotated is not None:
                ts        = time.strftime("%Y%m%d_%H%M%S")
                snap_path = _SNAPSHOT_DIR / f"snap_{ts}.jpg"
                cv2.imwrite(str(snap_path), last_annotated)
                sys.stdout.write(f"\r[SNAP] 저장: {snap_path.name}\n")
                sys.stdout.flush()
            elif key in ("q", "\x03"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_tty)
        pipeline.stop()
        logger.info("종료 | %d프레임 | 스냅샷 경로: %s", frame_count, _SNAPSHOT_DIR)


if __name__ == "__main__":
    main()
