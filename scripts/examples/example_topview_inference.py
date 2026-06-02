"""top_view v3-2 모델 추론 예제.

이미지 파일 또는 웹캠(D455f 불필요)으로 공구 탐지를 테스트한다.

사전 조건:
    pip install ultralytics opencv-python

    model_library/top_view_model/v3-2/weights/best.pt 배치 필요
    (Drive 링크: model_info.yaml 참조)

사용법:
    # 이미지 파일로 테스트
    python3 scripts/examples/example_topview_inference.py --source path/to/image.jpg

    # D455f 실시간 테스트 (권장)
    python3 scripts/examples/example_topview_inference.py --source realsense

    # 웹캠으로 실시간 테스트 (기본 device 0)
    python3 scripts/examples/example_topview_inference.py --source webcam

    # 결과 이미지 저장
    python3 scripts/examples/example_topview_inference.py --source path/to/image.jpg --save
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

import cv2

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_MODEL_PATH = (
    _PROJECT_ROOT
    / "ros2_ws/src/vision/model_library/top_view_model/v3-2/weights/best.pt"
)

_CLASS_NAMES = [
    "multi_tool",
    "ratchet_wrench",
    "screwdriver",
    "socket_19mm",
    "spanner_16mm",
    "utility_knife",
]

_CONF = 0.5
_IOU  = 0.45


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="top_view v3-2 추론 예제")
    p.add_argument("--source", default="realsense",
                   help="이미지 경로, 'realsense' (D455f), 또는 'webcam' (기본값: realsense)")
    p.add_argument("--device", type=int, default=0,
                   help="웹캠 device 인덱스 (기본값: 0)")
    p.add_argument("--conf", type=float, default=_CONF,
                   help=f"confidence threshold (기본값: {_CONF})")
    p.add_argument("--save", action="store_true",
                   help="결과 이미지 저장 (output/ 디렉토리)")
    return p.parse_args()


def main() -> None:
    try:
        from ultralytics import YOLO
    except ImportError:
        logger.error("ultralytics 미설치 — pip install ultralytics")
        sys.exit(1)

    if not _MODEL_PATH.exists():
        logger.error("모델 파일 없음: %s", _MODEL_PATH)
        logger.error("model_library/top_view_model/v3-2/weights/best.pt 배치 또는 Drive 다운로드 필요")
        sys.exit(1)

    args = _parse_args()
    logger.info("모델 로드: %s", _MODEL_PATH)
    model = YOLO(str(_MODEL_PATH))
    logger.info("로드 완료 | conf=%.2f iou=%.2f", args.conf, _IOU)

    save_dir = _PROJECT_ROOT / "scripts" / "examples" / "output"
    if args.save:
        save_dir.mkdir(exist_ok=True)

    # ── D455f 실시간 추론 ────────────────────────────────────
    if args.source == "realsense":
        try:
            import numpy as np
            import pyrealsense2 as rs
        except ImportError:
            logger.error("pyrealsense2 미설치 — pip install pyrealsense2")
            sys.exit(1)

        pipeline = rs.pipeline()
        rs_cfg = rs.config()
        rs_cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        pipeline.start(rs_cfg)
        logger.info("D455f 스트림 시작 | q: 종료")
        frame_idx = 0
        try:
            while True:
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                frame = np.asanyarray(color_frame.get_data())
                results   = model(frame, conf=args.conf, iou=_IOU, verbose=False)
                annotated = results[0].plot()
                cv2.imshow("top_view v3-2 | D455f", annotated)
                if args.save and frame_idx % 30 == 0:
                    save_dir.mkdir(exist_ok=True)
                    cv2.imwrite(str(save_dir / f"frame_{frame_idx:05d}.jpg"), annotated)
                frame_idx += 1
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
            logger.info("종료 | %d프레임", frame_idx)
        return

    # ── 이미지 파일 추론 ──────────────────────────────────────
    if args.source != "webcam":
        img_path = Path(args.source)
        if not img_path.exists():
            logger.error("파일 없음: %s", img_path)
            sys.exit(1)

        img     = cv2.imread(str(img_path))
        results = model(img, conf=args.conf, iou=_IOU)
        annotated = results[0].plot()

        boxes = results[0].boxes
        if boxes is not None and len(boxes):
            for cls, conf in zip(boxes.cls.tolist(), boxes.conf.tolist()):
                name = _CLASS_NAMES[int(cls)] if int(cls) < len(_CLASS_NAMES) else str(int(cls))
                logger.info("  %s: %.3f", name, conf)
        else:
            logger.info("탐지된 공구 없음")

        cv2.imshow("top_view v3-2 inference", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        if args.save:
            out_path = save_dir / f"result_{img_path.stem}.jpg"
            cv2.imwrite(str(out_path), annotated)
            logger.info("결과 저장: %s", out_path)
        return

    # ── 웹캠 실시간 추론 ──────────────────────────────────────
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        logger.error("웹캠 device=%d 열기 실패", args.device)
        sys.exit(1)

    logger.info("웹캠 스트림 시작 | q: 종료")
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        results   = model(frame, conf=args.conf, iou=_IOU, verbose=False)
        annotated = results[0].plot()
        cv2.imshow("top_view v3-2 inference", annotated)
        if args.save and frame_idx % 30 == 0:
            out_path = save_dir / f"frame_{frame_idx:05d}.jpg"
            cv2.imwrite(str(out_path), annotated)
        frame_idx += 1
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    logger.info("종료 | %d프레임", frame_idx)


if __name__ == "__main__":
    main()
