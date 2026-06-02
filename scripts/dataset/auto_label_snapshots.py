"""infer_snapshots 이미지를 V2 best.pt로 자동 라벨링한다.

생성된 라벨은 Roboflow에서 수동 검수 후 사용할 것.
multi_tool / ratchet_wrench + screwdriver 혼합 장면 위주로 확인 필요.

사용법:
    python scripts/dataset/auto_label_snapshots.py
    python scripts/dataset/auto_label_snapshots.py --conf 0.25
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
MODEL_PATH   = PROJECT_ROOT / "ros2_ws/src/vision/model_library/top_view_model/v2/weights/best.pt"
IMAGE_DIR    = PROJECT_ROOT / "scripts/infer_snapshots"
OUTPUT_DIR   = PROJECT_ROOT / "scripts/infer_snapshots/auto_labels"

# V2 클래스 순서 (알파벳순) — 절대 바꾸지 말 것
CLASS_NAMES = [
    "multi_tool",
    "ratchet_wrench",
    "screwdriver",
    "socket_19mm",
    "spanner_16mm",
    "utility_knife",
]


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="infer_snapshots 자동 라벨링")
    p.add_argument("--conf", type=float, default=0.25,
                   help="confidence 임계값 (기본 0.25 — 낮을수록 더 많이 잡음)")
    p.add_argument("--iou",  type=float, default=0.45,
                   help="NMS IoU 임계값 (기본 0.45)")
    return p.parse_args()


def _save_yolo_label(pred, img_path: Path, output_dir: Path) -> int:
    label_path = output_dir / (img_path.stem + ".txt")
    boxes = pred.boxes

    if boxes is None or len(boxes) == 0:
        label_path.write_text("")
        return 0

    lines = []
    for box in boxes:
        cls = int(box.cls.item())
        cx, cy, w, h = box.xywhn[0].tolist()
        lines.append(f"{cls} {cx:.6f} {cy:.6f} {w:.6f} {h:.6f}")

    label_path.write_text("\n".join(lines))
    return len(lines)


def main() -> None:
    args = _parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics 미설치 — pip install ultralytics")

    if not MODEL_PATH.exists():
        raise SystemExit(f"모델 없음: {MODEL_PATH}")

    images = sorted(IMAGE_DIR.glob("*.jpg"))
    if not images:
        raise SystemExit(f"이미지 없음: {IMAGE_DIR}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    logger.info("모델: %s", MODEL_PATH.relative_to(PROJECT_ROOT))
    logger.info("대상 이미지: %d장", len(images))
    logger.info("conf=%.2f  iou=%.2f", args.conf, args.iou)
    logger.info("라벨 저장 위치: %s", OUTPUT_DIR.relative_to(PROJECT_ROOT))

    model = YOLO(str(MODEL_PATH))

    total_objects = 0
    empty_count   = 0

    for img_path in images:
        results = model.predict(
            source=str(img_path),
            conf=args.conf,
            iou=args.iou,
            verbose=False,
        )
        count = _save_yolo_label(results[0], img_path, OUTPUT_DIR)
        total_objects += count
        if count == 0:
            empty_count += 1
            logger.warning("객체 미검출: %s", img_path.name)

    logger.info("완료 — 라벨 생성: %d장 / 미검출: %d장 / 총 객체: %d개",
                len(images), empty_count, total_objects)

    if empty_count > 0:
        logger.warning("미검출 이미지는 Roboflow에서 수동 라벨링 필요")

    class_counts = [0] * len(CLASS_NAMES)
    for label_file in OUTPUT_DIR.glob("*.txt"):
        for line in label_file.read_text().splitlines():
            if line.strip():
                cls = int(line.split()[0])
                if cls < len(class_counts):
                    class_counts[cls] += 1

    logger.info("--- 클래스별 검출 수 ---")
    for name, cnt in zip(CLASS_NAMES, class_counts):
        flag = " ← 검수 필요" if name in ("multi_tool", "ratchet_wrench", "screwdriver") else ""
        logger.info("  %-20s %d개%s", name, cnt, flag)


if __name__ == "__main__":
    main()
