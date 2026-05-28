"""YOLOv8 공구 검출 모델 파인튜닝.

사전 조건:
  - datasets/tools/images/train/, val/ 에 이미지 수집 완료
  - datasets/tools/labels/train/, val/ 에 YOLO 형식 어노테이션 완료
  - pip install ultralytics

사용법:
    python scripts/train_yolo.py
    python scripts/train_yolo.py --epochs 200 --model yolov8s.pt
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_YAML = PROJECT_ROOT / "datasets" / "tools" / "data.yaml"
RUNS_DIR = PROJECT_ROOT / "runs" / "yolo"


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="YOLOv8 공구 검출 파인튜닝")
    p.add_argument("--model", default="yolov8n.pt",
                   help="베이스 모델 (yolov8n.pt / yolov8s.pt / yolov8m.pt)")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--device", default="cuda", help="cuda / cpu")
    p.add_argument("--name", default="tools_v1", help="runs/yolo/<name> 저장명")
    return p.parse_args()


def _check_dataset() -> None:
    for split in ("train", "val"):
        img_dir = PROJECT_ROOT / "datasets" / "tools" / "images" / split
        lbl_dir = PROJECT_ROOT / "datasets" / "tools" / "labels" / split
        imgs = list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.png"))
        lbls = list(lbl_dir.glob("*.txt"))
        if not imgs:
            raise FileNotFoundError(
                f"[{split}] 이미지 없음: {img_dir}\n"
                "scripts/dataset/collect_images.py 로 수집 먼저 진행"
            )
        if not lbls:
            raise FileNotFoundError(
                f"[{split}] 라벨 없음: {lbl_dir}\n"
                "YOLO 형식(.txt) 어노테이션 완료 후 재실행"
            )
        logger.info("[%s] images=%d  labels=%d", split, len(imgs), len(lbls))


def main() -> None:
    args = _parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics 미설치 — pip install ultralytics")

    _check_dataset()

    model = YOLO(args.model)
    logger.info("베이스 모델: %s", args.model)
    logger.info("data: %s", DATA_YAML)
    logger.info("epochs=%d  imgsz=%d  batch=%d  device=%s", args.epochs, args.imgsz, args.batch, args.device)

    results = model.train(
        data=str(DATA_YAML),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(RUNS_DIR),
        name=args.name,
        # 탑뷰 공구함 특화 augmentation — 과도한 원근 변환 억제
        degrees=180.0,    # 탑뷰이므로 전 방향 회전 허용
        translate=0.1,
        scale=0.3,
        fliplr=0.5,
        flipud=0.5,
        perspective=0.0,  # 탑뷰 고정 카메라이므로 원근 왜곡 없음
        mosaic=1.0,
        mixup=0.1,
        patience=30,      # early stopping
        save=True,
        plots=True,
        verbose=True,
    )

    best_pt = RUNS_DIR / args.name / "weights" / "best.pt"
    logger.info("학습 완료 — 최적 가중치: %s", best_pt)
    logger.info("config/vision.yaml model_path에 아래 경로 기입 후 yolo_node 재기동:")
    logger.info("  model_path: %s", best_pt)

    # 수락 기준 검증 (mAP50 ≥ 0.85)
    metrics = results.results_dict
    map50 = metrics.get("metrics/mAP50(B)", 0.0)
    logger.info("val mAP50=%.3f  (수락 기준 ≥ 0.85)", map50)
    if map50 < 0.85:
        logger.warning("수락 기준 미달 — 데이터 추가 수집 또는 epoch 증가 검토")
    else:
        logger.info("수락 기준 통과")


if __name__ == "__main__":
    main()
