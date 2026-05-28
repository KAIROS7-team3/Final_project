"""YOLOv11s 공구 검출 모델 파인튜닝 — 탑뷰(D455f) / 그리퍼(C270) 시점별 분리 학습.

사전 조건:
  - datasets/tools/{top_view|gripper}/images/train|val/ 에 이미지 수집 완료
  - datasets/tools/{top_view|gripper}/labels/train|val/ 에 YOLO 형식 어노테이션 완료
  - pip install ultralytics

사용법:
    python scripts/train_yolo.py --view top_view
    python scripts/train_yolo.py --view gripper
    python scripts/train_yolo.py --view top_view --epochs 200 --batch 16
"""
from __future__ import annotations

import argparse
import logging
from pathlib import Path

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RUNS_DIR = PROJECT_ROOT / "runs" / "yolo"

# 시점별 data.yaml 경로
_DATA_YAML = {
    "top_view": PROJECT_ROOT / "datasets" / "tools" / "top_view" / "data.yaml",
    "gripper":  PROJECT_ROOT / "datasets" / "tools" / "gripper"  / "data.yaml",
}

# 시점별 augmentation — 탑뷰는 수직 고정(원근 왜곡 없음), 그리퍼는 각도 변화 있음
_AUG = {
    "top_view": dict(
        degrees=180.0,   # 탑뷰이므로 전 방향 회전 허용
        translate=0.1,
        scale=0.3,
        fliplr=0.5,
        flipud=0.5,
        perspective=0.0, # 고정 마운트, 원근 왜곡 없음
        mosaic=1.0,
        mixup=0.1,
    ),
    "gripper": dict(
        degrees=30.0,    # 그리퍼 접근 방향이 대략 일정해 큰 회전은 불필요
        translate=0.1,
        scale=0.4,       # 그립 거리에 따라 크기 변화 큼
        fliplr=0.5,
        flipud=0.2,
        perspective=0.0005,  # 그리퍼 각도 변화 반영
        mosaic=1.0,
        mixup=0.1,
    ),
}

# config/vision.yaml에 기입할 경로 키 이름
_MODEL_KEY = {
    "top_view": "top_view_model_path",
    "gripper":  "gripper_model_path",
}


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="YOLOv11s 공구 검출 파인튜닝")
    p.add_argument("--view", required=True, choices=["top_view", "gripper"],
                   help="학습할 카메라 시점 (top_view: D455f / gripper: C270)")
    p.add_argument("--model", default="yolo11s.pt",
                   help="베이스 모델 (yolo11n.pt / yolo11s.pt / yolo11m.pt)")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--device", default="cuda", help="cuda / cpu")
    return p.parse_args()


def _check_dataset(view: str) -> None:
    base = PROJECT_ROOT / "datasets" / "tools" / view
    for split in ("train", "val"):
        img_dir = base / "images" / split
        lbl_dir = base / "labels" / split
        imgs = list(img_dir.glob("**/*.jpg")) + list(img_dir.glob("**/*.png"))
        lbls = list(lbl_dir.glob("**/*.txt"))
        if not imgs:
            raise FileNotFoundError(
                f"[{view}/{split}] 이미지 없음: {img_dir}\n"
                f"  → python scripts/dataset/collect_images.py --camera {view} --split {split} ..."
            )
        if not lbls:
            raise FileNotFoundError(
                f"[{view}/{split}] 라벨 없음: {lbl_dir}\n"
                "  → YOLO 형식(.txt) 어노테이션 완료 후 재실행"
            )
        logger.info("[%s/%s] images=%d  labels=%d", view, split, len(imgs), len(lbls))


def main() -> None:
    args = _parse_args()
    view = args.view
    run_name = f"{view}_v1"

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics 미설치 — pip install ultralytics")

    _check_dataset(view)

    data_yaml = _DATA_YAML[view]
    aug = _AUG[view]

    model = YOLO(args.model)
    logger.info("view=%s  베이스 모델: %s", view, args.model)
    logger.info("data: %s", data_yaml)
    logger.info("epochs=%d  imgsz=%d  batch=%d  device=%s", args.epochs, args.imgsz, args.batch, args.device)

    results = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(RUNS_DIR),
        name=run_name,
        patience=30,
        save=True,
        plots=True,
        verbose=True,
        **aug,
    )

    best_pt = RUNS_DIR / run_name / "weights" / "best.pt"
    logger.info("학습 완료 — 최적 가중치: %s", best_pt)
    logger.info("config/vision.yaml에 아래 경로 기입 후 yolo_node 재기동:")
    logger.info("  %s: %s", _MODEL_KEY[view], best_pt)

    metrics = results.results_dict
    map50 = metrics.get("metrics/mAP50(B)", 0.0)
    logger.info("val mAP50=%.3f  (수락 기준 ≥ 0.85)", map50)
    if map50 < 0.85:
        logger.warning("수락 기준 미달 — 데이터 추가 수집 또는 epoch 증가 검토")
    else:
        logger.info("수락 기준 통과")


if __name__ == "__main__":
    main()
