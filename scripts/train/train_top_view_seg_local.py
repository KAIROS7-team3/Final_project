"""top_view_seg 로컬 학습 스크립트 — RTX 2050 4GB.

사용법:
    cd /home/iys/Final_project
    python3 scripts/train/train_top_view_seg_local.py

완료 후:
    runs/yolo/top_view_seg/weights/best.pt 생성됨
    → Drive 업로드 후 v3-seg/model_info.yaml 의 drive_url 기입
"""
from __future__ import annotations

import logging
from pathlib import Path

from ultralytics import YOLO

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_DATA_YAML    = _PROJECT_ROOT / "datasets/tools/top_view_seg/data.yaml"
_RUNS_DIR     = _PROJECT_ROOT / "runs/yolo"
_WEIGHTS_OUT  = _RUNS_DIR / "top_view_seg/weights/best.pt"


def main() -> None:
    if not _DATA_YAML.exists():
        logger.error("data.yaml 없음: %s", _DATA_YAML)
        return

    logger.info("학습 시작 — 데이터: %s", _DATA_YAML)
    model = YOLO("yolo11n-seg.pt")

    model.train(
        data=str(_DATA_YAML),
        epochs=100,
        patience=30,
        batch=4,            # RAM OOM 방지 (8 → 4)
        workers=2,          # dataloader 워커 수 제한 (기본 8 → 2)
        imgsz=640,
        optimizer="auto",
        lr0=0.01,
        lrf=0.01,
        device=0,
        project=str(_RUNS_DIR),
        name="top_view_seg",
        exist_ok=True,      # 이전 실행 폴더 재사용
        plots=True,
        verbose=True,
    )

    if _WEIGHTS_OUT.exists():
        logger.info("학습 완료 → %s", _WEIGHTS_OUT)
        logger.info("다음 단계: Drive 업로드 후 v3-seg/model_info.yaml 의 drive_url 기입")
    else:
        logger.warning("best.pt 미생성 — 학습 로그 확인 필요")


if __name__ == "__main__":
    main()
