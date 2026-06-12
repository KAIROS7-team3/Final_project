"""COCO bbox 데이터셋을 SAM으로 마스크 변환 — Google Colab 또는 로컬 실행.

사용법 (Colab):
    !pip install ultralytics segment-anything -q
    %run scripts/train/convert_bbox_to_seg_sam.py \\
        --input /content/Final_project.coco \\
        --output /content/Final_project_coco_segmentation \\
        --sam-checkpoint /content/sam_vit_h_4b8939.pth

사용법 (로컬):
    python3 scripts/train/convert_bbox_to_seg_sam.py \\
        --input datasets/tools/top_view/coco_bbox \\
        --output datasets/tools/top_view/coco_seg \\
        --sam-checkpoint /path/to/sam_vit_h_4b8939.pth

SAM 체크포인트 다운로드:
    wget https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
"""
from __future__ import annotations

import argparse
import json
import logging
import shutil
from pathlib import Path

import cv2
import numpy as np

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

SPLITS = ("train", "valid", "test")


def build_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="COCO bbox → COCO Segmentation via SAM")
    p.add_argument("--input",  required=True, help="원본 COCO bbox 루트 (train/valid/test 하위)")
    p.add_argument("--output", required=True, help="변환 결과 저장 경로")
    p.add_argument("--sam-checkpoint", required=True, help="SAM ViT-H 체크포인트 경로 (.pth)")
    p.add_argument("--model-type", default="vit_h", choices=["vit_h", "vit_l", "vit_b"])
    p.add_argument("--device", default="cuda", choices=["cuda", "cpu"])
    return p.parse_args()


def load_sam(checkpoint: str, model_type: str, device: str):
    """SAM predictor 초기화."""
    try:
        from segment_anything import SamPredictor, sam_model_registry
    except ImportError:
        raise ImportError("segment-anything 패키지 필요: pip install segment-anything")

    sam = sam_model_registry[model_type](checkpoint=checkpoint)
    sam.to(device=device)
    return SamPredictor(sam)


def mask_to_polygon(mask: np.ndarray) -> list[list[float]]:
    """바이너리 마스크 → COCO polygon segmentation 형식."""
    contours, _ = cv2.findContours(
        mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    polygons: list[list[float]] = []
    for c in contours:
        if c.size >= 6:  # 최소 3점 삼각형 이상
            polygons.append(c.flatten().tolist())
    return polygons


def convert_split(
    split: str,
    input_dir: Path,
    output_dir: Path,
    predictor,
) -> None:
    src_img_dir = input_dir / split / "images"
    src_ann_path = input_dir / split / "_annotations.coco.json"

    if not src_ann_path.exists():
        logger.warning("[%s] _annotations.coco.json 없음 — 건너뜀", split)
        return

    dst_img_dir = output_dir / split / "images"
    dst_img_dir.mkdir(parents=True, exist_ok=True)

    with src_ann_path.open() as f:
        coco: dict = json.load(f)

    id_to_file = {img["id"]: img["file_name"] for img in coco["images"]}
    new_annotations: list[dict] = []
    skipped = 0

    for ann in coco["annotations"]:
        img_id = ann["image_id"]
        img_file = id_to_file.get(img_id)
        if img_file is None:
            skipped += 1
            continue

        img_path = src_img_dir / img_file
        if not img_path.exists():
            logger.debug("이미지 없음: %s", img_path)
            skipped += 1
            continue

        image_bgr = cv2.imread(str(img_path))
        if image_bgr is None:
            skipped += 1
            continue

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        predictor.set_image(image_rgb)

        # COCO bbox [x, y, w, h] → SAM box [x_min, y_min, x_max, y_max]
        x, y, w, h = ann["bbox"]
        box = np.array([x, y, x + w, y + h])

        masks, scores, _ = predictor.predict(
            point_coords=None,
            point_labels=None,
            box=box[None, :],
            multimask_output=False,
        )

        polygons = mask_to_polygon(masks[0])
        if not polygons:
            skipped += 1
            continue

        new_ann = dict(ann)
        new_ann["segmentation"] = polygons
        new_annotations.append(new_ann)

    # 이미지 복사
    for img_meta in coco["images"]:
        src = src_img_dir / img_meta["file_name"]
        dst = dst_img_dir / img_meta["file_name"]
        if src.exists() and not dst.exists():
            shutil.copy2(src, dst)

    # 변환된 annotation 저장
    out_coco = dict(coco)
    out_coco["annotations"] = new_annotations
    dst_ann_path = output_dir / split / "_annotations.coco.json"
    with dst_ann_path.open("w") as f:
        json.dump(out_coco, f)

    logger.info(
        "[%s] 완료 — 변환: %d / 전체: %d / 스킵: %d",
        split, len(new_annotations), len(coco["annotations"]), skipped,
    )


def main() -> None:
    args = build_args()
    input_dir  = Path(args.input)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    logger.info("SAM 모델 로드 중 (%s / %s)...", args.model_type, args.device)
    predictor = load_sam(args.sam_checkpoint, args.model_type, args.device)
    logger.info("SAM 로드 완료")

    for split in SPLITS:
        convert_split(split, input_dir, output_dir, predictor)

    logger.info("변환 완료 → %s", output_dir)
    logger.info("Roboflow 업로드: 폴더를 직접 업로드 (zip 직접 업로드 미지원)")


if __name__ == "__main__":
    main()
