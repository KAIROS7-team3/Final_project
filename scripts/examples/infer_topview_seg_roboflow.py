"""탑뷰 세그멘테이션 추론 — Roboflow Hosted API 방식.

로컬 weights 없이 Roboflow Deploy 모델을 직접 호출한다.
팀원이 best.pt 없이도 곧바로 추론 결과를 확인할 수 있다.

사전 준비:
    pip install inference supervision roboflow
    export ROBOFLOW_API_KEY="your_key"   # 또는 .env 파일에 기입

사용법:
    python3 scripts/examples/infer_topview_seg_roboflow.py --image sample.jpg
    python3 scripts/examples/infer_topview_seg_roboflow.py --camera   # 실시간 D455f

모델 ID:
    model_library/top_view_model/v3-seg/model_info.yaml 의 roboflow_deploy.model_id 참조
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

import cv2

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_SNAPSHOT_DIR = _PROJECT_ROOT / "scripts" / "infer_snapshots"

# model_info.yaml 의 roboflow_deploy.model_id 와 일치해야 함
_MODEL_ID = "final-project-seg/1"


def build_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Roboflow Hosted API 세그멘테이션 추론")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--image",  help="정적 이미지 경로")
    g.add_argument("--camera", action="store_true", help="D455f 실시간 스트림")
    p.add_argument("--model-id", default=_MODEL_ID, help="Roboflow 모델 ID (project/version)")
    p.add_argument("--conf", type=float, default=0.70)
    return p.parse_args()


def get_api_key() -> str:
    key = os.environ.get("ROBOFLOW_API_KEY", "")
    if not key:
        env_path = _PROJECT_ROOT / ".env"
        if env_path.exists():
            for line in env_path.read_text().splitlines():
                if line.startswith("ROBOFLOW_API_KEY="):
                    key = line.split("=", 1)[1].strip()
                    break
    if not key:
        logger.error("ROBOFLOW_API_KEY 미설정 — .env 파일 또는 환경 변수로 설정 필요")
        logger.error("팀 Roboflow 워크스페이스 초대 후 Settings > API Keys 에서 발급")
        sys.exit(1)
    return key


def annotate_and_show(image: "np.ndarray", result, win_name: str) -> "np.ndarray":
    import supervision as sv
    detections = sv.Detections.from_inference(result)
    annotated = image.copy()
    annotated = sv.MaskAnnotator().annotate(annotated, detections)
    annotated = sv.LabelAnnotator().annotate(annotated, detections)
    cv2.imshow(win_name, annotated)
    return annotated


def run_image(image_path: str, model, conf: float) -> None:
    image = cv2.imread(image_path)
    if image is None:
        logger.error("이미지 로드 실패: %s", image_path)
        sys.exit(1)

    result = model.infer(image, confidence=conf)[0]
    annotated = annotate_and_show(image, result, f"Seg — {Path(image_path).name}")

    _SNAPSHOT_DIR.mkdir(exist_ok=True)
    out_path = _SNAPSHOT_DIR / f"seg_{Path(image_path).stem}.jpg"
    cv2.imwrite(str(out_path), annotated)
    logger.info("결과 저장: %s", out_path)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def run_camera(model, conf: float) -> None:
    import numpy as np
    try:
        import pyrealsense2 as rs
    except ImportError:
        logger.error("pyrealsense2 없음 — pip install pyrealsense2")
        sys.exit(1)

    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    logger.info("D455f 스트림 시작 — s: 스냅샷 저장  q: 종료")
    _SNAPSHOT_DIR.mkdir(exist_ok=True)

    try:
        snap_idx = 0
        while True:
            frames = pipeline.wait_for_frames()
            color  = np.asanyarray(frames.get_color_frame().get_data())  # type: ignore[name-defined]

            result = model.infer(color, confidence=conf)[0]
            annotated = annotate_and_show(color, result, "TopView Seg — s:스냅샷  q:종료")

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                out = _SNAPSHOT_DIR / f"seg_snap_{snap_idx:03d}.jpg"
                cv2.imwrite(str(out), annotated)
                logger.info("스냅샷 저장: %s", out)
                snap_idx += 1
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def main() -> None:
    args    = build_args()
    api_key = get_api_key()

    try:
        from inference import get_model
    except ImportError:
        logger.error("inference 패키지 없음 — pip install inference")
        sys.exit(1)

    logger.info("Roboflow 모델 로드: %s", args.model_id)
    model = get_model(args.model_id, api_key=api_key)

    if args.image:
        run_image(args.image, model, args.conf)
    else:
        run_camera(model, args.conf)


if __name__ == "__main__":
    main()
