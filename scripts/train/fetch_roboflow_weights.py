"""Roboflow inference 캐시에서 모델 가중치 추출.

사용법:
    python3 scripts/train/fetch_roboflow_weights.py
"""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
from pathlib import Path

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_DST = _PROJECT_ROOT / "ros2_ws/src/vision/model_library/top_view_model/v3-seg/weights/best.pt"
_API_KEY = os.environ.get("ROBOFLOW_API_KEY", "")
_MODEL_ID = "final-project-seg/1"


def find_cached_weights() -> list[Path]:
    result = subprocess.run(
        ["find", "/root", str(Path.home()), "/tmp", "-name", "*.pt", "-size", "+1M"],
        capture_output=True, text=True
    )
    return [Path(p) for p in result.stdout.strip().splitlines() if p]


def main() -> None:
    # Step 1: inference 패키지 설치
    logger.info("inference 패키지 설치 중...")
    subprocess.run(["pip", "install", "inference", "-q"], check=True)

    # Step 2: 모델 호출 (캐시 생성)
    logger.info("Roboflow 모델 다운로드 중: %s", _MODEL_ID)
    os.environ["ROBOFLOW_API_KEY"] = _API_KEY
    try:
        from inference import get_model
        get_model(_MODEL_ID)
        logger.info("모델 로드 완료")
    except Exception as e:
        logger.warning("모델 로드 중 오류 (캐시는 생성됐을 수 있음): %s", e)

    # Step 3: 캐시에서 .pt 파일 탐색
    logger.info("캐시된 .pt 파일 탐색 중...")
    found = find_cached_weights()

    if not found:
        logger.warning(".pt 파일을 찾지 못했습니다 — ONNX 포맷으로 캐싱됐을 수 있음")
        onnx_result = subprocess.run(
            ["find", str(Path.home()), "/tmp", "-name", "*.onnx", "-size", "+1M"],
            capture_output=True, text=True
        )
        for p in onnx_result.stdout.strip().splitlines():
            logger.info("ONNX 캐시 발견: %s", p)
        return

    for p in found:
        logger.info("발견: %s (%s MB)", p, round(p.stat().st_size / 1e6, 1))

    # Step 4: 가장 큰 .pt 파일을 v3-seg weights로 복사
    best = max(found, key=lambda p: p.stat().st_size)
    _DST.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, _DST)
    logger.info("복사 완료: %s → %s", best, _DST)
    logger.info("다음 단계: v3-seg/model_info.yaml 의 drive_url 기입 후 커밋")


if __name__ == "__main__":
    main()
