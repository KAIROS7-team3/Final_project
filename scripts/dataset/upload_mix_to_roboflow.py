"""mix 200장 + 자동 라벨을 Roboflow 프로젝트에 업로드한다."""

import os
import time
from pathlib import Path

from dotenv import load_dotenv
from roboflow import Roboflow

load_dotenv(Path(__file__).parents[2] / ".env")

API_KEY = os.environ["ROBOFLOW_API_KEY"]
WORKSPACE = "yeonseop9999-gmail-com"
PROJECT = "final-project-kir4p"

IMAGE_DIR = Path(__file__).parents[2] / "datasets/tools/top_view/images/train/mix"
LABEL_DIR = Path(__file__).parents[2] / "datasets/tools/top_view/auto_labels/mix"

LABEL_MAP = {
    0: "multi_tool",
    1: "ratchet_wrench",
    2: "screwdriver",
    3: "socket_19mm",
    4: "spanner_16mm",
    5: "utility_knife",
}


def main() -> None:
    rf = Roboflow(api_key=API_KEY)
    project = rf.workspace(WORKSPACE).project(PROJECT)

    images = sorted(IMAGE_DIR.glob("*.jpg"))
    print(f"업로드 대상: {len(images)}장")

    success, fail = 0, 0
    for img_path in images:
        label_path = LABEL_DIR / (img_path.stem + ".txt")
        if not label_path.exists():
            print(f"  [SKIP] 라벨 없음: {img_path.name}")
            fail += 1
            continue

        try:
            project.upload(
                image_path=str(img_path),
                annotation_path=str(label_path),
                annotation_labelmap=LABEL_MAP,
                split="train",
                num_retry_uploads=3,
            )
            success += 1
            if success % 20 == 0:
                print(f"  진행: {success}/{len(images)}")
            time.sleep(0.3)  # API rate limit 방지
        except Exception as e:
            print(f"  [FAIL] {img_path.name}: {e}")
            fail += 1

    print(f"\n완료 — 성공: {success}, 실패: {fail}")


if __name__ == "__main__":
    main()
