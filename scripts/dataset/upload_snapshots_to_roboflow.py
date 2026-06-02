"""infer_snapshots 이미지 + 자동 라벨을 Roboflow에 업로드한다.

auto_label_snapshots.py 실행 후 사용할 것.
"""

import os
import time
from pathlib import Path

from dotenv import load_dotenv
from roboflow import Roboflow

load_dotenv(Path(__file__).parents[2] / ".env")

API_KEY   = os.environ["ROBOFLOW_API_KEY"]
WORKSPACE = "yeonseop9999-gmail-com"
PROJECT   = "final-project-kir4p"

IMAGE_DIR = Path(__file__).parents[2] / "scripts/infer_snapshots"
LABEL_DIR = IMAGE_DIR / "auto_labels"

# V2 클래스 순서 (알파벳순) — 절대 바꾸지 말 것
LABEL_MAP = {
    0: "multi_tool",
    1: "ratchet_wrench",
    2: "screwdriver",
    3: "socket_19mm",
    4: "spanner_16mm",
    5: "utility_knife",
}


def main() -> None:
    if not LABEL_DIR.exists():
        raise SystemExit("라벨 디렉토리 없음 — auto_label_snapshots.py 먼저 실행하세요")

    rf = Roboflow(api_key=API_KEY)
    project = rf.workspace(WORKSPACE).project(PROJECT)

    images = sorted(IMAGE_DIR.glob("*.jpg"))
    print(f"업로드 대상: {len(images)}장")

    success, fail, skipped = 0, 0, 0
    for img_path in images:
        label_path = LABEL_DIR / (img_path.stem + ".txt")
        # 빈 라벨 파일(미검출)은 이미지만 업로드
        is_empty = not label_path.exists() or label_path.read_text().strip() == ""

        try:
            if is_empty:
                project.upload(
                    image_path=str(img_path),
                    split="train",
                    num_retry_uploads=3,
                )
            else:
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
            time.sleep(0.3)
        except Exception as e:
            print(f"  [FAIL] {img_path.name}: {e}")
            fail += 1

    print(f"\n완료 — 성공: {success}, 실패: {fail}, 라벨없음 스킵: {skipped}")


if __name__ == "__main__":
    main()
