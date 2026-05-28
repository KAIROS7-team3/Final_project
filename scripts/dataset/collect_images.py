"""D455f RGB 스트림에서 탑뷰 이미지를 수집해 datasets/tools/images/에 저장.

사용법:
    python scripts/dataset/collect_images.py --tool socket_19mm --split train
    python scripts/dataset/collect_images.py --tool ratchet_handle --split val

조작:
    Space  — 현재 프레임 저장
    q      — 종료
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
VALID_TOOLS = [
    "socket_19mm",
    "ratchet_handle",
    "hex_key_set_folding",
    "screwdriver_flat_6x100",
    "cutter_knife",
]
VALID_SPLITS = ["train", "val"]


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="공구 탑뷰 이미지 수집")
    p.add_argument("--tool", required=True, choices=VALID_TOOLS, help="수집할 공구 tool_id")
    p.add_argument("--split", required=True, choices=VALID_SPLITS, help="train 또는 val")
    p.add_argument("--save-dir", type=Path,
                   default=PROJECT_ROOT / "datasets" / "tools" / "images",
                   help="저장 루트 (기본: datasets/tools/images)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    try:
        import cv2
        import pyrealsense2 as rs
    except ImportError as e:
        print(f"[ERROR] 의존성 없음: {e}")
        print("pip install pyrealsense2 opencv-python")
        sys.exit(1)

    save_dir = args.save_dir / args.split / args.tool
    save_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(save_dir.glob("*.jpg"))
    counter = int(existing[-1].stem.split("_")[-1]) + 1 if existing else 0

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(cfg)

    print(f"[collect] tool={args.tool} split={args.split} save_dir={save_dir}")
    print("[collect] Space: 저장 / q: 종료")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color = frames.get_color_frame()
            if not color:
                continue

            import numpy as np
            img = np.asanyarray(color.get_data())

            cv2.putText(img, f"{args.tool} [{args.split}] saved={counter}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("collect — Space:save  q:quit", img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                fname = save_dir / f"{args.tool}_{counter:04d}.jpg"
                cv2.imwrite(str(fname), img)
                print(f"  saved: {fname}")
                counter += 1
            elif key == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"[collect] 완료 — 총 {counter}장 저장됨")


if __name__ == "__main__":
    main()
