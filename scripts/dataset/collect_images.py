"""그리퍼 웹캠(Logitech C270)으로 공구 이미지를 수집해 datasets/tools/images/에 저장.

YOLOv11s 학습용 — 그리퍼 카메라 시점 기준으로 수집해야 추론 환경과 일치.

사용법:
    python scripts/dataset/collect_images.py --tool socket_19mm --split train
    python scripts/dataset/collect_images.py --tool ratchet_handle --split val

옵션:
    --device  웹캠 디바이스 인덱스 (기본 0, 인식 안 될 시 1·2 시도)

조작:
    Space  — 현재 프레임 저장
    q      — 종료
"""
from __future__ import annotations

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import argparse
import sys
from pathlib import Path

import numpy as np

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
    p = argparse.ArgumentParser(description="공구 이미지 수집 (그리퍼 웹캠)")
    p.add_argument("--tool", required=True, choices=VALID_TOOLS, help="수집할 공구 tool_id")
    p.add_argument("--split", required=True, choices=VALID_SPLITS, help="train 또는 val")
    p.add_argument("--device", type=int, default=0, help="웹캠 디바이스 인덱스 (기본 0)")
    p.add_argument("--save-dir", type=Path,
                   default=PROJECT_ROOT / "datasets" / "tools" / "images",
                   help="저장 루트 (기본: datasets/tools/images)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    try:
        import cv2
    except ImportError:
        print("[ERROR] opencv-python 미설치 — pip install opencv-python")
        sys.exit(1)

    save_dir = args.save_dir / args.split / args.tool
    save_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(save_dir.glob("*.jpg"))
    counter = int(existing[-1].stem.split("_")[-1]) + 1 if existing else 0

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"[ERROR] 웹캠 device={args.device} 열기 실패. --device 옵션으로 인덱스 변경 시도")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 자동노출 안정화 대기
    print("[collect] 카메라 워밍업 중...")
    for _ in range(20):
        cap.read()

    win_name = f"collect | {args.tool} [{args.split}] — Space:저장  q:종료"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 960, 540)

    print(f"[collect] tool={args.tool} split={args.split} device={args.device} save_dir={save_dir}")
    print("[collect] Space: 저장 / q: 종료")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] 프레임 읽기 실패, 재시도...")
                continue

            overlay = frame.copy()
            cv2.putText(overlay, f"{args.tool}  [{args.split}]  saved={counter}",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow(win_name, overlay)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                fname = save_dir / f"{args.tool}_{counter:04d}.jpg"
                cv2.imwrite(str(fname), frame)  # overlay 없는 원본 저장
                print(f"  saved: {fname}")
                counter += 1
            elif key == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"[collect] 완료 — 총 {counter}장 저장됨")


if __name__ == "__main__":
    main()
