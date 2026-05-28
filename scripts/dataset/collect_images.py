"""공구 이미지 수집 — 탑뷰(D455f) 또는 그리퍼(C270) 시점별 분리 저장.

YOLOv11s 학습용. 두 카메라는 촬영 시점이 달라 별도 데이터셋·모델 필요.
저장 경로: datasets/tools/{top_view|gripper}/images/{train|val}/{tool_id}/

사용법:
    # 그리퍼 캠 (C270, 기본)
    python scripts/dataset/collect_images.py --camera gripper --tool screwdriver --split train

    # 탑뷰 캠 (D455f)
    python scripts/dataset/collect_images.py --camera top_view --tool screwdriver --split train --device 2

옵션:
    --device  웹캠 디바이스 인덱스 (기본 0; D455f는 1·2 시도)

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

PROJECT_ROOT = Path(__file__).resolve().parents[2]
VALID_TOOLS = [
    "screwdriver",
    "utility_knife",
    "ratchet_wrench",
    "multi_tool",
    "spanner_16mm",
    "socket_19mm",
]
VALID_SPLITS = ["train", "val"]
VALID_CAMERAS = ["top_view", "gripper"]


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="공구 이미지 수집")
    p.add_argument("--camera", required=True, choices=VALID_CAMERAS,
                   help="top_view (D455f) 또는 gripper (C270)")
    p.add_argument("--tool", required=True, choices=VALID_TOOLS, help="수집할 공구 tool_id")
    p.add_argument("--split", required=True, choices=VALID_SPLITS, help="train 또는 val")
    p.add_argument("--device", type=int, default=0, help="웹캠 디바이스 인덱스 (기본 0)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    try:
        import cv2
    except ImportError:
        print("[ERROR] opencv-python 미설치 — pip install opencv-python")
        sys.exit(1)

    save_dir = (
        PROJECT_ROOT / "datasets" / "tools" / args.camera
        / "images" / args.split / args.tool
    )
    save_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(save_dir.glob("*.jpg"))
    counter = int(existing[-1].stem.split("_")[-1]) + 1 if existing else 0

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print(f"[ERROR] 웹캠 device={args.device} 열기 실패. --device 옵션으로 인덱스 변경 시도")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("[collect] 카메라 워밍업 중...")
    for _ in range(20):
        cap.read()

    win_name = f"collect | {args.camera} | {args.tool} [{args.split}] — Space:저장  q:종료"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 960, 540)

    print(f"[collect] camera={args.camera} tool={args.tool} split={args.split} "
          f"device={args.device} save_dir={save_dir}")
    print("[collect] Space: 저장 / q: 종료")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] 프레임 읽기 실패, 재시도...")
                continue

            overlay = frame.copy()
            cv2.putText(overlay, f"[{args.camera}] {args.tool}  [{args.split}]  saved={counter}",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow(win_name, overlay)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                fname = save_dir / f"{args.tool}_{counter:04d}.jpg"
                cv2.imwrite(str(fname), frame)
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
