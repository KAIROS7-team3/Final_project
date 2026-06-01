"""공구 이미지 수집 — 탑뷰(D455f) 또는 그리퍼(C270) 시점별 분리 저장.

⚠️  이 스크립트는 원본 이미지 수집 전용이다.
    수집 후 Roboflow 업로드 → 라벨링/검수 → YOLOv11 export 순서로 진행.
    train_yolo.py는 Roboflow export 레이아웃({split}/images/, {split}/labels/)을 기대한다.

저장 경로: datasets/tools/{top_view|gripper}/images/{train|val}/{tool_id}/

사용법:
    # 탑뷰 캠 (D455f — pyrealsense2 사용)
    python scripts/dataset/collect_images.py --camera top_view --tool screwdriver --split train

    # 그리퍼 캠 (C270 — OpenCV VideoCapture 사용)
    python scripts/dataset/collect_images.py --camera gripper --tool screwdriver --split train
    python scripts/dataset/collect_images.py --camera gripper --tool screwdriver --split train --device 1

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
    "mix",
]
VALID_SPLITS = ["train", "val", "test_jig"]
VALID_CAMERAS = ["top_view", "gripper"]


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="공구 이미지 수집")
    p.add_argument("--camera", required=True, choices=VALID_CAMERAS,
                   help="top_view (D455f) 또는 gripper (C270)")
    p.add_argument("--tool", required=True, choices=VALID_TOOLS, help="수집할 공구 tool_id")
    p.add_argument("--split", required=True, choices=VALID_SPLITS, help="train 또는 val")
    p.add_argument("--device", type=int, default=0,
                   help="그리퍼(C270) 디바이스 인덱스 (기본 0, top_view 시 무시)")
    return p.parse_args()


def _collect_top_view(save_dir: Path, tool: str, split: str, counter: int) -> None:
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("[ERROR] pyrealsense2 미설치 — pip install pyrealsense2")
        sys.exit(1)
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("[ERROR] opencv-python 미설치 — pip install opencv-python")
        sys.exit(1)

    ctx = rs.context()
    if not ctx.query_devices():
        print("[ERROR] RealSense 장치 없음 — USB 연결 확인")
        sys.exit(1)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(cfg)

    print("[collect] D455f 워밍업 중...")
    for _ in range(20):
        pipeline.wait_for_frames()

    win_name = f"collect | top_view(D455f) | {tool} [{split}] — Space:저장  q:종료"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 960, 540)

    print(f"[collect] camera=top_view tool={tool} split={split} save_dir={save_dir}")
    print("[collect] Space: 저장 / q: 종료")

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            overlay = frame.copy()
            cv2.putText(overlay, f"[top_view] {tool}  [{split}]  saved={counter}",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow(win_name, cv2.resize(overlay, (960, 540)))

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                fname = save_dir / f"{tool}_{counter:04d}.jpg"
                cv2.imwrite(str(fname), frame)
                print(f"  saved: {fname}")
                counter += 1
            elif key == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"[collect] 완료 — 총 {counter}장 저장됨")


def _collect_gripper(save_dir: Path, tool: str, split: str, counter: int, device: int) -> None:
    try:
        import cv2
    except ImportError:
        print("[ERROR] opencv-python 미설치 — pip install opencv-python")
        sys.exit(1)

    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"[ERROR] 웹캠 device={device} 열기 실패. --device 옵션으로 인덱스 변경 시도")
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("[collect] C270 워밍업 중...")
    for _ in range(20):
        cap.read()

    win_name = f"collect | gripper(C270) | {tool} [{split}] — Space:저장  q:종료"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 960, 540)

    print(f"[collect] camera=gripper tool={tool} split={split} device={device} save_dir={save_dir}")
    print("[collect] Space: 저장 / q: 종료")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] 프레임 읽기 실패, 재시도...")
                continue

            overlay = frame.copy()
            cv2.putText(overlay, f"[gripper] {tool}  [{split}]  saved={counter}",
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow(win_name, overlay)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                fname = save_dir / f"{tool}_{counter:04d}.jpg"
                cv2.imwrite(str(fname), frame)
                print(f"  saved: {fname}")
                counter += 1
            elif key == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"[collect] 완료 — 총 {counter}장 저장됨")


def main() -> None:
    args = _parse_args()

    save_dir = (
        PROJECT_ROOT / "datasets" / "tools" / args.camera
        / "images" / args.split / args.tool
    )
    save_dir.mkdir(parents=True, exist_ok=True)

    existing = sorted(save_dir.glob("*.jpg"))
    counter = int(existing[-1].stem.split("_")[-1]) + 1 if existing else 0

    if args.camera == "top_view":
        _collect_top_view(save_dir, args.tool, args.split, counter)
    else:
        _collect_gripper(save_dir, args.tool, args.split, counter, args.device)


if __name__ == "__main__":
    main()
