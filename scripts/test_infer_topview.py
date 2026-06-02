"""탑뷰 D455f 실시간 추론 테스트 스크립트 (ROS2 불필요).

사용법:
    python3 scripts/test_infer_topview.py [초수 기본값=30]

    예) python3 scripts/test_infer_topview.py 10   # 10초 동안 실행
        Ctrl+C 로 언제든 종료 가능

출력: scripts/infer_snapshots/ 에 1초마다 annotated 이미지 저장
"""
from __future__ import annotations

import select
import sys
import termios
import tty
import time
from pathlib import Path

import cv2
import pyrealsense2 as rs
import yaml
from ultralytics import YOLO

# ---------------------------------------------------------------------------
# 설정
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH  = PROJECT_ROOT / "config" / "vision.yaml"
SNAPSHOT_DIR = PROJECT_ROOT / "scripts" / "infer_snapshots"
SNAPSHOT_DIR.mkdir(exist_ok=True)

with CONFIG_PATH.open() as f:
    cfg = yaml.safe_load(f)["yolo"]

MODEL_PATH  = PROJECT_ROOT / cfg["top_view_model_path"]
CLASS_NAMES = cfg["class_names"]
CONF        = cfg["confidence_threshold"]
IOU         = cfg["iou_threshold"]

# ---------------------------------------------------------------------------
# 모델 로드
# ---------------------------------------------------------------------------
if not MODEL_PATH.exists():
    print(f"[ERROR] 모델 파일 없음: {MODEL_PATH}")
    print("  → model_library/top_view_model/v2/weights/best.pt 를 runs/yolo/top_view_v2/weights/ 에 복사하거나")
    print("    config/vision.yaml top_view_model_path 경로 확인")
    sys.exit(1)

print(f"[INFO] 모델 로드 중: {MODEL_PATH}")
model = YOLO(str(MODEL_PATH))
print(f"[INFO] 모델 로드 완료 | conf={CONF} iou={IOU} classes={CLASS_NAMES}")

# ---------------------------------------------------------------------------
# D455f 스트림 시작
# ---------------------------------------------------------------------------
pipeline = rs.pipeline()
pipeline_cfg = rs.config()
pipeline_cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

print("[INFO] D455f 스트림 시작...")
pipeline.start(pipeline_cfg)
print("[INFO] 스트림 시작 완료. 'q' 종료 / 's' 스냅샷 저장")

# ---------------------------------------------------------------------------
# 터미널 단일 키 입력 헬퍼 (Enter 없이 즉시 감지)
# ---------------------------------------------------------------------------
def _keypress() -> str | None:
    """non-blocking — 눌린 키 반환, 없으면 None."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


# ---------------------------------------------------------------------------
# 추론 루프 (GUI 없음 — s: 스냅샷 저장 / q 또는 Ctrl+C: 종료)
# ---------------------------------------------------------------------------
import numpy as np

print("[INFO] 추론 시작 | s: 스냅샷 저장  q / Ctrl+C: 종료")
print(f"[INFO] 스냅샷 저장 경로: {SNAPSHOT_DIR}")

frame_count  = 0
fps_time     = time.time()
last_annotated = None

# 터미널을 raw 모드로 전환 (Enter 없이 키 즉시 감지)
fd       = sys.stdin.fileno()
old_tty  = termios.tcgetattr(fd)

try:
    tty.setraw(fd)

    while True:
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        results   = model(img, conf=CONF, iou=IOU, verbose=False)
        annotated = results[0].plot()
        last_annotated = annotated

        # FPS
        frame_count += 1
        if frame_count % 30 == 0:
            fps = 30 / (time.time() - fps_time)
            fps_time = time.time()
            sys.stdout.write(f"\r[FPS] {fps:.1f}  ")
            sys.stdout.flush()

        # 탐지 결과 출력
        boxes = results[0].boxes
        if boxes is not None and len(boxes):
            detections = [
                f"{CLASS_NAMES[int(c)] if int(c) < len(CLASS_NAMES) else int(c)}({s:.2f})"
                for c, s in zip(boxes.cls.tolist(), boxes.conf.tolist())
            ]
            sys.stdout.write(f"\r[DETECT] {', '.join(detections)}          \n")
            sys.stdout.flush()

        # 키 입력 처리
        key = _keypress()
        if key == "s" and last_annotated is not None:
            ts        = time.strftime("%Y%m%d_%H%M%S")
            snap_path = SNAPSHOT_DIR / f"snap_{ts}.jpg"
            cv2.imwrite(str(snap_path), last_annotated)
            sys.stdout.write(f"\r[SNAP] 저장: {snap_path.name}\n")
            sys.stdout.flush()
        elif key in ("q", "\x03"):  # q 또는 Ctrl+C
            break

except KeyboardInterrupt:
    pass
finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_tty)  # 터미널 복원
    pipeline.stop()
    print(f"\n[INFO] 종료 | {frame_count}프레임 | 스냅샷: {SNAPSHOT_DIR}")
