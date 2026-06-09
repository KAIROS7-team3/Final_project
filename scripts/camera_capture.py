#!/usr/bin/env python3
"""
camera_capture.py — RealSense 카메라 프리뷰 + Space로 이미지 저장

사용법:
  python3 scripts/camera_capture.py
  SPACE: 저장 / Q: 종료

저장 경로: scripts/samples/sample_001.png, 002.png ...
"""
import os
import sys
import cv2
import numpy as np
import pyrealsense2 as rs

SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'samples')
os.makedirs(SAVE_DIR, exist_ok=True)

WIDTH, HEIGHT, FPS = 1280, 720, 30

pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

try:
    pipeline.start(cfg)
    print('카메라 연결 완료.')
except Exception as e:
    print(f'카메라 시작 실패: {e}')
    sys.exit(1)

# 기존 파일 번호 이어서 시작 (덮어쓰기 방지)
existing = [f for f in os.listdir(SAVE_DIR) if f.startswith('sample_') and f.endswith('.png')]
count = len(existing)
if count:
    print(f'기존 {count}장 유지 — {count+1}번부터 이어서 저장')

cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
print('SPACE=저장  Q=종료\n')

last_frame = None
try:
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            last_frame = np.asanyarray(frames.get_color_frame().get_data())
        except RuntimeError:
            print('카메라 일시 끊김 — 재연결 중...')
            try:
                pipeline.stop()
                pipeline.start(cfg)
                print('재연결 완료.')
            except Exception:
                pass
            continue

        display = last_frame.copy()
        cv2.putText(display, f'Saved: {count}  |  SPACE=save  Q=quit',
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.imshow('Camera', display)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), ord('Q'), 27):
            break
        if key == ord(' ') and last_frame is not None:
            count += 1
            path = os.path.join(SAVE_DIR, f'sample_{count:03d}.png')
            cv2.imwrite(path, last_frame)
            print(f'[{count:03d}] 저장: {path}')
finally:
    pipeline.stop()
    cv2.destroyAllWindows()

print(f'\n총 {count}장 저장 → {SAVE_DIR}')
