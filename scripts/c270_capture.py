#!/usr/bin/env python3
"""
c270_capture.py — Logitech C270 라이브 프리뷰 + 이미지 캡처

  SPACE : 이미지 저장 (scripts/samples_c270/ 폴더)
  C     : 현재 커서 위치 좌표 출력 (터미널)
  Q/ESC : 종료

마우스를 화면 위에 올리면 픽셀 좌표 (x, y)가 상단에 실시간 표시됨.
"""
import os
import sys
import cv2

DEVICE = 8          # /dev/video8 = C270
WIDTH, HEIGHT = 640, 480
FPS = 30
SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'samples_c270')
os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    print(f'오류: /dev/video{DEVICE} 열기 실패')
    sys.exit(1)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f'C270 연결 완료 — {actual_w}x{actual_h} @ {FPS}fps')
print('SPACE=저장  C=좌표출력  Q/ESC=종료\n')

existing = [f for f in os.listdir(SAVE_DIR) if f.startswith('c270_') and f.endswith('.png')]
count = len(existing)
if count:
    print(f'기존 {count}장 유지 — {count+1}번부터 이어서 저장')

mouse_x, mouse_y = 0, 0

def on_mouse(event, x, y, flags, param):
    global mouse_x, mouse_y
    mouse_x, mouse_y = x, y

cv2.namedWindow('C270', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('C270', on_mouse)

while True:
    ret, frame = cap.read()
    if not ret:
        print('프레임 읽기 실패')
        break

    display = frame.copy()

    # 마우스 커서 위치에 십자선
    cv2.drawMarker(display, (mouse_x, mouse_y), (0, 255, 255),
                   cv2.MARKER_CROSS, 20, 1)

    # 상단 HUD
    cv2.rectangle(display, (0, 0), (actual_w, 42), (0, 0, 0), -1)
    cv2.putText(display,
                f'({mouse_x:4d}, {mouse_y:4d})  |  saved:{count}  |  SPACE=save  Q=quit',
                (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow('C270', display)
    key = cv2.waitKey(1) & 0xFF

    if key in (ord('q'), ord('Q'), 27):
        break

    if key == ord(' '):
        count += 1
        path = os.path.join(SAVE_DIR, f'c270_{count:03d}.png')
        cv2.imwrite(path, frame)
        print(f'[{count:03d}] 저장: {path}')

    if key in (ord('c'), ord('C')):
        print(f'현재 좌표: ({mouse_x}, {mouse_y})')

cap.release()
cv2.destroyAllWindows()
print(f'\n총 {count}장 저장 → {SAVE_DIR}')
