#!/usr/bin/env python3
"""
c270_handeye_capture.py — C270 eye-in-hand 캘리브레이션 이미지 캡처

사용법:
  1. DART에서 직접교시로 자세 이동 + MoveJ 웨이포인트 저장 (tw 파일)
  2. 각 자세에서 이 스크립트로 이미지 한 장 캡처
  3. 완료 후 tw 파일 + samples_c270_handeye/ 폴더를 함께 전달

조작키:
  SPACE : 현재 프레임 저장 (ArUco 감지 여부 무관)
  D     : 마지막 저장 삭제
  Q/ESC : 종료

저장 경로: scripts/samples_c270_handeye/pose_NNN.png
"""
import logging
import os
import sys

import cv2
import numpy as np
import yaml

logging.basicConfig(level=logging.INFO, format='[c270_capture] %(message)s')
log = logging.getLogger(__name__)

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR  = os.path.join(_SCRIPT_DIR, '..', 'config')
SAVE_DIR     = os.path.join(_SCRIPT_DIR, 'samples_c270_handeye')
os.makedirs(SAVE_DIR, exist_ok=True)

DEVICE        = 8
WIDTH, HEIGHT = 640, 480

# ── 카메라 intrinsic (ArUco 프리뷰용) ──────────────────────────────────────────
def _load_cam():
    path = os.path.join(_CONFIG_DIR, 'c270_camera_info.yaml')
    try:
        with open(path) as f:
            intr = yaml.safe_load(f)['intrinsics']
        K = np.array([[intr['fx'], 0, intr['cx']],
                      [0, intr['fy'], intr['cy']],
                      [0, 0, 1]], dtype=np.float64)
        d = np.array(intr['coeffs'], dtype=np.float64)
        return K, d
    except Exception:
        log.warning('c270_camera_info.yaml 없음 — ArUco 축 표시 비활성')
        return None, None

K, DIST = _load_cam()
MARKER_SIZE_M = 0.05
_DICT   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
_PARAMS = cv2.aruco.DetectorParameters_create()

_half = MARKER_SIZE_M / 2.0
OBJ_PTS = np.array([[-_half,  _half, 0], [ _half,  _half, 0],
                     [ _half, -_half, 0], [-_half, -_half, 0]], dtype=np.float32)


def draw_preview(frame: np.ndarray) -> tuple[np.ndarray, bool]:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _DICT, parameters=_PARAMS)
    disp = frame.copy()
    detected = ids is not None

    if detected and K is not None:
        cv2.aruco.drawDetectedMarkers(disp, corners, ids)
        img_pts = corners[0].reshape(4, 2).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, K, DIST)
        if ok:
            dist_m = float(np.linalg.norm(tvec))
            cv2.drawFrameAxes(disp, K, DIST, rvec, tvec, MARKER_SIZE_M * 0.5)
            cv2.putText(disp, f'ID{ids.flatten()[0]}  {dist_m:.3f}m',
                        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    elif detected:
        cv2.aruco.drawDetectedMarkers(disp, corners, ids)
        cv2.putText(disp, f'ID{ids.flatten()[0]} detected',
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    else:
        cv2.putText(disp, 'NO MARKER', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

    return disp, detected


def main() -> None:
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        log.error('/dev/video%d 열기 실패', DEVICE)
        sys.exit(1)

    existing = sorted([f for f in os.listdir(SAVE_DIR) if f.startswith('pose_')])
    count = len(existing)
    if count:
        log.info('기존 %d장 유지 — %d번부터 이어서 저장', count, count + 1)

    saved_files = [os.path.join(SAVE_DIR, f) for f in existing]

    print('\n=== C270 Eye-in-Hand 캡처 ===')
    print('DART에서 자세 이동 후 SPACE로 저장 — tw 파일과 번호 순서 맞출 것')
    print('SPACE=저장  D=마지막삭제  Q=종료\n')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        disp, detected = draw_preview(frame)
        marker_hint = '✓ 마커 감지' if detected else '✗ 마커 없음'
        cv2.putText(disp, f'[{count}장]  {marker_hint}  SPACE=저장  D=삭제  Q=종료',
                    (10, HEIGHT - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        cv2.imshow('C270 Handeye Capture', disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            count += 1
            path = os.path.join(SAVE_DIR, f'pose_{count:03d}.png')
            cv2.imwrite(path, frame)
            saved_files.append(path)
            marker_tag = '(마커O)' if detected else '(마커X)'
            log.info('[%d] %s %s', count, path, marker_tag)

        elif key == ord('d') and saved_files:
            removed = saved_files.pop()
            os.remove(removed)
            count -= 1
            log.info('삭제: %s — 남은 %d장', removed, count)

        elif key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f'\n총 {count}장 저장 → {SAVE_DIR}')
    print('다음: tw 파일과 함께 Claude에게 전달')


if __name__ == '__main__':
    main()
