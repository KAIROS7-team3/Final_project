#!/usr/bin/env python3
"""
c270_intrinsic_calib.py — Logitech C270 내부 파라미터 캘리브레이션

체커보드(9×6 내부 코너, 25mm 격자)를 테이블에 고정하고
로봇팔로 C270을 이동시키면서 다양한 각도/거리에서 촬영.

조작키:
  SPACE  : 코너 감지 성공 시 프레임 저장
  D      : 마지막 저장 프레임 삭제
  C      : 캘리브레이션 계산 (최소 20장 이상)
  Q/ESC  : 종료

결과: config/c270_camera_info.yaml
"""
import logging
import os
import sys
import time

import cv2
import numpy as np
import yaml

# ── 설정 ──────────────────────────────────────────────────────────────────
DEVICE       = 8          # /dev/video8 = C270
WIDTH        = 640
HEIGHT       = 480
FPS          = 30

BOARD_COLS   = 9          # 내부 코너 열 수
BOARD_ROWS   = 6          # 내부 코너 행 수
SQUARE_MM    = 25.0       # 격자 한 칸 실제 크기 (mm)

MIN_FRAMES   = 20         # 캘리브레이션 최소 샘플 수
SAVE_DIR     = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'samples_c270_calib')
CONFIG_OUT   = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             '..', 'config', 'c270_camera_info.yaml')

logging.basicConfig(level=logging.INFO, format='[c270_calib] %(message)s')
log = logging.getLogger(__name__)

os.makedirs(SAVE_DIR, exist_ok=True)

# ── 체커보드 3D 기준점 (Z=0 평면) ─────────────────────────────────────────
objp = np.zeros((BOARD_ROWS * BOARD_COLS, 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
objp *= (SQUARE_MM / 1000.0)   # mm → m

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

def open_camera() -> cv2.VideoCapture:
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    if not cap.isOpened():
        log.error(f'/dev/video{DEVICE} 열기 실패')
        sys.exit(1)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    log.info(f'C270 연결 — {w}x{h} @ {FPS}fps')
    return cap

def run_calibration(obj_pts, img_pts, img_size):
    log.info(f'캘리브레이션 계산 중 ({len(obj_pts)}장)...')
    flags = cv2.CALIB_RATIONAL_MODEL
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_pts, img_pts, img_size, None, None, flags=flags)
    return ret, K, dist

def save_config(K, dist, img_size, n_frames, rms):
    data = {
        'schema_version': 1,
        '_comment': 'C270 그리퍼 카메라 intrinsic (c270_intrinsic_calib.py)',
        'calibration_date': time.strftime('%Y-%m-%d'),
        'rms_reprojection_error_px': float(rms),
        'sample_count': n_frames,
        'board': {
            'cols': BOARD_COLS,
            'rows': BOARD_ROWS,
            'square_m': SQUARE_MM / 1000.0,
        },
        'intrinsics': {
            'width':  img_size[0],
            'height': img_size[1],
            'fx': float(K[0, 0]),
            'fy': float(K[1, 1]),
            'cx': float(K[0, 2]),
            'cy': float(K[1, 2]),
            'distortion_model': 'rational_polynomial',
            'coeffs': dist.flatten().tolist(),
        },
        'camera_matrix_row_major': K.tolist(),
    }
    path = os.path.abspath(CONFIG_OUT)
    class _IndentDumper(yaml.Dumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, indentless=False)

    with open(path, 'w') as f:
        yaml.dump(data, f, Dumper=_IndentDumper,
                  default_flow_style=False, allow_unicode=True)
    log.info(f'저장 완료: {path}')
    log.info(f'RMS 재투영 오차: {rms:.4f} px  (목표 < 0.5px)')

def main():
    cap = open_camera()
    obj_pts, img_pts = [], []
    saved_files = []
    last_detected = False

    print('\n=== C270 Intrinsic Calibration ===')
    print(f'체커보드: {BOARD_COLS}×{BOARD_ROWS} 내부 코너, {SQUARE_MM}mm 격자')
    print('SPACE=저장  D=마지막삭제  C=계산(20장↑)  Q=종료\n')

    while True:
        ret, frame = cap.read()
        if not ret:
            log.error('프레임 읽기 실패')
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, (BOARD_COLS, BOARD_ROWS),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)

        disp = frame.copy()
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(disp, (BOARD_COLS, BOARD_ROWS), corners2, found)
            color = (0, 255, 0)
            status = f'코너 감지 OK — SPACE로 저장 ({len(obj_pts)}/{MIN_FRAMES})'
        else:
            color = (0, 0, 255)
            status = f'코너 미감지 ({len(obj_pts)}/{MIN_FRAMES})'

        last_detected = found
        cv2.putText(disp, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow('C270 Intrinsic Calib', disp)

        key = cv2.waitKey(1) & 0xFF

        if key == ord(' ') and found:
            obj_pts.append(objp)
            img_pts.append(corners2)
            fname = os.path.join(SAVE_DIR, f'calib_{len(obj_pts):03d}.png')
            cv2.imwrite(fname, frame)
            saved_files.append(fname)
            log.info(f'[{len(obj_pts)}] 저장: {fname}')

        elif key == ord('d') and saved_files:
            removed = saved_files.pop()
            obj_pts.pop()
            img_pts.pop()
            os.remove(removed)
            log.info(f'삭제: {removed}  남은 {len(obj_pts)}장')

        elif key == ord('c'):
            if len(obj_pts) < MIN_FRAMES:
                log.warning(f'샘플 부족 ({len(obj_pts)}/{MIN_FRAMES}) — 더 수집하세요')
            else:
                img_size = (gray.shape[1], gray.shape[0])
                rms, K, dist = run_calibration(obj_pts, img_pts, img_size)
                save_config(K, dist, img_size, len(obj_pts), rms)
                print(f'\n카메라 행렬:\n{K}')
                print(f'\n왜곡 계수: {dist.flatten()}')
                if rms > 0.5:
                    log.warning(f'RMS {rms:.3f}px > 0.5px — 이미지 더 추가하거나 흔들린 컷 삭제 권장')
                break

        elif key in (ord('q'), 27):
            log.info('종료')
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
