#!/usr/bin/env python3
"""
c270_handeye_collect.py — C270 그리퍼 카메라 eye-in-hand 캘리브레이션 데이터 수집

직접교시로 로봇을 원하는 자세로 이동 → DART 화면 TCP 값 입력 → 저장 (15~25쌍 권장)
결과: scripts/c270_handeye_data.npz → compute_handeye_opencv.py 에서 사용

사전 조건:
  1. DART에서 직접교시 모드 활성화
  2. ArUco ID0 마커 (5cm) 테이블에 고정 배치

사용 방법:
  1. python3 scripts/c270_handeye_collect.py
  2. 로봇을 직접교시로 이동 (C270이 ID0 마커를 볼 수 있는 자세)
  3. 카메라 창에 초록 OK 뜨면 ENTER
  4. DART 화면 TCP 값 (X Y Z Rx Ry Rz) 입력
  5. 15~25쌍 반복

DART TCP 단위: X Y Z = mm,  Rx Ry Rz = deg (ZYZ 오일러)
"""
import logging
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[c270_handeye] %(message)s')
log = logging.getLogger(__name__)

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR  = os.path.join(_SCRIPT_DIR, '..', 'config')
OUTPUT_PATH  = os.path.join(_SCRIPT_DIR, 'c270_handeye_data.npz')

DEVICE        = 2
WIDTH, HEIGHT = 640, 480
MIN_SAMPLES   = 15

# ── 설정 로드 ──────────────────────────────────────────────────────────────────
def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)

_cam  = _load_yaml(os.path.join(_CONFIG_DIR, 'c270_camera_info.yaml'))['intrinsics']
_rt   = _load_yaml(os.path.join(_CONFIG_DIR, 'runtime.yaml'))
_calib = _rt.get('calibration', {})

CAMERA_MATRIX = np.array([
    [_cam['fx'], 0.0,        _cam['cx']],
    [0.0,        _cam['fy'], _cam['cy']],
    [0.0,        0.0,        1.0       ],
], dtype=np.float64)
DIST_COEFFS   = np.array(_cam['coeffs'], dtype=np.float64)

MARKER_SIZE_M  : float = float(_calib.get('aruco_marker_size_m', 0.05))
ARUCO_DICT_ID  : int   = int(_calib.get('aruco_dict_id', cv2.aruco.DICT_4X4_50))
ARUCO_TARGET_ID: int   = 0   # 고정 마커 ID
BASE_FRAME     : str   = str(_calib.get('base_frame',    'base_link'))
GRIPPER_FRAME  : str   = str(_calib.get('gripper_frame', 'link_6'))

# 품질 필터
MAX_FACE_ANGLE_DEG = 40.0   # eye-in-hand: 각도 범위 넓게 허용
MIN_DIST_M         = 0.10
MAX_DIST_M         = 0.80

_half = MARKER_SIZE_M / 2.0
OBJ_PTS = np.array([
    [-_half,  _half, 0.0],
    [ _half,  _half, 0.0],
    [ _half, -_half, 0.0],
    [-_half, -_half, 0.0],
], dtype=np.float32)

_ARUCO_DICT   = cv2.aruco.Dictionary_get(ARUCO_DICT_ID)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()


# ── DART TCP 입력 파서 ─────────────────────────────────────────────────────────
def parse_dart_tcp(line: str) -> tuple[np.ndarray, np.ndarray] | None:
    """
    DART TCP 입력 파싱: 'X Y Z Rx Ry Rz' (mm, deg ZYZ)
    → (R_gripper2base 3×3, t_gripper2base 3,) [m, rad]
    """
    try:
        vals = [float(v) for v in line.strip().split()]
        if len(vals) != 6:
            raise ValueError(f'값 6개 필요 (입력: {len(vals)}개)')
    except ValueError as e:
        log.error('입력 오류: %s', e)
        return None

    x, y, z, rz1, ry, rz2 = vals
    t = np.array([x / 1000.0, y / 1000.0, z / 1000.0], dtype=np.float64)  # mm → m

    # DART ZYZ 오일러 → 회전행렬 (E-1: rad 변환)
    R = Rotation.from_euler('ZYZ', [rz1, ry, rz2], degrees=True).as_matrix()
    return R, t


# ── ArUco 감지 ─────────────────────────────────────────────────────────────────
def detect(frame: np.ndarray):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS)

    disp = frame.copy()
    if ids is None:
        cv2.putText(disp, 'NO MARKER', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return None, None, disp, ''

    # ID0 우선 선택
    idx = 0
    for i, mid in enumerate(ids.flatten()):
        if mid == ARUCO_TARGET_ID:
            idx = i
            break

    img_pts = corners[idx].reshape(4, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, CAMERA_MATRIX, DIST_COEFFS)
    if not ok:
        return None, None, disp, 'SOLVEPNP_FAIL'

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.flatten()
    dist_m = float(np.linalg.norm(t))
    cos_a  = abs(float(R[2, 2]))
    angle  = float(np.degrees(np.arccos(np.clip(cos_a, 0.0, 1.0))))

    qual_ok = (angle <= MAX_FACE_ANGLE_DEG and MIN_DIST_M <= dist_m <= MAX_DIST_M)
    color   = (0, 255, 0) if qual_ok else (0, 165, 255)
    tag     = 'OK' if qual_ok else f'BAD ang={angle:.0f}° dist={dist_m:.2f}m'

    cv2.drawFrameAxes(disp, CAMERA_MATRIX, DIST_COEFFS,
                      rvec, tvec, MARKER_SIZE_M * 0.5)
    cv2.putText(disp, f'ID{ids.flatten()[idx]}  dist={dist_m:.3f}m  [{tag}]',
                (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    if not qual_ok:
        return None, None, disp, tag
    return R, t, disp, 'OK'


# ── 메인 ───────────────────────────────────────────────────────────────────────
def main() -> None:
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        log.error('/dev/video%d 열기 실패', DEVICE)
        sys.exit(1)
    log.info('C270 연결 완료 — %dx%d', WIDTH, HEIGHT)

    R_g2b_list, t_g2b_list = [], []
    R_t2c_list, t_t2c_list = [], []

    print('\n=== C270 Eye-in-Hand Calibration 데이터 수집 ===')
    print(f'마커: ID{ARUCO_TARGET_ID}  {MARKER_SIZE_M*100:.0f}cm  |  목표: {MIN_SAMPLES}쌍 이상')
    print('① 직접교시로 로봇 이동  ② 카메라 창 초록 OK 확인  ③ ENTER → DART TCP 입력')
    print('ENTER=저장시작  D=마지막삭제  Q=종료\n')

    last_R_t2c = last_t_t2c = None
    waiting_input = False

    while True:
        ret, frame = cap.read()
        if not ret:
            log.error('프레임 읽기 실패')
            break

        R_t2c, t_t2c, disp, qual = detect(frame)
        if R_t2c is not None:
            last_R_t2c, last_t_t2c = R_t2c, t_t2c

        n = len(R_g2b_list)
        hint = 'ENTER:저장  D:삭제  Q:종료' if not waiting_input else 'TCP 입력 중...'
        cv2.putText(disp, f'saved={n}  {hint}',
                    (10, HEIGHT - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imshow('C270 Eye-in-Hand Collect', disp)

        key = cv2.waitKey(30) & 0xFF

        if key == 13:  # ENTER
            if last_R_t2c is None:
                log.warning('마커 미감지 — 초록 OK 뜰 때 ENTER 누르세요')
                continue
            cap_R, cap_t = last_R_t2c.copy(), last_t_t2c.copy()
            cv2.destroyWindow('C270 Eye-in-Hand Collect')
            tcp_str = input(f'  DART TCP 입력 (X Y Z Rx Ry Rz, mm/deg): ').strip()
            if not tcp_str:
                log.warning('입력 취소')
                cv2.namedWindow('C270 Eye-in-Hand Collect')
                continue
            result = parse_dart_tcp(tcp_str)
            if result is None:
                cv2.namedWindow('C270 Eye-in-Hand Collect')
                continue
            R_g2b, t_g2b = result
            R_g2b_list.append(R_g2b); t_g2b_list.append(t_g2b)
            R_t2c_list.append(cap_R); t_t2c_list.append(cap_t)
            log.info('[%d] 저장  TCP=[%.3f, %.3f, %.3f]m', n + 1, t_g2b[0], t_g2b[1], t_g2b[2])
            cv2.namedWindow('C270 Eye-in-Hand Collect')

        elif key == ord('d') and R_g2b_list:
            R_g2b_list.pop(); t_g2b_list.pop()
            R_t2c_list.pop(); t_t2c_list.pop()
            log.info('마지막 삭제 — 남은 %d쌍', len(R_g2b_list))

        elif key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()

    n = len(R_g2b_list)
    print(f'\n수집: {n}쌍')
    if n == 0:
        print('데이터 없음.')
        return
    if n < MIN_SAMPLES:
        log.warning('%d쌍 수집 — 최소 %d쌍 권장', n, MIN_SAMPLES)

    np.savez(OUTPUT_PATH,
             R_gripper2base=np.stack(R_g2b_list),
             t_gripper2base=np.stack(t_g2b_list),
             R_target2cam  =np.stack(R_t2c_list),
             t_target2cam  =np.stack(t_t2c_list))
    log.info('저장: %s', OUTPUT_PATH)
    print('다음: python3 scripts/compute_handeye_opencv.py --input c270_handeye_data.npz')


if __name__ == '__main__':
    main()
