#!/usr/bin/env python3
"""
c270_handeye_collect.py — C270 그리퍼 카메라 eye-in-hand 캘리브레이션 데이터 수집

타겟: ChArUco 보드 (5×7, 38mm 칸, 31.5mm 마커, DICT_4X4_50)

사용 방법 (TW 파일):
  python3 scripts/c270_handeye_collect.py --tw ~/Downloads/gripper아크로.tw
  DART 스텝 모드로 포즈 하나씩 이동 → 완전 정지 → 초록 OK → ENTER

사용 방법 (수동 입력):
  python3 scripts/c270_handeye_collect.py
  직접교시로 로봇 이동 → 초록 OK → ENTER → DART TCP 입력

중단 후 재시작 시 자동으로 이어받기 여부를 물어봄.
결과: scripts/c270_handeye_data.npz → compute_handeye_opencv.py 에서 사용
"""
import argparse
import base64
import json
import logging
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[c270_handeye] %(message)s')
log = logging.getLogger(__name__)

_SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR  = os.path.join(_SCRIPT_DIR, '..', 'config')
OUTPUT_PATH  = os.path.join(_SCRIPT_DIR, 'c270_handeye_data.npz')
RESUME_PATH  = os.path.join(_SCRIPT_DIR, 'c270_handeye_data_tmp.npz')  # 자동 백업

DEVICE        = 8
WIDTH, HEIGHT = 640, 480
MIN_SAMPLES   = 15

# ── 설정 로드 ──────────────────────────────────────────────────────────────────
def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)

_cam   = _load_yaml(os.path.join(_CONFIG_DIR, 'c270_camera_info.yaml'))['intrinsics']
_rt    = _load_yaml(os.path.join(_CONFIG_DIR, 'runtime.yaml'))
_calib = _rt.get('calibration', {})

CAMERA_MATRIX = np.array([
    [_cam['fx'], 0.0,        _cam['cx']],
    [0.0,        _cam['fy'], _cam['cy']],
    [0.0,        0.0,        1.0       ],
], dtype=np.float64)
DIST_COEFFS = np.array(_cam['coeffs'], dtype=np.float64)

SQUARES_X      : int   = int(_calib.get('charuco_squares_x',    5))
SQUARES_Y      : int   = int(_calib.get('charuco_squares_y',    7))
SQUARE_SIZE_M  : float = float(_calib.get('charuco_square_size_m',  0.038))
MARKER_SIZE_M  : float = float(_calib.get('charuco_marker_size_m',  0.0315))
ARUCO_DICT_ID  : int   = int(_calib.get('aruco_dict_id',        cv2.aruco.DICT_4X4_50))
MIN_CORNERS    : int   = int(_calib.get('min_charuco_corners',   6))
BASE_FRAME     : str   = str(_calib.get('base_frame',   'base_link'))
GRIPPER_FRAME  : str   = str(_calib.get('gripper_frame', 'link_6'))

MAX_FACE_ANGLE_DEG = 45.0
MIN_DIST_M         = 0.10
MAX_DIST_M         = 0.80

_ARUCO_DICT   = cv2.aruco.Dictionary_get(ARUCO_DICT_ID)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()
CHARUCO_BOARD = cv2.aruco.CharucoBoard_create(
    SQUARES_X, SQUARES_Y, SQUARE_SIZE_M, MARKER_SIZE_M, _ARUCO_DICT)


# ── 자동 저장 / 이어받기 ────────────────────────────────────────────────────────
def autosave(R_g2b: list, t_g2b: list, R_t2c: list, t_t2c: list,
             pose_idx: int) -> None:
    arrays = dict(
        R_gripper2base=np.stack(R_g2b),
        t_gripper2base=np.stack(t_g2b),
        R_target2cam  =np.stack(R_t2c),
        t_target2cam  =np.stack(t_t2c),
    )
    np.savez(RESUME_PATH, **arrays, pose_idx=np.array(pose_idx))
    # 기존 OUTPUT_PATH보다 적은 수로 덮어쓰지 않음 (세션 초기화로 인한 데이터 손실 방지)
    if os.path.exists(OUTPUT_PATH):
        try:
            n_existing = len(np.load(OUTPUT_PATH)['R_gripper2base'])
            if len(R_g2b) < n_existing:
                return
        except Exception:
            pass
    np.savez(OUTPUT_PATH, **arrays)


def load_resume() -> tuple[list, list, list, list, int] | None:
    # tmp 파일 우선, 없으면 최종 출력 파일 확인
    if os.path.exists(RESUME_PATH):
        path, has_idx = RESUME_PATH, True
    elif os.path.exists(OUTPUT_PATH):
        path, has_idx = OUTPUT_PATH, False
    else:
        return None
    d = np.load(path)
    n = len(d['R_gripper2base'])
    if n == 0:
        return None
    # pose_idx: tmp 파일엔 저장돼 있고, 출력 파일엔 없으므로 샘플 수로 추정
    pose_idx = int(d['pose_idx']) if (has_idx and 'pose_idx' in d) else n
    print(f'\n[이어받기] 기존 데이터 발견 — {n}쌍 저장됨 (다음 TW 포즈: {pose_idx + 1}번)')
    ans = input('이어서 진행할까요? [Y/n]: ').strip().lower()
    if ans in ('', 'y'):
        return (list(d['R_gripper2base']), list(d['t_gripper2base']),
                list(d['R_target2cam']),   list(d['t_target2cam']),
                pose_idx)
    return None  # 새로 시작


# ── TW 파일 파서 ───────────────────────────────────────────────────────────────
def _extract_poses_from_node(node: object, results: list) -> None:
    """MoveLNode(Cartesian TCP)만 추출 — MoveJNode는 관절각이므로 TCP 계산에 사용 불가."""
    if isinstance(node, dict):
        if node.get('_type') == 'MoveLNode':
            p    = node['_pojo']
            pose = p.get('pose', {})
            results.append({
                'ann': p.get('annotation', ''),
                'X': float(pose['pose1']), 'Y': float(pose['pose2']),
                'Z': float(pose['pose3']), 'A': float(pose['pose4']),
                'B': float(pose['pose5']), 'C': float(pose['pose6']),
            })
        for v in node.values():
            _extract_poses_from_node(v, results)
    elif isinstance(node, list):
        for item in node:
            _extract_poses_from_node(item, results)


def load_tw_poses(tw_path: str) -> list[dict]:
    with open(tw_path, 'rb') as f:
        data = base64.b64decode(f.read())
    poses: list[dict] = []
    _extract_poses_from_node(json.loads(data), poses)
    return poses


# ── DART TCP → 회전행렬 변환 ───────────────────────────────────────────────────
def dart_tcp_to_Rt(x_mm: float, y_mm: float, z_mm: float,
                   a_deg: float, b_deg: float, c_deg: float
                   ) -> tuple[np.ndarray, np.ndarray]:
    t = np.array([x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0], dtype=np.float64)
    R = Rotation.from_euler('ZYZ', [a_deg, b_deg, c_deg], degrees=True).as_matrix()
    return R, t


def parse_dart_tcp(line: str) -> tuple[np.ndarray, np.ndarray] | None:
    try:
        vals = [float(v) for v in line.strip().split()]
        if len(vals) != 6:
            raise ValueError(f'값 6개 필요 (입력: {len(vals)}개)')
    except ValueError as e:
        log.error('입력 오류: %s', e)
        return None
    return dart_tcp_to_Rt(*vals)


# ── ChArUco 감지 ───────────────────────────────────────────────────────────────
def detect(frame: np.ndarray):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS)

    disp = frame.copy()

    if ids is None or len(ids) < 1:
        cv2.putText(disp, 'NO MARKER', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return None, None, disp, ''

    cv2.aruco.drawDetectedMarkers(disp, corners, ids)

    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, CHARUCO_BOARD, CAMERA_MATRIX, DIST_COEFFS)

    if retval < MIN_CORNERS:
        cv2.putText(disp, f'corners={retval} (min {MIN_CORNERS})', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 165, 255), 2)
        return None, None, disp, f'corners={retval}'

    rvec_init = np.zeros((3, 1), dtype=np.float64)
    tvec_init = np.zeros((3, 1), dtype=np.float64)
    valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, CHARUCO_BOARD,
        CAMERA_MATRIX, DIST_COEFFS, rvec_init, tvec_init)

    if not valid:
        cv2.putText(disp, 'POSE FAIL', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return None, None, disp, 'POSE_FAIL'

    R, _ = cv2.Rodrigues(rvec)
    t    = tvec.flatten()
    dist_m = float(np.linalg.norm(t))
    cos_a  = abs(float(R[2, 2]))
    angle  = float(np.degrees(np.arccos(np.clip(cos_a, 0.0, 1.0))))

    qual_ok = (angle <= MAX_FACE_ANGLE_DEG and MIN_DIST_M <= dist_m <= MAX_DIST_M)
    color   = (0, 255, 0) if qual_ok else (0, 165, 255)
    tag     = 'OK' if qual_ok else f'BAD ang={angle:.0f}° dist={dist_m:.2f}m'

    cv2.drawFrameAxes(disp, CAMERA_MATRIX, DIST_COEFFS,
                      rvec, tvec, SQUARE_SIZE_M)
    cv2.putText(disp,
                f'corners={retval}  dist={dist_m:.3f}m  ang={angle:.1f}deg  [{tag}]',
                (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)

    if not qual_ok:
        return None, None, disp, tag
    return R, t, disp, 'OK'


# ── 메인 ───────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--tw', default=None,
                        help='DART TW 파일 경로. 지정 시 TCP 자동 로드.')
    parser.add_argument('--offset', type=int, default=0,
                        help='저장 순번 오프셋 (예: --offset 23 → 첫 저장이 [24]로 표시)')
    args = parser.parse_args()

    tw_poses: list[dict] = []
    tw_mode = False
    if args.tw:
        tw_path = os.path.expanduser(args.tw)
        if not os.path.exists(tw_path):
            log.error('TW 파일 없음: %s', tw_path)
            sys.exit(1)
        tw_poses = load_tw_poses(tw_path)
        tw_mode  = True
        log.info('TW 파일 로드 — %d개 포즈: %s', len(tw_poses), tw_path)

    # 이어받기 시도
    R_g2b_list: list = []
    t_g2b_list: list = []
    R_t2c_list: list = []
    t_t2c_list: list = []
    pose_idx = 0

    resumed = load_resume()
    if resumed is not None:
        R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, pose_idx = resumed
        log.info('이어받기 완료 — %d쌍, 다음 TW 포즈: %d번', len(R_g2b_list), pose_idx + 1)

    def _save_and_exit(sig=None, frame=None) -> None:
        n = len(R_g2b_list)
        if n > 0:
            autosave(R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, pose_idx)
            log.info('인터럽트 감지 — %d쌍 저장: %s', n, OUTPUT_PATH)
        cap.release()
        cv2.destroyAllWindows()
        sys.exit(0)

    import signal
    signal.signal(signal.SIGINT,  _save_and_exit)
    signal.signal(signal.SIGTERM, _save_and_exit)

    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        log.error('/dev/video%d 열기 실패', DEVICE)
        sys.exit(1)
    log.info('C270 연결 완료 — %dx%d', WIDTH, HEIGHT)

    print('\n=== C270 Eye-in-Hand Calibration 데이터 수집 ===')
    print(f'타겟: ChArUco {SQUARES_X}×{SQUARES_Y}  칸 {SQUARE_SIZE_M*1000:.0f}mm  '
          f'마커 {MARKER_SIZE_M*1000:.1f}mm  |  목표: {MIN_SAMPLES}쌍 이상')
    if tw_mode:
        print(f'모드: TW 파일 ({len(tw_poses)}개 포즈) — 스텝 모드, 완전 정지 확인 후 ENTER')
    else:
        print('모드: 수동 — 직접교시 이동 → 초록 OK → ENTER → TCP 입력')
    print('ENTER=저장  D=마지막삭제  Q=종료\n')

    last_R_t2c = last_t_t2c = None

    while True:
        ret, frame = cap.read()
        if not ret:
            log.error('프레임 읽기 실패')
            break

        R_t2c, t_t2c, disp, qual = detect(frame)
        if R_t2c is not None:
            last_R_t2c, last_t_t2c = R_t2c, t_t2c

        n = len(R_g2b_list)

        if tw_mode and pose_idx < len(tw_poses):
            p = tw_poses[pose_idx]
            cv2.putText(disp,
                        f'Pose {pose_idx+1}/{len(tw_poses)}  ann={p["ann"]}  saved={n + args.offset}',
                        (10, HEIGHT - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)
            cv2.putText(disp,
                        f'X={p["X"]:.1f} Y={p["Y"]:.1f} Z={p["Z"]:.1f}',
                        (10, HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        else:
            cv2.putText(disp, f'saved={n}  ENTER:저장  D:삭제  Q:종료',
                        (10, HEIGHT - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        cv2.imshow('C270 Eye-in-Hand Collect', disp)
        key = cv2.waitKey(30) & 0xFF

        if key == 13:  # ENTER
            if last_R_t2c is None:
                log.warning('보드 미감지 — 초록 OK 뜰 때 ENTER 누르세요')
                continue

            cap_R, cap_t = last_R_t2c.copy(), last_t_t2c.copy()

            if tw_mode:
                if pose_idx >= len(tw_poses):
                    log.warning('TW 포즈 모두 사용됨 (총 %d개)', len(tw_poses))
                    continue
                p = tw_poses[pose_idx]
                R_g2b, t_g2b = dart_tcp_to_Rt(p['X'], p['Y'], p['Z'],
                                               p['A'], p['B'], p['C'])
                log.info('[%d] 저장  포즈%s  TCP=[%.3f, %.3f, %.3f]m',
                         n + 1 + args.offset, p['ann'], t_g2b[0], t_g2b[1], t_g2b[2])
                pose_idx += 1
            else:
                cv2.destroyWindow('C270 Eye-in-Hand Collect')
                tcp_str = input('  DART TCP 입력 (X Y Z A B C, mm/deg): ').strip()
                cv2.namedWindow('C270 Eye-in-Hand Collect')
                if not tcp_str:
                    log.warning('입력 취소')
                    continue
                result = parse_dart_tcp(tcp_str)
                if result is None:
                    continue
                R_g2b, t_g2b = result
                log.info('[%d] 저장  TCP=[%.3f, %.3f, %.3f]m',
                         n + 1 + args.offset, t_g2b[0], t_g2b[1], t_g2b[2])

            R_g2b_list.append(R_g2b); t_g2b_list.append(t_g2b)
            R_t2c_list.append(cap_R); t_t2c_list.append(cap_t)

            # 저장마다 자동 백업
            autosave(R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, pose_idx)

        elif key == ord('d') and R_g2b_list:
            R_g2b_list.pop(); t_g2b_list.pop()
            R_t2c_list.pop(); t_t2c_list.pop()
            if tw_mode and pose_idx > 0:
                pose_idx -= 1
            log.info('마지막 삭제 — 남은 %d쌍', len(R_g2b_list))
            if R_g2b_list:
                autosave(R_g2b_list, t_g2b_list, R_t2c_list, t_t2c_list, pose_idx)

        elif key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()

    n = len(R_g2b_list)
    print(f'\n수집: {n}쌍')
    if n == 0:
        print('데이터 없음.')
        if os.path.exists(RESUME_PATH):
            os.remove(RESUME_PATH)
        return
    if n < MIN_SAMPLES:
        log.warning('%d쌍 수집 — 최소 %d쌍 권장', n, MIN_SAMPLES)

    # OUTPUT_PATH는 autosave에서 이미 매 포즈마다 갱신됨
    log.info('완료 — %d쌍: %s', n, OUTPUT_PATH)

    if os.path.exists(RESUME_PATH):
        os.remove(RESUME_PATH)

    print('다음: python3 scripts/compute_handeye_opencv.py')


if __name__ == '__main__':
    main()
