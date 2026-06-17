#!/usr/bin/env python3
"""
augment_handeye_data.py — eye-in-hand 캘리브레이션 데이터 합성 증폭

기존 23쌍 실측 데이터를 기반으로 소폭 변형된 합성 포즈를 생성한다.
생성 조건: 각 합성 포즈에서 ChArUco 코너가 MIN_VISIBLE_CORNERS 이상 투영될 것.

정중앙 TCP 좌표(보드 중심 기준점)는 보드 위치 역산 결과의 sanity check에 사용.

사용법:
  python3 scripts/augment_handeye_data.py
  python3 scripts/augment_handeye_data.py --perturb 8 --out scripts/c270_handeye_data_aug.npz
"""
import argparse
import logging
import os

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[augment] %(message)s')
log = logging.getLogger(__name__)

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR = os.path.join(_SCRIPT_DIR, '..', 'config')

INPUT_PATH  = os.path.join(_SCRIPT_DIR, 'c270_handeye_data.npz')
DEFAULT_OUT = os.path.join(_SCRIPT_DIR, 'c270_handeye_data_aug.npz')

# ── 카메라 설정 로드 ──────────────────────────────────────────────────────────
def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)

_cam = _load_yaml(os.path.join(_CONFIG_DIR, 'c270_camera_info.yaml'))['intrinsics']
_rt  = _load_yaml(os.path.join(_CONFIG_DIR, 'runtime.yaml'))
_cal = _rt.get('calibration', {})

K = np.array([
    [_cam['fx'], 0.0,        _cam['cx']],
    [0.0,        _cam['fy'], _cam['cy']],
    [0.0,        0.0,        1.0       ],
], dtype=np.float64)
IMG_W, IMG_H = 640, 480

# ── ChArUco 보드 코너 모델 ─────────────────────────────────────────────────────
SQUARES_X   = int(_cal.get('charuco_squares_x',   5))
SQUARES_Y   = int(_cal.get('charuco_squares_y',   7))
SQUARE_SIZE = float(_cal.get('charuco_square_size_m', 0.038))

# 내부 코너 (squaresX-1) × (squaresY-1) = 4×6 = 24개, 보드 로컬 프레임 [m]
_CORNERS_BOARD = np.array(
    [[j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0]
     for i in range(1, SQUARES_Y)
     for j in range(1, SQUARES_X)],
    dtype=np.float64,
)  # (24, 3)

# 보드 중심 (full board 기준)
BOARD_CENTER_LOCAL = np.array([
    SQUARES_X * SQUARE_SIZE / 2,
    SQUARES_Y * SQUARE_SIZE / 2,
    0.0,
], dtype=np.float64)

# ── 정중앙 TCP 기준점 (보드 sanity check) ────────────────────────────────────
# DART MoveL 값: X=577.26mm Y=70.16mm Z=-30.00mm A=43.76° B=-180.0° C=43.76°
# 로봇 TCP가 보드 정중앙에 위치할 때의 base frame 좌표
CENTER_TCP_BASE_M = np.array([0.57726, 0.07016, -0.03000])

# ── 가시성 필터 파라미터 ──────────────────────────────────────────────────────
MIN_VISIBLE_CORNERS = 6
MAX_FACE_ANGLE_DEG  = 45.0
MIN_DIST_M          = 0.10
MAX_DIST_M          = 0.80

# ── 증폭 파라미터 ─────────────────────────────────────────────────────────────
ROT_PERTURB_DEG  = 12.0   # 최대 회전 섭동 [°]
TRANS_PERTURB_M  = 0.025  # 최대 이동 섭동 [m]  (±25mm)
MAX_ATTEMPTS     = 50     # 포즈당 시도 횟수 상한


def _mat4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t.flatten()
    return T


def _inv4(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3,  3] = -R.T @ t
    return Ti


def _is_visible(R_t2c: np.ndarray, t_t2c: np.ndarray) -> bool:
    """합성 포즈에서 ChArUco 보드 가시성 판별."""
    dist = float(np.linalg.norm(t_t2c))
    if not (MIN_DIST_M <= dist <= MAX_DIST_M):
        return False

    # 보드 법선(Z축)과 카메라 Z축 사이 각도 (정면성 체크)
    cos_a = abs(float(R_t2c[2, 2]))
    angle = float(np.degrees(np.arccos(np.clip(cos_a, 0.0, 1.0))))
    if angle > MAX_FACE_ANGLE_DEG:
        return False

    # 코너를 카메라 프레임으로 변환 후 투영
    pts_cam = (R_t2c @ _CORNERS_BOARD.T + t_t2c.reshape(3, 1)).T  # (24, 3)
    in_front = pts_cam[:, 2] > 0.0
    if in_front.sum() < MIN_VISIBLE_CORNERS:
        return False

    uvw = (K @ pts_cam.T).T                          # (24, 3)
    uv  = uvw[:, :2] / uvw[:, 2:3]                  # (24, 2)
    in_img = (
        (uv[:, 0] >= 0) & (uv[:, 0] < IMG_W) &
        (uv[:, 1] >= 0) & (uv[:, 1] < IMG_H) &
        in_front
    )
    return int(in_img.sum()) >= MIN_VISIBLE_CORNERS


def _calibrate_initial(R_g2b, t_g2b, R_t2c, t_t2c) -> tuple[np.ndarray, np.ndarray]:
    """기존 데이터로 초기 T_cam2gripper 추정 (증폭 계산용)."""
    R_c2g, t_c2g = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    return R_c2g, t_c2g.flatten()


def _verify_board_position(R_g2b, t_g2b, R_t2c, t_t2c,
                           R_c2g, t_c2g) -> None:
    """정중앙 TCP 기준점과 보드 위치 역산 결과 비교 (sanity check)."""
    board_positions = []
    T_c2g_mat = _mat4(R_c2g, t_c2g)
    for i in range(len(R_g2b)):
        T_g2b_mat = _mat4(R_g2b[i], t_g2b[i])
        T_c2b_mat = T_g2b_mat @ T_c2g_mat
        T_t2c_mat = _mat4(R_t2c[i], t_t2c[i])
        T_t2b_mat = T_c2b_mat @ T_t2c_mat
        # 보드 로컬 중심 → base frame
        center_base = T_t2b_mat[:3, :3] @ BOARD_CENTER_LOCAL + T_t2b_mat[:3, 3]
        board_positions.append(center_base)

    board_mean = np.mean(board_positions, axis=0)
    board_std  = np.std(board_positions, axis=0)
    err = np.linalg.norm(board_mean - CENTER_TCP_BASE_M) * 1000

    log.info('보드 중심 추정 (base): [%.1f, %.1f, %.1f] mm  (std: %.1f mm)',
             *board_mean * 1000, np.linalg.norm(board_std) * 1000)
    log.info('정중앙 TCP 기준점:     [%.1f, %.1f, %.1f] mm',
             *CENTER_TCP_BASE_M * 1000)
    log.info('기준점 대비 오차:       %.1f mm', err)
    if err > 100:
        log.warning('오차 %.1fmm — T_cam2gripper 초기값이 부정확할 수 있음. '
                    '증폭 결과 품질 저하 가능.', err)


def augment(n_perturb: int, seed: int = 42) -> tuple[list, list, list, list]:
    rng = np.random.default_rng(seed)

    data = np.load(INPUT_PATH)
    R_g2b = list(data['R_gripper2base'])
    t_g2b = list(data['t_gripper2base'])
    R_t2c = list(data['R_target2cam'])
    t_t2c = list(data['t_target2cam'])
    n_orig = len(R_g2b)
    log.info('원본 데이터: %d쌍', n_orig)

    # 초기 T_cam2gripper 추정
    R_c2g, t_c2g = _calibrate_initial(
        np.stack(R_g2b), t_g2b,
        np.stack(R_t2c), t_t2c,
    )
    log.info('초기 T_cam2gripper 추정 완료')
    _verify_board_position(R_g2b, t_g2b, R_t2c, t_t2c, R_c2g, t_c2g)

    T_c2g = _mat4(R_c2g, t_c2g)
    T_g2c = _inv4(T_c2g)  # gripper → cam

    aug_R_g2b, aug_t_g2b = [], []
    aug_R_t2c, aug_t_t2c = [], []

    for i in range(n_orig):
        T_g2b_i = _mat4(R_g2b[i], t_g2b[i])
        T_t2c_i = _mat4(R_t2c[i], t_t2c[i])

        # 보드 위치 (base frame): T_board2base = T_cam2base @ T_board2cam
        T_c2b_i = T_g2b_i @ T_c2g
        T_t2b_i = T_c2b_i @ T_t2c_i

        accepted = 0
        for _ in range(MAX_ATTEMPTS * n_perturb):
            if accepted >= n_perturb:
                break

            # 소폭 회전 섭동 (카메라 프레임 기준)
            angle = rng.uniform(0, np.radians(ROT_PERTURB_DEG))
            axis  = rng.standard_normal(3)
            axis /= np.linalg.norm(axis)
            R_perturb = Rotation.from_rotvec(angle * axis).as_matrix()

            # 소폭 이동 섭동 (카메라 프레임 기준)
            dt = rng.uniform(-TRANS_PERTURB_M, TRANS_PERTURB_M, 3)

            # 새 카메라 pose (base frame)
            T_c2b_new = T_c2b_i.copy()
            T_c2b_new[:3, :3] = R_perturb @ T_c2b_i[:3, :3]
            T_c2b_new[:3,  3] = T_c2b_i[:3, 3] + T_c2b_i[:3, :3] @ dt

            # 새 board→camera 변환
            T_t2c_new = _inv4(T_c2b_new) @ T_t2b_i
            R_t2c_new = T_t2c_new[:3, :3]
            t_t2c_new = T_t2c_new[:3,  3]

            # 가시성 필터
            if not _is_visible(R_t2c_new, t_t2c_new):
                continue

            # 새 gripper pose (base frame)
            T_g2b_new = T_c2b_new @ T_g2c
            aug_R_g2b.append(T_g2b_new[:3, :3])
            aug_t_g2b.append(T_g2b_new[:3,  3])
            aug_R_t2c.append(R_t2c_new)
            aug_t_t2c.append(t_t2c_new)
            accepted += 1

        if accepted < n_perturb:
            log.warning('포즈 %d: %d/%d 합성 성공 (가시성 조건 미충족)',
                        i + 1, accepted, n_perturb)

    log.info('합성 데이터: %d쌍', len(aug_R_g2b))
    return aug_R_g2b, aug_t_g2b, aug_R_t2c, aug_t_t2c


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--perturb', type=int, default=5,
                        help='포즈당 합성 데이터 수 (기본 5)')
    parser.add_argument('--out', default=DEFAULT_OUT,
                        help='출력 npz 경로')
    parser.add_argument('--aug-only', action='store_true',
                        help='원본 제외, 합성 데이터만 저장')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    aug_R_g2b, aug_t_g2b, aug_R_t2c, aug_t_t2c = augment(args.perturb, args.seed)

    if not aug_R_g2b:
        log.error('합성 데이터 없음 — 종료')
        return

    if not args.aug_only:
        orig = np.load(INPUT_PATH)
        all_R_g2b = np.concatenate([orig['R_gripper2base'], np.stack(aug_R_g2b)])
        all_t_g2b = np.concatenate([orig['t_gripper2base'], np.stack(aug_t_g2b)])
        all_R_t2c = np.concatenate([orig['R_target2cam'],   np.stack(aug_R_t2c)])
        all_t_t2c = np.concatenate([orig['t_target2cam'],   np.stack(aug_t_t2c)])
    else:
        all_R_g2b = np.stack(aug_R_g2b)
        all_t_g2b = np.stack(aug_t_g2b)
        all_R_t2c = np.stack(aug_R_t2c)
        all_t_t2c = np.stack(aug_t_t2c)

    np.savez(args.out,
             R_gripper2base=all_R_g2b,
             t_gripper2base=all_t_g2b,
             R_target2cam  =all_R_t2c,
             t_target2cam  =all_t_t2c)

    log.info('저장: %s  (총 %d쌍 = 원본 %d + 합성 %d)',
             args.out,
             len(all_R_g2b),
             0 if args.aug_only else len(np.load(INPUT_PATH)['R_gripper2base']),
             len(aug_R_g2b))
    print(f'\n다음: python3 scripts/compute_handeye_opencv.py '
          f'--input {args.out} --all --no-filter')


if __name__ == '__main__':
    main()
