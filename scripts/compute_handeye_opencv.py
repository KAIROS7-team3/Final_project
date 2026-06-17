#!/usr/bin/env python3
"""
compute_handeye_opencv.py — cv2.calibrateHandEye로 카메라↔로봇 변환 행렬 계산

입력: scripts/handeye_data.npz  (collect_handeye_data.py 출력)
출력: config/hand_eye.yaml

검증 방식: AX=XB 잔차 (A=상대 그리퍼 모션, B=상대 마커 모션, X=T_cam2base)
          잔차가 작을수록 데이터와 결과가 일치함.

품질 필터 (자동 적용):
  - 마커 기울기 <= MAX_FACE_ANGLE_DEG (카메라 정면 기준)
  - 마커 거리 MIN_DIST_M ~ MAX_DIST_M

사용 방법:
  python3 scripts/compute_handeye_opencv.py
  python3 scripts/compute_handeye_opencv.py --method PARK
  python3 scripts/compute_handeye_opencv.py --all
  python3 scripts/compute_handeye_opencv.py --no-filter   # 필터 없이 전체 사용
"""
import argparse
import datetime
import logging
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger = logging.getLogger('compute_handeye')

# ── 경로 ──────────────────────────────────────────────────────────────────────
_SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_DATA  = os.path.join(_SCRIPT_DIR, 'c270_handeye_data.npz')
_CONFIG_PATH   = os.path.join(_SCRIPT_DIR, '..', 'config', 'c270_hand_eye.yaml')
_RUNTIME_PATH = os.path.join(_SCRIPT_DIR, '..', 'config', 'runtime.yaml')

# ArUco 품질 필터 기준값
MAX_FACE_ANGLE_DEG = 22.0   # 마커가 카메라를 향한 각도 최대값
MIN_DIST_M         = 0.30   # 카메라-마커 최소 거리
MAX_DIST_M         = 0.85   # 카메라-마커 최대 거리

# ── 알고리즘 목록 ──────────────────────────────────────────────────────────────
METHODS: dict[str, int] = {
    'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
    'PARK':       cv2.CALIB_HAND_EYE_PARK,
    'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


# ── 헬퍼 ──────────────────────────────────────────────────────────────────────
def _make_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _inv_T(T: np.ndarray) -> np.ndarray:
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def filter_samples(
    R_g2b: list[np.ndarray],
    t_g2b: list[np.ndarray],
    R_t2c: list[np.ndarray],
    t_t2c: list[np.ndarray],
) -> tuple[list, list, list, list, list[int]]:
    """기울기·거리 기준으로 품질 낮은 샘플 제거. 남은 인덱스 목록도 반환."""
    good_idx: list[int] = []
    for i in range(len(R_t2c)):
        cos_a = abs(float(R_t2c[i][2, 2]))
        angle = float(np.degrees(np.arccos(np.clip(cos_a, 0.0, 1.0))))
        dist = float(t_t2c[i][2])
        if angle <= MAX_FACE_ANGLE_DEG and MIN_DIST_M <= dist <= MAX_DIST_M:
            good_idx.append(i)

    return (
        [R_g2b[i] for i in good_idx],
        [t_g2b[i] for i in good_idx],
        [R_t2c[i] for i in good_idx],
        [t_t2c[i] for i in good_idx],
        good_idx,
    )


def invert_transforms(
    R_list: list[np.ndarray],
    t_list: list[np.ndarray],
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """T_gripper2base → T_base2gripper  (eye-to-hand 입력용)."""
    return [R.T for R in R_list], [-R.T @ t for R, t in zip(R_list, t_list)]


def validate_axb(
    R_cam2gripper: np.ndarray,
    t_cam2gripper: np.ndarray,
    R_g2b: list[np.ndarray],
    t_g2b: list[np.ndarray],
    R_t2c: list[np.ndarray],
    t_t2c: list[np.ndarray],
) -> tuple[float, float, np.ndarray]:
    """
    AX=XB 잔차로 캘리브레이션 품질 평가 (eye-in-hand).
    A_ij = inv(T_g2b[i+1]) * T_g2b[i]  (그리퍼 상대 모션, base frame)
    B_ij = T_t2c[i+1] * inv(T_t2c[i])  (마커 상대 모션, camera frame)
    X    = T_cam2gripper
    residual = ||A*X - X*B||_translation  [mm]
    """
    X = _make_T(R_cam2gripper, t_cam2gripper)
    errs: list[float] = []
    n = len(R_g2b)
    for i in range(n - 1):
        A = _inv_T(_make_T(R_g2b[i + 1], t_g2b[i + 1])) @ _make_T(R_g2b[i], t_g2b[i])
        B = _make_T(R_t2c[i + 1], t_t2c[i + 1]) @ _inv_T(_make_T(R_t2c[i], t_t2c[i]))
        errs.append(float(np.linalg.norm((A @ X - X @ B)[:3, 3]) * 1000.0))
    arr = np.array(errs)
    return float(arr.mean()), float(arr.max()), arr


def run_calibration(
    R_g2b: list[np.ndarray],
    t_g2b: list[np.ndarray],
    R_t2c: list[np.ndarray],
    t_t2c: list[np.ndarray],
    method_name: str,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """
    eye-in-hand calibrateHandEye 실행.
    입력: T_gripper2base (그대로), T_target2cam (그대로)
    반환: (R_cam2gripper, t_cam2gripper, mean_axb_mm, max_axb_mm)
    """
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_g2b, t_g2b,
        R_t2c, [t.reshape(3, 1) for t in t_t2c],
        method=METHODS[method_name],
    )
    t_cam2gripper = t_cam2gripper.flatten()
    mean_mm, max_mm, _ = validate_axb(R_cam2gripper, t_cam2gripper, R_g2b, t_g2b, R_t2c, t_t2c)
    return R_cam2gripper, t_cam2gripper, mean_mm, max_mm


def save_yaml(
    R: np.ndarray,
    t: np.ndarray,
    n_samples: int,
    method: str,
    mean_mm: float,
    max_mm: float,
) -> None:
    # calibrateHandEye가 det≈-1인 행렬을 반환하는 경우 SO(3)으로 투영
    U, _, Vt = np.linalg.svd(R)
    R_so3 = U @ Vt
    if np.linalg.det(R_so3) < 0:
        U[:, -1] *= -1
        R_so3 = U @ Vt
    quat = Rotation.from_matrix(R_so3).as_quat()  # [x, y, z, w]
    cam_tilt = float(np.degrees(np.arccos(np.clip(abs(float(R_so3[2, 2])), 0.0, 1.0))))
    data = {
        'schema_version': 1,
        'transformation': {
            'rotation': {
                'x': float(quat[0]),
                'y': float(quat[1]),
                'z': float(quat[2]),
                'w': float(quat[3]),
            },
            'translation': {
                'x': float(t[0]),
                'y': float(t[1]),
                'z': float(t[2]),
            },
        },
        'metadata': {
            'calibration_date': datetime.date.today().isoformat(),
            'sample_count': n_samples,
            'method': f'calibrateHandEye_{method}',
            'axb_mean_mm': mean_mm,
            'axb_max_mm': max_mm,
            'cam_tilt_deg': cam_tilt,
            'cam_height_m': float(t[2]),
            'operator': None,
            'tool': 'c270_handeye_collect.py + compute_handeye_opencv.py',
            'frames': {
                'from': 'c270_optical_frame',
                'to':   'link_6',
                'note': 'T_cam2gripper: camera origin expressed in link_6 frame',
            },
        },
    }
    class _IndentDumper(yaml.Dumper):
        def increase_indent(self, flow=False, indentless=False):
            return super().increase_indent(flow, indentless=False)

    with open(_CONFIG_PATH, 'w') as f:
        yaml.dump(data, f, Dumper=_IndentDumper, explicit_start=True,
                  default_flow_style=False, allow_unicode=True, sort_keys=False)
    logger.info('config/c270_hand_eye.yaml 저장 완료')


# ── 메인 ───────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default=_DEFAULT_DATA,
                        help='입력 npz 파일 경로 (기본: c270_handeye_data.npz)')
    parser.add_argument('--method', default='PARK', choices=list(METHODS),
                        help='사용할 알고리즘 (기본: PARK)')
    parser.add_argument('--all', action='store_true',
                        help='5가지 알고리즘 전부 실행 후 최적 선택')
    parser.add_argument('--no-filter', action='store_true',
                        help='품질 필터 비활성화 (전체 샘플 사용)')
    args = parser.parse_args()

    data_path = os.path.abspath(args.input)
    if not os.path.exists(data_path):
        logger.error('데이터 파일 없음: %s', data_path)
        logger.error('c270_handeye_collect.py 를 먼저 실행하세요.')
        sys.exit(1)

    data = np.load(data_path)
    R_g2b = list(data['R_gripper2base'])
    t_g2b = list(data['t_gripper2base'])
    R_t2c = list(data['R_target2cam'])
    t_t2c = list(data['t_target2cam'])
    n_total = len(R_g2b)
    logger.info('데이터 로드: %d쌍', n_total)

    # 품질 필터
    if args.no_filter:
        R_g2b_f, t_g2b_f, R_t2c_f, t_t2c_f = R_g2b, t_g2b, R_t2c, t_t2c
        logger.info('품질 필터 비활성화 — 전체 %d쌍 사용', n_total)
    else:
        R_g2b_f, t_g2b_f, R_t2c_f, t_t2c_f, good_idx = filter_samples(
            R_g2b, t_g2b, R_t2c, t_t2c)
        n_f = len(R_g2b_f)
        removed = [i + 1 for i in range(n_total) if i not in good_idx]
        logger.info(
            '품질 필터 (기울기≤%.0fdeg, 거리%.2f~%.2fm): %d/%d쌍 사용  — 제거: %s',
            MAX_FACE_ANGLE_DEG, MIN_DIST_M, MAX_DIST_M, n_f, n_total, removed,
        )
        if n_f < 8:
            logger.error('필터 후 샘플 %d개 — 최소 8개 필요. 재수집 권장.', n_f)
            sys.exit(1)

    n_used = len(R_g2b_f)
    if n_used < 3:
        logger.error('최소 3쌍 필요 (현재 %d쌍)', n_used)
        sys.exit(1)

    # 알고리즘 실행
    if args.all:
        print('\n=== 전체 알고리즘 비교 ===')
        results: dict[str, tuple] = {}
        for name in METHODS:
            try:
                R, t, mean_mm, max_mm = run_calibration(
                    R_g2b_f, t_g2b_f, R_t2c_f, t_t2c_f, name)
                cam_tilt = float(np.degrees(np.arccos(np.clip(abs(float(R[2, 2])), 0.0, 1.0))))
                results[name] = (R, t, mean_mm, max_mm)
                print(f'  {name:12s}  AXB평균 {mean_mm:7.1f}mm  최대 {max_mm:7.1f}mm'
                      f'  Z(link6기준) {t[2]:.3f}m  기울기 {cam_tilt:.1f}deg')
            except Exception as e:
                print(f'  {name:12s}  실패: {e}')

        if not results:
            logger.error('모든 알고리즘 실패')
            sys.exit(1)

        best = min(results, key=lambda k: results[k][2])
        print(f'\n최적 알고리즘: {best}  (AXB 평균 {results[best][2]:.1f}mm)')
        R_best, t_best, mean_best, max_best = results[best]
        method_used = best

    else:
        method_used = args.method
        print(f'\n알고리즘: {method_used}')
        try:
            R_best, t_best, mean_best, max_best = run_calibration(
                R_g2b_f, t_g2b_f, R_t2c_f, t_t2c_f, method_used)
        except Exception as e:
            logger.error('calibrateHandEye 실패: %s', e)
            sys.exit(1)

    # 결과 출력
    T = np.eye(4)
    T[:3, :3] = R_best
    T[:3, 3] = t_best
    cam_tilt = float(np.degrees(np.arccos(np.clip(abs(float(R_best[2, 2])), 0.0, 1.0))))
    print(f'\n=== 결과 T_cam2gripper (c270_optical_frame → link_6) ===')
    print(f'변환 행렬:\n{np.round(T, 5)}')
    print(f'카메라 원점 (link_6 기준): [{t_best[0]:.3f}, {t_best[1]:.3f}, {t_best[2]:.3f}] m')
    print(f'카메라 기울기 (link_6 Z축 기준): {cam_tilt:.1f}deg')
    print(f'AXB 평균 잔차: {mean_best:.1f}mm  /  최대: {max_best:.1f}mm')
    print(f'사용 샘플: {n_used}/{n_total}')

    # 수락 기준 판정
    rt: dict = {}
    try:
        with open(_RUNTIME_PATH) as f:
            rt = yaml.safe_load(f) or {}
    except Exception:
        pass

    if max_best <= 50.0:
        print('✓ AXB 잔차 양호 (최대 ≤50mm) — 물리 검증 진행 권장')
    elif max_best <= 150.0:
        print('△ AXB 잔차 보통 (≤150mm) — 재수집 또는 물리 검증으로 판단')
    else:
        print('✗ AXB 잔차 과다 (>150mm) — 데이터 품질 문제. 재수집 강력 권장')
        print('  원인 후보: 마커 곡면 부착 (평판 베이스 필요), 카메라 USB 흔들림')

    save_yaml(R_best, t_best, n_used, method_used, mean_best, max_best)

    print('\n물리 검증 방법:')
    print('  1. 카메라 화면에서 알려진 특징점(슬롯 모서리 등) 픽셀 좌표 기록')
    print('  2. config/hand_eye.yaml 로 base frame 좌표 변환')
    print('  3. 로봇을 해당 좌표로 이동 → TCP와 특징점 거리 측정')
    print('  기준: 실측 오차 ≤10mm → 결과 확정')


if __name__ == '__main__':
    main()
