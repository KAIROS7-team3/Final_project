#!/usr/bin/env python3
"""
compute_handeye_opencv.py — cv2.calibrateHandEye로 카메라↔로봇 변환 행렬 계산

입력: scripts/handeye_data.npz  (collect_handeye_data.py 출력)
출력: config/hand_eye.yaml

검증 방식: ArUco 마커는 작업 공간 바닥에 고정 → 모든 로봇 자세에서 마커의
          base 좌표계 위치는 동일해야 함. 위치 분산(std)을 오차로 사용.

사용 방법:
  python3 scripts/compute_handeye_opencv.py
  python3 scripts/compute_handeye_opencv.py --method DANIILIDIS   # 알고리즘 지정
  python3 scripts/compute_handeye_opencv.py --all                  # 5가지 전부 비교
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
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DATA_PATH   = os.path.join(_SCRIPT_DIR, 'handeye_data.npz')
_CONFIG_PATH = os.path.join(_SCRIPT_DIR, '..', 'config', 'hand_eye.yaml')
_RUNTIME_PATH = os.path.join(_SCRIPT_DIR, '..', 'config', 'runtime.yaml')

# ── 설정 로드 ──────────────────────────────────────────────────────────────────
def _load_yaml(path: str) -> dict:
    try:
        with open(path) as f:
            return yaml.safe_load(f)
    except FileNotFoundError:
        logger.error('설정 파일 없음: %s', path)
        sys.exit(1)
    except yaml.YAMLError as e:
        logger.error('YAML 파싱 실패 (%s): %s', path, e)
        sys.exit(1)


# ── 알고리즘 목록 ──────────────────────────────────────────────────────────────
METHODS: dict[str, int] = {
    'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
    'PARK':       cv2.CALIB_HAND_EYE_PARK,
    'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


# ── 헬퍼 ──────────────────────────────────────────────────────────────────────
def invert_transforms(
    R_list: list[np.ndarray],
    t_list: list[np.ndarray],
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """T_gripper2base → T_base2gripper  (eye-to-hand 입력용)."""
    R_inv = [R.T for R in R_list]
    t_inv = [-R.T @ t for R, t in zip(R_list, t_list)]
    return R_inv, t_inv


def validate(
    R_cam2base: np.ndarray,
    t_cam2base: np.ndarray,
    R_t2c_list: list[np.ndarray],
    t_t2c_list: list[np.ndarray],
) -> tuple[float, float, np.ndarray]:
    """
    검증: 마커는 바닥에 고정 → 모든 자세에서 base 좌표계 마커 위치가 동일해야 함.
    반환: (mean_err_mm, max_err_mm, 오차 배열)
    """
    positions = np.array([
        R_cam2base @ t + t_cam2base
        for t in t_t2c_list
    ])
    mean_pos = positions.mean(axis=0)
    errors_mm = np.linalg.norm(positions - mean_pos, axis=1) * 1000.0
    return float(errors_mm.mean()), float(errors_mm.max()), errors_mm


def run_calibration(
    R_g2b: list[np.ndarray],
    t_g2b: list[np.ndarray],
    R_t2c: list[np.ndarray],
    t_t2c: list[np.ndarray],
    method_name: str,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """
    eye-to-hand calibrateHandEye 실행.
    반환: (R_cam2base, t_cam2base, mean_err_mm, max_err_mm)
    """
    R_b2g, t_b2g = invert_transforms(R_g2b, t_g2b)

    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_b2g, t_b2g,
        R_t2c, [t.reshape(3, 1) for t in t_t2c],
        method=METHODS[method_name],
    )
    t_cam2base = t_cam2base.flatten()
    mean_mm, max_mm, _ = validate(R_cam2base, t_cam2base, R_t2c, t_t2c)
    return R_cam2base, t_cam2base, mean_mm, max_mm


def save_yaml(
    R: np.ndarray,
    t: np.ndarray,
    n_samples: int,
    method: str,
    mean_mm: float,
    max_mm: float,
) -> None:
    quat = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]
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
            'mean_error_mm': mean_mm,
            'max_error_mm': max_mm,
            'position_error_mm': mean_mm,
            'orientation_error_deg': None,
            'operator': None,
            'tool': 'collect_handeye_data.py + compute_handeye_opencv.py',
            'frames': {
                'from': 'camera_color_optical_frame',
                'to': 'base_link',
            },
        },
    }
    with open(_CONFIG_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    logger.info('config/hand_eye.yaml 저장 완료')


# ── 메인 ───────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--method', default='DANIILIDIS', choices=list(METHODS),
                        help='사용할 알고리즘 (기본: DANIILIDIS)')
    parser.add_argument('--all', action='store_true',
                        help='5가지 알고리즘 전부 실행 후 최적 선택')
    args = parser.parse_args()

    # 데이터 로드
    if not os.path.exists(_DATA_PATH):
        logger.error('데이터 파일 없음: %s', _DATA_PATH)
        logger.error('collect_handeye_data.py 를 먼저 실행하세요.')
        sys.exit(1)

    data = np.load(_DATA_PATH)
    R_g2b = list(data['R_gripper2base'])
    t_g2b = list(data['t_gripper2base'])
    R_t2c = list(data['R_target2cam'])
    t_t2c = list(data['t_target2cam'])
    n = len(R_g2b)
    logger.info('데이터 로드: %d쌍', n)

    if n < 3:
        logger.error('최소 3쌍 필요 (현재 %d쌍)', n)
        sys.exit(1)

    # 알고리즘 실행
    if args.all:
        print('\n=== 전체 알고리즘 비교 ===')
        results: dict[str, tuple] = {}
        for name in METHODS:
            try:
                R, t, mean_mm, max_mm = run_calibration(R_g2b, t_g2b, R_t2c, t_t2c, name)
                results[name] = (R, t, mean_mm, max_mm)
                print(f'  {name:12s}  평균 {mean_mm:6.2f} mm  최대 {max_mm:6.2f} mm')
            except Exception as e:
                print(f'  {name:12s}  실패: {e}')

        if not results:
            logger.error('모든 알고리즘 실패')
            sys.exit(1)

        best = min(results, key=lambda k: results[k][2])
        print(f'\n최적 알고리즘: {best}  (평균 오차 {results[best][2]:.2f} mm)')
        R_best, t_best, mean_best, max_best = results[best]
        method_used = best

    else:
        method_used = args.method
        print(f'\n알고리즘: {method_used}')
        try:
            R_best, t_best, mean_best, max_best = run_calibration(
                R_g2b, t_g2b, R_t2c, t_t2c, method_used
            )
        except Exception as e:
            logger.error('calibrateHandEye 실패: %s', e)
            sys.exit(1)

    # 결과 출력
    T = np.eye(4)
    T[:3, :3] = R_best
    T[:3, 3]  = t_best
    print(f'\n=== 결과 (camera_color_optical_frame → base_link) ===')
    print(f'변환 행렬:\n{np.round(T, 5)}')
    print(f'평균 오차: {mean_best:.2f} mm  /  최대 오차: {max_best:.2f} mm')

    # 수락 기준 판정
    rt = _load_yaml(_RUNTIME_PATH)
    ok_mm  = float(rt.get('calibration', {}).get('error_ok_mm',  3.0))
    warn_mm = float(rt.get('calibration', {}).get('error_warn_mm', 5.0))

    if max_best <= ok_mm:
        print(f'✓ 수락 기준 충족 (최대 ≤ {ok_mm}mm)')
    elif max_best <= warn_mm:
        print(f'△ 추가 수집 권장 (최대 ≤ {warn_mm}mm 목표)')
    else:
        print(f'✗ 오차 과다 ({warn_mm}mm 초과) — 데이터 재수집 권장')

    # 저장
    save_yaml(R_best, t_best, n, method_used, mean_best, max_best)

    print('\n다음 단계:')
    print('  검증: 새 위치 카메라 클릭 → 예측 로봇 좌표로 이동 → TCP 오차 실측')
    print('  기준: 실측 오차 ≤ 10mm → config/hand_eye.yaml 확정')


if __name__ == '__main__':
    main()
