#!/usr/bin/env python3
"""
compute_fk_and_calibrate.py — TW 파일 관절각 → FK → calibrateHandEye

사용법:
  python3 scripts/compute_fk_and_calibrate.py
  python3 scripts/compute_fk_and_calibrate.py --tw /path/to/file.tw
  python3 scripts/compute_fk_and_calibrate.py --all   # 5가지 알고리즘 비교
  python3 scripts/compute_fk_and_calibrate.py --dump-fk  # FK 결과만 출력

입력:
  scripts/tcp_calib/cam_data.npz   (collect_handeye_data.py 출력)
  TW 파일                          (DART 저장 .tw)

출력:
  config/topview_calib.yaml
"""
import argparse
import base64
import datetime
import json
import logging
import os
import sys

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger = logging.getLogger('fk_calibrate')

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CAM_DATA   = os.path.join(_SCRIPT_DIR, 'tcp_calib', 'cam_data.npz')
_OUT_YAML   = os.path.join(_SCRIPT_DIR, '..', 'config', 'topview_calib.yaml')
_DEFAULT_TW = os.path.join(os.path.expanduser('~'), 'Downloads', 'topview아르코.tw')

METHODS: dict[str, int] = {
    'TSAI':       cv2.CALIB_HAND_EYE_TSAI,
    'PARK':       cv2.CALIB_HAND_EYE_PARK,
    'HORAUD':     cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF':    cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


# ── Doosan e0509 FK (URDF 기반) ────────────────────────────────────────────────
# 각 joint의 origin(xyz, rpy) — URDF 에서 직접 추출
# rpy 적용 순서 (URDF 표준): R = Rz(yaw) * Ry(pitch) * Rx(roll)
_JOINT_ORIGINS = [
    # joint_1: base_link → link_1
    dict(xyz=[0.0,      0.0,     0.2045], rpy=[0.0,            0.0,    0.0]),
    # joint_2: link_1   → link_2
    dict(xyz=[0.0,      0.0,     0.0   ], rpy=[0.0,           -np.pi/2, -np.pi/2]),
    # joint_3: link_2   → link_3
    dict(xyz=[0.373,    0.0,     0.0   ], rpy=[0.0,            0.0,     np.pi/2]),
    # joint_4: link_3   → link_4
    dict(xyz=[0.0,     -0.373,   0.0   ], rpy=[np.pi/2,        0.0,     0.0]),
    # joint_5: link_4   → link_5
    dict(xyz=[0.0,      0.0,     0.0   ], rpy=[-np.pi/2,       0.0,     0.0]),
    # joint_6: link_5   → link_6
    dict(xyz=[0.0,     -0.1725,  0.0   ], rpy=[np.pi/2,        0.0,     0.0]),
]


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy → 회전 행렬  R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    return (Rotation.from_euler('ZYX', [yaw, pitch, roll])).as_matrix()


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


def forward_kinematics(joint_angles_deg: list[float]) -> np.ndarray:
    """
    Doosan e0509 순방향 기구학.
    joint_angles_deg: [J1, J2, J3, J4, J5, J6] (degree)
    반환: T_link6_in_base  (4×4 homogeneous matrix)
    """
    T = np.eye(4)
    for orig, q_deg in zip(_JOINT_ORIGINS, joint_angles_deg):
        xyz = np.array(orig['xyz'])
        R_orig = _rpy_to_matrix(*orig['rpy'])
        q_rad = np.deg2rad(q_deg)
        R_z = Rotation.from_euler('Z', q_rad).as_matrix()
        T_joint = _make_T(R_orig @ R_z, xyz)
        T = T @ T_joint
    return T


# ── TW 파싱 ───────────────────────────────────────────────────────────────────
def load_tw_joints(tw_path: str) -> list[list[float]]:
    """
    DART .tw 파일에서 MoveJNode 관절각 파싱.
    반환: [[J1..J6], ...] degree 단위, 파일 순서대로
    pose 필드는 {'pose1': v1, ..., 'pose6': v6} dict 형식.
    """
    with open(tw_path, 'rb') as f:
        raw = f.read()
    try:
        data = json.loads(raw)
    except Exception:
        data = json.loads(base64.b64decode(raw).decode('utf-8'))

    nodes = data['taskFile']['file']['children'][2]['_pojo']['children']
    joints: list[list[float]] = []
    for node in nodes:
        pojo = node.get('_pojo', node)
        pose = pojo.get('pose', {})
        if isinstance(pose, dict) and 'pose1' in pose:
            joints.append([float(pose[f'pose{k}']) for k in range(1, 7)])
        elif isinstance(pose, list) and len(pose) >= 6:
            joints.append([float(v) for v in pose[:6]])
    logger.info('TW 파싱: %d개 포즈 발견', len(joints))
    return joints


# ── AXB 잔차 검증 ─────────────────────────────────────────────────────────────
def validate_axb(
    R_cam2base: np.ndarray,
    t_cam2base: np.ndarray,
    R_g2b: list[np.ndarray],
    t_g2b: list[np.ndarray],
    R_t2c: list[np.ndarray],
    t_t2c: list[np.ndarray],
) -> tuple[float, float, np.ndarray]:
    X = _make_T(R_cam2base, t_cam2base)
    errs: list[float] = []
    n = len(R_g2b)
    for i in range(n - 1):
        A = _make_T(R_g2b[i+1], t_g2b[i+1]) @ _inv_T(_make_T(R_g2b[i], t_g2b[i]))
        B = _make_T(R_t2c[i+1], t_t2c[i+1]) @ _inv_T(_make_T(R_t2c[i], t_t2c[i]))
        errs.append(float(np.linalg.norm((A @ X - X @ B)[:3, 3]) * 1000.0))
    arr = np.array(errs)
    return float(arr.mean()), float(arr.max()), arr


# ── 캘리브레이션 ─────────────────────────────────────────────────────────────
def run_calibration(
    R_g2b: list[np.ndarray],
    t_g2b: list[np.ndarray],
    R_t2c: list[np.ndarray],
    t_t2c: list[np.ndarray],
    method_name: str,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    R_b2g = [R.T for R in R_g2b]
    t_b2g = [-R.T @ t for R, t in zip(R_g2b, t_g2b)]
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_b2g, t_b2g,
        R_t2c, [t.reshape(3, 1) for t in t_t2c],
        method=METHODS[method_name],
    )
    t_cam2base = t_cam2base.flatten()
    mean_mm, max_mm, _ = validate_axb(R_cam2base, t_cam2base, R_g2b, t_g2b, R_t2c, t_t2c)
    return R_cam2base, t_cam2base, mean_mm, max_mm


def save_yaml(R: np.ndarray, t: np.ndarray, n: int, method: str,
              mean_mm: float, max_mm: float) -> None:
    quat = Rotation.from_matrix(R).as_quat()
    cam_tilt = float(np.degrees(np.arccos(np.clip(abs(float(R[2, 2])), 0.0, 1.0))))
    data = {
        'schema_version': 1,
        'transformation': {
            'rotation': {'x': float(quat[0]), 'y': float(quat[1]),
                         'z': float(quat[2]), 'w': float(quat[3])},
            'translation': {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2])},
        },
        'metadata': {
            'calibration_date': datetime.date.today().isoformat(),
            'sample_count': n,
            'method': f'calibrateHandEye_{method}',
            'axb_mean_mm': round(mean_mm, 2),
            'axb_max_mm':  round(max_mm, 2),
            'cam_tilt_deg': round(cam_tilt, 2),
            'cam_height_m': round(float(t[2]), 4),
            'tool': 'compute_fk_and_calibrate.py',
            'frames': {'from': 'camera_color_optical_frame', 'to': 'base_link'},
        },
    }
    out = os.path.abspath(_OUT_YAML)
    with open(out, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    logger.info('저장: %s', out)


# ── 메인 ─────────────────────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--tw', default=_DEFAULT_TW, help='.tw 파일 경로')
    parser.add_argument('--method', default='PARK', choices=list(METHODS))
    parser.add_argument('--all', action='store_true', help='5가지 알고리즘 전부 비교')
    parser.add_argument('--dump-fk', action='store_true', help='FK 결과만 출력 후 종료')
    args = parser.parse_args()

    # ─ TW 로드 ─
    if not os.path.exists(args.tw):
        logger.error('TW 파일 없음: %s', args.tw)
        sys.exit(1)
    joint_list = load_tw_joints(args.tw)

    # ─ FK 계산 ─
    fk_results: list[np.ndarray] = []
    print('\n=== FK 결과 (base_link 기준 link_6 위치) ===')
    for idx, jq in enumerate(joint_list, 1):
        T = forward_kinematics(jq)
        fk_results.append(T)
        pos = T[:3, 3]
        print(f'  pose_{idx:02d}  J=[{", ".join(f"{v:7.2f}" for v in jq)}] deg')
        print(f'          TCP = [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m')

    if args.dump_fk:
        return

    # ─ cam_data 로드 ─
    if not os.path.exists(_CAM_DATA):
        logger.error('카메라 데이터 없음: %s', _CAM_DATA)
        sys.exit(1)
    cam = np.load(_CAM_DATA)
    R_t2c_all = list(cam['R_target2cam'])
    t_t2c_all = list(cam['t_target2cam'])
    n_cam = len(R_t2c_all)
    n_fk  = len(fk_results)
    logger.info('cam_data: %d쌍  /  FK: %d포즈', n_cam, n_fk)

    n = min(n_cam, n_fk)
    if n < 3:
        logger.error('최소 3쌍 필요 (현재 %d쌍)', n)
        sys.exit(1)
    if n_cam != n_fk:
        logger.warning('샘플 수 불일치 — 앞 %d쌍만 사용', n)

    R_g2b = [fk_results[i][:3, :3] for i in range(n)]
    t_g2b = [fk_results[i][:3, 3]  for i in range(n)]
    R_t2c = R_t2c_all[:n]
    t_t2c = t_t2c_all[:n]

    # ─ 알고리즘 실행 ─
    if args.all:
        print('\n=== 전체 알고리즘 비교 ===')
        results: dict = {}
        for name in METHODS:
            try:
                R, t, mean_mm, max_mm = run_calibration(R_g2b, t_g2b, R_t2c, t_t2c, name)
                cam_tilt = float(np.degrees(np.arccos(np.clip(abs(float(R[2,2])), 0.0, 1.0))))
                results[name] = (R, t, mean_mm, max_mm)
                print(f'  {name:12s}  AXB평균 {mean_mm:7.1f}mm  최대 {max_mm:7.1f}mm'
                      f'  높이 {t[2]:.3f}m  기울기 {cam_tilt:.1f}deg')
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
        R_best, t_best, mean_best, max_best = run_calibration(
            R_g2b, t_g2b, R_t2c, t_t2c, method_used)

    # ─ 결과 출력 ─
    T = np.eye(4)
    T[:3, :3] = R_best
    T[:3, 3] = t_best
    cam_tilt = float(np.degrees(np.arccos(np.clip(abs(float(R_best[2,2])), 0.0, 1.0))))
    print(f'\n=== 결과 ({method_used}) ===')
    print(f'변환 행렬:\n{np.round(T, 5)}')
    print(f'카메라 위치: [{t_best[0]:.4f}, {t_best[1]:.4f}, {t_best[2]:.4f}] m')
    print(f'카메라 기울기 (수직 기준): {cam_tilt:.2f} deg')
    print(f'AXB 평균: {mean_best:.1f}mm  /  최대: {max_best:.1f}mm')
    print(f'사용 샘플: {n}쌍')

    # 수락 기준
    if max_best <= 50.0:
        print('✓ AXB 잔차 양호 (최대 ≤50mm)')
    elif max_best <= 150.0:
        print('△ AXB 잔차 보통 (≤150mm) — 물리 검증으로 판단')
    else:
        print('✗ AXB 잔차 과다 (>150mm) — 데이터 재수집 권장')

    save_yaml(R_best, t_best, n, method_used, mean_best, max_best)

    print('\n물리 검증:')
    print('  1. 카메라 화면에서 슬롯 모서리 픽셀 좌표 기록')
    print('  2. topview_calib.yaml 변환 적용 → base_link 좌표')
    print('  3. 로봇 TCP를 해당 좌표로 이동 → 실측 오차 ≤10mm 확인')


if __name__ == '__main__':
    main()
