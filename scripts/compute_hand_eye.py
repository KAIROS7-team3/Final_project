#!/usr/bin/env python3
"""
compute_hand_eye.py — Point Correspondence로 카메라↔로봇 변환 행렬 계산

사용 방법:
  1. point_collect.py 로 얻은 카메라 좌표를 camera_points 에 입력
  2. DART 터치 펜던트에서 읽은 TCP 좌표를 robot_points 에 입력
     (단위: mm → 자동으로 m 변환)
  3. python3 compute_hand_eye.py 실행
  4. 결과가 config/hand_eye.yaml 에 저장됨

최소 3쌍 필요. 4쌍 이상 권장 (오차 검증 가능).
"""
import logging
import os
import yaml
import numpy as np

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger = logging.getLogger('compute_hand_eye')

# ────────────────────────────────────────────────────────
# ▼▼▼ 여기에 수집한 좌표 입력 ▼▼▼
# ────────────────────────────────────────────────────────

# 카메라 좌표 [m] — point_collect.py 출력값 (color_frame intrinsics 적용, ver_1_2)
# 제외: #8(특이점), #13/#15(특이점), #16/#17/#22/#23(작업반경 외)
camera_points = [
    [ 0.0140,  0.1045,  0.9560],  # cam#1
    [-0.2204, -0.0391,  0.9660],  # cam#2
    [ 0.0365, -0.1334,  0.9630],  # cam#3
    [-0.1096,  0.0578,  0.9590],  # cam#4
    [ 0.1378, -0.0041,  0.9590],  # cam#5
    [-0.1405, -0.1615,  0.9660],  # cam#6
    [-0.0057, -0.0262,  0.9520],  # cam#7
    [ 0.2248,  0.0827,  0.9590],  # cam#9
    [ 0.1785, -0.1662,  0.9630],  # cam#10
    [-0.0183, -0.2823,  0.9670],  # cam#11
    [-0.2371, -0.2560,  0.9700],  # cam#12
    [-0.2273,  0.0625,  0.9530],  # cam#14
    [ 0.1159,  0.0928,  0.9620],  # cam#18
    [-0.2482, -0.1343,  0.9660],  # cam#19
    [ 0.1995, -0.2624,  0.9660],  # cam#20
    [-0.0702, -0.0935,  0.9620],  # cam#21
    [ 0.0515, -0.2011,  0.9640],  # cam#24
]

# 로봇 TCP 좌표 [mm] — DART Base 좌표계 (calibration_ver_1_2.tw)
robot_points_mm = [
    [ 672.48,   66.82,  -28.01],  # robot#1
    [ 516.40, -152.81,  -29.94],  # robot#2
    [ 428.90,   97.75,  -31.16],  # robot#3
    [ 622.80,  -47.49,  -27.94],  # robot#4
    [ 566.91,  191.21,  -29.61],  # robot#5
    [ 399.16,  -66.90,  -31.53],  # robot#6
    [ 535.89,   53.38,  -30.91],  # robot#7
    [ 654.13,  273.35,  -28.12],  # robot#9
    [ 410.33,  237.09,  -31.27],  # robot#10
    [ 281.61,   51.64,  -33.61],  # robot#11
    [ 304.60, -163.61,  -33.60],  # robot#12
    [ 616.33, -165.41,  -29.35],  # robot#14
    [ 662.36,  161.51,  -28.25],  # robot#18
    [ 428.10, -183.79,  -31.85],  # robot#19
    [ 318.10,  261.22,  -32.60],  # robot#20
    [ 472.68,   -7.54,  -31.85],  # robot#21
    [ 371.93,  119.23,  -32.30],  # robot#24
]

# ────────────────────────────────────────────────────────
# ▲▲▲ 입력 끝 ▲▲▲
# ────────────────────────────────────────────────────────

_CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'hand_eye.yaml')


def compute_transform(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """SVD 기반 최소제곱 rigid transform (src → dst)."""
    assert src.shape == dst.shape and src.shape[0] >= 3

    c_src = src.mean(axis=0)
    c_dst = dst.mean(axis=0)
    A = src - c_src
    B = dst - c_dst

    U, _, Vt = np.linalg.svd(B.T @ A)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    t = c_dst - R @ c_src
    return R, t


def main() -> None:
    if len(camera_points) < 3 or len(robot_points_mm) < 3:
        logger.error('camera_points 와 robot_points_mm 를 3쌍 이상 입력하세요.')
        return

    if len(camera_points) != len(robot_points_mm):
        logger.error('camera_points 와 robot_points_mm 개수가 다릅니다.')
        return

    cam = np.array(camera_points, dtype=float)
    rob = np.array(robot_points_mm, dtype=float) / 1000.0  # mm → m

    R, t = compute_transform(cam, rob)

    # 검증 — 각 점의 오차
    logger.info('=== 변환 결과 ===')
    errors = []
    for i, (c, r) in enumerate(zip(cam, rob)):
        pred = R @ c + t
        err = np.linalg.norm(pred - r) * 1000  # m → mm
        errors.append(err)
        logger.info('  #%d  오차: %.2f mm', i + 1, err)
    logger.info('  평균 오차: %.2f mm  /  최대: %.2f mm', np.mean(errors), max(errors))

    if max(errors) > 5:
        logger.warning('최대 오차 5mm 초과 — 좌표 재확인 권장 (목표: 1~3mm)')
    elif max(errors) > 3:
        logger.warning('최대 오차 3mm 초과 — 추가 포인트 수집 권장')

    # 4×4 행렬
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t

    logger.info('=== 변환 행렬 (camera → robot_base) ===\n%s', np.round(T, 6))

    # config/hand_eye.yaml 저장
    data = {
        'hand_eye': {
            'method': 'point_correspondence',
            'transform': {
                'rotation': R.tolist(),
                'translation_m': t.tolist(),
            },
            'matrix_4x4': T.tolist(),
            'num_points': len(camera_points),
            'mean_error_mm': float(np.mean(errors)),
            'max_error_mm': float(max(errors)),
        }
    }

    with open(_CONFIG_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

    logger.info('config/hand_eye.yaml 저장 완료')


if __name__ == '__main__':
    main()
