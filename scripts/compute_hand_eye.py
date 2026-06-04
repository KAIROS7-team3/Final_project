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

# 카메라 좌표 [m] — point_collect.py 출력값
# 카메라 #6, #7 오클릭으로 제외 → 로봇 6번부터 카메라 #8부터 매칭
camera_points = [
    [-0.1901, -0.1317, 0.8807],  # cam#1  ↔ robot#1
    [-0.0333, -0.0622, 0.9019],  # cam#2  ↔ robot#2
    [ 0.1043,  0.0102, 0.9026],  # cam#3  ↔ robot#3
    [ 0.1837, -0.1763, 0.9616],  # cam#4  ↔ robot#4
    [-0.0913,  0.0892, 0.8991],  # cam#5  ↔ robot#5
    [ 0.1104,  0.1143, 0.8948],  # cam#8  ↔ robot#6
    [-0.1794,  0.0121, 0.8745],  # cam#9  ↔ robot#7
    [ 0.0289, -0.1202, 0.9545],  # cam#10 ↔ robot#8
    [ 0.0151,  0.0368, 0.8717],  # cam#11 ↔ robot#9
    [-0.1444,  0.1493, 0.8955],  # cam#12 ↔ robot#10
    [ 0.1542, -0.0908, 0.9071],  # cam#13 ↔ robot#11
    [ 0.0165,  0.1293, 0.9544],  # cam#14 ↔ robot#12
    [-0.1140, -0.0559, 0.8768],  # cam#15 ↔ robot#13
    [ 0.0714, -0.0597, 0.8998],  # cam#16 ↔ robot#14
]

# 로봇 TCP 좌표 [mm] — DART Base 좌표계
# 5번부터 Z값은 파일 실측값 그대로 사용
robot_points_mm = [
    [ 426.36, -127.34,  50.47],  # robot#1  plc
    [ 499.61,   26.32,  30.20],  # robot#2  큐브
    [ 574.12,  154.25,  23.97],  # robot#3  마우스
    [ 393.10,  239.32, -30.74],  # robot#4  바닥
    [ 642.80,  -43.87,  35.79],  # robot#5  마우스
    [ 670.87,  161.37,  33.11],  # robot#6  큐브
    [ 575.05, -125.51,  67.68],  # robot#7  plc
    [ 445.30,   81.09,  -9.10],  # robot#8  바닥
    [ 603.54,   66.21,  67.47],  # robot#9  plc
    [ 706.30,  -91.86,  45.62],  # robot#10 큐브
    [ 474.91,  213.26,  46.17],  # robot#11 마우스
    [ 689.85,   49.15, -16.16],  # robot#12 바닥
    [ 508.48,  -62.01,  55.82],  # robot#13 plc
    [ 506.06,  128.12,  44.36],  # robot#14 큐브
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

    if max(errors) > 50:
        logger.warning('최대 오차 50mm 초과 — 좌표 재확인 권장')

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
