#!/usr/bin/env python3
"""
validate_handeye.py — hand-eye 캘리브레이션 물리 검증

사용법:
  python3 scripts/validate_handeye.py

절차:
  1. 마커(ID=1, 5cm)를 작업 테이블 위 임의 위치에 평평하게 놓기
  2. 화면에서 마커 인식 확인 (초록 테두리)
  3. SPACE: 현재 좌표 스냅샷 저장 (콘솔에 base_link 좌표 출력)
  4. DART에서 저장된 [x, y, z] 좌표로 로봇 TCP 이동
  5. 자로 TCP ↔ 마커 중심 실측 거리 기록
  기준: ≤10mm → 캘리브레이션 확정

Q: 종료
"""
import os
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
from scipy.spatial.transform import Rotation

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_YAML_PATH  = os.path.join(_SCRIPT_DIR, '..', 'config', 'topview_calib.yaml')
_RUNTIME    = os.path.join(_SCRIPT_DIR, '..', 'config', 'runtime.yaml')

MARKER_SIZE_M = 0.05   # 5cm
DICT_ID       = cv2.aruco.DICT_4X4_50
TARGET_ID     = 1


def load_calibration() -> tuple[np.ndarray, np.ndarray]:
    """topview_calib.yaml → (R_cam2base 3×3, t_cam2base 3,)."""
    with open(_YAML_PATH) as f:
        d = yaml.safe_load(f)
    tr = d['transformation']
    q = [tr['rotation']['x'], tr['rotation']['y'],
         tr['rotation']['z'], tr['rotation']['w']]
    R = Rotation.from_quat(q).as_matrix()
    t = np.array([tr['translation']['x'],
                  tr['translation']['y'],
                  tr['translation']['z']])
    return R, t


def cam_to_base(pt_cam: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """카메라 프레임 3D 점 → base_link 프레임."""
    return R @ pt_cam + t


def build_camera(w: int = 1280, h: int = 800, fps: int = 30):
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16,  fps)
    profile = pipeline.start(cfg)
    align   = rs.align(rs.stream.color)

    # 인트린식
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K    = np.array([[intr.fx, 0, intr.ppx],
                     [0, intr.fy, intr.ppy],
                     [0,       0,        1]], dtype=np.float64)
    dist = np.array(intr.coeffs, dtype=np.float64)
    print(f'[camera] {w}×{h}@{fps}fps  fx={intr.fx:.1f} fy={intr.fy:.1f}')
    return pipeline, align, K, dist


def detect_marker(frame_bgr: np.ndarray, K: np.ndarray, dist: np.ndarray
                  ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """
    ArUco 마커 ID=TARGET_ID 검출.
    반환: (rvec, tvec, corners)  없으면 (None, None, None)
    """
    gray   = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    aruco_dict   = cv2.aruco.Dictionary_get(DICT_ID)
    aruco_params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None:
        return None, None, None
    for i, mid in enumerate(ids.flatten()):
        if mid == TARGET_ID:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]], MARKER_SIZE_M, K, dist)
            return rvec[0], tvec[0].flatten(), corners[i]
    return None, None, None


def depth_at_pixel(depth_frame, u: int, v: int, k: int = 5) -> float | None:
    """(u,v) 주변 k×k 패치의 중앙값 깊이 [m]. 데이터 없으면 None."""
    h = depth_frame.get_height()
    w = depth_frame.get_width()
    vals = []
    for dv in range(-k, k+1):
        for du in range(-k, k+1):
            pu, pv = u+du, v+dv
            if 0 <= pu < w and 0 <= pv < h:
                z = depth_frame.get_distance(pu, pv)
                if z > 0.01:
                    vals.append(z)
    return float(np.median(vals)) if vals else None


def marker_rot_to_dart_abc(rvec: np.ndarray, R_c2b: np.ndarray) -> tuple[float, float, float]:
    """
    ArUco rvec (카메라 프레임) → base_link 프레임 회전 → DART ZYZ 오일러 (a, b, c) [deg].
    마커 평면에 수직으로 접근하는 TCP 자세 기준.
    DART MoveL 회전 관례: R = Rz(a) * Ry(b) * Rz(c)
    """
    R_m2c, _ = cv2.Rodrigues(rvec)
    R_m2b = R_c2b @ R_m2c
    # 마커 +Z가 위를 향하므로 TCP가 아래에서 접근 → 180° 뒤집기
    R_flip = Rotation.from_euler('X', 180, degrees=True).as_matrix()
    R_tcp2b = R_m2b @ R_flip
    a, b, c = Rotation.from_matrix(R_tcp2b).as_euler('ZYZ', degrees=True)
    return float(a), float(b), float(c)


def draw_overlay(frame: np.ndarray, pt_base: np.ndarray,
                 pt_cam: np.ndarray, corners,
                 abc: tuple[float, float, float] | None = None) -> None:
    """화면에 마커 박스 + 좌표 오버레이."""
    cv2.aruco.drawDetectedMarkers(frame, [corners])

    x, y, z = pt_base
    xc, yc, zc = pt_cam
    a_str = f'  a={abc[0]:+6.1f} b={abc[1]:+6.1f} c={abc[2]:+6.1f}' if abc else ''

    lines = [
        f'base_link:  x={x*100:+6.1f}cm  y={y*100:+6.1f}cm  z={z*100:+6.1f}cm',
        f'DART abc:{a_str}  (deg)',
        f'cam:  z={zc*100:.1f}cm',
        'SPACE: 스냅샷   Q: 종료',
    ]
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (15, 35 + 28*i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)


def main() -> None:
    if not os.path.exists(_YAML_PATH):
        print(f'[ERROR] topview_calib.yaml 없음: {_YAML_PATH}')
        sys.exit(1)

    R_c2b, t_c2b = load_calibration()
    print(f'캘리브레이션 로드: Z={t_c2b[2]:.3f}m  (AXB 파라미터 topview_calib.yaml 기준)')
    print()
    print('=== 물리 검증 절차 ===')
    print('1. 마커(ID=1, 5cm)를 작업 테이블 위에 평평하게 놓기')
    print('2. 화면에서 초록 테두리 확인')
    print('3. SPACE로 좌표 기록 → DART에서 그 좌표로 TCP 이동')
    print('4. 실제 거리 측정 (기준: ≤10mm)')
    print()

    pipeline, align, K, dist_coeffs = build_camera()
    snapshots: list[dict] = []

    try:
        while True:
            frames      = pipeline.wait_for_frames()
            aligned     = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            rvec, tvec_cam, corners = detect_marker(frame, K, dist_coeffs)

            if tvec_cam is not None:
                pt_base = cam_to_base(tvec_cam, R_c2b, t_c2b)
                abc = marker_rot_to_dart_abc(rvec, R_c2b)
                draw_overlay(frame, pt_base, tvec_cam, corners, abc)

                cx = int(corners[0][:, 0].mean())
                cy = int(corners[0][:, 1].mean())
                dz = depth_at_pixel(depth_frame, cx, cy)
                if dz:
                    cv2.putText(frame, f'depth={dz*100:.1f}cm', (15, 155),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
            else:
                abc = None
                cv2.putText(frame, 'No marker (ID=1)', (15, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow('Hand-Eye Validation', frame)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            if key == ord(' ') and tvec_cam is not None:
                pt_base = cam_to_base(tvec_cam, R_c2b, t_c2b)
                abc = marker_rot_to_dart_abc(rvec, R_c2b)
                snap_id = len(snapshots) + 1
                snapshots.append({'id': snap_id, 'base': pt_base.tolist(),
                                  'cam': tvec_cam.tolist(), 'abc': list(abc)})
                x, y, z = (pt_base * 1000).tolist()
                a, b, c = abc
                print(f'\n[스냅샷 {snap_id}]')
                print(f'  위치 (base_link): x={x:+7.1f}  y={y:+7.1f}  z={z:+7.1f}  mm')
                print(f'  방향 (DART ZYZ):  a={a:+7.1f}  b={b:+7.1f}  c={c:+7.1f}  deg')
                print(f'  → DART MoveL: [{x:.1f}, {y:.1f}, {z:.1f}, {a:.1f}, {b:.1f}, {c:.1f}]')
                print(f'  → 검증용 (Z+160 여유): [{x:.1f}, {y:.1f}, {z+160:.1f}, {a:.1f}, {b:.1f}, {c:.1f}]')
                print('  TCP 이동 후 실측 거리를 입력하세요.')

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    if snapshots:
        print('\n=== 검증 결과 요약 ===')
        for s in snapshots:
            b = s['base']
            print(f"  [{s['id']}] base_link [{b[0]*1000:+7.1f}, "
                  f"{b[1]*1000:+7.1f}, {b[2]*1000:+7.1f}] mm")
        print()
        print('각 스냅샷 좌표로 DART MoveL 이동 후 TCP ↔ 마커 중심 실측 거리:')
        errors = []
        for s in snapshots:
            try:
                val = float(input(f"  스냅샷 [{s['id']}] 실측 오차 (mm): "))
                errors.append(val)
            except (ValueError, EOFError):
                pass
        if errors:
            mean_e = np.mean(errors)
            max_e  = np.max(errors)
            print(f'\n평균 오차: {mean_e:.1f}mm  /  최대: {max_e:.1f}mm')
            if max_e <= 10.0:
                print('✓ 캘리브레이션 확정 — config/topview_calib.yaml 사용 가능')
            elif max_e <= 20.0:
                print('△ 오차 보통 — 사용 가능하나 추가 샘플 수집 권장')
            else:
                print('✗ 오차 과다 — 재캘리브레이션 필요')


if __name__ == '__main__':
    main()
