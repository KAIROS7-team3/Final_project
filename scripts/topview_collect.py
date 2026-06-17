#!/usr/bin/env python3
"""
topview_collect.py — 탑뷰 핸드-아이 캘리브레이션 데이터 수집

사용법:
  python3 scripts/topview_collect.py

절차:
  1. DART에서 첫 번째 포즈로 이동
  2. 화면에서 마커 초록 테두리 확인
  3. SPACE: 현재 포즈 저장 (이미지 + solvePnP 결과)
  4. 다음 포즈로 이동 → 반복
  5. Q: 종료 → scripts/tcp_calib/cam_data.npz 저장

출력:
  scripts/tcp_calib/cam_data.npz   (R_target2cam, t_target2cam)
  scripts/tcp_calib/pose_NNN.png   (포즈 이미지)

⚠️  pyrealsense2 직접 점유 — realsense_bringup ROS2 노드와 동시 실행 금지
"""
import os
import sys

import cv2
import numpy as np
import pyrealsense2 as rs

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_OUT_DIR    = os.path.join(_SCRIPT_DIR, 'tcp_calib')
os.makedirs(_OUT_DIR, exist_ok=True)

MARKER_SIZE_M = 0.05
DICT_ID       = cv2.aruco.DICT_4X4_50
TARGET_ID     = 1


def build_camera(w: int = 1280, h: int = 800, fps: int = 30):
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    profile = pipeline.start(cfg)
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K    = np.array([[intr.fx, 0, intr.ppx],
                     [0, intr.fy, intr.ppy],
                     [0, 0, 1]], dtype=np.float64)
    dist = np.array(intr.coeffs, dtype=np.float64)
    print(f'[camera] {w}×{h}@{fps}fps  fx={intr.fx:.1f} fy={intr.fy:.1f}')
    return pipeline, K, dist


def detect_marker(frame_bgr, K, dist):
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    aruco_dict   = cv2.aruco.Dictionary_get(DICT_ID)
    aruco_params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    if ids is None:
        return None, None, None
    for i, mid in enumerate(ids.flatten()):
        if mid == TARGET_ID:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]], MARKER_SIZE_M, K, dist)
            R, _ = cv2.Rodrigues(rvec[0])
            return R, tvec[0].flatten(), corners[i]
    return None, None, None


def main() -> None:
    # 기존 데이터 확인
    npz_path = os.path.join(_OUT_DIR, 'cam_data.npz')
    R_list: list[np.ndarray] = []
    t_list: list[np.ndarray] = []

    if os.path.exists(npz_path):
        existing = np.load(npz_path)
        R_list = list(existing['R_target2cam'])
        t_list = list(existing['t_target2cam'])
        print(f'기존 데이터 {len(R_list)}쌍 로드 — 이어서 수집합니다.')
    else:
        print('새 수집 시작.')

    print()
    print('SPACE: 포즈 저장  /  D: 마지막 삭제  /  Q: 종료 및 저장')
    print()

    pipeline, K, dist = build_camera()
    count = len(R_list)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            frame  = np.asanyarray(frames.get_color_frame().get_data())
            R, tvec, corners = detect_marker(frame, K, dist)

            display = frame.copy()
            if corners is not None:
                cv2.aruco.drawDetectedMarkers(display, [corners])
                dist_m = float(np.linalg.norm(tvec))
                face_angle = float(np.degrees(np.arccos(np.clip(abs(float(R[2, 2])), 0.0, 1.0))))
                ok = face_angle <= 22.0
                status = f'OK  angle={face_angle:.1f}deg  dist={dist_m*100:.1f}cm' if ok else \
                         f'BAD angle={face_angle:.1f}deg (>22)'
                color  = (0, 255, 0) if ok else (0, 60, 255)
                cv2.putText(display, status, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            else:
                cv2.putText(display, 'No marker (ID=1)', (15, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.putText(display, f'saved: {count}  |  SPACE=save  D=del  Q=quit',
                        (15, display.shape[0] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow('Topview Collect', display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            if key == ord(' ') and R is not None:
                ok = float(np.degrees(np.arccos(np.clip(abs(float(R[2, 2])), 0.0, 1.0)))) <= 22.0
                if not ok:
                    print(f'[SKIP] 기울기 과다 — SPACE 다시 눌러서 강제 저장, 또는 자세 조정')
                    # 한 번 더 누르면 강제 저장
                    key2 = cv2.waitKey(1500) & 0xFF
                    if key2 != ord(' '):
                        continue
                count += 1
                R_list.append(R)
                t_list.append(tvec)
                img_path = os.path.join(_OUT_DIR, f'pose_{count:03d}.png')
                cv2.imwrite(img_path, frame)
                print(f'[{count:03d}] 저장  dist={np.linalg.norm(tvec)*100:.1f}cm  → {img_path}')

            if key == ord('d') and count > 0:
                R_list.pop()
                t_list.pop()
                old_img = os.path.join(_OUT_DIR, f'pose_{count:03d}.png')
                if os.path.exists(old_img):
                    os.remove(old_img)
                print(f'[{count:03d}] 삭제')
                count -= 1

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    if count == 0:
        print('저장된 데이터 없음.')
        return

    np.savez(npz_path,
             R_target2cam=np.array(R_list),
             t_target2cam=np.array(t_list))
    print(f'\n총 {count}쌍 저장 → {npz_path}')
    if count < 15:
        print(f'권장 샘플 수: 15~25쌍 (현재 {count}쌍)')
    else:
        print('샘플 수 충분 — compute_fk_and_calibrate.py 실행 가능')


if __name__ == '__main__':
    main()
