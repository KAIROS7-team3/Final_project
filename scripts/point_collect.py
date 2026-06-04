#!/usr/bin/env python3
"""
point_collect.py — 핸드-아이 캘리브레이션용 카메라 좌표 수집 도구

사용 방법:
  1. ROS2 RealSense 노드 종료 후 실행 (카메라 직접 점유)
  2. 컬러 화면에서 기준점(테이프 X자, 모서리 등) 좌클릭
  3. 출력된 Camera XYZ 메모
  4. DART에서 로봇 TCP를 동일 물리 위치로 이동 → TCP 좌표 메모
  5. 3쌍 이상 수집 후 compute_hand_eye.py 에 입력

⚠️  pyrealsense2 직접 점유 — realsense_bringup ROS2 노드와 동시 실행 금지
"""
import logging
import pyrealsense2 as rs
import numpy as np
import cv2

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger = logging.getLogger('point_collect')

WIDTH = 640
HEIGHT = 480
FILTER_SIZE = 3  # 평균 필터 반경 → (2*3+1)² = 7×7 영역


def get_average_depth(depth_frame, x: int, y: int, size: int = FILTER_SIZE) -> float:
    depths = []
    for dx in range(-size, size + 1):
        for dy in range(-size, size + 1):
            px, py = x + dx, y + dy
            if px < 0 or py < 0 or px >= WIDTH or py >= HEIGHT:
                continue
            d = depth_frame.get_distance(px, py)
            if d > 0:
                depths.append(d)
    return float(np.mean(depths)) if depths else 0.0


pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
pipeline.start(config)
align = rs.align(rs.stream.color)

clicked_point: tuple[int, int] | None = None
collected: list[dict] = []


def mouse_callback(event, x, y, flags, param):
    global clicked_point
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_point = (x, y)


cv2.namedWindow('color')
cv2.setMouseCallback('color', mouse_callback)

logger.info('=== 캘리브레이션 기준점 수집 ===')
logger.info('좌클릭: 해당 위치 카메라 좌표 출력 / ESC: 종료')

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        # 수집된 점 표시
        for i, pt in enumerate(collected):
            px, py = pt['pixel']
            cv2.circle(color_image, (px, py), 5, (0, 255, 0), -1)
            cv2.putText(color_image, str(i + 1), (px + 6, py - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        if clicked_point is not None:
            x, y = clicked_point
            depth = get_average_depth(depth_frame, x, y)

            if depth == 0:
                logger.warning('Depth 값 없음 — 다른 위치 클릭')
                clicked_point = None
                continue

            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
            X, Y, Z = point_3d

            entry = {'pixel': (x, y), 'camera_xyz_m': (X, Y, Z)}
            collected.append(entry)
            idx = len(collected)

            logger.info(
                '기준점 #%d | Pixel=(%d,%d) | Camera XYZ: X=%.4f m  Y=%.4f m  Z=%.4f m'
                ' → DART에서 TCP를 이 위치로 이동 후 좌표 기록',
                idx, x, y, X, Y, Z)

            cv2.circle(color_image, (x, y), 5, (0, 0, 255), -1)
            cv2.putText(color_image, f'#{idx}', (x + 6, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            clicked_point = None

        cv2.imshow('color', color_image)
        if cv2.waitKey(1) == 27:
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

    if collected:
        logger.info('=== 수집 완료 — 아래 값을 compute_hand_eye.py camera_points 에 입력 ===')
        for i, pt in enumerate(collected):
            X, Y, Z = pt['camera_xyz_m']
            logger.info('  [%.4f, %.4f, %.4f],  # #%d', X, Y, Z, i + 1)
