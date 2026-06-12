#!/usr/bin/env python3
"""
collect_handeye_data.py — cv2.calibrateHandEye 데이터 수집

CALIB_POSES(관절각)로 로봇을 이동 → TF로 TCP 6DOF 읽기 → ArUco 마커 감지
결과: scripts/handeye_data.npz  (compute_handeye_opencv.py 에서 사용)

사용 방법:
  1. ROS2 드라이버 실행 (로봇 연결)
  2. realsense_bringup 노드 종료 (카메라 직접 점유)
  3. python3 scripts/collect_handeye_data.py
  4. 카메라 프리뷰 창에서:
       SPACE  : 현재 프레임으로 저장 (마커 감지 시)
       s      : 이 자세 스킵
       q      : 즉시 종료

⚠️  pyrealsense2 직접 점유 — realsense_bringup ROS2 노드와 동시 실행 금지
⚠️  E-1 예외: MoveJoint API는 degree 입력 (하드웨어 경계 인터페이스)
"""
import logging
import os
import sys

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
import yaml
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from dsr_msgs2.srv import MoveJoint

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger = logging.getLogger('collect_handeye')

# ── 경로 ──────────────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR = os.path.join(_SCRIPT_DIR, '..', 'config')
_OUTPUT_PATH = os.path.join(_SCRIPT_DIR, 'handeye_data.npz')


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


_runtime = _load_yaml(os.path.join(_CONFIG_DIR, 'runtime.yaml'))
_cam_info = _load_yaml(os.path.join(_CONFIG_DIR, 'camera_info.yaml')).get('intrinsics', {})
_calib = _runtime.get('calibration', {})

VEL: float = float(_calib.get('motion_vel_deg_per_s', 20.0))
ACC: float = float(_calib.get('motion_acc_deg_per_s2', 10.0))
SVC_TIMEOUT: float = float(_calib.get('service_timeout_sec', 5.0))
MARKER_SIZE_M: float = float(_calib.get('aruco_marker_size_m', 0.065))
ARUCO_DICT_ID: int = int(_calib.get('aruco_dict_id', cv2.aruco.DICT_4X4_50))
ARUCO_TARGET_ID: int = int(_calib.get('aruco_target_id', 0))
MIN_SAMPLES: int = int(_calib.get('min_sample_count', 15))

# 수집 품질 필터 — compute_handeye_opencv.py와 동일 기준 적용
MAX_FACE_ANGLE_DEG: float = 22.0  # 마커가 카메라에 대해 기울어진 최대 각도
MIN_MARKER_DIST_M:  float = 0.30  # 마커-카메라 최소 거리
MAX_MARKER_DIST_M:  float = 0.85  # 마커-카메라 최대 거리
BASE_FRAME: str = str(_calib.get('base_frame', 'base_link'))
GRIPPER_FRAME: str = str(_calib.get('gripper_frame', 'link_6'))

CAMERA_MATRIX = np.array([
    [_cam_info['fx'], 0.0,              _cam_info['cx']],
    [0.0,             _cam_info['fy'],  _cam_info['cy']],
    [0.0,             0.0,              1.0            ],
], dtype=np.float64)
DIST_COEFFS = np.array(_cam_info['coeffs'], dtype=np.float64)
IMG_W: int = _cam_info['width']
IMG_H: int = _cam_info['height']

# ── 캘리브레이션 자세 — TFcalibration.tw 기반 실측 자세 (calib_poses.py와 별개) ──
# calib_poses.py: handeye_calib_motion.py 전용 격자형 37자세
# 이 파일: DART 직접교시로 수집한 R9+V21=30자세 (D455f 탑뷰 캘리브레이션 전용)
# ⚠️ E-1 예외: Doosan MoveJoint API는 degree 입력 (하드웨어 경계 인터페이스)
CALIB_POSES: list[list[float]] = [
    # ── 직접교시 샘플 기반 자세 (TFcalibration.tw 2~10번 참조) ────────────────
    # ⚠️ J1 안전 제한 ±30° 적용 (TW 9번·10번 J1 31.x → 30.0 클램프)
    # 마커 위치: link_6 옆면 (탑뷰 카메라 방향) — 프리뷰에서 미감지 시 S 스킵

    # ── 그룹 R: 실측 기준 자세 9개 (TW 2~10번) ──────────────────────────────
    [  -5.76,  39.57,  65.88,  85.36,  92.78, -119.27],  # R1  TW2
    [  30.00,  38.15,  68.98, -77.75,  92.78,  111.28],  # R2  TW3 (J1 클램프)
    [   5.67,   9.03,  51.71,  11.84,  59.95,   -9.10],  # R3  TW4
    [   9.28,  41.84, 107.92,  11.85, -65.48,   -9.10],  # R4  TW5
    [  -9.90,  44.54, 102.82, -67.54, -65.48,   18.22],  # R5  TW6
    [  22.91,  44.74,  86.97, -67.36,  85.49,  121.39],  # R6  TW7
    [   0.79,  16.51,  69.24, -89.98,  85.49,   82.78],  # R7  TW8
    [  30.00,  14.48,  69.25, -89.98,  40.66,   82.78],  # R8  TW9 (J1 클램프)
    [  30.00,  18.40, 101.34, -72.60,  40.66,   82.78],  # R9  TW10 (J1 클램프)

    # ── 그룹 V: 기준 자세 변형 (±5~10° 퍼터베이션) ─────────────────────────
    # R1 변형
    [  -5.76,  30.00,  65.88,  85.36,  92.78, -119.27],  # V01 R1 J2-10
    [  -5.76,  39.57,  73.00,  80.00,  92.78, -119.27],  # V02 R1 J3+7/J4-5
    [  -5.76,  39.57,  65.88,  85.36,  83.00, -119.27],  # V03 R1 J5-10
    # R2 변형
    [  20.00,  38.15,  68.98, -77.75,  92.78,  111.28],  # V04 R2 J1-10
    [  30.00,  30.00,  68.98, -77.75,  92.78,  111.28],  # V05 R2 J2-8
    [  30.00,  38.15,  76.00, -70.00,  92.78,  111.28],  # V06 R2 J3+7/J4+8
    # R3 변형
    [   5.67,   9.03,  51.71,  11.84,  68.00,   -9.10],  # V07 R3 J5+8
    [  -5.00,   9.03,  51.71,  11.84,  59.95,   -9.10],  # V08 R3 J1-11
    [   5.67,  17.00,  58.00,  11.84,  59.95,   -9.10],  # V09 R3 J2+8/J3+6
    # R4 변형
    [   9.28,  41.84, 107.92,  11.85, -55.00,   -9.10],  # V10 R4 J5+10
    [   9.28,  33.00, 100.00,  11.85, -65.48,   -9.10],  # V11 R4 J2-9/J3-8
    [  18.00,  41.84, 107.92,  11.85, -65.48,   -9.10],  # V12 R4 J1+9
    # R5 변형
    [   0.00,  44.54, 102.82, -67.54, -65.48,   18.22],  # V13 R5 J1+10
    [  -9.90,  36.00,  95.00, -67.54, -65.48,   18.22],  # V14 R5 J2-9/J3-8
    [  -9.90,  44.54, 102.82, -57.00, -55.00,   18.22],  # V15 R5 J4+11/J5+10
    # R6 변형
    [  13.00,  44.74,  86.97, -67.36,  85.49,  121.39],  # V16 R6 J1-10
    [  22.91,  36.00,  79.00, -67.36,  85.49,  121.39],  # V17 R6 J2-9/J3-8
    [  22.91,  44.74,  86.97, -57.00,  75.00,  121.39],  # V18 R6 J4+10/J5-10
    # R7 변형
    [  10.00,  16.51,  69.24, -89.98,  85.49,   82.78],  # V19 R7 J1+9
    [   0.79,  25.00,  76.00, -82.00,  85.49,   82.78],  # V20 R7 J2+9/J3+7/J4+8
    # R8 변형
    [  20.00,  14.48,  69.25, -89.98,  40.66,   82.78],  # V21 R8 J1-10
]
HOME: list[float] = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# ── ArUco 마커 3D 기준점 (마커 평면, 마커 중심 원점) ──────────────────────────
def _make_obj_pts(size_m: float) -> np.ndarray:
    h = size_m / 2.0
    return np.array([
        [-h,  h, 0.0],
        [ h,  h, 0.0],
        [ h, -h, 0.0],
        [-h, -h, 0.0],
    ], dtype=np.float32)


OBJ_PTS = _make_obj_pts(MARKER_SIZE_M)

# OpenCV 4.5.x 구 API — 모듈 로드 시 1회만 생성 (4.7+ ArucoDetector 미지원)
_ARUCO_DICT = cv2.aruco.Dictionary_get(ARUCO_DICT_ID)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()


# ── ROS2 노드 ──────────────────────────────────────────────────────────────────
class CollectNode(Node):
    def __init__(self) -> None:
        super().__init__('collect_handeye')
        self._move_cli = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info('move_joint 서비스 연결 대기 중...')
        if not self._move_cli.wait_for_service(timeout_sec=SVC_TIMEOUT):
            raise RuntimeError('move_joint 서비스 연결 실패 — 로봇 드라이버 실행 여부 확인')
        self.get_logger().info('연결 완료.')

    def move_to(self, pos: list[float]) -> bool:
        req = MoveJoint.Request()
        req.pos = [float(p) for p in pos]
        req.vel = VEL
        req.acc = ACC
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0       # ABSOLUTE
        req.blend_type = 0
        req.sync_type = 0  # SYNC (이동 완료까지 블로킹)
        future = self._move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('move_joint 타임아웃')
            return False
        return bool(future.result().success)

    def get_gripper2base(self) -> tuple[np.ndarray, np.ndarray] | None:
        """TF에서 base_link → gripper_frame 변환 읽기."""
        try:
            tf = self._tf_buffer.lookup_transform(
                BASE_FRAME, GRIPPER_FRAME, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error('TF lookup 실패 (%s→%s): %s', BASE_FRAME, GRIPPER_FRAME, e)
            return None

        tr = tf.transform.translation
        ro = tf.transform.rotation
        t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
        R = Rotation.from_quat([ro.x, ro.y, ro.z, ro.w]).as_matrix()
        return R, t


# ── ArUco 감지 ─────────────────────────────────────────────────────────────────
def detect_aruco(
    color_img: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray]:
    """
    ArUco 마커 감지 → (R_target2cam, t_target2cam, display_img)
    마커 미감지 시 R, t = None
    """
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS)

    display = color_img.copy()

    if ids is None:
        cv2.putText(display, 'NO MARKER', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        return None, None, display

    # ARUCO_TARGET_ID 우선, 없으면 첫 번째
    target_idx = 0
    for idx, mid in enumerate(ids.flatten()):
        if mid == ARUCO_TARGET_ID:
            target_idx = idx
            break

    img_pts = corners[target_idx].reshape(4, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, CAMERA_MATRIX, DIST_COEFFS)
    if not ok:
        cv2.putText(display, 'SOLVEPNP FAIL', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 165, 255), 2)
        return None, None, display

    R_target2cam, _ = cv2.Rodrigues(rvec)
    t_target2cam: np.ndarray = tvec.flatten()

    cv2.drawFrameAxes(display, CAMERA_MATRIX, DIST_COEFFS,
                      rvec, tvec, MARKER_SIZE_M * 0.5)
    dist_m = float(np.linalg.norm(t_target2cam))

    # 마커 기울기 계산 (카메라 정면 기준)
    cos_a = abs(float(R_target2cam[2, 2]))
    face_angle = float(np.degrees(np.arccos(np.clip(cos_a, 0.0, 1.0))))

    qual_ok = (face_angle <= MAX_FACE_ANGLE_DEG
               and MIN_MARKER_DIST_M <= dist_m <= MAX_MARKER_DIST_M)
    qual_color = (0, 255, 0) if qual_ok else (0, 165, 255)
    qual_tag   = 'OK' if qual_ok else f'BAD(ang={face_angle:.0f}deg,dist={dist_m:.2f}m)'
    label = f'ID={ids.flatten()[target_idx]}  dist={dist_m:.3f}m  [{qual_tag}]'
    cv2.putText(display, label, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, qual_color, 2)
    cv2.putText(display, 'SPACE:save  S:skip  Q:quit', (10, IMG_H - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)

    return R_target2cam, t_target2cam, display


# ── 메인 ───────────────────────────────────────────────────────────────────────
def main() -> None:
    rclpy.init()
    try:
        node = CollectNode()
    except RuntimeError as e:
        logger.error('%s', e)
        rclpy.shutdown()
        sys.exit(1)

    pipeline = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.bgr8, 30)
    try:
        pipeline.start(rs_cfg)
    except Exception as e:
        logger.error('RealSense 파이프라인 시작 실패: %s', e)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    cv2.namedWindow('ArUco Preview', cv2.WINDOW_NORMAL)

    R_g2b_list: list[np.ndarray] = []
    t_g2b_list: list[np.ndarray] = []
    R_t2c_list: list[np.ndarray] = []
    t_t2c_list: list[np.ndarray] = []

    total = len(CALIB_POSES)
    skipped = 0
    quit_flag = False
    i = -1

    print(f'\n=== collect_handeye_data.py  ({total}자세 | 마커 {MARKER_SIZE_M*100:.1f}cm) ===')
    print(f'TF: {BASE_FRAME} → {GRIPPER_FRAME}')
    print('프리뷰 창 조작: SPACE=저장  S=스킵  Q=종료\n')

    try:
        for i, pose in enumerate(CALIB_POSES):
            if quit_flag:
                break

            # 홈 복귀 후 목표 자세 이동 (안전한 중간 경유점)
            print(f'[{i+1:02d}/{total}] 홈 복귀 중...')
            node.move_to(HOME)

            print(f'[{i+1:02d}/{total}] 이동 중... {pose}')
            if not node.move_to(pose):
                logger.warning('이동 실패 — 홈 복귀 후 스킵')
                node.move_to(HOME)
                skipped += 1
                continue

            # TF 업데이트 대기 (이동 직후 약간의 지연 필요)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.05)

            result = node.get_gripper2base()
            if result is None:
                logger.warning('TCP TF 읽기 실패 — 스킵')
                skipped += 1
                continue

            R_g2b, t_g2b = result
            print(f'  TCP t=[{t_g2b[0]:.3f}, {t_g2b[1]:.3f}, {t_g2b[2]:.3f}] m')

            # 카메라 프리뷰 루프
            action = ''
            last_R_t2c: np.ndarray | None = None
            last_t_t2c: np.ndarray | None = None

            while not action:
                rclpy.spin_once(node, timeout_sec=0)

                try:
                    frames = pipeline.wait_for_frames(timeout_ms=200)
                    if frames and frames.get_color_frame():
                        color_img = np.asanyarray(frames.get_color_frame().get_data())
                        last_R_t2c, last_t_t2c, display = detect_aruco(color_img)

                        sample_n = len(R_g2b_list)
                        cv2.putText(display, f'Pose {i+1}/{total}  Saved:{sample_n}',
                                    (10, IMG_H - 45),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
                        cv2.imshow('ArUco Preview', display)
                except RuntimeError:
                    logger.warning('카메라 프레임 끊김 — 재연결 시도...')
                    try:
                        pipeline.stop()
                        pipeline.start(rs_cfg)
                        last_R_t2c = None
                        logger.info('카메라 재연결 완료')
                    except Exception as e:
                        logger.error('카메라 재연결 실패: %s', e)

                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    action = 'save'
                elif key in (ord('s'), ord('S')):
                    action = 'skip'
                elif key in (ord('q'), ord('Q'), 27):
                    action = 'quit'

            if action == 'quit':
                print('사용자 종료.')
                node.move_to(HOME)
                quit_flag = True
                break

            if action == 'skip':
                print('  → 스킵')
                skipped += 1
                continue

            # action == 'save'
            if last_R_t2c is None:
                logger.warning('마커 미감지 상태에서 저장 시도 — 스킵')
                skipped += 1
                continue

            # 품질 검사 — 기울기·거리 기준
            dist_now = float(last_t_t2c[2])
            cos_a_now = abs(float(last_R_t2c[2, 2]))
            face_ang_now = float(np.degrees(np.arccos(np.clip(cos_a_now, 0.0, 1.0))))
            if face_ang_now > MAX_FACE_ANGLE_DEG:
                logger.warning(
                    '마커 기울기 %.1fdeg > %.0fdeg — 평판 베이스에 마커 재부착 필요. 스킵.',
                    face_ang_now, MAX_FACE_ANGLE_DEG)
                skipped += 1
                continue
            if not (MIN_MARKER_DIST_M <= dist_now <= MAX_MARKER_DIST_M):
                logger.warning(
                    '마커 거리 %.3fm (허용 %.2f~%.2fm) — 스킵.',
                    dist_now, MIN_MARKER_DIST_M, MAX_MARKER_DIST_M)
                skipped += 1
                continue

            R_g2b_list.append(R_g2b)
            t_g2b_list.append(t_g2b)
            R_t2c_list.append(last_R_t2c)
            t_t2c_list.append(last_t_t2c)
            print(f'  ✓ 저장 (누적 {len(R_g2b_list)}쌍)'
                  f'  기울기={face_ang_now:.1f}deg  거리={dist_now:.3f}m')

    except KeyboardInterrupt:
        print('\nCtrl+C — 종료합니다.')
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print('\n홈 자세로 복귀 중...')
        node.move_to(HOME)
        node.destroy_node()
        rclpy.shutdown()

    n = len(R_g2b_list)
    print(f'\n수집: {n}쌍  /  스킵: {skipped}쌍')

    if n == 0:
        print('수집된 데이터 없음.')
        return

    if n < MIN_SAMPLES:
        logger.warning('%d쌍 수집 — calibrateHandEye 권장 최소 %d쌍 미달', n, MIN_SAMPLES)
        print('계속 저장하려면 y, 취소하려면 n: ', end='')
        if input().strip().lower() != 'y':
            print('저장 취소.')
            return

    np.savez(
        _OUTPUT_PATH,
        R_gripper2base=np.stack(R_g2b_list),
        t_gripper2base=np.stack(t_g2b_list),
        R_target2cam=np.stack(R_t2c_list),
        t_target2cam=np.stack(t_t2c_list),
    )
    logger.info('저장 완료: %s  (%d쌍)', _OUTPUT_PATH, n)
    print('다음: python3 scripts/compute_handeye_opencv.py')


if __name__ == '__main__':
    main()
