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

# ── 캘리브레이션 자세 (handeye_calib_motion.py와 동일) ─────────────────────────
# ⚠️ E-1 예외: Doosan MoveJoint API는 degree 입력 (하드웨어 경계 인터페이스)
CALIB_POSES: list[list[float]] = [
    # ── 그룹 A: 위치 다양성 (J1~J3 sweep) ──────────────────────────────────
    [   0.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A1  홈
    [  30.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A2
    [  60.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A3
    [ -30.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A4
    [ -60.0,   0.0,  90.0,   0.0,  90.0,   0.0],  # A5
    [   0.0, -20.0,  90.0,   0.0,  90.0,   0.0],  # A6
    [   0.0, -35.0,  90.0,   0.0,  90.0,   0.0],  # A7
    [   0.0,   0.0,  75.0,   0.0,  90.0,   0.0],  # A8
    [   0.0,   0.0, 105.0,   0.0,  90.0,   0.0],  # A9
    [  30.0, -20.0,  80.0,   0.0,  90.0,   0.0],  # A10
    # ── 그룹 B: J4 변화 (팔뚝 축 회전) ────────────────────────────────────
    [   0.0,   0.0,  90.0,  30.0,  90.0,   0.0],  # B1
    [   0.0,   0.0,  90.0, -30.0,  90.0,   0.0],  # B2
    [   0.0,   0.0,  90.0,  50.0,  90.0,   0.0],  # B3
    [   0.0,   0.0,  90.0, -50.0,  90.0,   0.0],  # B4
    [  30.0,   0.0,  90.0,  35.0,  90.0,   0.0],  # B5
    [ -30.0,   0.0,  90.0, -35.0,  90.0,   0.0],  # B6
    [   0.0, -20.0,  90.0,  30.0,  90.0,   0.0],  # B7
    [   0.0, -20.0,  90.0, -30.0,  90.0,   0.0],  # B8
    # ── 그룹 C: J5 변화 (손목 상하 꺾임) ──────────────────────────────────
    [   0.0,   0.0,  90.0,   0.0,  75.0,   0.0],  # C1
    [   0.0,   0.0,  90.0,   0.0, 105.0,   0.0],  # C2
    [   0.0,   0.0,  90.0,   0.0,  70.0,   0.0],  # C3
    [   0.0,   0.0,  90.0,   0.0, 110.0,   0.0],  # C4
    [  30.0,   0.0,  90.0,   0.0,  75.0,   0.0],  # C5
    [ -30.0,   0.0,  90.0,   0.0, 105.0,   0.0],  # C6
    [   0.0, -20.0,  90.0,   0.0,  75.0,   0.0],  # C7
    [   0.0, -20.0,  90.0,   0.0, 105.0,   0.0],  # C8
    # ── 그룹 D: J6 변화 (툴 축 회전) ──────────────────────────────────────
    [   0.0,   0.0,  90.0,   0.0,  90.0,  30.0],  # D1
    [   0.0,   0.0,  90.0,   0.0,  90.0, -30.0],  # D2
    [   0.0,   0.0,  90.0,   0.0,  90.0,  50.0],  # D3
    [   0.0,   0.0,  90.0,   0.0,  90.0, -50.0],  # D4
    [  30.0,   0.0,  90.0,   0.0,  90.0,  30.0],  # D5
    [ -30.0,   0.0,  90.0,   0.0,  90.0, -30.0],  # D6
    # ── 그룹 E: J4+J5+J6 복합 ──────────────────────────────────────────────
    [   0.0,   0.0,  90.0,  30.0,  75.0,  30.0],  # E1
    [   0.0,   0.0,  90.0, -30.0, 105.0, -30.0],  # E2
    [  30.0, -20.0,  90.0,  25.0,  80.0,  20.0],  # E3
    [ -30.0, -20.0,  90.0, -25.0, 100.0, -20.0],  # E4
    [   0.0, -30.0,  85.0,  20.0,  80.0,  20.0],  # E5
]
HOME: list[float] = CALIB_POSES[0]

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
    label = f'ID={ids.flatten()[target_idx]}  dist={dist_m:.3f}m'
    cv2.putText(display, label, (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
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

            print(f'[{i+1:02d}/{total}] 이동 중... {pose}')

            if not node.move_to(pose):
                logger.warning('이동 실패 — 자동 스킵')
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

                frames = pipeline.poll_for_frames()
                if frames and frames.get_color_frame():
                    color_img = np.asanyarray(frames.get_color_frame().get_data())
                    last_R_t2c, last_t_t2c, display = detect_aruco(color_img)

                    sample_n = len(R_g2b_list)
                    cv2.putText(display, f'Pose {i+1}/{total}  Saved:{sample_n}',
                                (10, IMG_H - 45),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
                    cv2.imshow('ArUco Preview', display)

                key = cv2.waitKey(1) & 0xFF
                if key == ord(' '):
                    action = 'save'
                elif key in (ord('s'), ord('S')):
                    action = 'skip'
                elif key in (ord('q'), ord('Q'), 27):
                    action = 'quit'

            if action == 'quit':
                print('사용자 종료.')
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

            R_g2b_list.append(R_g2b)
            t_g2b_list.append(t_g2b)
            R_t2c_list.append(last_R_t2c)
            t_t2c_list.append(last_t_t2c)
            print(f'  ✓ 저장 완료 (누적 {len(R_g2b_list)}쌍)')

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
