#!/usr/bin/env python3
"""
teach_collect.py — 직접교시로 hand-eye 캘리브레이션 데이터 수집

사용법:
  1. python3 scripts/teach_collect.py
  2. 로봇 팔을 원하는 자세로 직접 이동 (직접교시 모드)
  3. SPACE: 저장 / S: 스킵 / Q: 종료
  4. 종료 시 홈 복귀 + handeye_data.npz + 관절각 출력

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

from dsr_msgs2.srv import MoveJoint, SetRobotMode
from dsr_msgs2.msg import RobotState

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(levelname)s: %(message)s')
logger = logging.getLogger('teach_collect')

# ── 경로 ──────────────────────────────────────────────────────────────────────
_SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
_CONFIG_DIR  = os.path.join(_SCRIPT_DIR, '..', 'config')
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


_runtime  = _load_yaml(os.path.join(_CONFIG_DIR, 'runtime.yaml'))
_cam_info = _load_yaml(os.path.join(_CONFIG_DIR, 'camera_info.yaml')).get('intrinsics', {})
_calib    = _runtime.get('calibration', {})

VEL:           float = float(_calib.get('motion_vel_deg_per_s', 10.0))
ACC:           float = float(_calib.get('motion_acc_deg_per_s2',  5.0))
SVC_TIMEOUT:   float = float(_calib.get('service_timeout_sec',    5.0))
MARKER_SIZE_M: float = float(_calib.get('aruco_marker_size_m',  0.05))
ARUCO_DICT_ID: int   = int(_calib.get('aruco_dict_id', cv2.aruco.DICT_4X4_50))
ARUCO_TARGET_ID: int = int(_calib.get('aruco_target_id', 0))
BASE_FRAME:    str   = str(_calib.get('base_frame',    'base_link'))
GRIPPER_FRAME: str   = str(_calib.get('gripper_frame', 'link_6'))

CAMERA_MATRIX = np.array([
    [_cam_info['fx'], 0.0,             _cam_info['cx']],
    [0.0,            _cam_info['fy'],  _cam_info['cy']],
    [0.0,            0.0,              1.0            ],
], dtype=np.float64)
DIST_COEFFS = np.array(_cam_info['coeffs'], dtype=np.float64)
IMG_W: int = _cam_info['width']
IMG_H: int = _cam_info['height']

HOME: list[float] = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# Robot mode 상수 (dsr_msgs2/srv/SetRobotMode 기준)
# 0=MANUAL, 1=AUTONOMOUS, 2=MEASURE
ROBOT_MODE_MANUAL:     int = 0
ROBOT_MODE_AUTONOMOUS: int = 1

# OpenCV 4.5.x 구 API
_ARUCO_DICT   = cv2.aruco.Dictionary_get(ARUCO_DICT_ID)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters_create()

def _make_obj_pts(size_m: float) -> np.ndarray:
    h = size_m / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
                    dtype=np.float32)

OBJ_PTS = _make_obj_pts(MARKER_SIZE_M)


# ── ROS2 노드 ──────────────────────────────────────────────────────────────────
class TeachCollectNode(Node):
    def __init__(self) -> None:
        super().__init__('teach_collect')
        self._move_cli = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self._mode_cli = self.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._current_posj: list[float] = [0.0] * 6
        self.create_subscription(RobotState, '/dsr01/state', self._state_cb, 10)

        self.get_logger().info('move_joint 서비스 연결 대기 중...')
        if not self._move_cli.wait_for_service(timeout_sec=SVC_TIMEOUT):
            raise RuntimeError('move_joint 서비스 연결 실패 — 로봇 드라이버 실행 여부 확인')
        self.get_logger().info('연결 완료.')

    def _state_cb(self, msg: RobotState) -> None:
        if len(msg.current_posj) >= 6:
            self._current_posj = list(msg.current_posj[:6])

    def set_robot_mode(self, mode: int) -> bool:
        if not self._mode_cli.wait_for_service(timeout_sec=2.0):
            return False
        req = SetRobotMode.Request()
        req.robot_mode = mode
        future = self._mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.done() and future.result() is not None

    def move_to(self, pos: list[float]) -> bool:
        req = MoveJoint.Request()
        req.pos        = [float(p) for p in pos]
        req.vel        = VEL
        req.acc        = ACC
        req.time       = 0.0
        req.radius     = 0.0
        req.mode       = 0
        req.blend_type = 0
        req.sync_type  = 0
        future = self._move_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('move_joint 타임아웃')
            return False
        return bool(future.result().success)

    def get_gripper2base(self) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                BASE_FRAME, GRIPPER_FRAME, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().error('TF lookup 실패: %s', e)
            return None
        tr = tf.transform.translation
        ro = tf.transform.rotation
        t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
        R = Rotation.from_quat([ro.x, ro.y, ro.z, ro.w]).as_matrix()
        return R, t

    def get_posj_deg(self) -> list[float]:
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        return list(self._current_posj)


# ── ArUco 감지 ─────────────────────────────────────────────────────────────────
def detect_aruco(
    color_img: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray]:
    gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS)
    display = color_img.copy()

    if ids is None:
        cv2.putText(display, 'NO MARKER', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        return None, None, display

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

    R_t2c, _ = cv2.Rodrigues(rvec)
    t_t2c: np.ndarray = tvec.flatten()

    cv2.drawFrameAxes(display, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, MARKER_SIZE_M * 0.5)
    dist_m = float(np.linalg.norm(t_t2c))
    cv2.putText(display, f'ID={ids.flatten()[target_idx]}  dist={dist_m:.3f}m',
                (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(display, 'SPACE:save  S:skip  Q:quit',
                (10, IMG_H - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
    return R_t2c, t_t2c, display


# ── 메인 ───────────────────────────────────────────────────────────────────────
def main() -> None:
    rclpy.init()
    try:
        node = TeachCollectNode()
    except RuntimeError as e:
        logger.error('%s', e)
        rclpy.shutdown()
        sys.exit(1)

    # 홈으로 이동
    print('\n홈 자세로 이동 중...')
    node.move_to(HOME)

    # 직접교시 모드 전환 시도
    print('\n직접교시 모드 전환 시도 중...')
    if node.set_robot_mode(ROBOT_MODE_MANUAL):
        print('✓ 직접교시 모드 활성화 — 로봇 팔을 원하는 자세로 이동하세요.')
    else:
        print('△ 자동 전환 실패 — 로봇 교시패널에서 직접교시 버튼을 눌러주세요.')
        print('  준비되면 Enter 키를 누르세요.', end=' ')
        input()

    # RealSense 시작
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

    R_g2b_list:   list[np.ndarray] = []
    t_g2b_list:   list[np.ndarray] = []
    R_t2c_list:   list[np.ndarray] = []
    t_t2c_list:   list[np.ndarray] = []
    posj_list:    list[list[float]] = []

    print('\n=== 직접교시 데이터 수집 ===')
    print('조작: SPACE=저장  S=스킵  Q=종료\n')

    last_R_t2c: np.ndarray | None = None
    last_t_t2c: np.ndarray | None = None

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0)

            frames = pipeline.poll_for_frames()
            if frames and frames.get_color_frame():
                color_img = np.asanyarray(frames.get_color_frame().get_data())
                last_R_t2c, last_t_t2c, display = detect_aruco(color_img)

                n = len(R_g2b_list)
                status = f'Saved:{n}  {"MARKER OK" if last_R_t2c is not None else "NO MARKER"}'
                cv2.putText(display, status, (10, IMG_H - 45),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1)
                cv2.imshow('ArUco Preview', display)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), ord('Q'), 27):
                print('종료.')
                break

            if key in (ord('s'), ord('S')):
                print('  → 스킵')
                continue

            if key == ord(' '):
                if last_R_t2c is None:
                    logger.warning('마커 미감지 — 저장 불가')
                    continue

                result = node.get_gripper2base()
                if result is None:
                    logger.warning('TF 읽기 실패 — 재시도하세요')
                    continue

                R_g2b, t_g2b = result
                posj = node.get_posj_deg()

                R_g2b_list.append(R_g2b)
                t_g2b_list.append(t_g2b)
                R_t2c_list.append(last_R_t2c)
                t_t2c_list.append(last_t_t2c)
                posj_list.append(posj)

                n = len(R_g2b_list)
                print(f'  ✓ [{n:02d}] 저장  posj(deg)=[{posj[0]:.1f}, {posj[1]:.1f}, '
                      f'{posj[2]:.1f}, {posj[3]:.1f}, {posj[4]:.1f}, {posj[5]:.1f}]')
                print(f'       t_gripper=[{t_g2b[0]:.3f}, {t_g2b[1]:.3f}, {t_g2b[2]:.3f}] m')

    except KeyboardInterrupt:
        print('\nCtrl+C — 종료합니다.')
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    n = len(R_g2b_list)
    print(f'\n수집 완료: {n}쌍')

    # 직접교시 모드 해제 후 홈 복귀
    print('자율 모드로 전환 후 홈 복귀 중...')
    node.set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    node.move_to(HOME)
    node.destroy_node()
    rclpy.shutdown()

    if n == 0:
        print('저장된 데이터 없음.')
        return

    # 관절각 요약 출력 (캘리브레이션 자세 생성용)
    print('\n' + '='*60)
    print('  저장된 관절각 (캘리브레이션 자세 생성용)')
    print('  단위: degree  [J1, J2, J3, J4, J5, J6]')
    print('='*60)
    for i, posj in enumerate(posj_list):
        print(f'  [{i+1:02d}]  [{posj[0]:6.1f}, {posj[1]:6.1f}, {posj[2]:6.1f},'
              f' {posj[3]:6.1f}, {posj[4]:6.1f}, {posj[5]:6.1f}]')
    print('='*60)

    # npz 저장
    np.savez(
        _OUTPUT_PATH,
        R_gripper2base=np.stack(R_g2b_list),
        t_gripper2base=np.stack(t_g2b_list),
        R_target2cam=np.stack(R_t2c_list),
        t_target2cam=np.stack(t_t2c_list),
        joint_angles_deg=np.array(posj_list),
    )
    logger.info('저장 완료: %s  (%d쌍)', _OUTPUT_PATH, n)
    print('\n다음: 위 관절각을 보고 캘리브레이션 자세 세트 생성')
    print('      또는: python3 scripts/compute_handeye_opencv.py --all')


if __name__ == '__main__':
    main()
