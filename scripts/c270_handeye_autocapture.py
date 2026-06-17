#!/usr/bin/env python3
"""
c270_handeye_autocapture.py — GUI 없이 단발 캡처하는 ChArUco AXB 데이터 수집 도구

사용 시나리오: 로봇은 Windows/DART에서 사용자가 직접 조작하고, 카메라(C270)만 이
컴퓨터에 연결되어 있을 때, 에이전트(Claude Code)가 매 포즈마다 한 번씩 호출해서
캡처→판정→저장을 대신 수행한다. cv2.imshow 같은 GUI 창을 띄우지 않는다.

사용법:
  python3 scripts/c270_handeye_autocapture.py --tw v5.tw --pose 2
    → TW 파일의 2번째 MoveL 포즈를 R_gripper2base로 사용, 카메라 한 프레임 캡처해
      ChArUco 보드 인식 후 품질 기준 통과 시 scripts/c270_handeye_auto_data.npz에 추가.

  python3 scripts/c270_handeye_autocapture.py --status
    → 지금까지 저장된 샘플 수/품질 요약 출력.
"""
import argparse, base64, json, logging, os, sys
import cv2, numpy as np, yaml
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[autocapture] %(message)s')
log = logging.getLogger('c270_handeye_autocapture')

_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(_DIR, 'c270_handeye_auto_data.npz')
FRAME_DIR = os.path.join(_DIR, 'c270_handeye_auto_frames')

# 품질 기준 — 수집 단계는 c270_handeye_collect.py와 동일하게 넉넉히(45°) 받고,
# compute_handeye_opencv.py의 엄격한 필터(22°)는 계산 단계에서 별도로 적용한다.
# (Rz 회전처럼 face_angle은 그대로지만 롤 다양성을 주는 포즈를 수집 단계에서
# 섣불리 버리지 않기 위함 — 22°로 막아두면 정작 필요한 롤 정보가 다 날아간다.)
MAX_FACE_ANGLE_DEG = 45.0
MIN_DIST_M = 0.10
MAX_DIST_M = 0.80
N_WARMUP_FRAMES = 5  # 카메라 노출 안정화를 위해 앞쪽 프레임은 버림


def _load_yaml(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


_CFG_DIR = os.path.join(_DIR, '..', 'config')
_cam = _load_yaml(os.path.join(_CFG_DIR, 'c270_camera_info.yaml'))['intrinsics']
_rt = _load_yaml(os.path.join(_CFG_DIR, 'runtime.yaml'))
_calib = _rt.get('calibration', {})

CAMERA_MATRIX = np.array([
    [_cam['fx'], 0.0, _cam['cx']],
    [0.0, _cam['fy'], _cam['cy']],
    [0.0, 0.0, 1.0],
], dtype=np.float64)
DIST_COEFFS = np.array(_cam['coeffs'], dtype=np.float64)

DEVICE: str = _calib.get('c270_device', '/dev/video2')
WIDTH: int = int(_calib.get('c270_width', 640))
HEIGHT: int = int(_calib.get('c270_height', 480))
SQUARES_X: int = int(_calib.get('charuco_squares_x', 5))
SQUARES_Y: int = int(_calib.get('charuco_squares_y', 7))
SQUARE_SIZE_M: float = float(_calib.get('charuco_square_size_m', 0.038))
MARKER_SIZE_M: float = float(_calib.get('charuco_marker_size_m', 0.0315))
ARUCO_DICT_ID: int = int(_calib.get('aruco_dict_id', cv2.aruco.DICT_4X4_50))
MIN_CORNERS: int = int(_calib.get('min_charuco_corners', 6))

_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
_ARUCO_PARAMS = cv2.aruco.DetectorParameters()
CHARUCO_BOARD = cv2.aruco.CharucoBoard(
    (SQUARES_X, SQUARES_Y), SQUARE_SIZE_M, MARKER_SIZE_M, _ARUCO_DICT)


# ── TW 파서 (c270_tcp_center_calib.py와 동일 형식) ───────────────────────────
def _extract_movel(node, results: list) -> None:
    if isinstance(node, dict):
        if node.get('_type') == 'MoveLNode':
            p = node['_pojo']
            pose = p.get('pose', {})
            if 'pose1' in pose:
                results.append({
                    'ann': p.get('annotation', ''),
                    'X': float(pose['pose1']), 'Y': float(pose['pose2']),
                    'Z': float(pose['pose3']), 'A': float(pose['pose4']),
                    'B': float(pose['pose5']), 'C': float(pose['pose6']),
                })
        for v in node.values():
            _extract_movel(v, results)
    elif isinstance(node, list):
        for item in node:
            _extract_movel(item, results)


def load_tw(tw_path: str) -> list[dict]:
    with open(tw_path, 'rb') as f:
        data = base64.b64decode(f.read())
    poses: list[dict] = []
    _extract_movel(json.loads(data), poses)
    return poses


def dart_to_Rt(x: float, y: float, z: float,
               a: float, b: float, c: float) -> tuple[np.ndarray, np.ndarray]:
    """Doosan 공식: TCP (a,b,c)는 Euler ZYZ."""
    t = np.array([x / 1000, y / 1000, z / 1000])
    R = Rotation.from_euler('ZYZ', [a, b, c], degrees=True).as_matrix()
    return R, t


# ── 캡처 + 검출 ──────────────────────────────────────────────────────────────
def grab_frame() -> np.ndarray:
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        log.error('카메라 열기 실패: %s', DEVICE)
        sys.exit(1)
    frame = None
    for _ in range(N_WARMUP_FRAMES + 1):
        ret, frame = cap.read()
        if not ret:
            cap.release()
            log.error('프레임 읽기 실패')
            sys.exit(1)
    cap.release()
    return frame


def detect_board(frame: np.ndarray):
    """ChArUco 보드 인식. 성공 시 (R_target2cam, t_target2cam, face_angle_deg, dist_m), 실패 시 None 4개."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, _ARUCO_DICT, parameters=_ARUCO_PARAMS)
    if ids is None or len(ids) < 1:
        return None, None, None, None, 'NO_MARKER'

    retval, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, CHARUCO_BOARD, CAMERA_MATRIX, DIST_COEFFS)
    if retval < MIN_CORNERS:
        return None, None, None, None, f'corners={retval}(min {MIN_CORNERS})'

    rvec0 = np.zeros((3, 1)); tvec0 = np.zeros((3, 1))
    valid, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
        ch_corners, ch_ids, CHARUCO_BOARD, CAMERA_MATRIX, DIST_COEFFS, rvec0, tvec0)
    if not valid:
        return None, None, None, None, 'POSE_ESTIMATION_FAILED'

    R, _ = cv2.Rodrigues(rvec)
    t = tvec.flatten()
    face_angle = float(np.degrees(np.arccos(np.clip(abs(float(R[2, 2])), 0.0, 1.0))))
    dist = float(np.linalg.norm(t))
    return R, t, face_angle, dist, 'OK'


def _load_existing() -> tuple[list, list, list, list]:
    if not os.path.exists(DATA_PATH):
        return [], [], [], []
    d = np.load(DATA_PATH)
    return (list(d['R_gripper2base']), list(d['t_gripper2base']),
            list(d['R_target2cam']), list(d['t_target2cam']))


def _save(R_g2b, t_g2b, R_t2c, t_t2c) -> None:
    np.savez(DATA_PATH,
             R_gripper2base=np.stack(R_g2b), t_gripper2base=np.stack(t_g2b),
             R_target2cam=np.stack(R_t2c), t_target2cam=np.stack(t_t2c))


def capture_one(tw_path: str, pose_num: int, force: bool) -> None:
    tw_poses = load_tw(os.path.expanduser(tw_path))
    if not (1 <= pose_num <= len(tw_poses)):
        log.error('포즈 번호 범위 초과 (1~%d)', len(tw_poses))
        sys.exit(1)
    p = tw_poses[pose_num - 1]
    R_g2b, t_g2b = dart_to_Rt(p['X'], p['Y'], p['Z'], p['A'], p['B'], p['C'])
    log.info('TW 포즈 %d  ann=%s  TCP=[%.1f,%.1f,%.1f]mm',
             pose_num, p['ann'], p['X'], p['Y'], p['Z'])

    frame = grab_frame()
    os.makedirs(FRAME_DIR, exist_ok=True)
    frame_path = os.path.join(FRAME_DIR, f'pose{pose_num:02d}.jpg')
    cv2.imwrite(frame_path, frame)  # intrinsic 변경 등으로 재계산 필요할 때를 위해 원본 보관
    R_t2c, t_t2c, face_angle, dist, status = detect_board(frame)

    if status != 'OK':
        log.warning('검출 실패: %s — 저장 안 함. 마커가 화면에 잘 보이는지 확인 후 다시 호출하세요.', status)
        return

    quality_ok = (face_angle <= MAX_FACE_ANGLE_DEG and MIN_DIST_M <= dist <= MAX_DIST_M)
    tag = 'OK' if quality_ok else 'WARN(기준 미달)'
    log.info('검출 성공 — face_angle=%.1f°(기준≤%.0f)  dist=%.3fm(기준%.2f~%.2f)  [%s]',
             face_angle, MAX_FACE_ANGLE_DEG, dist, MIN_DIST_M, MAX_DIST_M, tag)

    if not quality_ok and not force:
        log.warning('품질 기준 미달 — 저장 안 함. 마커를 더 정면으로/적정 거리로 맞추고 다시 호출하거나 --force로 강제 저장하세요.')
        return

    R_g2b_l, t_g2b_l, R_t2c_l, t_t2c_l = _load_existing()
    R_g2b_l.append(R_g2b); t_g2b_l.append(t_g2b)
    R_t2c_l.append(R_t2c); t_t2c_l.append(t_t2c)
    _save(R_g2b_l, t_g2b_l, R_t2c_l, t_t2c_l)
    log.info('저장 완료 — 누적 %d쌍 → %s', len(R_g2b_l), DATA_PATH)


def recompute_from_frames(tw_path: str) -> None:
    """FRAME_DIR에 저장된 원본 사진들을 현재 intrinsic/보드 설정으로 다시 검출.

    카메라 intrinsic을 바꾼 뒤 로봇을 다시 움직이지 않고 재계산할 때 사용.
    """
    if not os.path.isdir(FRAME_DIR):
        log.error('저장된 프레임 없음: %s', FRAME_DIR)
        sys.exit(1)
    tw_poses = load_tw(os.path.expanduser(tw_path))

    R_g2b_l, t_g2b_l, R_t2c_l, t_t2c_l = [], [], [], []
    files = sorted(glob.glob(os.path.join(FRAME_DIR, 'pose*.jpg')))
    log.info('저장된 프레임 %d개로 재계산 시작', len(files))
    for f in files:
        pose_num = int(os.path.basename(f)[4:6])
        p = tw_poses[pose_num - 1]
        R_g2b, t_g2b = dart_to_Rt(p['X'], p['Y'], p['Z'], p['A'], p['B'], p['C'])
        frame = cv2.imread(f)
        R_t2c, t_t2c, face_angle, dist, status = detect_board(frame)
        if status != 'OK':
            log.warning('포즈 %d: 재검출 실패(%s) — 제외', pose_num, status)
            continue
        quality_ok = (face_angle <= MAX_FACE_ANGLE_DEG and MIN_DIST_M <= dist <= MAX_DIST_M)
        log.info('포즈 %d: face_angle=%.1f° dist=%.3fm [%s]',
                 pose_num, face_angle, dist, 'OK' if quality_ok else 'WARN')
        if not quality_ok:
            continue
        R_g2b_l.append(R_g2b); t_g2b_l.append(t_g2b)
        R_t2c_l.append(R_t2c); t_t2c_l.append(t_t2c)

    _save(R_g2b_l, t_g2b_l, R_t2c_l, t_t2c_l)
    log.info('재계산 완료 — %d/%d쌍 저장 → %s', len(R_g2b_l), len(files), DATA_PATH)


def status() -> None:
    if not os.path.exists(DATA_PATH):
        log.info('저장된 데이터 없음')
        return
    R_g2b, t_g2b, R_t2c, t_t2c = _load_existing()
    n = len(R_g2b)
    log.info('누적 %d쌍', n)
    n_ok = 0
    for i in range(n):
        cos_a = abs(float(R_t2c[i][2, 2]))
        angle = float(np.degrees(np.arccos(np.clip(cos_a, 0.0, 1.0))))
        dist = float(np.linalg.norm(t_t2c[i]))
        ok = angle <= MAX_FACE_ANGLE_DEG and MIN_DIST_M <= dist <= MAX_DIST_M
        n_ok += ok
        log.info('  %2d: face_angle=%5.1f°  dist=%.3fm  [%s]',
                 i + 1, angle, dist, 'OK' if ok else 'WARN')
    log.info('품질 기준 통과: %d/%d', n_ok, n)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--tw', help='TW 파일 경로')
    parser.add_argument('--pose', type=int, help='TW 파일 내 포즈 번호 (1-indexed)')
    parser.add_argument('--force', action='store_true', help='품질 기준 미달이어도 강제 저장')
    parser.add_argument('--status', action='store_true', help='누적 데이터 현황 출력')
    args = parser.parse_args()

    if args.status:
        status()
    elif args.tw and args.pose:
        capture_one(args.tw, args.pose, args.force)
    else:
        parser.print_help()
