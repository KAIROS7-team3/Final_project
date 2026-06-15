#!/usr/bin/env python3
"""
c270_tcp_center_calib.py — 고정 마커 기준 핸드아이 캘리브레이션

원리:
  테이블에 고정 마커(점/핀/스티커)를 하나 붙여둔다.
  로봇을 높이별로 이동해서 마커가 카메라 정중앙 십자선에 오도록 XY 조정.
  각 포즈의 TCP 좌표 기록 → T_cam2ee 계산.

사용법:
  python3 scripts/c270_tcp_center_calib.py --collect --tw file.tw  # TW 파일 자동 로드
  python3 scripts/c270_tcp_center_calib.py --collect               # 수동 입력
  python3 scripts/c270_tcp_center_calib.py --compute
  python3 scripts/c270_tcp_center_calib.py --compute --t 0.0 0.033 0.0825
"""
import argparse, base64, json, logging, os, sys, yaml, datetime
import cv2, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(message)s')
log = logging.getLogger('c270_tcp_center_calib')

_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(_DIR, 'c270_tcp_center_data.npz')


def _load_runtime() -> dict:
    p = os.path.join(_DIR, '..', 'config', 'runtime.yaml')
    with open(p) as f:
        return yaml.safe_load(f)


def _load_K() -> tuple[np.ndarray, np.ndarray, float, float]:
    p = os.path.join(_DIR, '..', 'config', 'c270_camera_info.yaml')
    with open(p) as f:
        intr = yaml.safe_load(f)['intrinsics']
    K = np.array([[intr['fx'], 0, intr['cx']],
                  [0, intr['fy'], intr['cy']],
                  [0, 0, 1]], dtype=np.float64)
    D = np.array(intr['coeffs'], dtype=np.float64)
    return K, D, intr['cx'], intr['cy']


_RT = _load_runtime()
_CAL = _RT['calibration']
DEVICE: str = _CAL['c270_device']
WIDTH: int = _CAL['c270_width']
HEIGHT: int = _CAL['c270_height']

K, D, CX, CY = _load_K()


def dart_to_Rt(x: float, y: float, z: float,
               a: float, b: float, c: float) -> tuple[np.ndarray, np.ndarray]:
    t = np.array([x / 1000, y / 1000, z / 1000])
    R = Rotation.from_euler('ZYZ', [a, b, c], degrees=True).as_matrix()
    return R, t


def parse_tcp(line: str) -> tuple[np.ndarray | None, np.ndarray | None]:
    vals = [float(v) for v in line.strip().split()]
    if len(vals) != 6:
        return None, None
    return dart_to_Rt(*vals)


# ── TW 파일 파서 ────────────────────────────────────────────────────────────
def _extract_movel(node: dict | list, results: list) -> None:
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


def proj_SO3(R: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(R)
    R2 = U @ Vt
    if np.linalg.det(R2) < 0:
        U[:, -1] *= -1
        R2 = U @ Vt
    return R2


# ── 수집 모드 ───────────────────────────────────────────────────────────────
def collect(tw_path: str | None = None) -> None:
    tw_poses: list[dict] = []
    if tw_path:
        tw_path = os.path.expanduser(tw_path)
        if not os.path.exists(tw_path):
            log.error('TW 파일 없음: %s', tw_path)
            sys.exit(1)
        tw_poses = load_tw(tw_path)
        log.info('TW 파일 로드 — %d개 MoveL 포즈', len(tw_poses))

    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        log.error('카메라 열기 실패: %s', DEVICE)
        sys.exit(1)

    R_list: list[np.ndarray] = []
    t_list: list[np.ndarray] = []

    if os.path.exists(DATA_PATH):
        d = np.load(DATA_PATH)
        R_list = list(d['R_ee2base'])
        t_list = list(d['t_ee2base'])
        log.info('기존 %d포즈 이어받기', len(R_list))

    pose_idx = len(R_list)

    log.info('=== 고정 마커 기준 핸드아이 캘리브레이션 ===')
    log.info('준비: 테이블에 마커(차르코보드 코너 등) 하나 고정')
    if tw_poses:
        log.info('TW 모드: 로봇을 각 포즈로 이동 후 마커를 십자선에 맞추고 ENTER')
        log.info('현재 포즈: %d/%d', pose_idx + 1, len(tw_poses))
    else:
        log.info('수동 모드: 마커가 십자선에 오도록 로봇 XY 조정 → ENTER → TCP 입력')
    log.info('ENTER=저장  D=마지막삭제  Q=종료')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        disp = frame.copy()
        h, w = disp.shape[:2]
        cx_i, cy_i = int(CX), int(CY)

        cv2.line(disp, (0, cy_i), (w, cy_i), (0, 255, 0), 1)
        cv2.line(disp, (cx_i, 0), (cx_i, h), (0, 255, 0), 1)
        cv2.circle(disp, (cx_i, cy_i), 20, (0, 255, 255), 2)

        if tw_poses:
            info = f'pose {pose_idx+1}/{len(tw_poses)}  saved={len(R_list)}  ENTER:save  D:del  Q:quit'
        else:
            info = f'saved={len(R_list)}  ENTER:save  D:del  Q:quit'
        cv2.putText(disp, info, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(disp, 'Center marker at crosshair, then ENTER',
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow('TCP-in-Center Calib', disp)
        key = cv2.waitKey(30) & 0xFF

        if key == 13:  # ENTER
            if tw_poses:
                if pose_idx >= len(tw_poses):
                    log.info('TW 파일의 모든 포즈 완료')
                    break
                p = tw_poses[pose_idx]
                R, t = dart_to_Rt(p['X'], p['Y'], p['Z'], p['A'], p['B'], p['C'])
                log.info('[%d] TW포즈%d ann=%s TCP=[%.3f,%.3f,%.3f]m 저장',
                         len(R_list) + 1, pose_idx + 1, p['ann'], t[0], t[1], t[2])
                pose_idx += 1
            else:
                cv2.destroyWindow('TCP-in-Center Calib')
                tcp = input('  DART TCP (X Y Z A B C  mm/deg): ').strip()
                cv2.namedWindow('TCP-in-Center Calib')
                R, t = parse_tcp(tcp)
                if R is None:
                    log.warning('입력 오류 (숫자 6개)')
                    continue
                log.info('[%d] 저장 — TCP=[%.3f,%.3f,%.3f]m', len(R_list) + 1, t[0], t[1], t[2])

            R_list.append(R)
            t_list.append(t)
            np.savez(DATA_PATH,
                     R_ee2base=np.stack(R_list),
                     t_ee2base=np.stack(t_list))

            if tw_poses and pose_idx >= len(tw_poses):
                log.info('TW 파일의 모든 포즈 완료')
                break

        elif key == ord('d') and R_list:
            R_list.pop()
            t_list.pop()
            pose_idx = max(0, pose_idx - 1)
            if R_list:
                np.savez(DATA_PATH,
                         R_ee2base=np.stack(R_list),
                         t_ee2base=np.stack(t_list))
            log.info('마지막 삭제 — 남은 %d포즈  TW idx=%d', len(R_list), pose_idx + 1)

        elif key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    log.info('수집 완료: %d포즈 → %s', len(R_list), DATA_PATH)
    if len(R_list) >= 5:
        log.info('compute 실행: python3 scripts/c270_tcp_center_calib.py --compute')


# ── 계산 모드 ───────────────────────────────────────────────────────────────
def compute(t_fixed: np.ndarray | None = None) -> None:
    if not os.path.exists(DATA_PATH):
        log.error('데이터 없음: %s', DATA_PATH)
        sys.exit(1)

    d = np.load(DATA_PATH)
    R_ee2base = list(d['R_ee2base'])
    t_ee2base = list(d['t_ee2base'])
    N = len(R_ee2base)
    log.info('포즈 수: %d', N)

    if N < 4:
        log.error('최소 4포즈 필요')
        sys.exit(1)

    # 원리:
    #   고정 마커 월드 좌표 P (미지수)
    #   포즈 i에서 마커가 이미지 정중앙 → 광학축이 마커를 가리킴
    #   카메라 월드 위치: p_cam_i = t_ee2base[i] + R_ee2base[i] @ t_cam2ee
    #   광학축 방향(월드): d_i = R_ee2base[i] @ R_cam2ee @ [0,0,1]
    #   제약: P = p_cam_i + λ_i * d_i
    #
    # 미지수: R_cam2ee(3), t_cam2ee(3), P(3), λ_i(N)  → 9+N
    # 방정식: 3*N  →  N≥5 이면 overdetermined

    ray_cam = np.array([0., 0., 1.])

    def _build_residuals(t_ce: np.ndarray):
        def residuals(p: np.ndarray) -> list[float]:
            rv = p[:3]
            P_world = p[3:6]
            lam = p[6:6 + N]
            R_ce = Rotation.from_rotvec(rv).as_matrix()
            res = []
            for i in range(N):
                Ree = R_ee2base[i]; tee = t_ee2base[i]
                p_cam = tee + Ree @ t_ce
                d = Ree @ R_ce @ ray_cam
                P_est = p_cam + lam[i] * d
                res.extend(((P_est - P_world) * 1000).tolist())
            return res
        return residuals

    t_mean = np.mean(t_ee2base, axis=0)
    P0 = t_mean.copy(); P0[2] = 0.0
    lam0 = np.full(N, t_mean[2])

    if t_fixed is not None:
        log.info('translation 고정: %s', t_fixed)
        residuals_fn = _build_residuals(np.array(t_fixed))
        p0 = np.concatenate([[0, 0, 0], P0, lam0])
        result = least_squares(residuals_fn, p0, method='lm',
                               max_nfev=5000, ftol=1e-12, xtol=1e-12)
        p_opt = result.x
        R_opt = proj_SO3(Rotation.from_rotvec(p_opt[:3]).as_matrix())
        t_opt = np.array(t_fixed)
        lam_opt = p_opt[6:6 + N]
        P_opt = p_opt[3:6]
    else:
        def residuals_full(p: np.ndarray) -> list[float]:
            rv = p[:3]; t_ce = p[3:6]; P_world = p[6:9]; lam = p[9:9 + N]
            R_ce = Rotation.from_rotvec(rv).as_matrix()
            res = []
            for i in range(N):
                Ree = R_ee2base[i]; tee = t_ee2base[i]
                p_cam = tee + Ree @ t_ce
                d = Ree @ R_ce @ ray_cam
                P_est = p_cam + lam[i] * d
                res.extend(((P_est - P_world) * 1000).tolist())
            return res

        p0 = np.concatenate([[0, 0, 0], [0, 0.033, 0.0825], P0, lam0])
        result = least_squares(residuals_full, p0, method='lm',
                               max_nfev=5000, ftol=1e-12, xtol=1e-12)
        p_opt = result.x
        R_opt = proj_SO3(Rotation.from_rotvec(p_opt[:3]).as_matrix())
        t_opt = p_opt[3:6]
        P_opt = p_opt[6:9]
        lam_opt = p_opt[9:9 + N]

    residuals_fn = _build_residuals(t_opt)
    p_check = np.concatenate([Rotation.from_matrix(R_opt).as_rotvec(), P_opt, lam_opt])
    res_val = np.array(residuals_fn(p_check)).reshape(N, 3)
    errs = np.linalg.norm(res_val, axis=1)

    log.info('=== 결과 ===')
    log.info('마커 추정 위치: [%.1f, %.1f, %.1f] mm',
             P_opt[0] * 1000, P_opt[1] * 1000, P_opt[2] * 1000)
    log.info('잔차: 평균=%.1fmm  최대=%.1fmm', errs.mean(), errs.max())
    for i in range(N):
        log.info('  포즈 %d: %.1fmm  λ=%.0fmm', i + 1, errs[i], lam_opt[i] * 1000)

    quat = Rotation.from_matrix(R_opt).as_quat()
    tilt = float(np.degrees(np.arccos(np.clip(abs(float(R_opt[2, 2])), 0, 1))))
    T_show = np.eye(4); T_show[:3, :3] = R_opt; T_show[:3, 3] = t_opt
    log.info('T_cam2gripper:\n%s', np.round(T_show, 4))
    log.info('translation: x=%.4f  y=%.4f  z=%.4f m', t_opt[0], t_opt[1], t_opt[2])
    log.info('quaternion : x=%.4f  y=%.4f  z=%.4f  w=%.4f',
             quat[0], quat[1], quat[2], quat[3])
    log.info('카메라 기울기: %.1f°', tilt)

    cfg_path = os.path.join(_DIR, '..', 'config', 'c270_hand_eye.yaml')
    data = {
        'schema_version': 1,
        'transformation': {
            'rotation': {'x': float(quat[0]), 'y': float(quat[1]),
                         'z': float(quat[2]), 'w': float(quat[3])},
            'translation': {'x': float(t_opt[0]), 'y': float(t_opt[1]), 'z': float(t_opt[2])},
        },
        'metadata': {
            'calibration_date': datetime.date.today().isoformat(),
            'sample_count': N,
            'method': 'fixed_marker_ray',
            'mean_err_mm': float(errs.mean()),
            'max_err_mm': float(errs.max()),
            'cam_tilt_deg': tilt,
            'marker_world_mm': [float(P_opt[0] * 1000), float(P_opt[1] * 1000),
                                 float(P_opt[2] * 1000)],
        }
    }
    with open(cfg_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    log.info('저장: %s', cfg_path)


# ── entry point ─────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--collect', action='store_true')
    parser.add_argument('--compute', action='store_true')
    parser.add_argument('--tw', default=None,
                        help='DART TW 파일 경로 (--collect 와 함께 사용)')
    parser.add_argument('--t', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='translation 고정 (m, 예: 0.0 0.033 0.0825)')
    args = parser.parse_args()

    if args.collect:
        collect(tw_path=args.tw)
    elif args.compute:
        t_fixed = np.array(args.t) if args.t else None
        compute(t_fixed)
    else:
        parser.print_help()
