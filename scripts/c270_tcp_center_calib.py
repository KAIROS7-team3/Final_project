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
import argparse, base64, json, os, sys, yaml, datetime
import cv2, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(_DIR, 'c270_tcp_center_data.npz')


def _load_K():
    p = os.path.join(_DIR, '..', 'config', 'c270_camera_info.yaml')
    with open(p) as f:
        intr = yaml.safe_load(f)['intrinsics']
    K = np.array([[intr['fx'], 0, intr['cx']],
                  [0, intr['fy'], intr['cy']],
                  [0, 0, 1]], dtype=np.float64)
    D = np.array(intr['coeffs'], dtype=np.float64)
    return K, D, intr['cx'], intr['cy']

K, D, CX, CY = _load_K()


def dart_to_Rt(x, y, z, a, b, c):
    t = np.array([x / 1000, y / 1000, z / 1000])
    R = Rotation.from_euler('ZYZ', [a, b, c], degrees=True).as_matrix()
    return R, t


def parse_tcp(line):
    vals = [float(v) for v in line.strip().split()]
    if len(vals) != 6:
        return None, None
    return dart_to_Rt(*vals)


# ── TW 파일 파서 ────────────────────────────────────────────────────────────
def _extract_movel(node, results):
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


def load_tw(tw_path):
    with open(tw_path, 'rb') as f:
        data = base64.b64decode(f.read())
    poses = []
    _extract_movel(json.loads(data), poses)
    return poses


def proj_SO3(R):
    U, _, Vt = np.linalg.svd(R)
    R2 = U @ Vt
    if np.linalg.det(R2) < 0:
        U[:, -1] *= -1
        R2 = U @ Vt
    return R2


# ── 수집 모드 ───────────────────────────────────────────────────────────────
def collect(tw_path=None):
    tw_poses = []
    if tw_path:
        tw_path = os.path.expanduser(tw_path)
        if not os.path.exists(tw_path):
            print(f"TW 파일 없음: {tw_path}"); sys.exit(1)
        tw_poses = load_tw(tw_path)
        print(f"TW 파일 로드 — {len(tw_poses)}개 MoveL 포즈")

    cap = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print("카메라 열기 실패"); sys.exit(1)

    R_list, t_list = [], []

    if os.path.exists(DATA_PATH):
        d = np.load(DATA_PATH)
        R_list = list(d['R_ee2base'])
        t_list = list(d['t_ee2base'])
        print(f"기존 {len(R_list)}포즈 이어받기")

    pose_idx = len(R_list)  # TW 모드: 이미 저장된 포즈 이후부터 이어받기

    print("\n=== 고정 마커 기준 핸드아이 캘리브레이션 ===")
    print("준비: 테이블에 마커(차르코보드 코너 등) 하나 고정")
    if tw_poses:
        print(f"TW 모드: 로봇을 각 포즈로 이동 후 마커를 십자선에 맞추고 ENTER")
        print(f"현재 포즈: {pose_idx + 1}/{len(tw_poses)}")
    else:
        print("수동 모드: 마커가 십자선에 오도록 로봇 XY 조정 → ENTER → TCP 입력")
    print("ENTER=저장  D=마지막삭제  Q=종료\n")

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
            remaining = len(tw_poses) - pose_idx
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
                    print("TW 파일의 모든 포즈 완료"); break
                p = tw_poses[pose_idx]
                R, t = dart_to_Rt(p['X'], p['Y'], p['Z'], p['A'], p['B'], p['C'])
                print(f"  [{len(R_list)+1}] TW포즈{pose_idx+1} ann={p['ann']} "
                      f"TCP=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}]m 저장")
                pose_idx += 1
            else:
                cv2.destroyWindow('TCP-in-Center Calib')
                tcp = input('  DART TCP (X Y Z A B C  mm/deg): ').strip()
                cv2.namedWindow('TCP-in-Center Calib')
                R, t = parse_tcp(tcp)
                if R is None:
                    print("입력 오류 (숫자 6개)"); continue
                print(f"  [{len(R_list)+1}] 저장 — TCP=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}]m")

            R_list.append(R)
            t_list.append(t)
            np.savez(DATA_PATH,
                     R_ee2base=np.stack(R_list),
                     t_ee2base=np.stack(t_list))

            if tw_poses and pose_idx >= len(tw_poses):
                print("TW 파일의 모든 포즈 완료"); break

        elif key == ord('d') and R_list:
            R_list.pop(); t_list.pop()
            pose_idx = max(0, pose_idx - 1)
            if R_list:
                np.savez(DATA_PATH,
                         R_ee2base=np.stack(R_list),
                         t_ee2base=np.stack(t_list))
            print(f"마지막 삭제 — 남은 {len(R_list)}포즈  TW idx={pose_idx+1}")

        elif key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    print(f"\n수집 완료: {len(R_list)}포즈 → {DATA_PATH}")
    if len(R_list) >= 5:
        print("compute 실행: python3 scripts/c270_tcp_center_calib.py --compute")


# ── 계산 모드 ───────────────────────────────────────────────────────────────
def compute(t_fixed=None):
    if not os.path.exists(DATA_PATH):
        print(f"데이터 없음: {DATA_PATH}"); sys.exit(1)

    d = np.load(DATA_PATH)
    R_ee2base = list(d['R_ee2base'])
    t_ee2base = list(d['t_ee2base'])
    N = len(R_ee2base)
    print(f"포즈 수: {N}")

    if N < 4:
        print("최소 4포즈 필요"); sys.exit(1)

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

    def _build_residuals(t_ce):
        def residuals(p):
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

    # P 초기값: TCP 위치들 평균에서 약간 아래 (마커는 로봇 아래 테이블에 있음)
    t_mean = np.mean(t_ee2base, axis=0)
    P0 = t_mean.copy(); P0[2] = 0.0  # Z=0 (테이블 높이 근사)
    lam0 = np.full(N, t_mean[2])     # 대략 TCP 높이만큼 거리

    if t_fixed is not None:
        print(f"translation 고정: {t_fixed}")
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
        # t_cam2ee도 최적화
        def residuals_full(p):
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

    # 잔차 계산
    residuals_fn = _build_residuals(t_opt)
    p_check = np.concatenate([Rotation.from_matrix(R_opt).as_rotvec(), P_opt, lam_opt])
    res_val = np.array(residuals_fn(p_check)).reshape(N, 3)
    errs = np.linalg.norm(res_val, axis=1)

    print(f"\n=== 결과 ===")
    print(f"마커 추정 위치: [{P_opt[0]*1000:.1f}, {P_opt[1]*1000:.1f}, {P_opt[2]*1000:.1f}] mm")
    print(f"잔차: 평균={errs.mean():.1f}mm  최대={errs.max():.1f}mm")
    for i in range(N):
        print(f"  포즈 {i+1}: {errs[i]:.1f}mm  λ={lam_opt[i]*1000:.0f}mm")

    quat = Rotation.from_matrix(R_opt).as_quat()
    tilt = float(np.degrees(np.arccos(np.clip(abs(float(R_opt[2, 2])), 0, 1))))
    T_show = np.eye(4); T_show[:3, :3] = R_opt; T_show[:3, 3] = t_opt
    print(f"\nT_cam2gripper:")
    print(np.round(T_show, 4))
    print(f"translation: x={t_opt[0]:.4f}  y={t_opt[1]:.4f}  z={t_opt[2]:.4f} m")
    print(f"quaternion : x={quat[0]:.4f}  y={quat[1]:.4f}  z={quat[2]:.4f}  w={quat[3]:.4f}")
    print(f"카메라 기울기: {tilt:.1f}°")

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
            'marker_world_mm': [float(P_opt[0]*1000), float(P_opt[1]*1000), float(P_opt[2]*1000)],
        }
    }
    with open(cfg_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"\n저장: {cfg_path}")


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
