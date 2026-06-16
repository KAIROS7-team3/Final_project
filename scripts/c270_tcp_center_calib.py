#!/usr/bin/env python3
"""
c270_tcp_center_calib.py — 고정 마커 기준 핸드아이 캘리브레이션 (fixed_marker_ray)

원리:
  테이블에 고정 마커(점/핀/스티커)를 하나 붙여둔다.
  로봇을 높이별로 이동해서 마커가 카메라 정중앙 십자선에 오도록 XY 조정.
  각 포즈의 TCP 좌표 기록 → T_cam2ee 계산.

  TW 포즈의 annotation에 "마커"와 "중심"이 모두 포함되면 "TCP=마커 직접 일치" 포즈로
  간주하고, 일반 광선(ray) 샘플과 분리해 ground truth 마커 위치(P_world)로 사용한다
  (예: "차르코마커 중심점_TCP 중심").

사용법:
  python3 scripts/c270_tcp_center_calib.py --collect --tw file.tw  # TW 파일 자동 로드
  python3 scripts/c270_tcp_center_calib.py --collect               # 수동 입력
  python3 scripts/c270_tcp_center_calib.py --compute
  python3 scripts/c270_tcp_center_calib.py --compute --input a.npz b.npz --tw v5.tw --exclude 8
  python3 scripts/c270_tcp_center_calib.py --compute --t 0.0 0.033 0.0825  # translation 고정
  python3 scripts/c270_tcp_center_calib.py --compute --p 0.578 0.0724 -0.0296  # P_world 고정
"""
import argparse, base64, json, logging, os, sys, yaml, datetime
import cv2, numpy as np
from scipy.optimize import least_squares, differential_evolution
from scipy.spatial.transform import Rotation

logging.basicConfig(level=logging.INFO, format='[%(name)s] %(message)s')
log = logging.getLogger('c270_tcp_center_calib')

_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(_DIR, 'c270_tcp_center_data.npz')

# 잔차 robust loss 임계값 (mm). soft_l1은 이 값 근방까지는 일반 제곱오차처럼,
# 그 이상은 가중치를 줄여 outlier가 전체 해를 끌고 가지 못하게 한다.
ROBUST_F_SCALE_MM = 2.0
# bounds용 위치 여유폭 (m). 수집된 EE 위치 범위 밖으로 마커/카메라가 있을 가능성은
# 낮으므로 데이터 범위 + 이 여유폭으로 탐색 범위를 한정한다 (하드코딩 매직넘버 대신
# 실제 수집 데이터 범위에서 유도).
BOUNDS_MARGIN_M = 0.3
T_CE_BOUND_M = 0.5  # 카메라가 그리퍼 몸체에 붙어있다는 전제의 물리적 상한


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
    """DART TCP(x,y,z,a,b,c)[mm/deg] → (R_ee2base, t_ee2base)[m].

    (a,b,c)는 Doosan 공식 SDK 정의대로 Euler ZYZ (dsr_common2/include/DRFS.h:
    "follows Euler ZYZ notation")를 따른다.
    """
    t = np.array([x / 1000, y / 1000, z / 1000])
    R = Rotation.from_euler('ZYZ', [a, b, c], degrees=True).as_matrix()
    return R, t


def parse_tcp(line: str) -> tuple[np.ndarray | None, np.ndarray | None]:
    vals = [float(v) for v in line.strip().split()]
    if len(vals) != 6:
        return None, None
    return dart_to_Rt(*vals)


def is_ground_truth_annotation(ann: str) -> bool:
    """TCP를 마커 중심에 직접 일치시켜 얻은 ground truth 포즈인지 annotation으로 판별.

    예: "차르코마커 중심점_TCP 중심" → True (일반 ray 샘플과 분리해 P_world로 사용).
    """
    return ('마커' in ann) and ('중심' in ann)


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


def classify_tw_poses(
    tw_poses: list[dict],
) -> tuple[list[tuple[np.ndarray, np.ndarray]], list[tuple[np.ndarray, np.ndarray, str]]]:
    """TW 포즈 목록을 (ray 샘플, ground-truth 샘플)로 분리."""
    rays: list[tuple[np.ndarray, np.ndarray]] = []
    gts: list[tuple[np.ndarray, np.ndarray, str]] = []
    for p in tw_poses:
        R, t = dart_to_Rt(p['X'], p['Y'], p['Z'], p['A'], p['B'], p['C'])
        if is_ground_truth_annotation(p['ann']):
            gts.append((R, t, p['ann']))
        else:
            rays.append((R, t))
    return rays, gts


class _IndentDumper(yaml.Dumper):
    """yamllint indent-sequences 규칙 호환 — 블록 시퀀스도 매핑 키만큼 들여씀 (issue #40)."""
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, indentless=False)


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
    gt_R_list: list[np.ndarray] = []
    gt_t_list: list[np.ndarray] = []
    gt_ann_list: list[str] = []

    if os.path.exists(DATA_PATH):
        d = np.load(DATA_PATH, allow_pickle=True)
        R_list = list(d['R_ee2base'])
        t_list = list(d['t_ee2base'])
        if 'gt_R' in d.files:
            gt_R_list = list(d['gt_R'])
            gt_t_list = list(d['gt_t'])
            gt_ann_list = list(d['gt_ann'])
        log.info('기존 %d개 ray 포즈 + %d개 ground-truth 포즈 이어받기',
                 len(R_list), len(gt_R_list))

    pose_idx = len(R_list) + len(gt_R_list)

    log.info('=== 고정 마커 기준 핸드아이 캘리브레이션 ===')
    log.info('준비: 테이블에 마커(차르코보드 코너 등) 하나 고정')
    if tw_poses:
        log.info('TW 모드: 로봇을 각 포즈로 이동 후 마커를 십자선에 맞추고 ENTER')
        log.info('현재 포즈: %d/%d', pose_idx + 1, len(tw_poses))
    else:
        log.info('수동 모드: 마커가 십자선에 오도록 로봇 XY 조정 → ENTER → TCP 입력')
    log.info('ENTER=저장  D=마지막삭제  Q=종료')

    def _save() -> None:
        np.savez(DATA_PATH,
                 R_ee2base=np.stack(R_list) if R_list else np.empty((0, 3, 3)),
                 t_ee2base=np.stack(t_list) if t_list else np.empty((0, 3)),
                 gt_R=np.stack(gt_R_list) if gt_R_list else np.empty((0, 3, 3)),
                 gt_t=np.stack(gt_t_list) if gt_t_list else np.empty((0, 3)),
                 gt_ann=np.array(gt_ann_list, dtype=object))

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
            cur_ann = tw_poses[pose_idx]['ann'] if pose_idx < len(tw_poses) else ''
            gt_tag = '  [GROUND-TRUTH]' if is_ground_truth_annotation(cur_ann) else ''
            info = (f'pose {pose_idx+1}/{len(tw_poses)}  rays={len(R_list)} '
                    f'gt={len(gt_R_list)}  ENTER:save  D:del  Q:quit{gt_tag}')
        else:
            info = f'rays={len(R_list)}  ENTER:save  D:del  Q:quit'
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
                is_gt = is_ground_truth_annotation(p['ann'])
                if is_gt:
                    gt_R_list.append(R); gt_t_list.append(t); gt_ann_list.append(p['ann'])
                    log.info('[GT] TW포즈%d ann=%s TCP=[%.3f,%.3f,%.3f]m 저장 (ground truth)',
                             pose_idx + 1, p['ann'], t[0], t[1], t[2])
                else:
                    R_list.append(R); t_list.append(t)
                    log.info('[ray %d] TW포즈%d ann=%s TCP=[%.3f,%.3f,%.3f]m 저장',
                             len(R_list), pose_idx + 1, p['ann'], t[0], t[1], t[2])
                pose_idx += 1
            else:
                cv2.destroyWindow('TCP-in-Center Calib')
                tcp = input('  DART TCP (X Y Z A B C  mm/deg): ').strip()
                cv2.namedWindow('TCP-in-Center Calib')
                R, t = parse_tcp(tcp)
                if R is None:
                    log.warning('입력 오류 (숫자 6개)')
                    continue
                R_list.append(R); t_list.append(t)
                log.info('[ray %d] 저장 — TCP=[%.3f,%.3f,%.3f]m', len(R_list), t[0], t[1], t[2])

            _save()

            if tw_poses and pose_idx >= len(tw_poses):
                log.info('TW 파일의 모든 포즈 완료')
                break

        elif key == ord('d') and (R_list or gt_R_list):
            prev_ann = tw_poses[pose_idx - 1]['ann'] if (tw_poses and pose_idx > 0) else ''
            if tw_poses and pose_idx > 0 and is_ground_truth_annotation(prev_ann):
                gt_R_list.pop(); gt_t_list.pop(); gt_ann_list.pop()
            elif R_list:
                R_list.pop(); t_list.pop()
            pose_idx = max(0, pose_idx - 1)
            _save()
            log.info('마지막 삭제 — 남은 ray %d개 / gt %d개  TW idx=%d',
                     len(R_list), len(gt_R_list), pose_idx + 1)

        elif key in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    log.info('수집 완료: ray %d개 / gt %d개 → %s', len(R_list), len(gt_R_list), DATA_PATH)
    if len(R_list) >= 5:
        log.info('compute 실행: python3 scripts/c270_tcp_center_calib.py --compute')


# ── 데이터 로딩/병합 ────────────────────────────────────────────────────────
def _load_npz_source(path: str) -> tuple[list, list, list]:
    """npz 1개 → (ray R/t 쌍 목록, gt R/t/ann 목록)."""
    d = np.load(path, allow_pickle=True)
    R_ee2base = list(d['R_ee2base'])
    t_ee2base = list(d['t_ee2base'])
    rays = list(zip(R_ee2base, t_ee2base))
    gts: list[tuple[np.ndarray, np.ndarray, str]] = []
    if 'gt_R' in d.files and len(d['gt_R']) > 0:
        gts = list(zip(list(d['gt_R']), list(d['gt_t']), list(d['gt_ann'])))
    return rays, gts


_RaySources = tuple[
    list[tuple[np.ndarray, np.ndarray]],
    list[tuple[np.ndarray, np.ndarray, str]],
    list[str],
]


def _gather_sources(inputs: list[str], tw_paths: list[str]) -> _RaySources:
    """여러 npz/tw 소스를 순서대로 병합. 각 ray 포즈의 출처 라벨도 같이 반환."""
    rays: list[tuple[np.ndarray, np.ndarray]] = []
    gts: list[tuple[np.ndarray, np.ndarray, str]] = []
    labels: list[str] = []

    for path in inputs:
        if not os.path.exists(path):
            log.error('입력 파일 없음: %s', path)
            sys.exit(1)
        r, g = _load_npz_source(path)
        rays.extend(r)
        gts.extend(g)
        labels.extend(f'{os.path.basename(path)}' for _ in r)
        log.info('%s: ray %d개 / gt %d개 로드', os.path.basename(path), len(r), len(g))

    for path in tw_paths:
        path = os.path.expanduser(path)
        if not os.path.exists(path):
            log.error('TW 파일 없음: %s', path)
            sys.exit(1)
        tw_poses = load_tw(path)
        r, g = classify_tw_poses(tw_poses)
        rays.extend(r)
        gts.extend(g)
        labels.extend(f'{os.path.basename(path)}' for _ in r)
        log.info('%s: ray %d개 / gt %d개 로드 (TW 직접 로딩)', os.path.basename(path), len(r), len(g))

    return rays, gts, labels


# ── 계산 모드 ───────────────────────────────────────────────────────────────
def _solve(
    R_list: list[np.ndarray], t_list: list[np.ndarray], N: int,
    t_fixed: np.ndarray | None, P_fixed: np.ndarray | None,
    use_global: bool,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """광선-포인트 모델 최적화.

    - t_fixed / P_fixed가 주어지면 해당 변수는 최적화 대상에서 빠지고 고정값으로 사용된다.
    - bounds: 모든 변수에 물리적으로 타당한 범위를 줘서 λ(광선 깊이)가 음수
      (카메라 뒤쪽 "교차")로 가는 비물리적 해를 차단한다.
    - loss='soft_l1' + method='trf': outlier 포즈 하나가 전체 해를 왜곡하지 못하도록
      robust loss 적용 (scipy.optimize.least_squares 공식 옵션).
    - use_global=True: scipy.optimize.differential_evolution으로 먼저 전역 탐색한 뒤
      그 결과를 초기값으로 least_squares를 돌려 정밀화 (local minimum 회피).
    """
    has_t = t_fixed is None
    has_P = P_fixed is None
    ray_cam = np.array([0., 0., 1.])
    t_arr = np.array(t_list)

    def unpack(p: np.ndarray):
        idx = 3
        rv = p[:3]
        if has_t:
            t_ce = p[idx:idx + 3]; idx += 3
        else:
            t_ce = t_fixed
        if has_P:
            P = p[idx:idx + 3]; idx += 3
        else:
            P = P_fixed
        lam = p[idx:idx + N]
        return rv, t_ce, P, lam

    def residuals(p: np.ndarray) -> list[float]:
        rv, t_ce, P, lam = unpack(p)
        R_ce = Rotation.from_rotvec(rv).as_matrix()
        res = []
        for i in range(N):
            p_cam = t_list[i] + R_list[i] @ t_ce
            d = R_list[i] @ R_ce @ ray_cam
            P_est = p_cam + lam[i] * d
            res.extend(((P_est - P) * 1000).tolist())
        return res

    def objective(p: np.ndarray) -> float:
        r = residuals(p)
        return float(np.sum(np.square(r)))

    # bounds: 데이터 범위에서 유도 (매직넘버 대신 실제 수집 범위 + 여유폭)
    pos_lo = t_arr.min(axis=0) - BOUNDS_MARGIN_M
    pos_hi = t_arr.max(axis=0) + BOUNDS_MARGIN_M
    span = float(np.linalg.norm(t_arr.max(axis=0) - t_arr.min(axis=0)))
    lam_hi = span + 1.0  # 광선 깊이 상한 (m), 작업공간 대각 거리 + 여유

    lb = [-np.pi] * 3
    ub = [np.pi] * 3
    if has_t:
        lb += [-T_CE_BOUND_M] * 3
        ub += [T_CE_BOUND_M] * 3
    if has_P:
        lb += pos_lo.tolist()
        ub += pos_hi.tolist()
    lb += [0.0] * N           # ② λ >= 0  (카메라 앞쪽만 허용)
    ub += [lam_hi] * N
    lb = np.array(lb); ub = np.array(ub)

    p0 = np.zeros(len(lb))
    idx = 3
    if has_t:
        p0[idx:idx + 3] = [0.0, 0.0, 0.05]; idx += 3
    if has_P:
        p0[idx:idx + 3] = t_arr.mean(axis=0); idx += 3
    p0[idx:] = np.full(N, float(np.median(t_arr[:, 2])))
    p0 = np.clip(p0, lb, ub)

    if use_global:
        log.info('differential_evolution 전역 탐색 중 (local minimum 회피)...')
        de = differential_evolution(objective, bounds=list(zip(lb, ub)),
                                     seed=0, maxiter=300, polish=False, tol=1e-9)
        p0 = np.clip(de.x, lb, ub)
        log.info('전역 탐색 완료 — cost=%.3f', de.fun)

    result = least_squares(residuals, p0, bounds=(lb, ub), method='trf',
                           loss='soft_l1', f_scale=ROBUST_F_SCALE_MM,
                           max_nfev=20000, ftol=1e-14, xtol=1e-14)

    rv, t_ce, P, lam = unpack(result.x)
    R_ce = proj_SO3(Rotation.from_rotvec(rv).as_matrix())
    res = np.array(residuals(result.x)).reshape(N, 3)
    errs = np.linalg.norm(res, axis=1)
    return R_ce, np.asarray(t_ce), np.asarray(P), lam, errs


def compute(
    inputs: list[str], tw_paths: list[str],
    t_fixed: np.ndarray | None, p_fixed: np.ndarray | None,
    exclude: list[int], gt_index: int | None, use_global: bool,
) -> None:
    rays, gts, labels = _gather_sources(inputs, tw_paths)
    N_total = len(rays)
    if N_total == 0:
        log.error('ray 데이터 없음')
        sys.exit(1)

    log.info('=== 병합된 ray 포즈 (1-indexed) ===')
    for i, (label, (_, t)) in enumerate(zip(labels, rays), start=1):
        mark = ' <-- 제외' if i in exclude else ''
        log.info('  %2d [%s] TCP=[%.3f,%.3f,%.3f]m%s', i, label, t[0], t[1], t[2], mark)

    keep_idx = [i for i in range(N_total) if (i + 1) not in exclude]
    if len(keep_idx) != N_total:
        log.info('exclude 처리: %d개 제외 → %d개 사용', N_total - len(keep_idx), len(keep_idx))
    R_list = [rays[i][0] for i in keep_idx]
    t_list = [rays[i][1] for i in keep_idx]
    N = len(R_list)
    if N < 4:
        log.error('최소 4포즈 필요 (현재 %d)', N)
        sys.exit(1)

    # ground truth (P_world) 결정
    P_fixed = p_fixed
    gt_source = 'CLI --p'
    if P_fixed is None and gts:
        if gt_index is not None:
            if not (1 <= gt_index <= len(gts)):
                log.error('--gt-index 범위 초과 (1~%d)', len(gts))
                sys.exit(1)
            _, P_fixed, ann = gts[gt_index - 1]
            gt_source = f'gt #{gt_index} ({ann})'
        elif len(gts) == 1:
            _, P_fixed, ann = gts[0]
            gt_source = f'자동 감지 ground-truth ({ann})'
        else:
            log.error('ground-truth 포즈 %d개 발견 — --gt-index로 선택하거나 --p로 직접 지정',
                       len(gts))
            for i, (_, t, ann) in enumerate(gts, start=1):
                log.error('  gt #%d: %s  TCP=[%.3f,%.3f,%.3f]m', i, ann, t[0], t[1], t[2])
            sys.exit(1)
    if P_fixed is not None:
        log.info('P_world 고정: [%.4f, %.4f, %.4f] m  (출처: %s)',
                 P_fixed[0], P_fixed[1], P_fixed[2], gt_source)
    else:
        log.info('P_world ground-truth 없음 — 자유 변수로 추정 (self-consistency만 검증됨)')

    if t_fixed is not None:
        log.info('translation 고정: %s', t_fixed)

    R_ce, t_ce, P_opt, lam, errs = _solve(R_list, t_list, N, t_fixed, P_fixed, use_global)

    log.info('=== 결과 ===')
    log.info('마커 위치: [%.1f, %.1f, %.1f] mm', P_opt[0] * 1000, P_opt[1] * 1000, P_opt[2] * 1000)
    log.info('잔차: 평균=%.2fmm  최대=%.2fmm', errs.mean(), errs.max())
    worst = np.argsort(-errs)[:5]
    for i in worst:
        log.info('  worst: 포즈 %d  %.2fmm  λ=%.0fmm', keep_idx[i] + 1, errs[i], lam[i] * 1000)
    if np.any(lam < 1e-6):
        log.warning('λ가 0에 매우 가까운 포즈 존재 — 광선 깊이 추정 불안정 가능')

    quat = Rotation.from_matrix(R_ce).as_quat()
    tilt = float(np.degrees(np.arccos(np.clip(abs(float(R_ce[2, 2])), 0, 1))))
    T_show = np.eye(4); T_show[:3, :3] = R_ce; T_show[:3, 3] = t_ce
    log.info('T_cam2gripper:\n%s', np.round(T_show, 4))
    log.info('translation: x=%.4f  y=%.4f  z=%.4f m', t_ce[0], t_ce[1], t_ce[2])
    log.info('quaternion : x=%.4f  y=%.4f  z=%.4f  w=%.4f', quat[0], quat[1], quat[2], quat[3])
    log.info('카메라 기울기: %.1f°', tilt)

    cfg_path = os.path.join(_DIR, '..', 'config', 'c270_hand_eye.yaml')
    data = {
        'schema_version': 1,
        'transformation': {
            'rotation': {'x': float(quat[0]), 'y': float(quat[1]),
                         'z': float(quat[2]), 'w': float(quat[3])},
            'translation': {'x': float(t_ce[0]), 'y': float(t_ce[1]), 'z': float(t_ce[2])},
        },
        'metadata': {
            'calibration_date': datetime.date.today().isoformat(),
            'sample_count': N,
            'sample_count_total': N_total,
            'excluded_poses': sorted(exclude),
            'method': 'fixed_marker_ray',
            'mean_err_mm': float(errs.mean()),
            'max_err_mm': float(errs.max()),
            'cam_tilt_deg': tilt,
            'marker_world_mm': [float(P_opt[0] * 1000), float(P_opt[1] * 1000),
                                 float(P_opt[2] * 1000)],
            'ground_truth_used': bool(P_fixed is not None and gts),
            'ground_truth_source': gt_source if (P_fixed is not None and gts) else None,
            'translation_fixed': t_fixed is not None,
            'optimizer': {
                'loss': 'soft_l1', 'method': 'trf', 'f_scale_mm': ROBUST_F_SCALE_MM,
                'global_presearch': use_global,
            },
            'frames': {
                'from': 'c270_optical_frame',
                'to':   'link_6',
                'note': 'T_cam2gripper: camera origin expressed in link_6 frame',
            },
        }
    }
    with open(cfg_path, 'w') as f:
        yaml.dump(data, f, Dumper=_IndentDumper, explicit_start=True,
                  default_flow_style=False, allow_unicode=True, sort_keys=False)
    log.info('저장: %s', cfg_path)


# ── entry point ─────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--collect', action='store_true')
    parser.add_argument('--compute', action='store_true')
    parser.add_argument('--tw', nargs='*', default=[],
                        help='DART TW 파일 경로 (--collect: 1개, --compute: 여러 개 직접 병합 가능)')
    parser.add_argument('--input', nargs='*', default=[DATA_PATH],
                        help='--compute 입력 npz 파일 목록 (기본: %(default)s)')
    parser.add_argument('--exclude', nargs='*', type=int, default=[],
                        help='제외할 ray 포즈 번호 (병합 후 1-indexed, outlier 제거용)')
    parser.add_argument('--gt-index', type=int, default=None,
                        help='ground-truth 포즈가 여러 개일 때 사용할 번호 (1-indexed)')
    parser.add_argument('--t', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='translation 고정 (m)')
    parser.add_argument('--p', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                        help='P_world(마커 위치) 고정 (m) — 지정 시 ground-truth 자동감지 무시')
    parser.add_argument('--no-global', action='store_true',
                        help='differential_evolution 전역 탐색 생략 (빠른 반복용)')
    args = parser.parse_args()

    if args.collect:
        collect(tw_path=args.tw[0] if args.tw else None)
    elif args.compute:
        t_fixed = np.array(args.t) if args.t else None
        p_fixed = np.array(args.p) if args.p else None
        compute(args.input, args.tw, t_fixed, p_fixed, args.exclude, args.gt_index,
                use_global=not args.no_global)
    else:
        parser.print_help()
