"""toolbox_motion.py
────────────────────
공구함 서랍 열기/닫기 + 공구 접근 Step 시퀀스.
출처: TaskWriter toolboxapproach_box1.tw / toolboxapproach_box2.tw (e0509 실측)

⚠️ 스코프: 데모 / Track B Phase 1 한정.
  - 아래 모든 좌표·속도·펄스 상수는 `.tw` 파일에서 디코딩한 **실측 하드코딩 값**이다.
  - 운영자 튜닝 대상이 아닌 demonstration 재현용이므로, config/*.yaml 분리(E-4)는
    의도적으로 보류한다. 실제 운영용 fetch/return unit action으로 일반화할 때
    웨이포인트·임계값을 config로 이관할 것.
  - 따라서 이 파일을 프로덕션 fetch/return의 기반으로 그대로 확장하지 말 것.

단위 주의:
  - 여기서 정의된 모든 좌표·속도는 DSR 네이티브 단위 (mm, deg, mm/s, deg/s).
  - ArmInterface (m/rad) 와는 다름 — unit_action_server.py 래퍼에서 변환 필요.
  - chamjo cube-solver motion_library.py 와 동일한 StepKind/Step 패턴 사용.

ROS2 의존성 없음 — 단독 테스트 가능.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Literal, Optional


# ── StepKind / Step (chamjo motion_library.py 동일 패턴) ──────────────────

class StepKind(Enum):
    MOVE_L_ABS      = auto()
    MOVE_L_REL      = auto()
    MOVE_J_ABS      = auto()
    MOVE_J_REL      = auto()
    GRIP            = auto()
    WAIT            = auto()
    VISUAL_SERVO_XZ = auto()   # 손잡이 XZ 정렬 VS — runner에서 HandleServoController 실행
    MOVE_L_TOP_XY       = auto()   # 탑뷰 XY + 고정 Z 이동 — runner가 /vision/tool_top_pose 좌표 사용
    VISUAL_SERVO_XY     = auto()   # 공구 XY 정렬 VS — runner에서 ToolServoController 실행
    MOVE_L_TOOL_XYZ     = auto()   # 그리퍼 캠 XYZ 하강 — runner가 /vision/tool_gripper_pose 좌표 사용
    MOVE_L_SLOT_XY      = auto()   # 탑뷰 slot XY + 고정 Z 이동 — runner가 /vision/slot_top_pose 좌표 사용
    WAIT_VISION_TOP_XY  = auto()   # 탑뷰 캐시 초기화 후 /vision/tool_top_pose 신규 수신 대기


PickPlaceMarker = Literal["pick", "place"]


@dataclass
class Step:
    kind:  StepKind
    pose:  Optional[list] = None
    vel:   Optional[float] = None
    acc:   Optional[float] = None
    pulse: Optional[int]   = None
    sec:   Optional[float] = None
    marker: Optional[PickPlaceMarker] = None  # 물리적 집기/놓기 시점 표시 (action feedback용)


def marked(step: Step, marker: PickPlaceMarker) -> Step:
    """시퀀스 빌더가 특정 step을 pick/place로 표시한다.

    tool_action_server는 marker가 설정된 step 실행 직후 action feedback으로
    phase=marker를 발행한다 (예: orchestrator의 DB 상태 전이 트리거).
    좌표가 비전 기반으로 동적 계산되더라도, 시퀀스 빌더는 항상 "이 step이
    pick/place다"라는 역할을 알고 있으므로 동일하게 적용 가능하다.

    marker는 "pick"/"place"만 허용한다 — 오타로 인한 DB 상태 전이 누락(S-8 영향)을
    방지하기 위해 런타임에도 검증한다.
    """
    if marker not in ("pick", "place"):
        raise ValueError(f"marker는 'pick' 또는 'place'여야 함: {marker!r}")
    step.marker = marker
    return step


# ── 속도 / 가속도 기본값 (TaskWriter MainRoutine 설정값) ──────────────────
# translationalVelocity=250.0 mm/s, translationalAcceleration=1000.0 mm/s²
# rotationalVelocity=76.5 deg/s,   rotationalAcceleration=306.0 deg/s²
# jointVelocity=60.0 deg/s,        jointAcceleration=100.0 deg/s²

VEL_L: float = 50.0
ACC_L: float = 200.0
VEL_R: float = 15.3
ACC_R: float = 61.2
VEL_J: float = 12.0
ACC_J: float = 20.0


# ── 그리퍼 pulse 상수 ─────────────────────────────────────────────────────

# 주의: 여기 PULSE_RELEASE=450 은 "공구를 놓기 위한 release stroke"(부분 개방)이며,
#       hal/doosan/gripper_driver.py 의 PULSE_OPEN=0 (완전 개방)과 의미·값이 다르다.
#       두 모듈의 pulse 상수를 혼용하지 말 것.
PULSE_RELEASE:      int = 450   # gripper_release stroke (TW SubRoutine 실측) — 부분 개방
PULSE_GRIP_BOX:     int = 600   # gripper_grap_boxhand stroke (TW SubRoutine 실측)
PULSE_GRIP_SOCKET:  int = 650   # socket 파지 stroke (TW SubRoutine 실측)


# ── 웨이포인트 상수 (TaskWriter 실측값, DSR BASE 좌표계, mm/deg) ───────────
# 형식: [x, y, z, rx, ry, rz]  (MoveL) 또는 [j1..j6] deg (MoveJ)

# 실물 로봇 홈 자세 (deg) — chamjo JOINT_HOME_POS 동일
JOINT_HOME_DEG:    list = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# ── layer 0 (1층 서랍) ────────────────────────────────────────────────────
# 출처: toolboxapproach_box1_open.tw / toolboxapproach_box1_close.tw

# open MoveJ: 서랍 접근 전 안전 자세
LAYER0_SETUP_J:        list = [-19.53, 53.85, 110.47, 71.14, 95.19, -75.18]

# close MoveJ: inner 위치에서 손잡이로 이동 전 관절 공간 안전 자세 (open의 SETUP_J와 다름)
LAYER0_CLOSE_SETUP_J:  list = [-23.33, 56.66, 107.63, 67.45, 96.16, -75.52]

# 서랍 손잡이 바로 앞 (파지 직전)
LAYER0_APPROACH:       list = [378.88, 433.02, 65.45, 90.0, 90.0, 90.0]

# 서랍 당긴 후 열린 위치
LAYER0_OPEN:           list = [378.88, 243.86, 65.46, 90.0, 90.0, 90.0]

# 열린 후 Z 약간 낮춤 (서랍 안쪽 자세 진입)
LAYER0_SILENCE:        list = [378.88, 243.86, 56.43, 90.0, 90.0, 90.0]

# 서랍 내부 공구 접근 위치 (공구 집기/놓기)
LAYER0_INNER:          list = [378.88, 169.1,  50.45, 90.0, 90.0, 90.0]

# 서랍 닫기 직전 하강 위치 (gripper_grap_boxhand 전)
LAYER0_OPENDOWN:       list = [378.88, 243.86, 55.45, 90.0, 90.0, 90.0]

# close 완료 후 최종 위치 (= INNER, box1_close 마지막 스텝)
LAYER0_CLOSE_END:      list = LAYER0_INNER


# ── layer 0 v2 (1층 서랍, toolboxapproach_box1_{open,close}_v2.tw) ───────
# x 좌표 수정 (378.88 → 369.0), open/silence/opendown y 조정 (243.86 → 213.86)
# setup_j / close_setup_j 는 v1과 동일

LAYER0_V2_APPROACH:    list = [369.0, 433.02, 65.45, 90.0, 90.0, 90.0]
LAYER0_V2_OPEN:        list = [369.0, 213.86, 65.46, 90.0, 90.0, 90.0]
LAYER0_V2_SILENCE:     list = [369.0, 213.86, 56.43, 90.0, 90.0, 90.0]
LAYER0_V2_INNER:       list = [369.0, 169.1,  50.45, 90.0, 90.0, 90.0]
LAYER0_V2_OPENDOWN:    list = [369.0, 213.86, 55.45, 90.0, 90.0, 90.0]
LAYER0_V2_CLOSE_END:   list = LAYER0_V2_INNER


# ── layer 1 (2층 서랍) ────────────────────────────────────────────────────
# 출처: toolboxapproach_box2_open.tw / toolboxapproach_box2_close.tw

# open MoveJ
LAYER1_SETUP_J:        list = [-6.14, 44.85, 116.43, 84.19, 91.97, -71.38]

# close MoveJ: open의 SETUP_J와 다름
LAYER1_CLOSE_SETUP_J:  list = [-23.64, 48.6, 110.08, 67.82, 98.38, -70.33]

LAYER1_APPROACH:       list = [380.57, 427.51, 115.68, 90.0, 90.0, 90.0]

LAYER1_OPEN:           list = [380.56, 237.79, 115.69, 90.0, 90.0, 90.0]

LAYER1_SILENCE:        list = [380.56, 237.79, 106.7,  90.0, 90.0, 90.0]

LAYER1_INNER:          list = [380.56, 165.94, 103.69, 90.0, 90.0, 90.0]

LAYER1_OPENDOWN:       list = [380.56, 237.79, 103.69, 90.0, 90.0, 90.0]

# close 완료 후 최종 위치 (box2_close 마지막 스텝 — INNER와 APPROACH 사이 중간 위치)
LAYER1_CLOSE_END:      list = [380.61, 291.56, 115.7,  89.99, 89.99, 90.0]

# layer_height_z 실측: 115.68 - 65.45 ≈ 50 mm (toolbox.yaml layer_height_z 갱신 필요)
LAYER_HEIGHT_Z_MM: float = 115.68 - 65.45   # ≈ 50.23 mm


# ── layer 1 v2 (2층 서랍, toolboxapproach_box2_{open,close}_v2.tw) ───────
# x 좌표 수정 (380.5x → 369.0), open/silence/opendown y 조정 (237.79 → 213.86)
# setup_j / close_setup_j 는 v1과 동일

LAYER1_V2_APPROACH:    list = [369.0, 427.51, 115.68, 90.0, 90.0, 90.0]
LAYER1_V2_OPEN:        list = [369.0, 213.86, 115.69, 90.0, 90.0, 90.0]
LAYER1_V2_SILENCE:     list = [369.0, 213.86, 106.7,  90.0, 90.0, 90.0]
LAYER1_V2_INNER:       list = [369.0, 165.94, 103.69, 90.0, 90.0, 90.0]
LAYER1_V2_OPENDOWN:    list = [369.0, 213.86, 103.69, 90.0, 90.0, 90.0]
LAYER1_V2_CLOSE_END:   list = [369.0, 291.56, 115.7,  89.99, 89.99, 90.0]


# ── 공구 접근 파라미터 ──────────────────────────────────────────────────────────
# E-4: config/toolbox.yaml vision_motion 섹션으로 이관 완료.
# toolbox_seq_runner.py가 __init__에서 로드 후 self._tool_approach_z_mm 등으로 사용.


# ── socket 공구 위치 (toolboxapproach_box2_socket_*.tw 실측값, DSR BASE 좌표계, mm/deg) ──
SOCKET_APPROACH_XY:  list = [269.98, 362.81, 234.0,   53.23,  180.0,  -38.07]
SOCKET_APPROACH_Z:   list = [269.98, 362.8,  122.8,   48.74, -180.0,  -42.55]
SOCKET_BOTTOM_XY:    list = [550.0, -172.72, 235.73, 160.43,  180.0,   73.74]
SOCKET_BOTTOM:       list = [550.0, -172.72,  -0.12, 158.87,  180.0,   72.17]
SOCKET_CATCH_HOME_L: list = [373.0,    0.0,  245.0,    3.13, -180.0,    3.13]


# ── 웨이포인트 테이블 ─────────────────────────────────────────────────────

_LAYER_WP = {
    0: {
        "setup_j":       LAYER0_SETUP_J,
        "close_setup_j": LAYER0_CLOSE_SETUP_J,
        "approach":      LAYER0_APPROACH,
        "open":          LAYER0_OPEN,
        "silence":       LAYER0_SILENCE,
        "inner":         LAYER0_INNER,
        "opendown":      LAYER0_OPENDOWN,
        "close_end":     LAYER0_CLOSE_END,
    },
    1: {
        "setup_j":       LAYER1_SETUP_J,
        "close_setup_j": LAYER1_CLOSE_SETUP_J,
        "approach":      LAYER1_APPROACH,
        "open":          LAYER1_OPEN,
        "silence":       LAYER1_SILENCE,
        "inner":         LAYER1_INNER,
        "opendown":      LAYER1_OPENDOWN,
        "close_end":     LAYER1_CLOSE_END,
    },
}

_LAYER_WP_V2 = {
    0: {
        "setup_j":       LAYER0_SETUP_J,
        "close_setup_j": LAYER0_CLOSE_SETUP_J,
        "approach":      LAYER0_V2_APPROACH,
        "open":          LAYER0_V2_OPEN,
        "silence":       LAYER0_V2_SILENCE,
        "inner":         LAYER0_V2_INNER,
        "opendown":      LAYER0_V2_OPENDOWN,
        "close_end":     LAYER0_V2_CLOSE_END,
    },
    1: {
        "setup_j":       LAYER1_SETUP_J,
        "close_setup_j": LAYER1_CLOSE_SETUP_J,
        "approach":      LAYER1_V2_APPROACH,
        "open":          LAYER1_V2_OPEN,
        "silence":       LAYER1_V2_SILENCE,
        "inner":         LAYER1_V2_INNER,
        "opendown":      LAYER1_V2_OPENDOWN,
        "close_end":     LAYER1_V2_CLOSE_END,
    },
}


# ── Step 헬퍼 팩토리 (chamjo 동일) ────────────────────────────────────────

def ml_abs(pos: list, vel: float = VEL_L, acc: float = ACC_L) -> Step:
    return Step(kind=StepKind.MOVE_L_ABS, pose=list(pos), vel=vel, acc=acc)

def ml_rel(pos: list, vel: float = VEL_L, acc: float = ACC_L) -> Step:
    return Step(kind=StepKind.MOVE_L_REL, pose=list(pos), vel=vel, acc=acc)

def mj_abs(joints: list, vel: float = VEL_J, acc: float = ACC_J) -> Step:
    return Step(kind=StepKind.MOVE_J_ABS, pose=list(joints), vel=vel, acc=acc)

def grip(pulse: int) -> Step:
    return Step(kind=StepKind.GRIP, pulse=pulse)

def wait_step(sec: float) -> Step:
    return Step(kind=StepKind.WAIT, sec=sec)

GRIP_RELEASE = lambda: grip(PULSE_RELEASE)
GRIP_BOX     = lambda: grip(PULSE_GRIP_BOX)
GRIP_SOCKET  = lambda: grip(PULSE_GRIP_SOCKET)
JOINT_HOME   = lambda: mj_abs(JOINT_HOME_DEG)


# ── 시퀀스 함수 ───────────────────────────────────────────────────────────

def home_seq() -> list[Step]:
    """홈 자세 복귀 시퀀스."""
    return [GRIP_RELEASE(), JOINT_HOME()]


def _wp(layer: int, key: str) -> list:
    if layer not in _LAYER_WP:
        raise ValueError(f"layer는 0 또는 1만 지원: {layer}")
    return _LAYER_WP[layer][key]


def _wp_v2(layer: int, key: str) -> list:
    if layer not in _LAYER_WP_V2:
        raise ValueError(f"layer는 0 또는 1만 지원: {layer}")
    return _LAYER_WP_V2[layer][key]


def drawer_open_seq(
    layer: int,
    tool_pose: Optional[list] = None,
) -> list[Step]:
    """서랍 열기 시퀀스 (TW: toolboxapproach_box{n}_open.tw 기준).

    layer: 0 = 1층, 1 = 2층
    tool_pose: 비전에서 받은 공구 위치 [x, y, z, rx, ry, rz] — DSR BASE 좌표계 (mm, deg).
               None이면 하드코딩 INNER 웨이포인트 사용.
    종료 후 팔은 tool_pose 또는 LAYER{n}_INNER 위치에 있음.
    """
    inner = tool_pose if tool_pose is not None else _wp(layer, "inner")
    return [
        GRIP_RELEASE(),
        mj_abs(_wp(layer, "setup_j")),
        ml_abs(_wp(layer, "approach")),
        GRIP_BOX(),
        ml_abs(_wp(layer, "open")),
        ml_abs(_wp(layer, "silence")),
        GRIP_RELEASE(),
        ml_abs(inner),
        JOINT_HOME(),
    ]


def vision_drawer_open_seq(layer: int) -> list[Step]:
    """손잡이 XZ Visual Servoing 포함 서랍 열기 시퀀스.

    ③ APPROACH까지 하드코딩 웨이포인트로 이동 후 VS로 XZ 정렬.
    VS(DETECT→ALIGN_XZ) 완료 후 GRIP_BOX → 서랍 당기기.

    layer: 0 = 1층, 1 = 2층
    """
    return [
        GRIP_RELEASE(),                            # ①
        mj_abs(_wp(layer, "setup_j")),             # ②
        ml_abs(_wp(layer, "approach")),            # ③ 하드코딩 APPROACH — 손잡이 정면
        Step(kind=StepKind.VISUAL_SERVO_XZ),       # ④⑤ VS: DETECT → ALIGN_XZ
        GRIP_BOX(),                                # ⑥
        ml_abs(_wp(layer, "open")),                # ⑦ 서랍 당김
        ml_abs(_wp(layer, "silence")),             # ⑧ Z 9mm 하강
        GRIP_RELEASE(),                            # ⑨
        ml_abs(_wp(layer, "inner")),               # ⑩
        JOINT_HOME(),                              # ⑪
    ]


def drawer_close_seq(
    layer: int,
    tool_pose: Optional[list] = None,
) -> list[Step]:
    """서랍 닫기 시퀀스 (TW: toolboxapproach_box{n}_close.tw 기준).

    layer: 0 = 1층, 1 = 2층
    drawer_open_seq() 이후 호출 전제 (팔이 INNER에 있는 상태).
    tool_pose: 공구 반납 위치 [x, y, z, rx, ry, rz] — DSR BASE 좌표계 (mm, deg).
               None이면 하드코딩 CLOSE_END 웨이포인트 사용.
    종료 후 팔은 tool_pose 또는 LAYER{n}_CLOSE_END 위치에 있음.
    """
    close_end = tool_pose if tool_pose is not None else _wp(layer, "close_end")
    return [
        GRIP_RELEASE(),
        mj_abs(_wp(layer, "close_setup_j")),
        ml_abs(_wp(layer, "opendown")),
        GRIP_BOX(),
        ml_abs(_wp(layer, "open")),
        ml_abs(_wp(layer, "approach")),
        GRIP_RELEASE(),
        ml_abs(close_end),
        JOINT_HOME(),
    ]


def drawer_open_seq_v2(layer: int) -> list[Step]:
    """서랍 열기 시퀀스 v2 (toolboxapproach_box{n}_{open,close}_v2.tw 기준).

    v1 대비 변경: x=369.0mm, open/silence/opendown y 좌표 조정.
    layer: 0 = 1층, 1 = 2층
    """
    return [
        GRIP_RELEASE(),
        mj_abs(_wp_v2(layer, "setup_j")),
        ml_abs(_wp_v2(layer, "approach")),
        GRIP_BOX(),
        ml_abs(_wp_v2(layer, "open")),
        ml_abs(_wp_v2(layer, "silence")),
        GRIP_RELEASE(),
        ml_abs(_wp_v2(layer, "inner")),
        JOINT_HOME(),
    ]


def drawer_close_seq_v2(layer: int) -> list[Step]:
    """서랍 닫기 시퀀스 v2 (toolboxapproach_box{n}_{open,close}_v2.tw 기준).

    v1 대비 변경: x=369.0mm, open/silence/opendown y 좌표 조정.
    layer: 0 = 1층, 1 = 2층
    """
    return [
        GRIP_RELEASE(),
        mj_abs(_wp_v2(layer, "close_setup_j")),
        ml_abs(_wp_v2(layer, "opendown")),
        GRIP_BOX(),
        ml_abs(_wp_v2(layer, "open")),
        ml_abs(_wp_v2(layer, "approach")),
        GRIP_RELEASE(),
        ml_abs(_wp_v2(layer, "close_end")),
        JOINT_HOME(),
    ]


def vision_drawer_close_seq(layer: int) -> list[Step]:
    """손잡이 XZ Visual Servoing 포함 서랍 닫기 시퀀스.

    ③ OPENDOWN까지 하드코딩 웨이포인트로 이동 후 VS로 XZ 정렬.
    VS 완료 후 GRIP_BOX → 서랍 밀기.

    layer: 0 = 1층, 1 = 2층
    """
    return [
        GRIP_RELEASE(),                            # ①
        mj_abs(_wp(layer, "close_setup_j")),       # ②
        ml_abs(_wp(layer, "opendown")),            # ③ 하드코딩 OPENDOWN — 손잡이 정면
        Step(kind=StepKind.VISUAL_SERVO_XZ),       # ④⑤ VS: DETECT → ALIGN_XZ
        GRIP_BOX(),                                # ⑥
        ml_abs(_wp(layer, "open")),                # ⑦
        ml_abs(_wp(layer, "approach")),            # ⑧ 서랍 밀기
        GRIP_RELEASE(),                            # ⑨
        ml_abs(_wp(layer, "close_end")),           # ⑩
        JOINT_HOME(),                              # ⑪
    ]


def approach_tool_seq(
    layer: int,
    tool_pose: Optional[list] = None,
) -> list[Step]:
    """서랍이 이미 열린 상태에서 공구 접근 위치로만 이동.

    drawer_open_seq() 후 팔이 이미 inner 위치에 있으면 불필요.
    서랍 열기 없이 inner 위치만 필요할 때 단독 사용.
    tool_pose: 비전에서 받은 공구 위치 [x, y, z, rx, ry, rz] — DSR BASE 좌표계 (mm, deg).
               None이면 하드코딩 INNER 웨이포인트 사용.
    """
    inner = tool_pose if tool_pose is not None else _wp(layer, "inner")
    return [
        mj_abs(_wp(layer, "setup_j")),
        ml_abs(inner),
    ]


def fetch_from_drawer_seq(
    layer: int,
    tool_pose: Optional[list] = None,
) -> list[Step]:
    """서랍 열기 → 공구 접근까지 전체 시퀀스 (공구 파지는 caller가 수행).

    drawer_open_seq()의 alias. 가독성용.
    tool_pose: 비전에서 받은 공구 위치 [x, y, z, rx, ry, rz] — DSR BASE 좌표계 (mm, deg).
               None이면 하드코딩 INNER 웨이포인트 사용.
    """
    return drawer_open_seq(layer, tool_pose=tool_pose)


def socket_fetch_seq() -> list[Step]:
    """공구함(bottom) → staging area 소켓 전달 시퀀스 (TW: box2_socket_catch_ver2).

    호출 전 팔이 홈 자세에 있어야 함.
    종료 후 팔은 SOCKET_CATCH_HOME_L 위치(MoveL 복귀).
    """
    return [
        JOINT_HOME(),
        GRIP_RELEASE(),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_APPROACH_Z),
        GRIP_SOCKET(),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_BOTTOM),
        GRIP_RELEASE(),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_CATCH_HOME_L),
    ]


VISION_FETCH_SCAN_J: list = [26.2, 18.04, 35.57, 25.49, 111.91, 42.14]  # 탑뷰 스캔 자세 (deg)


def vision_fetch_seq() -> list[Step]:
    """탑뷰 XY + 그리퍼 캠 XYZ 기반 공구 fetch 시퀀스 (11단계).

    ① JOINT_HOME
    ② GRIP_RELEASE
    ③ MoveJ → VISION_FETCH_SCAN_J  (탑뷰 스캔 자세)
    ④ WAIT_VISION_TOP_XY — 캐시 초기화 후 /vision/tool_top_pose 신규 수신 대기
    ⑤ MOVE_L_TOP_XY   — runner가 /vision/tool_top_pose XY + TOOL_APPROACH_Z_MM 로 이동
    ⑥ MOVE_L_TOOL_XYZ — 그리퍼 캠 XYZ 로 공구 위치로 하강
    ⑦ GRIP_SOCKET
    ⑧ MOVE_L_TOP_XY   — 다시 TOOL_APPROACH_Z_MM 높이로 상승 (⑤과 동일)
    ⑨ MoveL → SOCKET_BOTTOM_XY  (staging 위)
    ⑩ MoveL → SOCKET_BOTTOM     (staging 하강)
    ⑪ GRIP_RELEASE
    ⑫ MoveL → SOCKET_BOTTOM_XY  (staging 위)
    ⑬ JOINT_HOME

    호출 전 팔이 홈 자세에 있어야 함.
    좌표는 모두 runner가 토픽에서 실시간으로 읽어 처리 (파라미터 불필요).
    """
    return [
        JOINT_HOME(),                               # ①
        GRIP_RELEASE(),                             # ②
        mj_abs(VISION_FETCH_SCAN_J),                # ③ 탑뷰 스캔 자세
        Step(kind=StepKind.WAIT_VISION_TOP_XY),     # ④ 신규 탑뷰 좌표 수신 대기
        Step(kind=StepKind.MOVE_L_TOP_XY),          # ⑤ 탑뷰 XY + 고정 Z
        Step(kind=StepKind.MOVE_L_TOOL_XYZ),        # ⑥ 그리퍼 캠 XYZ 하강
        GRIP_SOCKET(),                              # ⑦
        Step(kind=StepKind.MOVE_L_TOP_XY),          # ⑧ 상승 (⑤과 동일)
        ml_abs(SOCKET_BOTTOM_XY),                   # ⑨ staging 위
        ml_abs(SOCKET_BOTTOM),                      # ⑩ staging 하강
        GRIP_RELEASE(),                             # ⑪
        ml_abs(SOCKET_BOTTOM_XY),                   # ⑫ staging 위
        JOINT_HOME(),                               # ⑬
    ]


def socket_return_seq() -> list[Step]:
    """staging area → 공구함(bottom) 소켓 반납 시퀀스 (TW: box2_socket_drop_ver2).

    호출 전 팔이 홈 자세에 있어야 함.
    종료 후 팔은 JOINT_HOME 자세.
    """
    return [
        JOINT_HOME(),
        GRIP_RELEASE(),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_BOTTOM),
        GRIP_SOCKET(),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_APPROACH_Z),
        GRIP_RELEASE(),
        ml_abs(SOCKET_APPROACH_XY),
        JOINT_HOME(),
    ]


def vision_return_seq() -> list[Step]:
    """탑뷰 XY + 그리퍼 캠 VS 기반 공구 return 시퀀스 (13단계).

    ① JOINT_HOME
    ② GRIP_RELEASE
    ③ MOVE_L_TOP_XY   — 탑뷰 /vision/tool_top_pose XY + 고정 Z (staging 위)
    ④ VISUAL_SERVO_XY — 그리퍼 캠 XY VS → staging 공구 정밀 정렬
    ⑤ MOVE_L_TOOL_XYZ — 그리퍼 캠 XYZ 로 staging 공구 위치로 하강
    ⑥ GRIP_SOCKET
    ⑦ MOVE_L_TOP_XY   — staging 위로 상승 (③과 동일)
    ⑧ MOVE_L_SLOT_XY  — 탑뷰 /vision/slot_top_pose XY + 고정 Z (slot 위)
    ⑨ VISUAL_SERVO_XY — 그리퍼 캠 XY VS → slot 위치 정밀 정렬
    ⑩ MOVE_L_TOOL_XYZ — 그리퍼 캠 XYZ 로 slot 위치로 하강
    ⑪ GRIP_RELEASE
    ⑫ MOVE_L_SLOT_XY  — slot 위로 상승 (⑧과 동일)
    ⑬ JOINT_HOME

    호출 전 팔이 홈 자세에 있어야 함.
    좌표는 모두 runner가 토픽에서 실시간으로 읽어 처리 (파라미터 불필요).
    """
    return [
        JOINT_HOME(),                           # ①
        GRIP_RELEASE(),                         # ②
        Step(kind=StepKind.MOVE_L_TOP_XY),      # ③ staging 탑뷰 XY + 고정 Z
        Step(kind=StepKind.VISUAL_SERVO_XY),    # ④ staging 그리퍼 캠 XY VS
        Step(kind=StepKind.MOVE_L_TOOL_XYZ),   # ⑤ staging 그리퍼 캠 XYZ 하강
        GRIP_SOCKET(),                          # ⑥
        Step(kind=StepKind.MOVE_L_TOP_XY),      # ⑦ staging 위로 상승
        Step(kind=StepKind.MOVE_L_SLOT_XY),     # ⑧ slot 탑뷰 XY + 고정 Z
        Step(kind=StepKind.VISUAL_SERVO_XY),    # ⑨ slot 그리퍼 캠 XY VS
        Step(kind=StepKind.MOVE_L_TOOL_XYZ),   # ⑩ slot 그리퍼 캠 XYZ 하강
        GRIP_RELEASE(),                         # ⑪
        Step(kind=StepKind.MOVE_L_SLOT_XY),     # ⑫ slot 위로 상승
        JOINT_HOME(),                           # ⑬
    ]


def return_to_drawer_seq(
    layer: int,
    tool_pose: Optional[list] = None,
) -> list[Step]:
    """공구 반납 후 서랍 닫기 전체 시퀀스 (공구 release는 caller가 수행).

    drawer_close_seq()의 alias. 가독성용.
    tool_pose: 공구 반납 위치 [x, y, z, rx, ry, rz] — DSR BASE 좌표계 (mm, deg).
               None이면 하드코딩 CLOSE_END 웨이포인트 사용.
    """
    return drawer_close_seq(layer, tool_pose=tool_pose)


def full_socket_fetch_seq() -> list[Step]:
    """서랍 열기 → 소켓 파지 → 스테이징 거치 → 서랍 닫기 전체 시퀀스.

    layer 1(2층 서랍) 사용. 데모 전용 — vision 미구현, 좌표 하드코딩.

    흐름:
      1. drawer_open_seq(1): home + 서랍 열기, 종료 위치=LAYER1_INNER
      2. SOCKET_APPROACH_XY→Z: 소켓 슬롯 하강 후 파지
      3. SOCKET_BOTTOM_XY→BOTTOM: 스테이징 거치
      4. drawer_close_seq(1): joint 이동으로 시작하므로 직전 위치 무관
      5. JOINT_HOME
    """
    return [
        *drawer_open_seq(1),
        JOINT_HOME(),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_APPROACH_Z),
        marked(GRIP_SOCKET(), "pick"),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_BOTTOM),
        marked(GRIP_RELEASE(), "place"),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_CATCH_HOME_L),
        JOINT_HOME(),
        *drawer_close_seq(1),
        JOINT_HOME(),
    ]


def full_socket_return_seq() -> list[Step]:
    """서랍 열기 → 스테이징 픽업 → 서랍 슬롯 반납 → 서랍 닫기 전체 시퀀스.

    layer 1(2층 서랍) 사용. 데모 전용 — vision 미구현, 좌표 하드코딩.

    흐름:
      1. drawer_open_seq(1): home + 서랍 열기, 종료 위치=LAYER1_INNER
      2. SOCKET_BOTTOM_XY→BOTTOM: 스테이징에서 소켓 픽업
      3. SOCKET_APPROACH_XY→Z: 서랍 슬롯 반납
      4. drawer_close_seq(1): joint 이동으로 시작하므로 직전 위치 무관
      5. JOINT_HOME
    """
    return [
        *drawer_open_seq(1),
        JOINT_HOME(),
        ml_abs(SOCKET_CATCH_HOME_L),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_BOTTOM),
        marked(GRIP_SOCKET(), "pick"),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_APPROACH_Z),
        marked(GRIP_RELEASE(), "place"),
        ml_abs(SOCKET_APPROACH_XY),
        JOINT_HOME(),
        *drawer_close_seq(1),
        JOINT_HOME(),
    ]
