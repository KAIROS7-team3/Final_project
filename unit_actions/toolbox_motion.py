"""toolbox_motion.py
────────────────────
공구함 서랍 열기/닫기 + 공구 접근 Step 시퀀스.
출처: TaskWriter toolboxapproach_box1.tw / toolboxapproach_box2.tw (e0509 실측)

단위 주의:
  - 여기서 정의된 모든 좌표·속도는 DSR 네이티브 단위 (mm, deg, mm/s, deg/s).
  - ArmInterface (m/rad) 와는 다름 — unit_action_server.py 래퍼에서 변환 필요.
  - chamjo cube-solver motion_library.py 와 동일한 StepKind/Step 패턴 사용.

ROS2 의존성 없음 — 단독 테스트 가능.
"""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


# ── StepKind / Step (chamjo motion_library.py 동일 패턴) ──────────────────

class StepKind(Enum):
    MOVE_L_ABS = auto()
    MOVE_L_REL = auto()
    MOVE_J_ABS = auto()
    MOVE_J_REL = auto()
    GRIP       = auto()
    WAIT       = auto()


@dataclass
class Step:
    kind:  StepKind
    pose:  Optional[list] = None
    vel:   Optional[float] = None
    acc:   Optional[float] = None
    pulse: Optional[int]   = None
    sec:   Optional[float] = None


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

PULSE_OPEN:         int = 450   # gripper_release stroke (TW SubRoutine 실측)
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


# ── socket 공구 위치 (toolboxapproach_box2_socket_*.tw 실측값, DSR BASE 좌표계, mm/deg) ──
SOCKET_APPROACH_XY:  list = [269.98, 362.81, 234.0,   53.23,  180.0,  -38.07]
SOCKET_APPROACH_Z:   list = [269.98, 362.8,  122.8,   48.74, -180.0,  -42.55]
SOCKET_BOTTOM_XY:    list = [428.0, -172.72, 235.73, 160.43,  180.0,   73.74]
SOCKET_BOTTOM:       list = [428.0, -172.72,  -0.12, 158.87,  180.0,   72.17]
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

GRIP_OPEN    = lambda: grip(PULSE_OPEN)
GRIP_BOX     = lambda: grip(PULSE_GRIP_BOX)
GRIP_SOCKET  = lambda: grip(PULSE_GRIP_SOCKET)
JOINT_HOME   = lambda: mj_abs(JOINT_HOME_DEG)


# ── 시퀀스 함수 ───────────────────────────────────────────────────────────

def _wp(layer: int, key: str) -> list:
    if layer not in _LAYER_WP:
        raise ValueError(f"layer는 0 또는 1만 지원: {layer}")
    return _LAYER_WP[layer][key]


def drawer_open_seq(layer: int) -> list[Step]:
    """서랍 열기 시퀀스 (TW: toolboxapproach_box{n}_open.tw 기준).

    layer: 0 = 1층, 1 = 2층
    종료 후 팔은 LAYER{n}_INNER 위치에 있음.
    """
    return [
        GRIP_OPEN(),
        mj_abs(_wp(layer, "setup_j")),
        ml_abs(_wp(layer, "approach")),
        GRIP_BOX(),
        ml_abs(_wp(layer, "open")),
        ml_abs(_wp(layer, "silence")),
        GRIP_OPEN(),
        ml_abs(_wp(layer, "inner")),
    ]


def drawer_close_seq(layer: int) -> list[Step]:
    """서랍 닫기 시퀀스 (TW: toolboxapproach_box{n}_close.tw 기준).

    layer: 0 = 1층, 1 = 2층
    drawer_open_seq() 이후 호출 전제 (팔이 INNER에 있는 상태).
    종료 후 팔은 LAYER{n}_CLOSE_END 위치에 있음.
    """
    return [
        GRIP_OPEN(),
        mj_abs(_wp(layer, "close_setup_j")),
        ml_abs(_wp(layer, "opendown")),
        GRIP_BOX(),
        ml_abs(_wp(layer, "open")),
        ml_abs(_wp(layer, "approach")),
        GRIP_OPEN(),
        ml_abs(_wp(layer, "close_end")),
    ]


def approach_tool_seq(layer: int) -> list[Step]:
    """서랍이 이미 열린 상태에서 공구 접근 위치로만 이동.

    drawer_open_seq() 후 팔이 이미 inner 위치에 있으면 불필요.
    서랍 열기 없이 inner 위치만 필요할 때 단독 사용.
    """
    return [
        mj_abs(_wp(layer, "setup_j")),
        ml_abs(_wp(layer, "inner")),
    ]


def fetch_from_drawer_seq(layer: int) -> list[Step]:
    """서랍 열기 → 공구 접근까지 전체 시퀀스 (공구 파지는 caller가 수행).

    drawer_open_seq()의 alias. 가독성용.
    """
    return drawer_open_seq(layer)


def socket_fetch_seq() -> list[Step]:
    """공구함(bottom) → staging area 소켓 전달 시퀀스 (TW: box2_socket_catch_ver2).

    호출 전 팔이 홈 자세에 있어야 함.
    종료 후 팔은 SOCKET_CATCH_HOME_L 위치(MoveL 복귀).
    """
    return [
        JOINT_HOME(),
        GRIP_OPEN(),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_APPROACH_Z),
        GRIP_SOCKET(),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_BOTTOM),
        GRIP_OPEN(),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_CATCH_HOME_L),
    ]


def socket_return_seq() -> list[Step]:
    """staging area → 공구함(bottom) 소켓 반납 시퀀스 (TW: box2_socket_drop_ver2).

    호출 전 팔이 홈 자세에 있어야 함.
    종료 후 팔은 JOINT_HOME 자세.
    """
    return [
        JOINT_HOME(),
        GRIP_OPEN(),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_BOTTOM),
        GRIP_SOCKET(),
        ml_abs(SOCKET_BOTTOM_XY),
        ml_abs(SOCKET_APPROACH_XY),
        ml_abs(SOCKET_APPROACH_Z),
        GRIP_OPEN(),
        ml_abs(SOCKET_APPROACH_XY),
        JOINT_HOME(),
    ]


def return_to_drawer_seq(layer: int) -> list[Step]:
    """공구 반납 후 서랍 닫기 전체 시퀀스 (공구 release는 caller가 수행).

    drawer_close_seq()의 alias. 가독성용.
    """
    return drawer_close_seq(layer)
