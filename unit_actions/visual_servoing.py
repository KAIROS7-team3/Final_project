"""visual_servoing.py
─────────────────────
Visual Servoing 제어기 모음 (Track B, Phase 1).

HandleServoController — 서랍 손잡이 XZ 정렬 (vision_open / vision_close)
  그리퍼가 Y 방향으로 손잡이를 향하므로 XZ만 보정, Y 고정.
  제어식: vx = Kp×err_x, vz = Kp×err_z, vy = 0

ToolServoController   — 공구 접근 XY 정렬 (vision_fetch)
  그리퍼가 Z 방향으로 공구 위에서 하강하므로 XY만 보정, Z 고정.
  제어식: vx = Kp×err_x, vy = Kp×err_y, vz = 0

상태 기계 (공통):
  DETECT  → 좌표 N프레임 연속 수신 대기
  ALIGN   → P제어 (축 종류만 다름)
  DONE    → 정렬 완료, 호출자가 다음 스텝 실행
  ERROR   → 타임아웃 / 소실 → 호출자가 홈 복귀 + PLC + DB 처리

단위: DSR 네이티브 (mm, mm/s) — E-1 예외, unit_action_server.py 래퍼에서 변환.
ROS2 의존성 없음 — 단독 테스트 가능.
"""

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Callable, Optional

logger = logging.getLogger(__name__)


# ── 데이터 클래스 ──────────────────────────────────────────────────────────────

@dataclass
class ToolPose:
    """그리퍼 카메라가 제공하는 공구 중심 좌표.

    x, y : robot base frame 기준 (mm) — XY 이동에 사용
    z    : robot base frame 기준 (mm) — 하강 거리에 사용
    rz   : robot base frame 기준 (deg) — 공구 방위각 (yaw)
    valid: 해당 프레임 좌표 유효 여부
    """
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rz: float = 0.0
    valid: bool = False


@dataclass
class HandlePose:
    """비전 노드가 TF 변환 후 제공하는 손잡이 중심 좌표.

    x, z  : robot base frame 기준 (mm)
    valid : 해당 프레임 좌표 유효 여부
    """
    x: float = 0.0
    z: float = 0.0
    valid: bool = False


@dataclass
class VelocityCommand:
    """로봇 말단 속도 명령 (DSR BASE 좌표계, mm/s).

    vx, vy, vz : 선속도
    stop       : True면 즉시 정지 (E-stop 포함)
    """
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    stop: bool = False


@dataclass
class ServoConfig:
    """Visual Servoing 파라미터. config/visual_servo.yaml에서 로드."""
    kp: float = 1.0
    xz_align_thr_mm: float = 3.0   # HandleServoController 용
    xy_align_thr_mm: float = 3.0   # ToolServoController 용
    detect_stable_frames: int = 3
    vel_limit_mm_s: float = 20.0
    timeout_sec: float = 15.0

    @classmethod
    def load_from_yaml(cls, path: str, section: str = "") -> "ServoConfig":
        """section: "handle" | "tool" | "" (루트, 구버전 호환)."""
        import yaml
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        data = raw.get(section, raw) if section else raw
        return cls(
            kp                   = data.get("gains",     {}).get("kp",                cls.kp),
            xz_align_thr_mm      = data.get("thresholds",{}).get("xz_align_thr_mm",   cls.xz_align_thr_mm),
            xy_align_thr_mm      = data.get("thresholds",{}).get("xy_align_thr_mm",   cls.xy_align_thr_mm),
            detect_stable_frames = data.get("detect",    {}).get("stable_frames",     cls.detect_stable_frames),
            vel_limit_mm_s       = data.get("vel_limit_mm_s",                         cls.vel_limit_mm_s),
            timeout_sec          = data.get("timeout_sec",                            cls.timeout_sec),
        )


# ── 상태 열거형 ────────────────────────────────────────────────────────────────

class ServoState(Enum):
    DETECT   = auto()   # 좌표 안정 수신 대기 (공통)
    ALIGN_XZ = auto()   # XZ P 제어 — HandleServoController
    ALIGN_XY = auto()   # XY P 제어 — ToolServoController
    DONE     = auto()   # 정렬 완료 → 호출자가 다음 스텝 실행
    ERROR    = auto()   # 타임아웃 / 소실 → 호출자가 실패 처리


# ── 인터페이스 콜백 타입 ──────────────────────────────────────────────────────

HandleGetterFn = Callable[[], HandlePose]               # 손잡이 TF 좌표 반환
EEPoseGetterFn = Callable[[], tuple[float, float, float]]  # EE 위치 (x, y, z) mm


# ── Visual Servoing 제어기 ─────────────────────────────────────────────────────

class HandleServoController:
    """서랍 손잡이 XZ 정렬 상태 기계.

    사용 예 (runner에서):
        cfg  = ServoConfig.load_from_yaml("config/visual_servo.yaml")
        ctrl = HandleServoController(
            cfg,
            get_handle=lambda: vision_node.get_handle_pose(),
            get_ee_pose=lambda: robot.get_ee_pose_mm(),
            estop_event=self._estop_event,   # S-3: runner의 Event 주입
        )
        while not ctrl.is_terminal():
            cmd = ctrl.tick()
            send_velocity(cmd)
            time.sleep(0.033)   # 30 Hz

        if ctrl.state == ServoState.DONE:
            grip_box()          # ⑥ GRIP_BOX
        else:
            handle_error()      # 홈 복귀 + PLC + DB (E-5)
    """

    def __init__(
        self,
        cfg: ServoConfig,
        get_handle: HandleGetterFn,
        get_ee_pose: EEPoseGetterFn,
        estop_event: threading.Event,        # S-3: runner에서 주입
    ) -> None:
        self._cfg        = cfg
        self._get_handle = get_handle
        self._get_ee     = get_ee_pose
        self._estop      = estop_event       # S-3: 매 tick 체크

        self._state: ServoState = ServoState.DETECT
        self._stable_count: int = 0
        self._start: float = time.monotonic()  # 상태 전환과 무관하게 고정 (Finding 5)

        logger.info("[servo] 초기화 — state=DETECT thr=%.1fmm timeout=%.1fs",
                    cfg.xz_align_thr_mm, cfg.timeout_sec)

    # ── 공개 API ─────────────────────────────────────────────────────────────

    @property
    def state(self) -> ServoState:
        return self._state

    def is_terminal(self) -> bool:
        return self._state in (ServoState.DONE, ServoState.ERROR)

    def tick(self) -> VelocityCommand:
        """제어 루프 1 tick. 30 Hz 호출 권장."""

        # S-3: E-stop 최우선 체크 — 매 tick 진입 시
        if self._estop.is_set():
            logger.error("[servo] E-stop 감지 — 즉시 정지")
            return self._to_error("E-stop")

        # 타임아웃: 루프 시작 시각 기준 단조증가 (상태 전환 시 리셋 없음)
        if time.monotonic() - self._start > self._cfg.timeout_sec:
            return self._to_error(f"타임아웃 {self._cfg.timeout_sec}s 초과")

        if self._state == ServoState.DETECT:
            return self._tick_detect()
        if self._state == ServoState.ALIGN_XZ:
            return self._tick_align_xz()

        return VelocityCommand(stop=True)

    # ── STATE 1: DETECT ───────────────────────────────────────────────────────

    def _tick_detect(self) -> VelocityCommand:
        """손잡이 TF 좌표 N프레임 연속 수신 대기. 로봇 정지 유지."""
        handle = self._get_handle()

        if handle.valid:
            self._stable_count += 1
            logger.debug("[servo][DETECT] 안정 %d/%d",
                         self._stable_count, self._cfg.detect_stable_frames)
        else:
            if self._stable_count > 0:
                logger.debug("[servo][DETECT] 수신 끊김 — count 리셋")
            self._stable_count = 0

        if self._stable_count >= self._cfg.detect_stable_frames:
            logger.info("[servo][DETECT→ALIGN_XZ] 손잡이 좌표 안정 확보")
            self._state = ServoState.ALIGN_XZ
            return VelocityCommand(stop=True)

        return VelocityCommand(stop=True)

    # ── STATE 2: ALIGN_XZ ────────────────────────────────────────────────────

    def _tick_align_xz(self) -> VelocityCommand:
        """XZ 오차 P 제어.

        수식:
            err_x = target_x - ee_x   (mm)
            err_z = target_z - ee_z   (mm)
            vx    = Kp × err_x        (mm/s)
            vz    = Kp × err_z        (mm/s)
            vy    = 0.0               (Y 고정 — 서랍 당기는 방향)
        """
        handle = self._get_handle()

        if not handle.valid:
            logger.warning("[servo][ALIGN_XZ] 손잡이 좌표 소실 — DETECT 복귀")
            self._state = ServoState.DETECT
            self._stable_count = 0
            return VelocityCommand(stop=True)

        ee_x, _, ee_z = self._get_ee()

        err_x = handle.x - ee_x   # mm
        err_z = handle.z - ee_z   # mm
        dist  = _norm2(err_x, err_z)

        # 정렬 완료
        if dist <= self._cfg.xz_align_thr_mm:
            logger.info("[servo][ALIGN_XZ→DONE] 정렬 완료 dist=%.2fmm", dist)
            self._state = ServoState.DONE
            return VelocityCommand(stop=True)

        # P 제어
        vx = _clamp(self._cfg.kp * err_x, self._cfg.vel_limit_mm_s)
        vz = _clamp(self._cfg.kp * err_z, self._cfg.vel_limit_mm_s)

        logger.debug("[servo][ALIGN_XZ] err=(%.2f, %.2f)mm dist=%.2fmm → vx=%.2f vz=%.2f mm/s",
                     err_x, err_z, dist, vx, vz)

        return VelocityCommand(vx=vx, vy=0.0, vz=vz)

    # ── 내부 유틸 ─────────────────────────────────────────────────────────────

    def _to_error(self, reason: str) -> VelocityCommand:
        logger.error("[servo] ERROR: %s", reason)
        self._state = ServoState.ERROR
        return VelocityCommand(stop=True)


# ── 공구 접근 XY VS 제어기 ────────────────────────────────────────────────────

ToolGetterFn = Callable[[], ToolPose]


class ToolServoController:
    """공구 접근 XY 정렬 상태 기계 (vision_fetch 전용).

    그리퍼가 Z 방향으로 공구 위에서 하강하므로 XY만 보정하고 Z는 고정.
    정렬 완료(DONE) 후 호출자가 그리퍼 캠 Z 좌표로 하강(MOVE_L_TOOL_XYZ) 실행.

    사용 예 (runner에서):
        cfg  = ServoConfig.load_from_yaml("config/visual_servo.yaml", section="tool")
        ctrl = ToolServoController(
            cfg,
            get_tool=lambda: vision_node.get_tool_pose(),
            get_ee_pose=lambda: robot.get_ee_pose_mm(),
            estop_event=self._estop_event,
        )
        while not ctrl.is_terminal():
            cmd = ctrl.tick()
            send_velocity(cmd)
            time.sleep(0.033)

        if ctrl.state == ServoState.DONE:
            move_to_tool_xyz()   # ④ 그리퍼 캠 XYZ 로 하강
        else:
            handle_error()
    """

    def __init__(
        self,
        cfg: ServoConfig,
        get_tool: ToolGetterFn,
        get_ee_pose: EEPoseGetterFn,
        estop_event: threading.Event,
    ) -> None:
        self._cfg      = cfg
        self._get_tool = get_tool
        self._get_ee   = get_ee_pose
        self._estop    = estop_event

        self._state: ServoState = ServoState.DETECT
        self._stable_count: int = 0
        self._start: float = time.monotonic()

        logger.info("[tool_servo] 초기화 — state=DETECT thr=%.1fmm timeout=%.1fs",
                    cfg.xy_align_thr_mm, cfg.timeout_sec)

    @property
    def state(self) -> ServoState:
        return self._state

    def is_terminal(self) -> bool:
        return self._state in (ServoState.DONE, ServoState.ERROR)

    def tick(self) -> VelocityCommand:
        if self._estop.is_set():
            logger.error("[tool_servo] E-stop 감지 — 즉시 정지")
            return self._to_error("E-stop")

        if time.monotonic() - self._start > self._cfg.timeout_sec:
            return self._to_error(f"타임아웃 {self._cfg.timeout_sec}s 초과")

        if self._state == ServoState.DETECT:
            return self._tick_detect()
        if self._state == ServoState.ALIGN_XY:
            return self._tick_align_xy()

        return VelocityCommand(stop=True)

    def _tick_detect(self) -> VelocityCommand:
        tool = self._get_tool()

        if tool.valid:
            self._stable_count += 1
            logger.debug("[tool_servo][DETECT] 안정 %d/%d",
                         self._stable_count, self._cfg.detect_stable_frames)
        else:
            if self._stable_count > 0:
                logger.debug("[tool_servo][DETECT] 수신 끊김 — count 리셋")
            self._stable_count = 0

        if self._stable_count >= self._cfg.detect_stable_frames:
            logger.info("[tool_servo][DETECT→ALIGN_XY] 공구 좌표 안정 확보")
            self._state = ServoState.ALIGN_XY
            return VelocityCommand(stop=True)

        return VelocityCommand(stop=True)

    def _tick_align_xy(self) -> VelocityCommand:
        """XY 오차 P 제어.

        수식:
            err_x = target_x - ee_x   (mm)
            err_y = target_y - ee_y   (mm)
            vx    = Kp × err_x        (mm/s)
            vy    = Kp × err_y        (mm/s)
            vz    = 0.0               (Z 고정 — 하강은 VS 완료 후 별도 실행)
        """
        tool = self._get_tool()

        if not tool.valid:
            logger.warning("[tool_servo][ALIGN_XY] 공구 좌표 소실 — DETECT 복귀")
            self._state = ServoState.DETECT
            self._stable_count = 0
            return VelocityCommand(stop=True)

        ee_x, ee_y, _ = self._get_ee()

        err_x = tool.x - ee_x
        err_y = tool.y - ee_y
        dist  = _norm2(err_x, err_y)

        if dist <= self._cfg.xy_align_thr_mm:
            logger.info("[tool_servo][ALIGN_XY→DONE] 정렬 완료 dist=%.2fmm", dist)
            self._state = ServoState.DONE
            return VelocityCommand(stop=True)

        vx = _clamp(self._cfg.kp * err_x, self._cfg.vel_limit_mm_s)
        vy = _clamp(self._cfg.kp * err_y, self._cfg.vel_limit_mm_s)

        logger.debug("[tool_servo][ALIGN_XY] err=(%.2f, %.2f)mm dist=%.2fmm → vx=%.2f vy=%.2f mm/s",
                     err_x, err_y, dist, vx, vy)

        return VelocityCommand(vx=vx, vy=vy, vz=0.0)

    def _to_error(self, reason: str) -> VelocityCommand:
        logger.error("[tool_servo] ERROR: %s", reason)
        self._state = ServoState.ERROR
        return VelocityCommand(stop=True)


# ── 순수 함수 유틸 ─────────────────────────────────────────────────────────────

def _clamp(value: float, limit: float) -> float:
    """속도 상한 클램프 (S-5)."""
    return max(-limit, min(limit, value))


def _norm2(a: float, b: float) -> float:
    return (a * a + b * b) ** 0.5
