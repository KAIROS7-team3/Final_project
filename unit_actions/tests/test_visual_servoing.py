"""unit_actions/tests/test_visual_servoing.py

HandleServoController 단위 테스트 (P-1).
ROS2 없이 실행: python3 -m pytest unit_actions/tests/test_visual_servoing.py -v
"""

import threading
import time
import pytest

from unit_actions.visual_servoing import (
    HandlePose,
    HandleServoController,
    ServoConfig,
    ServoState,
    ToolPose,
    ToolServoController,
    VelocityCommand,
    _clamp,
    _norm2,
)


# ── 픽스처 ────────────────────────────────────────────────────────────────────

def _cfg(**kw) -> ServoConfig:
    base = ServoConfig(
        kp=1.0,
        xz_align_thr_mm=3.0,
        detect_stable_frames=3,
        vel_limit_mm_s=20.0,
        timeout_sec=30.0,
    )
    for k, v in kw.items():
        setattr(base, k, v)
    return base


def _ctrl(
    cfg: ServoConfig,
    handle: HandlePose = None,
    ee_pose: tuple = (378.88, 433.02, 65.45),
    estop: threading.Event = None,
) -> HandleServoController:
    if handle is None:
        handle = HandlePose(x=378.88, z=65.45, valid=True)
    if estop is None:
        estop = threading.Event()
    return HandleServoController(
        cfg=cfg,
        get_handle=lambda: handle,
        get_ee_pose=lambda: ee_pose,
        estop_event=estop,
    )


# ── 유틸 함수 ─────────────────────────────────────────────────────────────────

def test_clamp_within():
    assert _clamp(10.0, 20.0) == 10.0

def test_clamp_positive_over():
    assert _clamp(30.0, 20.0) == 20.0

def test_clamp_negative_over():
    assert _clamp(-30.0, 20.0) == -20.0

def test_norm2():
    assert abs(_norm2(3.0, 4.0) - 5.0) < 1e-9


# ── 초기 상태 ─────────────────────────────────────────────────────────────────

def test_initial_state_is_detect():
    ctrl = _ctrl(_cfg())
    assert ctrl.state == ServoState.DETECT

def test_is_terminal_false_at_start():
    ctrl = _ctrl(_cfg())
    assert ctrl.is_terminal() is False


# ── DETECT 상태 ───────────────────────────────────────────────────────────────

def test_detect_stop_while_waiting():
    handle = HandlePose(valid=True, x=378.88, z=65.45)
    ctrl = _ctrl(_cfg(detect_stable_frames=3), handle=handle)
    cmd = ctrl.tick()
    assert cmd.stop is True
    assert ctrl.state == ServoState.DETECT   # 아직 3프레임 미만

def test_detect_transitions_after_stable_frames():
    handle = HandlePose(valid=True, x=378.88, z=65.45)
    ctrl = _ctrl(_cfg(detect_stable_frames=3), handle=handle,
                 ee_pose=(378.88, 433.02, 65.45))
    for _ in range(3):
        ctrl.tick()
    assert ctrl.state == ServoState.ALIGN_XZ

def test_detect_resets_count_on_invalid():
    frames = [
        HandlePose(valid=True, x=378.0, z=65.0),
        HandlePose(valid=True, x=378.0, z=65.0),
        HandlePose(valid=False),                  # 끊김
    ]
    idx = [0]
    def getter():
        h = frames[min(idx[0], len(frames)-1)]
        idx[0] += 1
        return h

    ctrl = HandleServoController(
        cfg=_cfg(detect_stable_frames=3),
        get_handle=getter,
        get_ee_pose=lambda: (378.88, 433.02, 65.45),
        estop_event=threading.Event(),
    )
    ctrl.tick()  # count=1
    ctrl.tick()  # count=2
    ctrl.tick()  # 끊김 → count=0
    assert ctrl._stable_count == 0
    assert ctrl.state == ServoState.DETECT


# ── ALIGN_XZ 상태 ─────────────────────────────────────────────────────────────

def test_align_xz_done_when_within_threshold():
    # EE가 이미 손잡이 위치와 일치
    handle = HandlePose(valid=True, x=378.88, z=65.45)
    ctrl = _ctrl(_cfg(detect_stable_frames=1, xz_align_thr_mm=3.0),
                 handle=handle, ee_pose=(378.88, 433.02, 65.45))
    ctrl.tick()   # DETECT → ALIGN_XZ (stable_frames=1)
    cmd = ctrl.tick()  # 오차=0 → DONE
    assert ctrl.state == ServoState.DONE
    assert cmd.stop is True

def test_align_xz_p_control_positive_error():
    # 손잡이가 EE보다 x+10, z+5 mm
    handle = HandlePose(valid=True, x=388.88, z=70.45)
    ctrl = _ctrl(_cfg(detect_stable_frames=1, kp=1.0, vel_limit_mm_s=20.0),
                 handle=handle, ee_pose=(378.88, 433.02, 65.45))
    ctrl.tick()   # DETECT → ALIGN_XZ
    cmd = ctrl.tick()
    assert abs(cmd.vx - 10.0) < 1e-6   # Kp×err_x = 1.0×10 = 10
    assert abs(cmd.vz -  5.0) < 1e-6   # Kp×err_z = 1.0×5  = 5
    assert cmd.vy == 0.0                # Y 고정

def test_align_xz_velocity_clamped():
    # 오차가 커서 vel_limit 초과
    handle = HandlePose(valid=True, x=500.0, z=200.0)
    ctrl = _ctrl(_cfg(detect_stable_frames=1, kp=1.0, vel_limit_mm_s=20.0),
                 handle=handle, ee_pose=(378.88, 433.02, 65.45))
    ctrl.tick()   # DETECT → ALIGN_XZ
    cmd = ctrl.tick()
    assert abs(cmd.vx) <= 20.0
    assert abs(cmd.vz) <= 20.0

def test_align_xz_vy_always_zero():
    handle = HandlePose(valid=True, x=390.0, z=70.0)
    ctrl = _ctrl(_cfg(detect_stable_frames=1),
                 handle=handle, ee_pose=(378.88, 433.02, 65.45))
    ctrl.tick()
    cmd = ctrl.tick()
    assert cmd.vy == 0.0

def test_align_xz_returns_to_detect_on_lost():
    handle_valid   = HandlePose(valid=True,  x=390.0, z=70.0)
    handle_invalid = HandlePose(valid=False)
    frames = [handle_valid, handle_invalid]
    idx = [0]
    def getter():
        h = frames[min(idx[0], len(frames)-1)]
        idx[0] += 1
        return h

    ctrl = HandleServoController(
        cfg=_cfg(detect_stable_frames=1),
        get_handle=getter,
        get_ee_pose=lambda: (378.88, 433.02, 65.45),
        estop_event=threading.Event(),
    )
    ctrl.tick()   # DETECT → ALIGN_XZ
    cmd = ctrl.tick()  # 소실 → DETECT 복귀
    assert ctrl.state == ServoState.DETECT
    assert cmd.stop is True


# ── DONE / ERROR / E-stop ─────────────────────────────────────────────────────

def test_is_terminal_true_after_done():
    handle = HandlePose(valid=True, x=378.88, z=65.45)
    ctrl = _ctrl(_cfg(detect_stable_frames=1, xz_align_thr_mm=3.0),
                 handle=handle, ee_pose=(378.88, 433.02, 65.45))
    ctrl.tick()
    ctrl.tick()
    assert ctrl.is_terminal() is True

def test_estop_returns_stop_and_error():
    estop = threading.Event()
    estop.set()
    ctrl = _ctrl(_cfg(), estop=estop)
    cmd = ctrl.tick()
    assert cmd.stop is True
    assert ctrl.state == ServoState.ERROR

def test_timeout_transitions_to_error():
    ctrl = _ctrl(_cfg(timeout_sec=0.0))
    time.sleep(0.01)
    cmd = ctrl.tick()
    assert ctrl.state == ServoState.ERROR
    assert cmd.stop is True

def test_timeout_monotonic_not_reset_on_state_change():
    """타임아웃 카운터가 상태 전환 시 리셋되지 않음을 검증."""
    handle = HandlePose(valid=True, x=390.0, z=70.0)
    ctrl = _ctrl(_cfg(detect_stable_frames=1, timeout_sec=0.05),
                 handle=handle, ee_pose=(378.88, 433.02, 65.45))
    ctrl.tick()   # DETECT → ALIGN_XZ
    time.sleep(0.06)
    cmd = ctrl.tick()  # 타임아웃 — 리셋 없이 ERROR
    assert ctrl.state == ServoState.ERROR
    assert cmd.stop is True

def test_is_terminal_true_after_error():
    ctrl = _ctrl(_cfg(timeout_sec=0.0))
    time.sleep(0.01)
    ctrl.tick()
    assert ctrl.is_terminal() is True


# ── ToolServoController 픽스처 ─────────────────────────────────────────────────

def _tool_cfg(**kw) -> ServoConfig:
    base = ServoConfig(
        kp=1.0,
        xy_align_thr_mm=3.0,
        detect_stable_frames=3,
        vel_limit_mm_s=20.0,
        timeout_sec=30.0,
    )
    for k, v in kw.items():
        setattr(base, k, v)
    return base


def _tool_ctrl(
    cfg: ServoConfig,
    tool: ToolPose = None,
    ee_pose: tuple = (100.0, 200.0, 234.0),
    estop: threading.Event = None,
) -> ToolServoController:
    if tool is None:
        tool = ToolPose(x=100.0, y=200.0, z=234.0, valid=True)
    if estop is None:
        estop = threading.Event()
    return ToolServoController(
        cfg=cfg,
        get_tool=lambda: tool,
        get_ee_pose=lambda: ee_pose,
        estop_event=estop,
    )


# ── ToolServoController 초기 상태 ─────────────────────────────────────────────

def test_tool_initial_state_is_detect():
    ctrl = _tool_ctrl(_tool_cfg())
    assert ctrl.state == ServoState.DETECT

def test_tool_is_terminal_false_at_start():
    ctrl = _tool_ctrl(_tool_cfg())
    assert ctrl.is_terminal() is False


# ── ToolServoController DETECT 상태 ──────────────────────────────────────────

def test_tool_detect_stop_while_waiting():
    tool = ToolPose(valid=True, x=100.0, y=200.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=3), tool=tool)
    cmd = ctrl.tick()
    assert cmd.stop is True
    assert ctrl.state == ServoState.DETECT

def test_tool_detect_transitions_after_stable_frames():
    tool = ToolPose(valid=True, x=100.0, y=200.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=3), tool=tool,
                      ee_pose=(100.0, 200.0, 234.0))
    for _ in range(3):
        ctrl.tick()
    assert ctrl.state == ServoState.ALIGN_XY

def test_tool_detect_resets_count_on_invalid():
    frames = [
        ToolPose(valid=True,  x=100.0, y=200.0, z=234.0),
        ToolPose(valid=True,  x=100.0, y=200.0, z=234.0),
        ToolPose(valid=False),
    ]
    idx = [0]
    def getter():
        p = frames[min(idx[0], len(frames) - 1)]
        idx[0] += 1
        return p

    ctrl = ToolServoController(
        cfg=_tool_cfg(detect_stable_frames=3),
        get_tool=getter,
        get_ee_pose=lambda: (100.0, 200.0, 234.0),
        estop_event=threading.Event(),
    )
    ctrl.tick()
    ctrl.tick()
    ctrl.tick()  # 끊김 → count 리셋
    assert ctrl._stable_count == 0
    assert ctrl.state == ServoState.DETECT


# ── ToolServoController ALIGN_XY 상태 ────────────────────────────────────────

def test_tool_align_xy_done_when_within_threshold():
    tool = ToolPose(valid=True, x=100.0, y=200.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=1, xy_align_thr_mm=3.0),
                      tool=tool, ee_pose=(100.0, 200.0, 234.0))
    ctrl.tick()   # DETECT → ALIGN_XY
    cmd = ctrl.tick()  # 오차=0 → DONE
    assert ctrl.state == ServoState.DONE
    assert cmd.stop is True

def test_tool_align_xy_p_control():
    # 공구가 EE보다 x+10, y+8 mm
    tool = ToolPose(valid=True, x=110.0, y=208.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=1, kp=1.0, vel_limit_mm_s=20.0),
                      tool=tool, ee_pose=(100.0, 200.0, 234.0))
    ctrl.tick()   # DETECT → ALIGN_XY
    cmd = ctrl.tick()
    assert abs(cmd.vx - 10.0) < 1e-6   # Kp × err_x = 1.0 × 10 = 10
    assert abs(cmd.vy -  8.0) < 1e-6   # Kp × err_y = 1.0 × 8  = 8
    assert cmd.vz == 0.0                # Z 고정

def test_tool_align_xy_velocity_clamped():
    tool = ToolPose(valid=True, x=500.0, y=600.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=1, kp=1.0, vel_limit_mm_s=20.0),
                      tool=tool, ee_pose=(100.0, 200.0, 234.0))
    ctrl.tick()   # DETECT → ALIGN_XY
    cmd = ctrl.tick()
    assert abs(cmd.vx) <= 20.0
    assert abs(cmd.vy) <= 20.0

def test_tool_align_xy_vz_always_zero():
    tool = ToolPose(valid=True, x=110.0, y=210.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=1), tool=tool,
                      ee_pose=(100.0, 200.0, 234.0))
    ctrl.tick()
    cmd = ctrl.tick()
    assert cmd.vz == 0.0

def test_tool_align_xy_returns_to_detect_on_lost():
    tool_valid   = ToolPose(valid=True,  x=110.0, y=210.0, z=234.0)
    tool_invalid = ToolPose(valid=False)
    frames = [tool_valid, tool_invalid]
    idx = [0]
    def getter():
        p = frames[min(idx[0], len(frames) - 1)]
        idx[0] += 1
        return p

    ctrl = ToolServoController(
        cfg=_tool_cfg(detect_stable_frames=1),
        get_tool=getter,
        get_ee_pose=lambda: (100.0, 200.0, 234.0),
        estop_event=threading.Event(),
    )
    ctrl.tick()   # DETECT → ALIGN_XY
    cmd = ctrl.tick()  # 소실 → DETECT 복귀
    assert ctrl.state == ServoState.DETECT
    assert cmd.stop is True


# ── ToolServoController DONE / ERROR / E-stop ─────────────────────────────────

def test_tool_is_terminal_true_after_done():
    tool = ToolPose(valid=True, x=100.0, y=200.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=1, xy_align_thr_mm=3.0),
                      tool=tool, ee_pose=(100.0, 200.0, 234.0))
    ctrl.tick()
    ctrl.tick()
    assert ctrl.is_terminal() is True

def test_tool_estop_returns_stop_and_error():
    estop = threading.Event()
    estop.set()
    ctrl = _tool_ctrl(_tool_cfg(), estop=estop)
    cmd = ctrl.tick()
    assert cmd.stop is True
    assert ctrl.state == ServoState.ERROR

def test_tool_timeout_transitions_to_error():
    ctrl = _tool_ctrl(_tool_cfg(timeout_sec=0.0))
    time.sleep(0.01)
    cmd = ctrl.tick()
    assert ctrl.state == ServoState.ERROR
    assert cmd.stop is True

def test_tool_timeout_monotonic_not_reset_on_state_change():
    """타임아웃 카운터가 상태 전환 시 리셋되지 않음을 검증."""
    tool = ToolPose(valid=True, x=110.0, y=210.0, z=234.0)
    ctrl = _tool_ctrl(_tool_cfg(detect_stable_frames=1, timeout_sec=0.05),
                      tool=tool, ee_pose=(100.0, 200.0, 234.0))
    ctrl.tick()   # DETECT → ALIGN_XY
    time.sleep(0.06)
    cmd = ctrl.tick()  # 타임아웃 — 리셋 없이 ERROR
    assert ctrl.state == ServoState.ERROR
    assert cmd.stop is True

def test_tool_is_terminal_true_after_error():
    ctrl = _tool_ctrl(_tool_cfg(timeout_sec=0.0))
    time.sleep(0.01)
    ctrl.tick()
    assert ctrl.is_terminal() is True
