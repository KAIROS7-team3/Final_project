"""place_on_hand.py — 손바닥 위에 공구를 올려놓는 유닛 액션 (direct 타입).

handover_type=direct 공구(스패너, 소켓, 멕가이버)용.

흐름:
  1. get_hand_pose()로 손바닥 XYZ + 쿼터니언 읽기
  2. 손바닥 위 approach_height_m 위치로 이동 (TCP 방향 = 손바닥 방향)
  3. is_hand_ready() 재확인
  4. Z 수직 하강 (하강 완료 후 Z 변화 확인)
  5. 그리퍼 열기 → 공구 놓음
  6. Z 수직 복귀

No rclpy dependency (unit_actions 규칙 E-2).
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Callable

from hal.arm_interface import ArmInterface, Pose
from hal.gripper_interface import GripperInterface

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class HandoverConfig:
    approach_height_m: float = 0.10       # 손바닥 위 대기 높이
    lift_height_m: float = 0.10           # 전달 후 복귀 높이
    velocity_scale: float = 0.20          # S-6: 속도 scale 0.2 이하 강제
    z_abort_m: float = 0.05              # 하강 후 Z 변화 허용치 (5cm)
    ready_recheck_timeout_s: float = 2.0  # 대기점 도달 후 ready 재확인 대기 시간


# /hand/pose (geometry_msgs/PoseStamped)를 간단히 담는 타입
@dataclass
class HandPose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def place_on_hand(
    arm: ArmInterface,
    gripper: GripperInterface,
    get_hand_pose: Callable[[], HandPose | None],
    is_hand_ready: Callable[[], bool],
    cfg: HandoverConfig | None = None,
) -> bool:
    """손바닥 위에 공구를 직접 올려놓는다 (direct 타입).

    Args:
        arm: ArmInterface 구현체.
        gripper: GripperInterface 구현체.
        get_hand_pose: 최신 /hand/pose를 HandPose로 반환하는 콜백.
        is_hand_ready: /hand/ready 상태 콜백.
        cfg: HandoverConfig. None이면 기본값.

    Returns:
        True: 전달 성공.
        False: abort → Staging fallback.
    """
    if cfg is None:
        cfg = HandoverConfig()

    # ── Phase 1: 손 위치 확인 ────────────────────────────────────────────
    hp = get_hand_pose()
    if hp is None or not is_hand_ready():
        logger.warning("[place_on_hand] 손 감지 없음 — abort")
        return False

    logger.info("[place_on_hand] 손 위치: x=%.3f y=%.3f z=%.3f", hp.x, hp.y, hp.z)

    # ── Phase 2: 손바닥 위 대기점으로 이동 ──────────────────────────────
    approach = Pose(
        position=(hp.x, hp.y, hp.z + cfg.approach_height_m),
        quaternion=(hp.qx, hp.qy, hp.qz, hp.qw),
    )
    logger.info(
        "[place_on_hand] 대기점 이동: z=%.3f (+%.2fm)",
        hp.z + cfg.approach_height_m, cfg.approach_height_m,
    )
    if not arm.move_to_pose(approach, velocity_scale=cfg.velocity_scale):
        logger.error("[place_on_hand] 대기점 이동 실패 — abort")
        return False

    # ── Phase 3: /hand/ready 재확인 ──────────────────────────────────────
    deadline = time.monotonic() + cfg.ready_recheck_timeout_s
    ready = False
    while time.monotonic() < deadline:
        if is_hand_ready():
            ready = True
            break
        time.sleep(0.1)
    if not ready:
        logger.warning("[place_on_hand] ready 재확인 실패 — abort")
        return False

    # 최신 손 위치로 갱신
    hp = get_hand_pose()
    if hp is None:
        logger.warning("[place_on_hand] 재확인 후 손 위치 소실 — abort")
        return False
    z_ref = hp.z

    # ── Phase 4: Z 수직 하강 ─────────────────────────────────────────────
    place = Pose(
        position=(hp.x, hp.y, z_ref),
        quaternion=(hp.qx, hp.qy, hp.qz, hp.qw),
    )
    logger.info("[place_on_hand] 하강: z=%.3f", z_ref)
    if not arm.move_to_pose(place, velocity_scale=cfg.velocity_scale):
        logger.error("[place_on_hand] 하강 실패 — abort")
        _lift(arm, place, cfg)
        return False

    # 하강 완료 후 손 Z 변화 체크
    hp_now = get_hand_pose()
    if hp_now is not None:
        z_delta = abs(hp_now.z - z_ref)
        if z_delta > cfg.z_abort_m:
            logger.warning(
                "[place_on_hand] 손 Z 변화 %.3fm > %.3fm — abort",
                z_delta, cfg.z_abort_m,
            )
            _lift(arm, place, cfg)
            return False

    # ── Phase 5: 그리퍼 열기 ─────────────────────────────────────────────
    logger.info("[place_on_hand] 그리퍼 열기")
    gripper.open()

    # ── Phase 6: 수직 복귀 ───────────────────────────────────────────────
    _lift(arm, place, cfg)
    logger.info("[place_on_hand] 전달 완료")
    return True


def _lift(arm: ArmInterface, from_pose: Pose, cfg: HandoverConfig) -> None:
    """현재 위치에서 Z만 lift_height_m 위로 복귀."""
    x, y, z = from_pose.position
    lift = Pose(
        position=(x, y, z + cfg.lift_height_m),
        quaternion=from_pose.quaternion,
    )
    arm.move_to_pose(lift, velocity_scale=cfg.velocity_scale)
