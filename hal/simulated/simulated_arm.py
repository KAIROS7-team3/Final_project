import logging
import time

from hal.arm_interface import ArmInterface, JointStates, Pose

logger = logging.getLogger(__name__)

# Doosan e0509 joint limits (rad) — from hardware.md
_JOINT_LIMITS_RAD = [
    (-3.14159, 3.14159),
    (-3.14159, 3.14159),
    (-3.14159, 3.14159),
    (-3.14159, 3.14159),
    (-3.14159, 3.14159),
    (-3.14159, 3.14159),
]
_HOME_JOINT_POSITIONS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class SimulatedArm(ArmInterface):
    """Mock arm that logs calls and returns plausible data. No hardware access."""

    def __init__(self) -> None:
        self._joint_positions = list(_HOME_JOINT_POSITIONS)
        self._current_pose = Pose(position=(0.5, 0.0, 0.5), quaternion=(0.0, 0.0, 0.0, 1.0))
        self._moving = False
        self._estop = False

    def move_to_pose(self, pose: Pose, velocity_scale: float = 0.3) -> bool:
        if self._estop:
            logger.warning("[SimulatedArm] move_to_pose blocked — E-stop active")
            return False
        logger.info("[SimulatedArm] move_to_pose - target=%s velocity_scale=%.2f", pose, velocity_scale)
        self._moving = True
        time.sleep(0.05)  # simulate motion delay
        self._current_pose = pose
        self._moving = False
        return True

    def move_to_joint_positions(self, positions: list[float], velocity_scale: float = 0.3) -> bool:
        if self._estop:
            logger.warning("[SimulatedArm] move_to_joint_positions blocked — E-stop active")
            return False
        for i, (pos, (lo, hi)) in enumerate(zip(positions, _JOINT_LIMITS_RAD)):
            if not (lo <= pos <= hi):
                logger.error("[SimulatedArm] joint limit violation - joint=%d value=%.3f", i, pos)
                return False
        self._moving = True
        time.sleep(0.05)
        self._joint_positions = list(positions)
        self._moving = False
        return True

    def get_joint_states(self) -> JointStates:
        return JointStates(
            positions=list(self._joint_positions),
            velocities=[0.0] * 6,
            efforts=[0.0] * 6,
        )

    def get_end_effector_pose(self) -> Pose:
        return self._current_pose

    def emergency_stop(self) -> None:
        self._estop = True
        self._moving = False
        logger.info("[SimulatedArm] E-stop triggered")

    def is_moving(self) -> bool:
        return self._moving
