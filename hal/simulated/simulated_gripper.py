import logging

from hal.gripper_interface import GripperInterface

logger = logging.getLogger(__name__)


class SimulatedGripper(GripperInterface):
    """Mock gripper that logs calls and tracks internal state. No hardware access."""

    def __init__(self) -> None:
        self._position = 0.0  # 0.0=open, 1.0=closed
        self._grasping = False
        self._estop = False

    def set_position(self, position: float, force: float = 20.0) -> bool:
        if self._estop:
            logger.warning("[SimulatedGripper] set_position blocked — E-stop active")
            return False
        if not (0.0 <= position <= 1.0):
            logger.error("[SimulatedGripper] invalid position=%.2f (must be 0.0–1.0)", position)
            return False
        logger.info("[SimulatedGripper] set_position - position=%.2f force=%.1fN", position, force)
        self._position = position
        self._grasping = position > 0.8
        return True

    def get_position(self) -> float:
        return self._position

    def is_grasping(self) -> bool:
        return self._grasping

    def open(self) -> bool:
        return self.set_position(0.0)

    def close(self, force: float = 20.0) -> bool:
        return self.set_position(1.0, force=force)

    def emergency_stop(self) -> None:
        self._estop = True
        logger.info("[SimulatedGripper] E-stop triggered")
