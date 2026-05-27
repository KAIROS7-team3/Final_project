import logging

from hal.gripper_interface import GripperInterface

logger = logging.getLogger(__name__)


def release(gripper: GripperInterface) -> bool:
    """Open gripper to release currently held tool.

    Returns:
        True if gripper opened successfully.
    No rclpy dependency.
    """
    logger.info("[release] opening gripper")
    success = gripper.open()
    if not success:
        logger.error("[release] gripper open failed")
    return success
