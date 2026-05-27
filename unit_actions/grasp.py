import logging

from hal.gripper_interface import GripperInterface

logger = logging.getLogger(__name__)

_DEFAULT_GRASP_FORCE_N = 20.0


def grasp(gripper: GripperInterface, tool_id: str, grasp_force: float = 0.0) -> bool:
    """Close gripper to grasp a tool.

    Args:
        gripper: GripperInterface implementation.
        tool_id: Target tool identifier (used for logging only at this layer).
        grasp_force: Force limit in N. 0.0 means use default from config/toolbox.yaml.

    Returns:
        True if gripper closed and is_grasping() is True.
    No rclpy dependency.
    """
    force = grasp_force if grasp_force > 0.0 else _DEFAULT_GRASP_FORCE_N
    logger.info("[grasp] closing gripper - tool_id=%s force=%.1fN", tool_id, force)
    success = gripper.close(force=force)
    if not success:
        logger.error("[grasp] gripper close failed - tool_id=%s", tool_id)
        return False
    if not gripper.is_grasping():
        logger.error("[grasp] gripper closed but no grasp detected - tool_id=%s", tool_id)
        return False
    logger.info("[grasp] success - tool_id=%s", tool_id)
    return True
