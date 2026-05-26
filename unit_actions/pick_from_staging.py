import logging

from hal.arm_interface import ArmInterface, Pose

logger = logging.getLogger(__name__)


def pick_from_staging(arm: ArmInterface, tool_id: str, staging_pose: Pose) -> bool:
    """Move arm to staging area to pick up a tool.

    The staging_pose must be pre-loaded from config/staging_area.yaml by the caller.

    Args:
        arm: ArmInterface implementation.
        tool_id: Tool being picked (for logging/DB).
        staging_pose: Target pose in robot_base_link frame (meters + quaternion).

    Returns:
        True on success.
    No rclpy dependency.
    """
    logger.info("[pick_from_staging] tool_id=%s target=%s", tool_id, staging_pose)
    success = arm.move_to_pose(staging_pose)
    if not success:
        logger.error("[pick_from_staging] arm motion failed - tool_id=%s", tool_id)
    return success
