import logging

from hal.arm_interface import ArmInterface, Pose

logger = logging.getLogger(__name__)


def return_to_slot(
    arm: ArmInterface,
    tool_id: str,
    slot_row: int,
    slot_col: int,
    slot_pose: Pose,
) -> bool:
    """Move arm to return a tool to its home slot.

    The slot_pose must be pre-loaded from config/toolbox.yaml by the caller.

    Args:
        arm: ArmInterface implementation.
        tool_id: Tool being returned.
        slot_row: Target slot row (0-indexed).
        slot_col: Target slot column (0-indexed).
        slot_pose: Target pose in robot_base_link frame (meters + quaternion).

    Returns:
        True on success.
    No rclpy dependency.
    """
    logger.info("[return_to_slot] tool_id=%s slot=(%d,%d) target=%s", tool_id, slot_row, slot_col, slot_pose)
    success = arm.move_to_pose(slot_pose)
    if not success:
        logger.error("[return_to_slot] arm motion failed - tool_id=%s slot=(%d,%d)", tool_id, slot_row, slot_col)
    return success
