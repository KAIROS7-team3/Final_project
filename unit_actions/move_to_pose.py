import logging

from hal.arm_interface import ArmInterface, Pose

logger = logging.getLogger(__name__)


def move_to_pose(arm: ArmInterface, pose: Pose, velocity_scale: float = 0.3) -> bool:
    """Command arm to move end-effector to target pose.

    Args:
        arm: ArmInterface implementation.
        pose: Target pose in robot_base_link frame. Coordinates in meters, rotation as quaternion.
        velocity_scale: Motion speed as fraction of max velocity (0.0–1.0).

    Returns:
        True on success, False on failure.
    Raises:
        ValueError: If velocity_scale is out of [0.0, 1.0].
    No rclpy dependency.
    """
    if not (0.0 <= velocity_scale <= 1.0):
        raise ValueError(f"velocity_scale must be in [0.0, 1.0], got {velocity_scale}")
    logger.info("[move_to_pose] target=%s velocity_scale=%.2f", pose, velocity_scale)
    success = arm.move_to_pose(pose, velocity_scale=velocity_scale)
    if not success:
        logger.error("[move_to_pose] motion failed - target=%s", pose)
    return success
