import logging
from dataclasses import dataclass

from hal.arm_interface import ArmInterface, Pose
from hal.camera_interface import CameraInterface

logger = logging.getLogger(__name__)


@dataclass
class WorkspaceScan:
    rgb_frame_shape: tuple[int, ...]
    depth_frame_shape: tuple[int, ...]
    end_effector_pose: Pose
    success: bool


def scan_workspace(arm: ArmInterface, camera: CameraInterface) -> WorkspaceScan:
    """Capture aligned RGB+depth frames and current arm pose for YOLOv11s detection.

    Returns WorkspaceScan with frame metadata and arm pose on success.
    Returns WorkspaceScan(success=False) on camera failure; does not suppress arm errors.
    No rclpy dependency.
    """
    logger.info("[scan_workspace] starting workspace scan")
    try:
        rgb, depth = camera.get_aligned_frames()
    except Exception as e:
        logger.error("[scan_workspace] camera capture failed - error=%s", e)
        pose = arm.get_end_effector_pose()
        return WorkspaceScan(rgb_frame_shape=(), depth_frame_shape=(), end_effector_pose=pose, success=False)
    pose = arm.get_end_effector_pose()
    logger.info("[scan_workspace] captured frames - rgb=%s depth=%s", rgb.shape, depth.shape)
    return WorkspaceScan(
        rgb_frame_shape=rgb.shape,
        depth_frame_shape=depth.shape,
        end_effector_pose=pose,
        success=True,
    )
