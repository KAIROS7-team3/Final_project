import pytest

from hal.arm_interface import Pose
from hal.simulated.simulated_arm import SimulatedArm
from hal.simulated.simulated_camera import SimulatedCamera
from hal.simulated.simulated_gripper import SimulatedGripper
from unit_actions.grasp import grasp
from unit_actions.move_to_pose import move_to_pose
from unit_actions.pick_from_staging import pick_from_staging
from unit_actions.place_at_staging import place_at_staging
from unit_actions.release import release
from unit_actions.return_to_slot import return_to_slot
from unit_actions.scan_workspace import scan_workspace


@pytest.fixture
def arm():
    return SimulatedArm()


@pytest.fixture
def gripper():
    return SimulatedGripper()


@pytest.fixture
def camera():
    c = SimulatedCamera()
    c.start()
    return c


@pytest.fixture
def target_pose():
    return Pose(position=(0.4, 0.1, 0.3), quaternion=(0.0, 0.0, 0.0, 1.0))


class TestScanWorkspace:
    def test_happy_path(self, arm, camera):
        result = scan_workspace(arm, camera)
        assert result.success is True
        assert result.rgb_frame_shape == (480, 640, 3)
        assert result.depth_frame_shape == (480, 640)

    def test_failure_camera_raises(self, arm):
        from unittest.mock import MagicMock
        broken_cam = MagicMock()
        broken_cam.get_aligned_frames.side_effect = RuntimeError("camera disconnected")
        result = scan_workspace(arm, broken_cam)
        assert result.success is False
        assert result.rgb_frame_shape == ()


class TestMoveToPose:
    def test_happy_path(self, arm, target_pose):
        assert move_to_pose(arm, target_pose) is True
        assert arm.get_end_effector_pose() == target_pose

    def test_invalid_velocity_scale(self, arm, target_pose):
        with pytest.raises(ValueError):
            move_to_pose(arm, target_pose, velocity_scale=1.5)

    def test_estop_blocks_motion(self, arm, target_pose):
        arm.emergency_stop()
        assert move_to_pose(arm, target_pose) is False


class TestGrasp:
    def test_happy_path(self, gripper):
        assert grasp(gripper, "wrench_8mm") is True
        assert gripper.is_grasping() is True

    def test_estop_blocks_grasp(self, gripper):
        gripper.emergency_stop()
        assert grasp(gripper, "wrench_8mm") is False

    def test_custom_force(self, gripper):
        assert grasp(gripper, "screwdriver_phillips_small", grasp_force=15.0) is True


class TestRelease:
    def test_happy_path(self, gripper):
        gripper.close()
        assert release(gripper) is True
        assert gripper.get_position() == 0.0

    def test_release_already_open(self, gripper):
        assert release(gripper) is True


class TestPlaceAtStaging:
    def test_happy_path(self, arm, target_pose):
        assert place_at_staging(arm, "wrench_8mm", target_pose) is True

    def test_estop_blocks(self, arm, target_pose):
        arm.emergency_stop()
        assert place_at_staging(arm, "wrench_8mm", target_pose) is False


class TestPickFromStaging:
    def test_happy_path(self, arm, target_pose):
        assert pick_from_staging(arm, "wrench_8mm", target_pose) is True

    def test_estop_blocks(self, arm, target_pose):
        arm.emergency_stop()
        assert pick_from_staging(arm, "wrench_8mm", target_pose) is False


class TestReturnToSlot:
    def test_happy_path(self, arm, target_pose):
        assert return_to_slot(arm, "wrench_8mm", 0, 0, target_pose) is True

    def test_estop_blocks(self, arm, target_pose):
        arm.emergency_stop()
        assert return_to_slot(arm, "wrench_8mm", 0, 0, target_pose) is False
