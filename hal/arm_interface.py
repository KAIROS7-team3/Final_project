from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class JointStates:
    positions: list[float]  # rad, len=6 for e0509
    velocities: list[float]  # rad/s
    efforts: list[float]  # Nm


@dataclass
class Pose:
    position: tuple[float, float, float]  # (x, y, z) in m, robot_base_link frame
    quaternion: tuple[float, float, float, float]  # (x, y, z, w)


class ArmInterface(ABC):
    @abstractmethod
    def move_to_pose(self, pose: Pose, velocity_scale: float = 0.3) -> bool:
        """Move end-effector to target pose. Returns True on success."""

    @abstractmethod
    def move_to_joint_positions(self, positions: list[float], velocity_scale: float = 0.3) -> bool:
        """Move to joint positions (rad). Returns True on success."""

    @abstractmethod
    def get_joint_states(self) -> JointStates:
        """Return current joint states."""

    @abstractmethod
    def get_end_effector_pose(self) -> Pose:
        """Return current end-effector pose in base_link frame."""

    @abstractmethod
    def emergency_stop(self) -> None:
        """Trigger emergency stop immediately. Must complete within 500ms (S-3)."""

    @abstractmethod
    def is_moving(self) -> bool:
        """Return True if robot is currently executing a motion."""
