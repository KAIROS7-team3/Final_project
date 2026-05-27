from abc import ABC, abstractmethod


class GripperInterface(ABC):
    @abstractmethod
    def set_position(self, position: float, force: float = 20.0) -> bool:
        """Set gripper position (0.0=open, 1.0=closed) with force limit in N. Returns True on success."""

    @abstractmethod
    def get_position(self) -> float:
        """Return current position (0.0=open, 1.0=closed)."""

    @abstractmethod
    def is_grasping(self) -> bool:
        """Return True if gripper detects a grasped object (force feedback)."""

    @abstractmethod
    def open(self) -> bool:
        """Fully open gripper. Returns True on success."""

    @abstractmethod
    def close(self, force: float = 20.0) -> bool:
        """Close gripper with force limit in N. Returns True on success."""

    @abstractmethod
    def emergency_stop(self) -> None:
        """Immediately halt gripper motion."""
