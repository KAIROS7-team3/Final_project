"""unit_action_server — unit_actions/ 순수 Python 모듈을 ROS2 action server로 래핑한다.

Track A/B 전용. Track C는 unit_actions를 사용하지 않는다 (CLAUDE.md).
rclpy 의존성은 이 파일에만 있고 unit_actions/ 자체에는 없다 (E-2).
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from interfaces.action import (
    Grasp,
    MoveToPose,
    PickFromStaging,
    PlaceAtStaging,
    Release,
    ReturnToSlot,
)


class UnitActionServer(Node):
    """6개 unit_action을 각각 ROS2 action server로 노출한다."""

    def __init__(self) -> None:
        super().__init__("unit_action_server")

        # TODO(Phase 4–5a): HAL 구현체 주입 (현재는 스텁)
        self._arm = None
        self._gripper = None
        self._camera = None

        self._servers = [
            ActionServer(self, Grasp, "grasp", self._execute_grasp),
            ActionServer(self, MoveToPose, "move_to_pose", self._execute_move_to_pose),
            ActionServer(self, PlaceAtStaging, "place_at_staging", self._execute_place_at_staging),
            ActionServer(
                self, PickFromStaging, "pick_from_staging", self._execute_pick_from_staging
            ),
            ActionServer(self, Release, "release", self._execute_release),
            ActionServer(self, ReturnToSlot, "return_to_slot", self._execute_return_to_slot),
        ]
        self.get_logger().info("[UnitActionServer] ready — 6 action servers registered")

    def _execute_grasp(self, goal_handle):
        # TODO(Phase 5a): unit_actions.grasp() 호출
        self.get_logger().warn("[UnitActionServer] grasp not yet implemented")
        goal_handle.abort()
        return Grasp.Result(success=False, message="not implemented")

    def _execute_move_to_pose(self, goal_handle):
        # TODO(Phase 5a): unit_actions.move_to_pose() 호출
        self.get_logger().warn("[UnitActionServer] move_to_pose not yet implemented")
        goal_handle.abort()
        return MoveToPose.Result(success=False, message="not implemented")

    def _execute_place_at_staging(self, goal_handle):
        # TODO(Phase 4): unit_actions.place_at_staging() 호출
        self.get_logger().warn("[UnitActionServer] place_at_staging not yet implemented")
        goal_handle.abort()
        return PlaceAtStaging.Result(success=False, message="not implemented")

    def _execute_pick_from_staging(self, goal_handle):
        # TODO(Phase 4): unit_actions.pick_from_staging() 호출
        self.get_logger().warn("[UnitActionServer] pick_from_staging not yet implemented")
        goal_handle.abort()
        return PickFromStaging.Result(success=False, message="not implemented")

    def _execute_release(self, goal_handle):
        # TODO(Phase 5a): unit_actions.release() 호출
        self.get_logger().warn("[UnitActionServer] release not yet implemented")
        goal_handle.abort()
        return Release.Result(success=False, message="not implemented")

    def _execute_return_to_slot(self, goal_handle):
        # TODO(Phase 5a): unit_actions.return_to_slot() 호출
        self.get_logger().warn("[UnitActionServer] return_to_slot not yet implemented")
        goal_handle.abort()
        return ReturnToSlot.Result(success=False, message="not implemented")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitActionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
