"""Manual test node that simulates motion completion from voice intents.

This is not a production control path. It exists so bring-up can verify:
/voice/raw_text -> /voice/intent -> DB update without moving hardware.
"""

from __future__ import annotations

import rclpy
from interfaces.msg import Intent
from interfaces.srv import CheckToolFeasibility, UpdateToolStatus
from rclpy.node import Node

from db.intent_status_mapping import simulated_status_for_intent


class IntentStatusSimulatorNode(Node):
    """Subscribe to /voice/intent and write a simulated completed motion result."""

    def __init__(self) -> None:
        super().__init__("intent_status_simulator_node")
        self.declare_parameter("track", "A")
        self._track = self.get_parameter("track").get_parameter_value().string_value
        # 테스트 노드지만 DB Gate를 다시 통과시켜 S-2 우회를 만들지 않는다.
        self._feasibility_client = self.create_client(
            CheckToolFeasibility,
            "check_tool_feasibility",
        )
        self._update_client = self.create_client(
            UpdateToolStatus,
            "update_tool_status",
        )
        self.create_subscription(Intent, "/voice/intent", self._handle_intent, 1)
        self.get_logger().warning(
            "manual simulator active: voice intents will update DB after DB gate"
        )

    def _handle_intent(self, message: Intent) -> None:
        """Convert fetch/return intents into simulated status updates."""

        update = simulated_status_for_intent(message.intent_type)
        tool_id = message.tool_id.strip()
        if update is None or not tool_id:
            self.get_logger().info(
                f"simulator ignored intent={message.intent_type} tool_id={message.tool_id}"
            )
            return

        if not self._feasibility_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("check_tool_feasibility service is not available")
            return

        request = CheckToolFeasibility.Request()
        request.intent = message.intent_type
        request.tool_id = tool_id
        future = self._feasibility_client.call_async(request)
        future.add_done_callback(
            lambda done: self._handle_feasibility(done, message, update.new_status)
        )

    def _handle_feasibility(
        self,
        future: rclpy.task.Future,
        message: Intent,
        new_status: str,
    ) -> None:
        """Call update_tool_status only after DB Gate approval."""

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"DB feasibility check failed: {exc}")
            return

        if not response.feasible:
            self.get_logger().warning(
                f"simulated DB update rejected by DB gate: {response.reason}"
            )
            return

        if not self._update_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("update_tool_status service is not available")
            return

        request = UpdateToolStatus.Request()
        request.tool_id = message.tool_id.strip()
        request.new_status = new_status
        request.event_type = message.intent_type
        request.track = self._track
        request.notes = (
            "manual voice intent simulation"
            if not message.raw_utterance
            else f"manual voice intent simulation: {message.raw_utterance}"
        )
        update_future = self._update_client.call_async(request)
        update_future.add_done_callback(self._handle_update_result)

    def _handle_update_result(self, future: rclpy.task.Future) -> None:
        """Log the final result of the simulated DB status write."""

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"DB status update failed: {exc}")
            return

        if response.success:
            self.get_logger().info(f"simulated DB update complete: {response.message}")
            return
        self.get_logger().error(f"simulated DB update rejected: {response.message}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IntentStatusSimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
