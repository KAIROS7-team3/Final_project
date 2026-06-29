"""FOD 상태 전이를 관리하는 상시 관제 노드 (S-8).

전이 규칙:
  staged  → missing : /vision/detections/top_view 에서 staging_vision_timeout_s 연속 미감지
  missing → staged  : /vision/detections/top_view 에서 staging_vision_timeout_s 연속 감지 (역방향 복구)
  out     → missing : checkout_timeout_minutes 타임아웃 (탑뷰 감지 불가)
  missing → fod_alert: missing_to_alert_seconds 경과 (30분 유예)
"""

from __future__ import annotations

import time
from datetime import timedelta

import rclpy
from db_core.repository import ToolRepository
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


class FodMonitorNode(Node):
    """Polls DB and vision detections to manage FOD state transitions."""

    def __init__(self) -> None:
        super().__init__("fod_monitor_node")
        self.declare_parameter("db_path", "robot_arm.db")
        self.declare_parameter("operator_id", "operator_01")
        self.declare_parameter("checkout_timeout_minutes", 10.0)
        self.declare_parameter("missing_to_alert_seconds", 1800.0)
        self.declare_parameter("staging_vision_timeout_s", 60.0)
        self.declare_parameter("poll_interval_seconds", 5.0)
        self.declare_parameter("busy_timeout_ms", 5000)

        self._repository = ToolRepository(
            self.get_parameter("db_path").get_parameter_value().string_value,
            self.get_parameter("operator_id").get_parameter_value().string_value,
            busy_timeout_ms=self.get_parameter("busy_timeout_ms")
            .get_parameter_value()
            .integer_value,
        )

        # staged 공구 탑뷰 최근 감지 시각 (tool_id → monotonic)
        self._last_seen: dict[str, float] = {}
        # missing 공구 탑뷰 연속 감지 시작 시각 (tool_id → monotonic)
        # 감지가 끊기면 초기화, 60초 유지되면 staged 복구
        self._missing_first_seen: dict[str, float] = {}

        self._init_staged_last_seen()

        self.create_subscription(
            Detection2DArray,
            "/vision/detections/top_view",
            self._on_top_view_detections,
            10,
        )

        poll_interval = (
            self.get_parameter("poll_interval_seconds").get_parameter_value().double_value
        )
        self._poll_interval = poll_interval
        self.create_timer(poll_interval, self._poll)

    def _init_staged_last_seen(self) -> None:
        """기동 시 staged 공구에 현재 시각 부여 — 60초 유예 시작."""
        try:
            staged = self._repository.get_tools_by_status("staged")
            now = time.monotonic()
            for tool in staged:
                self._last_seen[tool["tool_id"]] = now
                self.get_logger().info(
                    f"[fod_monitor] init grace: tool_id={tool['tool_id']} staged"
                )
        except Exception as exc:
            self.get_logger().warning(f"[fod_monitor] init staged lookup failed: {exc}")

    def _on_top_view_detections(self, msg: Detection2DArray) -> None:
        """탑뷰 감지 수신 시 공구별 last_seen 갱신."""
        now = time.monotonic()
        for det in msg.detections:
            if not det.results:
                continue
            tool_id = det.results[0].hypothesis.class_id
            if tool_id:
                self._last_seen[tool_id] = now

    def _poll(self) -> None:
        """S-8 전이 규칙을 주기적으로 적용한다."""
        self._check_staged_vision()
        self._check_missing_recovery()
        self._check_timeouts()

    def _check_staged_vision(self) -> None:
        """staged 공구 중 탑뷰 미감지 60초 초과 → missing 전이."""
        vision_timeout = (
            self.get_parameter("staging_vision_timeout_s").get_parameter_value().double_value
        )
        now = time.monotonic()

        try:
            staged_tools = self._repository.get_tools_by_status("staged")
        except Exception as exc:
            self.get_logger().error(f"[fod_monitor] staged query failed: {exc}")
            return

        for tool in staged_tools:
            tool_id = tool["tool_id"]
            last_seen = self._last_seen.get(tool_id)

            if last_seen is None:
                self._last_seen[tool_id] = now
                continue

            elapsed = now - last_seen
            if elapsed < vision_timeout:
                continue

            try:
                update = self._repository.mark_staged_vision_missing(tool_id)
            except Exception as exc:
                self.get_logger().error(
                    f"[fod_monitor] mark_staged_vision_missing failed: tool_id={tool_id} {exc}"
                )
                continue

            if update is not None:
                self.get_logger().warning(
                    f"[fod_monitor] FOD transition tool_id={tool_id}: "
                    f"staged -> missing (미감지 {elapsed:.0f}s)"
                )
                self._last_seen.pop(tool_id, None)

    def _check_missing_recovery(self) -> None:
        """missing 공구 중 탑뷰 연속 감지 60초 초과 → staged 복구 (역방향)."""
        vision_timeout = (
            self.get_parameter("staging_vision_timeout_s").get_parameter_value().double_value
        )
        # 감지가 끊겼다고 볼 기준: 마지막 감지로부터 poll_interval * 2 초과
        detection_stale_s = self._poll_interval * 2
        now = time.monotonic()

        try:
            missing_tools = self._repository.get_tools_by_status("missing")
        except Exception as exc:
            self.get_logger().error(f"[fod_monitor] missing query failed: {exc}")
            return

        missing_ids = {tool["tool_id"] for tool in missing_tools}

        # missing 상태 아닌데 _missing_first_seen에 남아있으면 정리
        for tool_id in list(self._missing_first_seen.keys()):
            if tool_id not in missing_ids:
                self._missing_first_seen.pop(tool_id, None)

        for tool_id in missing_ids:
            last_seen = self._last_seen.get(tool_id)

            if last_seen is None or now - last_seen > detection_stale_s:
                # 감지 안 됨 또는 끊김 → 연속 감지 윈도우 초기화
                self._missing_first_seen.pop(tool_id, None)
                continue

            # 감지 중 — 연속 감지 시작 시각 기록
            if tool_id not in self._missing_first_seen:
                self._missing_first_seen[tool_id] = now
                continue

            elapsed = now - self._missing_first_seen[tool_id]
            if elapsed < vision_timeout:
                continue

            # 60초 연속 감지 → staged 복구
            try:
                update = self._repository.mark_missing_staged_recovery(tool_id)
            except Exception as exc:
                self.get_logger().error(
                    f"[fod_monitor] mark_missing_staged_recovery failed: tool_id={tool_id} {exc}"
                )
                continue

            if update is not None:
                self.get_logger().info(
                    f"[fod_monitor] FOD recovery tool_id={tool_id}: "
                    f"missing -> staged (연속 감지 {elapsed:.0f}s)"
                )
                self._missing_first_seen.pop(tool_id, None)
                # staged 복구 시 last_seen 유지 — 즉시 재소실 방지를 위해 now로 갱신
                self._last_seen[tool_id] = now

    def _check_timeouts(self) -> None:
        """out → missing (타임아웃), missing → fod_alert (30분 유예) 전이."""
        try:
            updates = self._repository.mark_checkout_timeouts(
                checkout_timeout=timedelta(
                    minutes=self.get_parameter("checkout_timeout_minutes")
                    .get_parameter_value()
                    .double_value
                ),
                alert_grace=timedelta(
                    seconds=self.get_parameter("missing_to_alert_seconds")
                    .get_parameter_value()
                    .double_value
                ),
            )
        except Exception as exc:
            self.get_logger().error(f"[fod_monitor] timeout poll failed: {exc}")
            return

        for update in updates:
            msg = (
                f"FOD transition tool_id={update.tool_id}: "
                f"{update.previous_status} -> {update.new_status}"
            )
            if update.new_status == "fod_alert":
                self.get_logger().error(f"{msg} [FOD ALERT — logged to system_events]")
            else:
                self.get_logger().warning(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FodMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
