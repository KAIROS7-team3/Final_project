"""demo_trigger — 마이크 없이 `/voice/intent`를 1회 publish하는 데모 트리거.

음성 스택(whisper + rule_intent) 없이 orchestrator BT를 시연/테스트할 때 사용한다.
orchestrator가 `/db/CheckToolFeasibility` 서비스(S-2 DB Gate)를 수행하므로
이 경로로 intent를 주입해도 안전 게이트는 우회되지 않는다.

사용:
    ros2 run demo demo_trigger fetch
    ros2 run demo demo_trigger fetch socket_19mm
    ros2 run demo demo_trigger return socket_19mm
"""

from __future__ import annotations

import sys
from typing import Any

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from interfaces.msg import Intent

from demo._paths import find_repo_file
from demo._trigger_args import parse_args, strip_ros_args

_QOS_INTENT = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)


def _default_tool_id() -> str:
    """config/demo.yaml의 tool_id (못 찾으면 빈 문자열)."""
    try:
        path = find_repo_file("config/demo.yaml")
    except (RuntimeError, FileNotFoundError):
        return ""
    with open(path) as f:
        cfg: dict[str, Any] = yaml.safe_load(f) or {}
    return str(cfg.get("demo", {}).get("tool_id", ""))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    cli_argv = strip_ros_args(list(sys.argv[1:]))

    try:
        parsed = parse_args(cli_argv, default_tool_id=_default_tool_id())
    except ValueError as exc:
        print(f"[demo_trigger] {exc}", file=sys.stderr)
        rclpy.shutdown()
        sys.exit(2)

    node = Node("demo_trigger")
    pub = node.create_publisher(Intent, "/voice/intent", _QOS_INTENT)

    msg = Intent()
    msg.intent_type = parsed.intent_type
    msg.tool_id = parsed.tool_id
    msg.confidence = 1.0
    msg.raw_utterance = f"[demo_trigger] {parsed.intent_type} {parsed.tool_id}".strip()
    msg.timestamp = node.get_clock().now().to_msg()

    # RELIABLE 구독자가 연결될 시간을 준 뒤 publish (1회성 노드).
    deadline = node.get_clock().now().nanoseconds + 2_000_000_000
    while rclpy.ok() and pub.get_subscription_count() == 0:
        if node.get_clock().now().nanoseconds > deadline:
            node.get_logger().warn(
                "[demo_trigger] /voice/intent 구독자 없음 — 그래도 publish 시도"
            )
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    pub.publish(msg)
    node.get_logger().info(
        f"[demo_trigger] published: intent_type={parsed.intent_type} "
        f"tool_id={parsed.tool_id or '(config 기본값)'}"
    )
    # publish 플러시 여유.
    end = node.get_clock().now().nanoseconds + 500_000_000
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
