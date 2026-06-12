"""demo_trigger 인자 파싱 (ROS 비의존, 단위 테스트 가능).

demo_trigger.py는 rclpy/interfaces를 import하므로 ROS 환경 없이는 import할 수
없다. 순수 파싱 로직만 이 모듈로 분리해 plain pytest로 검증한다.
"""

from __future__ import annotations

from dataclasses import dataclass

VALID_INTENTS = ("fetch", "return", "home")


@dataclass(frozen=True)
class TriggerArgs:
    """파싱된 트리거 인자."""

    intent_type: str
    tool_id: str  # 빈 문자열이면 demo_runner가 config 기본값 사용


def parse_args(argv: list[str], default_tool_id: str = "") -> TriggerArgs:
    """`[intent_type, tool_id?]` 위치 인자를 파싱한다.

    Raises:
        ValueError: intent_type 누락 또는 fetch/return 외 값.
    """
    if not argv:
        raise ValueError(
            f"intent_type이 필요합니다 ({'|'.join(VALID_INTENTS)}). "
            "사용: demo_trigger <fetch|return> [tool_id]"
        )
    intent_type = argv[0].strip().lower()
    if intent_type not in VALID_INTENTS:
        raise ValueError(
            f"지원하지 않는 intent_type: {argv[0]!r} "
            f"(허용: {'|'.join(VALID_INTENTS)})"
        )
    tool_id = argv[1].strip() if len(argv) > 1 and argv[1].strip() else default_tool_id
    return TriggerArgs(intent_type=intent_type, tool_id=tool_id)


def strip_ros_args(argv: list[str]) -> list[str]:
    """`--ros-args ...` 이후 인자를 제거해 위치 인자만 남긴다."""
    if "--ros-args" in argv:
        return argv[: argv.index("--ros-args")]
    return argv
