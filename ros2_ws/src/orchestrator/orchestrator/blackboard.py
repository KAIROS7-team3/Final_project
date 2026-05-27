from dataclasses import dataclass, field
from typing import Optional


# Blackboard key 상수 — BT 노드 간 공유 상태 접근 시 이 상수를 사용한다.
KEY_ACTIVE_TOOL_ID = "active_tool_id"
KEY_TOOL_POSE = "tool_pose"
KEY_STAGING_STATE = "staging_state"
KEY_INTENT = "intent"
KEY_IS_MOVING = "is_moving"
KEY_FEASIBILITY_REASON = "feasibility_reason"

# staging_state 허용 값
STAGING_EMPTY = "empty"
STAGING_OCCUPIED = "occupied"


@dataclass
class BlackboardSchema:
    """py_trees Blackboard에 올라가는 공유 상태 전체 스키마.

    BT 노드는 이 클래스를 직접 인스턴스화하지 않는다.
    py_trees.blackboard.Client로 각 키를 개별 등록한다.
    여기서는 타입·기본값·허용 범위를 문서화하는 용도로만 사용.
    """

    active_tool_id: Optional[str] = None
    """현재 처리 중인 공구 ID. 작업 완료 시 None으로 초기화."""

    tool_pose: Optional[dict] = None
    """vision에서 받아온 공구 포즈.
    형식: {'position': [x, y, z], 'quaternion': [x, y, z, w], 'frame': str}
    단위: m, quaternion (E-1 준수)
    """

    staging_state: str = STAGING_EMPTY
    """Staging Area 상태. STAGING_EMPTY | STAGING_OCCUPIED"""

    intent: Optional[str] = None
    """voice 노드에서 파싱한 의도. 'fetch' | 'return'"""

    is_moving: bool = False
    """로봇 동작 중 여부 (S-7: True 동안 신규 명령 차단)."""

    feasibility_reason: str = ""
    """DB Gate 차단 사유. feasible=True 시 빈 문자열."""
