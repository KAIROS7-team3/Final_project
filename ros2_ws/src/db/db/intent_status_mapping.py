"""테스트 전용 voice intent -> DB 상태 전이 매핑."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SimulatedStatusUpdate:
    """시뮬레이터가 /db/UpdateToolStatus에 넘길 상태 전이."""

    new_status: str
    event_type: str


def simulated_status_for_intent(intent_type: str) -> SimulatedStatusUpdate | None:
    """수동 voice-to-DB simulator가 사용할 임시 상태 전이를 반환한다.

    이 매핑은 실제 motion 결과가 아니다. 하드웨어를 움직이지 않고
    `/voice/intent -> /db/UpdateToolStatus` 흐름을 확인하기 위한 테스트 전용 값이다.
    """

    # 실제 운영에서는 motion 완료 콜백이 이 결정을 해야 한다.
    if intent_type == "fetch":
        return SimulatedStatusUpdate(new_status="out", event_type="fetch")
    if intent_type == "return":
        return SimulatedStatusUpdate(new_status="in_slot", event_type="return")
    return None
