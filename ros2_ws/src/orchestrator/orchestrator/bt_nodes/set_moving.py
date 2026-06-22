"""SetMoving BT 리프 — /robot/status is_moving 발행 + PLC 상태 갱신."""
from __future__ import annotations

from typing import Callable, Optional

import py_trees


class SetMoving(py_trees.behaviour.Behaviour):
    """is_moving을 publish_fn으로 발행하고 선택적으로 PLC 상태를 갱신한다.

    항상 SUCCESS 반환. 발행 예외 시 FAILURE.

    Args:
        name: BT 노드 이름.
        publish_fn: is_moving(bool) → None.
        is_moving: 발행할 값.
        set_plc_fn: PLC 상태 갱신 함수 (선택). plc_state를 함께 지정해야 동작.
        plc_state: set_plc_fn에 전달할 상태 문자열 (예: "moving", "idle").
    """

    def __init__(
        self,
        name: str,
        publish_fn: Callable[[bool], None],
        is_moving: bool,
        set_plc_fn: Optional[Callable[[str], None]] = None,
        plc_state: Optional[str] = None,
    ) -> None:
        super().__init__(name=name)
        self._publish_fn = publish_fn
        self._is_moving = is_moving
        self._set_plc_fn = set_plc_fn
        self._plc_state = plc_state

    def update(self) -> py_trees.common.Status:
        try:
            self._publish_fn(self._is_moving)
        except Exception as exc:
            self.logger.error(f"[{self.name}] is_moving 발행 실패: {exc}")
            return py_trees.common.Status.FAILURE
        if self._set_plc_fn and self._plc_state:
            try:
                self._set_plc_fn(self._plc_state)
            except Exception as exc:
                self.logger.warning(f"[{self.name}] PLC 상태 갱신 실패: {exc}")
        return py_trees.common.Status.SUCCESS
