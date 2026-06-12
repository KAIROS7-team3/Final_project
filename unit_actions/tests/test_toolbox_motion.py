"""toolbox_motion.py — Step.marker / marked() 테스트.

orchestrator는 action feedback의 phase="pick"/"place"로 DB 상태를
물리적 집기/놓기 시점에 전이시킨다 (in_slot<->out<->staged). 이 마커가
fetch/return 시퀀스에 정확히 1쌍씩, GRIP step에만 붙어 있는지 검증한다.
"""

import pytest

from unit_actions.toolbox_motion import (
    Step,
    StepKind,
    full_socket_fetch_seq,
    full_socket_return_seq,
    grip,
    marked,
)


def test_marked_sets_marker_on_step() -> None:
    step = marked(grip(650), "pick")
    assert step.marker == "pick"
    assert step.kind == StepKind.GRIP
    assert step.pulse == 650


def test_marked_rejects_non_step() -> None:
    with pytest.raises(AttributeError):
        marked("not a step", "pick")  # type: ignore[arg-type]


def test_marked_rejects_invalid_marker() -> None:
    with pytest.raises(ValueError):
        marked(grip(650), "Pick")  # type: ignore[arg-type]


def test_step_kind_names_do_not_collide_with_markers() -> None:
    marker_values = {"pick", "place"}
    step_kind_names = {kind.name.lower() for kind in StepKind}
    assert marker_values.isdisjoint(step_kind_names)


def _markers(seq: list[Step]) -> list[tuple[int, str]]:
    return [(i, s.marker) for i, s in enumerate(seq) if s.marker is not None]


def test_full_socket_fetch_seq_has_one_pick_and_place() -> None:
    seq = full_socket_fetch_seq()
    marks = _markers(seq)
    assert [m for _, m in marks] == ["pick", "place"]
    pick_idx, place_idx = marks[0][0], marks[1][0]
    assert seq[pick_idx].kind == StepKind.GRIP
    assert seq[place_idx].kind == StepKind.GRIP
    assert pick_idx < place_idx


def test_full_socket_return_seq_has_one_pick_and_place() -> None:
    seq = full_socket_return_seq()
    marks = _markers(seq)
    assert [m for _, m in marks] == ["pick", "place"]
    pick_idx, place_idx = marks[0][0], marks[1][0]
    assert seq[pick_idx].kind == StepKind.GRIP
    assert seq[place_idx].kind == StepKind.GRIP
    assert pick_idx < place_idx
