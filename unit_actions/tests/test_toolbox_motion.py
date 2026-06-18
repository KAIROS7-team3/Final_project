"""toolbox_motion.py — Step.marker / marked() 테스트.

orchestrator는 action feedback의 phase="pick"/"place"로 DB 상태를
물리적 집기/놓기 시점에 전이시킨다 (in_slot<->out<->staged). 이 마커가
fetch/return 시퀀스에 정확히 1쌍씩, GRIP step에만 붙어 있는지 검증한다.
"""

import pytest

from unit_actions.toolbox_motion import (
    LAYER0_V2_APPROACH,
    LAYER1_V2_APPROACH,
    PULSE_GRIP_BOX,
    PULSE_RELEASE,
    Step,
    StepKind,
    drawer_close_seq_v2,
    drawer_open_seq_v2,
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


# ── drawer v2 시퀀스 (feat/motion-drawer-v2, PR #54) ──────────────────────
# v1 대비 x=369.0mm 보정 + open/silence/opendown y 조정. 구조는 9스텝 동일.

# open/close 공통 step kind 골격 — GRIP으로 시작·끝나는 MoveJ
_V2_KINDS = [
    StepKind.GRIP,
    StepKind.MOVE_J_ABS,
    StepKind.MOVE_L_ABS,
    StepKind.GRIP,
    StepKind.MOVE_L_ABS,
    StepKind.MOVE_L_ABS,
    StepKind.GRIP,
    StepKind.MOVE_L_ABS,
    StepKind.MOVE_J_ABS,
]


@pytest.mark.parametrize("layer", [0, 1])
def test_drawer_open_seq_v2_structure(layer: int) -> None:
    """happy path: open v2는 9스텝, 부분개방으로 시작·홈복귀로 종료, ④ 손잡이 GRIP_BOX."""
    seq = drawer_open_seq_v2(layer)
    assert [s.kind for s in seq] == _V2_KINDS
    assert seq[0].pulse == PULSE_RELEASE          # ① 파지 준비 개방
    assert seq[3].pulse == PULSE_GRIP_BOX         # ④ 손잡이 파지
    assert seq[6].pulse == PULSE_RELEASE          # ⑦ 손잡이 해제
    assert seq[-1].pose == [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # ⑨ JOINT_HOME


@pytest.mark.parametrize("layer", [0, 1])
def test_drawer_close_seq_v2_structure(layer: int) -> None:
    """happy path: close v2도 9스텝, ④ 손잡이 GRIP_BOX, 홈복귀로 종료."""
    seq = drawer_close_seq_v2(layer)
    assert [s.kind for s in seq] == _V2_KINDS
    assert seq[0].pulse == PULSE_RELEASE
    assert seq[3].pulse == PULSE_GRIP_BOX
    assert seq[-1].pose == [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


def test_drawer_open_seq_v2_uses_v2_waypoints() -> None:
    """v2 시퀀스가 v2 웨이포인트(x=369.0 보정값)를 사용하는지 검증."""
    # approach step(index 2)이 layer별 v2 상수와 일치
    assert drawer_open_seq_v2(0)[2].pose == LAYER0_V2_APPROACH
    assert drawer_open_seq_v2(1)[2].pose == LAYER1_V2_APPROACH
    # v2 x 보정값 확인 (v1 layer0 = 378.88 → v2 = 369.0)
    assert LAYER0_V2_APPROACH[0] == 369.0
    assert LAYER1_V2_APPROACH[0] == 369.0


@pytest.mark.parametrize("seq_fn", [drawer_open_seq_v2, drawer_close_seq_v2])
@pytest.mark.parametrize("bad_layer", [-1, 2, 99])
def test_drawer_seq_v2_rejects_invalid_layer(seq_fn, bad_layer: int) -> None:
    """failure path: layer는 0/1만 지원 — 그 외는 ValueError."""
    with pytest.raises(ValueError):
        seq_fn(bad_layer)
