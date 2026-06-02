from __future__ import annotations

from db.plc_state_publisher import PLC_STATE_ERROR, plc_state_for_fod_transition


def test_missing_transition_maps_to_plc_error() -> None:
    assert plc_state_for_fod_transition("missing") == PLC_STATE_ERROR


def test_fod_alert_transition_maps_to_plc_error() -> None:
    assert plc_state_for_fod_transition("fod_alert") == PLC_STATE_ERROR


def test_non_fod_transition_does_not_publish_plc_state() -> None:
    assert plc_state_for_fod_transition("in_slot") is None
