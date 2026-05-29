from __future__ import annotations

from db.intent_status_mapping import simulated_status_for_intent


def test_fetch_intent_maps_to_out_status() -> None:
    update = simulated_status_for_intent("fetch")

    assert update is not None
    assert update.new_status == "out"
    assert update.event_type == "fetch"


def test_return_intent_maps_to_in_slot_status() -> None:
    update = simulated_status_for_intent("return")

    assert update is not None
    assert update.new_status == "in_slot"
    assert update.event_type == "return"


def test_unknown_intent_has_no_simulated_update() -> None:
    assert simulated_status_for_intent("unknown") is None
