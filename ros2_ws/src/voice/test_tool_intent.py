from voice.tool_intent import (
    build_gemma_prompt,
    normalize_text,
    parse_tool_intent,
)


def test_spanner_false_friend_korean() -> None:
    parsed = parse_tool_intent("스페인어 가져와")

    assert parsed["tool_id"] == "spanner_16mm"


def test_spanner_false_friend_english() -> None:
    parsed = parse_tool_intent("Spanish 가져와")

    assert parsed["tool_id"] == "spanner_16mm"


def test_scanner_is_low_certainty_spanner() -> None:
    parsed = parse_tool_intent("스캐너 줘")

    assert parsed["tool_id"] == "spanner_16mm"
    assert parsed["need_confirm"] is True or parsed["confidence"] <= 0.65


def test_box_al_19mm_maps_to_socket() -> None:
    parsed = parse_tool_intent("박스알 19미리 가져와")

    assert parsed["tool_id"] == "socket_19mm"


def test_boks_socket_maps_to_socket() -> None:
    parsed = parse_tool_intent("복스 소켓 줘")

    assert parsed["tool_id"] == "socket_19mm"


def test_rocket_launch_maps_to_ratchet() -> None:
    parsed = parse_tool_intent("로켓 런치 가져와")

    assert parsed["tool_id"] == "ratchet_wrench"


def test_generic_wrench_needs_confirmation() -> None:
    parsed = parse_tool_intent("렌치 가져와")

    assert parsed["need_confirm"] is True


def test_power_strip_is_low_certainty_multitool() -> None:
    parsed = parse_tool_intent("멀티탭 가져와")

    assert parsed["tool_id"] == "multi_tool"
    assert parsed["need_confirm"] is True or parsed["confidence"] <= 0.65


def test_utility_life_maps_to_utility_knife() -> None:
    parsed = parse_tool_intent("유틸리티 라이프 줘")

    assert parsed["tool_id"] == "utility_knife"


def test_device_driver_is_low_certainty_screwdriver() -> None:
    parsed = parse_tool_intent("장치 드라이버 가져와")

    assert parsed["tool_id"] == "screwdriver"
    assert parsed["need_confirm"] is True or parsed["confidence"] <= 0.65


def test_socket_16mm_conflict_needs_confirmation() -> None:
    parsed = parse_tool_intent("소켓 16미리 가져와")

    assert parsed["need_confirm"] is True


def test_spanner_19mm_conflict_needs_confirmation() -> None:
    parsed = parse_tool_intent("스패너 19미리 가져와")

    assert parsed["need_confirm"] is True


def test_cutter_maps_to_utility_knife() -> None:
    parsed = parse_tool_intent("커터 가져와")

    assert parsed["tool_id"] == "utility_knife"


def test_driver_maps_to_screwdriver() -> None:
    parsed = parse_tool_intent("드라이버 줘")

    assert parsed["tool_id"] == "screwdriver"


def test_generic_tool_request_is_unknown_or_confirmed() -> None:
    parsed = parse_tool_intent("공구 가져와")

    assert parsed["tool_id"] == "unknown" or parsed["need_confirm"] is True


def test_normalize_text_is_case_and_space_insensitive() -> None:
    assert normalize_text("  Spanish   LANGUAGE ") == "spanish language"


def test_build_gemma_prompt_requires_json_only() -> None:
    prompt = build_gemma_prompt("스페인어 가져와")

    assert "반드시 JSON만 출력한다" in prompt
    assert "스페인어 가져와" in prompt


if __name__ == "__main__":
    tests = [
        test_spanner_false_friend_korean,
        test_spanner_false_friend_english,
        test_scanner_is_low_certainty_spanner,
        test_box_al_19mm_maps_to_socket,
        test_boks_socket_maps_to_socket,
        test_rocket_launch_maps_to_ratchet,
        test_generic_wrench_needs_confirmation,
        test_power_strip_is_low_certainty_multitool,
        test_utility_life_maps_to_utility_knife,
        test_device_driver_is_low_certainty_screwdriver,
        test_socket_16mm_conflict_needs_confirmation,
        test_spanner_19mm_conflict_needs_confirmation,
        test_cutter_maps_to_utility_knife,
        test_driver_maps_to_screwdriver,
        test_generic_tool_request_is_unknown_or_confirmed,
        test_normalize_text_is_case_and_space_insensitive,
        test_build_gemma_prompt_requires_json_only,
    ]
    for test in tests:
        test()
    print(f"{len(tests)} tool_intent tests passed.")
