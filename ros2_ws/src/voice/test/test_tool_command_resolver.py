from voice.tool_intent import resolve_tool_command


def test_corrected_tool_id_reaches_fetch_command_when_confident() -> None:
    parsed = resolve_tool_command("복스 소켓 줘")

    assert parsed.intent_type == "fetch"
    assert parsed.tool_id == "socket_19mm"
    assert parsed.need_confirm is False


def test_uncertain_tool_intent_is_blocked_by_default() -> None:
    parsed = resolve_tool_command("스캐너 줘")

    assert parsed.intent_type == "unknown"
    assert parsed.tool_id == "spanner_16mm"
    assert parsed.need_confirm is True


def test_uncertain_tool_intent_can_be_allowed_for_db_bench_test() -> None:
    parsed = resolve_tool_command(
        "스캐너 줘",
        allow_uncertain_tool_intent=True,
    )

    assert parsed.intent_type == "fetch"
    assert parsed.tool_id == "spanner_16mm"
    assert parsed.need_confirm is True


def test_generic_tool_request_does_not_reach_db_gate() -> None:
    parsed = resolve_tool_command("공구 가져와")

    assert parsed.intent_type == "unknown"
    assert parsed.tool_id == ""
    assert parsed.need_confirm is True


def test_return_uses_corrected_tool_id() -> None:
    parsed = resolve_tool_command("박스알 19미리 반납")

    assert parsed.intent_type == "return"
    assert parsed.tool_id == "socket_19mm"
