from voice.command_parser import parse_command


def test_parse_all_configured_fetch_aliases() -> None:
    examples = {
        "screwdriver": "십자 드라이버 가져다 줘",
        "utility_knife": "커터칼 가져다 줘",
        "ratchet_wrench": "라쳇 렌치 가져다 줘",
        "multi_tool": "멕가이버 가져다 줘",
        "spanner_16mm": "스패너 가져다 줘",
        "socket_19mm": "복스 소켓 가져다 줘",
    }

    for tool_id, utterance in examples.items():
        parsed = parse_command(utterance)
        assert parsed.intent_type == "fetch"
        assert parsed.tool_id == tool_id


def test_parse_fetch_korean_tool_command() -> None:
    parsed = parse_command("스패너 가져다 줘")

    assert parsed.intent_type == "fetch"
    assert parsed.tool_id == "spanner_16mm"
    assert parsed.confidence > 0.0


def test_parse_return_korean_tool_command() -> None:
    parsed = parse_command("복스 소켓 반납")

    assert parsed.intent_type == "return"
    assert parsed.tool_id == "socket_19mm"


def test_parse_cancel_command() -> None:
    parsed = parse_command("작업 취소")

    assert parsed.intent_type == "cancel"
    assert parsed.tool_id == ""


def test_parse_unknown_command() -> None:
    parsed = parse_command("오늘 날씨 알려줘")

    assert parsed.intent_type == "unknown"
    assert parsed.tool_id == ""
    assert parsed.confidence == 0.0
