from voice.wake_word import apply_wake_word_gate


def test_wake_word_gate_passes_through_when_disabled() -> None:
    result = apply_wake_word_gate("스패너 가져와", ["로봇"], False)

    assert result.accepted is True
    assert result.command_text == "스패너 가져와"


def test_wake_word_gate_accepts_and_strips_prefix() -> None:
    result = apply_wake_word_gate("로봇 스패너 가져와", ["로봇"], True)

    assert result.accepted is True
    assert result.command_text == "스패너 가져와"


def test_wake_word_gate_accepts_kobot_variants() -> None:
    result = apply_wake_word_gate("코버 스패너 가져와", ["코봇", "코버"], True)

    assert result.accepted is True
    assert result.command_text == "스패너 가져와"


def test_wake_word_gate_accepts_common_short_variants() -> None:
    result = apply_wake_word_gate("코부츠 스패너 반납", ["코부"], True)

    assert result.accepted is True
    assert result.command_text == "스패너 반납"


def test_wake_word_gate_rejects_missing_prefix() -> None:
    result = apply_wake_word_gate("스패너 가져와", ["로봇"], True)

    assert result.accepted is False
    assert result.command_text == ""
