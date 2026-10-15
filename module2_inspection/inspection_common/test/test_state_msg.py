"""상태 문자열 만들기·읽기."""

import pytest

from inspection_common.state_msg import format_state, parse_state


def test_format_joins_with_separator():
    assert format_state("READING", "gauge_line1", "4.23 bar") == "READING | gauge_line1 | 4.23 bar"


def test_state_only():
    assert format_state("IDLE") == "IDLE"


def test_roundtrip():
    text = format_state("PENDING", "hotspot_1 (7.65, -6.00)", "85.2 C", "goal (6.1, -6.0)")
    assert parse_state(text) == ("PENDING", ["hotspot_1 (7.65, -6.00)", "85.2 C", "goal (6.1, -6.0)"])


def test_parse_strips_spaces():
    assert parse_state("NO_GAUGE |unknown ") == ("NO_GAUGE", ["unknown"])


@pytest.mark.parametrize("bad", ["reading", "", "NO GAUGE", "A|B"])
def test_state_must_be_upper_single_word(bad):
    with pytest.raises(ValueError):
        format_state(bad)


def test_field_cannot_contain_bar():
    with pytest.raises(ValueError):
        format_state("READING", "a|b")
