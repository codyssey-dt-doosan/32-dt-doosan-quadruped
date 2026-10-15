"""작은 원판 판독 한계. 점검 위치에서 원판 지름은 약 26~34 px (설계 4-2절)."""

import math

import pytest

from gauge_ocr.reader import read_gauge
from test_reader import phi_of, SPEC, synth_gauge


@pytest.mark.parametrize("diam", [16, 20, 24, 32])
@pytest.mark.parametrize("value", [0.5, 2.5, 5.0, 7.5, 9.5])
@pytest.mark.parametrize("squash", [1.0, 0.9])
def test_small_dial_reads(diam, value, squash):
    v, _ = read_gauge(synth_gauge(phi_of(value), squash=squash, r=diam // 2), SPEC)
    assert abs(v - value) < 0.2


@pytest.mark.parametrize("value", [0.5, 5.0, 9.5])
def test_dial_12px_is_not_read(value):
    v, _ = read_gauge(synth_gauge(phi_of(value), r=6), SPEC)
    assert math.isnan(v)
