"""eval_slope: 관문 문턱(ω·내리막 피치·평지), 리셋 분류(타일 끝 이탈 vs 전도), yaw 감기. Genesis 초기화 없음."""
import math
import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from eval_slope import classify_reset, gate, wrap_pi  # noqa: E402


def test_gate_thresholds():
    ok = {(20, "slope_down"): {"w": [0.003, -0.002], "pitch_err": [-2.0, -2.5]}, (20, "flat"): {"w": [0.001], "pitch_err": [0.0]}}
    assert gate(ok)[0]
    bad = {(20, "slope_down"): {"w": [0.07, 0.08], "pitch_err": [-6.5, -7.0]}}  # model_2800 수준
    passed, why = gate(bad)
    assert not passed and any("ω" in w for w in why) and any("피치" in w for w in why)


def test_classify_reset():
    fell, exited = classify_reset(torch.tensor([True, True, False]), torch.tensor([1.0, 3.3, 3.3]), 3.325)
    assert fell.tolist() == [True, False, False] and exited.tolist() == [False, True, False]


def test_wrap_pi():
    assert wrap_pi(torch.tensor([3.5 * math.pi])).item() == pytest.approx(-0.5 * math.pi)
