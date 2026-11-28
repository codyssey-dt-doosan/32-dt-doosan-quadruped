"""eval_slope: 관문 문턱(ω·내리막 피치·평지), 리셋 분류(타일 끝 이탈 vs 전도), yaw 감기. Genesis 초기화 없음."""
import math
import sys
from pathlib import Path

import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import eval_slope  # noqa: E402
from eval_slope import TYPES, YAWS, classify_reset, gate, wrap_pi  # noqa: E402


def test_gate_thresholds():
    ok = {(20, "slope_down"): {"w": [0.003, -0.002], "pitch_err": [-2.0, -2.5]}, (20, "flat"): {"w": [0.001], "pitch_err": [0.0]}}
    assert gate(ok)[0]
    bad = {(20, "slope_down"): {"w": [0.07, 0.08], "pitch_err": [-6.5, -7.0]}}  # model_2800 수준
    passed, why = gate(bad)
    assert not passed and any("ω" in w for w in why) and any("피치" in w for w in why)


def test_gate_rules():  # 판정 R4: 오르막 피치는 판정 안 함, 평균만 넘어도 FAIL, 평지는 0.004
    assert gate({(20, "slope_up"): {"w": [0.007, -0.007], "pitch_err": [9.0, 9.0]}})[0]  # mean 0·max 0.007 ≤ 0.008
    assert not gate({(20, "slope_down"): {"w": [0.005, 0.005], "pitch_err": [0.0, 0.0]}})[0]  # max 0.005는 통과, mean만 초과
    assert not gate({(20, "flat"): {"w": [0.005]}})[0]


def test_gate_fails_closed():  # 경사 띠에 못 들어간 run(NaN)이나 빈 기록을 빼고 나머지로 PASS하지 않는다
    for w in ([], [0.001, math.nan], [math.nan]):
        for kind in ("flat", "slope_up", "slope_down"):
            passed, why = gate({(20, kind): {"w": w, "pitch_err": [0.0] * len(w)}})
            assert not passed and why, (kind, w)


def test_main_keeps_run_outside_band(monkeypatch, capsys):  # main이 띠에 못 들어간 run을 빼고 나머지 7개로 PASS하지 않는다
    n = len(YAWS) * len(TYPES)

    def fake(run_dir, ckpt, deg, miss):
        t_on = torch.full((n,), 10.0)
        t_on[miss] = 0.0
        return {"type": torch.arange(n) // len(YAWS), "t_on": t_on, "dpsi": torch.zeros(n), "dy": torch.zeros(n),
                "pitch": torch.where(t_on > 0, math.radians(deg), math.nan), "fell": torch.zeros(n, dtype=torch.bool),
                "exited": torch.zeros(n, dtype=torch.bool)}
    monkeypatch.setattr(eval_slope, "drift", lambda run_dir: [])
    args = ["--run-dir", "x", "--ckpt", "y", "--deg", "20"]
    monkeypatch.setattr(eval_slope, "measure", lambda *a: fake(*a, miss=[]))
    assert eval_slope.main(args) == 0
    monkeypatch.setattr(eval_slope, "measure", lambda *a: fake(*a, miss=[len(YAWS)]))  # slope_up 첫 run
    assert eval_slope.main(args) == 1 and "slope_up 경사 위에 못 들어간 run 1/8" in capsys.readouterr().out


def test_classify_reset():
    fell, exited = classify_reset(torch.tensor([True, True, False]), torch.tensor([1.0, 3.3, 3.3]), 3.325)
    assert fell.tolist() == [True, False, False] and exited.tolist() == [False, True, False]


def test_wrap_pi():
    assert wrap_pi(torch.tensor([3.5 * math.pi])).item() == pytest.approx(-0.5 * math.pi)
