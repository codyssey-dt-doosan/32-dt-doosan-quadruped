"""numpy 정책 형식: npz 왕복, ELU 은닉층 + 선형 출력, 관측 차원 검사."""
from pathlib import Path

import numpy as np
import pytest

from mpc_controller import rl_policy, sim2sim

CFG = sim2sim.load(Path(__file__).resolve().parents[1] / "config/sim2sim.json")
rng = np.random.default_rng(0)


def test_save_load_roundtrip(tmp_path):
    layers = [(rng.normal(size=(8, 45)), rng.normal(size=8)), (rng.normal(size=(12, 8)), rng.normal(size=12))]
    rl_policy.save(tmp_path / "p.npz", layers, CFG)
    pol, cfg = rl_policy.load(tmp_path / "p.npz")
    assert cfg == CFG
    x = rng.normal(size=45); h = layers[0][0] @ x + layers[0][1]; h = np.where(h > 0, h, np.expm1(h))
    assert np.allclose(pol.act(x), layers[1][0] @ h + layers[1][1])


def test_rejects_wrong_obs_dim(tmp_path):
    rl_policy.save(tmp_path / "p.npz", [(np.zeros((12, 45)), np.zeros(12))], CFG)
    pol, _ = rl_policy.load(tmp_path / "p.npz")
    with pytest.raises(ValueError):
        pol.act(np.zeros(44))
