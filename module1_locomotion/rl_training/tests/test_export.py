"""export.py: numpy 정책 = rsl_rl actor 결정적 평균, 학습 때 sim2sim 사본과 저장소 JSON이 다르면 거부, 출처 meta(학습 커밋 포함)."""
import json
import sys
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cfgs  # noqa: E402
import export  # noqa: E402
from export import export_actor, make_runner, torch_actor_mean  # noqa: E402
from mpc_controller import rl_policy  # noqa: E402

CFG = cfgs.load_sim2sim()


def test_numpy_matches_torch_actor(tmp_path):
    runner = make_runner(num_envs=2, cpu=True)                # cfgs + Go2BoxEnv + OnPolicyRunner
    export_actor(runner, CFG, tmp_path / "p.npz")
    pol, _ = rl_policy.load(tmp_path / "p.npz")
    obs = torch.randn(100, 45)
    ref = torch_actor_mean(runner, obs).numpy()               # 결정적 평균
    assert np.abs(np.stack([pol.act(o) for o in obs.numpy()]) - ref).max() < 1e-5


def test_export_refuses_config_drift(tmp_path, capsys):
    run = tmp_path / "run"
    run.mkdir()
    (run / "sim2sim.json").write_text(json.dumps({**CFG, "kp": CFG["kp"] + 1.0}))
    out = tmp_path / "p.npz"
    assert export.main(["--run-dir", str(run), "--ckpt", "model_0.pt", "--out", str(out)]) == 1
    assert "kp" in capsys.readouterr().out
    assert not out.exists()


def test_export_writes_meta(tmp_path):
    run = tmp_path / "run_x"
    run.mkdir()
    (run / "sim2sim.json").write_text(json.dumps(CFG))
    (run / "provenance.json").write_text(json.dumps({"git": "abc123", "dirty": False, "code": {}}))
    runner = make_runner(num_envs=2, cpu=True)
    runner.current_learning_iteration = 7
    runner.save(str(run / "model_7.pt"))
    out = tmp_path / "p.npz"
    assert export.main(["--run-dir", str(run), "--ckpt", "model_7.pt", "--out", str(out)]) == 0
    meta = rl_policy.load_meta(out)
    assert (meta["run"], meta["ckpt"], meta["iteration"]) == ("run_x", "model_7.pt", 7) and meta["git"]
    assert (meta["train_git"], meta["train_dirty"]) == ("abc123", False)  # 학습 커밋(내보낸 커밋과 다를 수 있다)
