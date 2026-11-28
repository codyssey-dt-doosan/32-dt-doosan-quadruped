"""train.py 로그 폴더 처리: 같은 실험명은 지우지 않고 거부, --resume는 마지막 체크포인트, 학습 코드가 바뀌면 resume 거부.
--init-from은 actor·critic 가중치만 불러 새 run을 시작한다(새 Adam·iteration 0), 출처는 provenance·npz meta init_from."""
import hashlib
import json
import sys
from pathlib import Path

import pytest
import torch
from rsl_rl.runners import OnPolicyRunner

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import genesis as gs  # noqa: E402

import cfgs  # noqa: E402
import export  # noqa: E402
import train  # noqa: E402
from go2_box_env import Go2BoxEnv  # noqa: E402
from mpc_controller import rl_policy  # noqa: E402
from train import CODE, code_drift, default_log_dir, prepare_log_dir, provenance  # noqa: E402


def test_prepare_log_dir_refuses_existing(tmp_path):          # Review Focus 1
    (tmp_path / "run").mkdir(); (tmp_path / "run" / "model_10.pt").write_bytes(b"x")
    with pytest.raises(FileExistsError):
        prepare_log_dir(tmp_path / "run", resume=False)
    assert (tmp_path / "run" / "model_10.pt").exists()
    assert prepare_log_dir(tmp_path / "run", resume=True).name == "model_10.pt"


def test_resume_picks_highest_iteration(tmp_path):  # 사전순이면 model_9가 뽑힌다
    for i in (9, 10, 100):
        (tmp_path / f"model_{i}.pt").write_bytes(b"x")
    assert prepare_log_dir(tmp_path, resume=True).name == "model_100.pt"


def test_new_dir_is_created(tmp_path):
    assert prepare_log_dir(tmp_path / "a" / "run", resume=False) is None
    assert (tmp_path / "a" / "run").is_dir()


def test_default_log_dir_ignores_cwd(tmp_path, monkeypatch):  # 저장소 루트에서 돌려도 rl_training/logs
    monkeypatch.chdir(tmp_path)
    monkeypatch.delenv("RL_LOG_DIR", raising=False)
    assert default_log_dir() == Path(__file__).resolve().parents[1] / "logs"
    monkeypatch.setenv("RL_LOG_DIR", str(tmp_path / "x"))
    assert default_log_dir() == tmp_path / "x"


def test_code_drift_refuses_changed_training_code(tmp_path):  # 코랩 재연결 사이 push로 보상·env가 바뀐 채 이어 학습하지 않게
    assert code_drift(tmp_path) == ["provenance.json 없음"]
    p = provenance()
    assert set(p["code"]) == set(CODE) and p["git"] and isinstance(p["dirty"], bool)
    (tmp_path / "provenance.json").write_text(json.dumps(p))
    assert code_drift(tmp_path) == []
    p["code"]["cfgs.py"] = "0" * 64
    (tmp_path / "provenance.json").write_text(json.dumps(p))
    assert code_drift(tmp_path) == ["cfgs.py"]


def test_code_drift_refuses_terrain_mode_change(tmp_path):  # 코랩 재연결 때 --terrain을 빠뜨리면 험지 run이 평지로 이어지지 않게
    p = provenance()
    (tmp_path / "provenance.json").write_text(json.dumps({**p, "terrain": True}))
    assert code_drift(tmp_path, terrain=True) == []
    assert code_drift(tmp_path, terrain=False) == ["--terrain"]
    (tmp_path / "provenance.json").write_text(json.dumps(p))  # terrain 키가 없는 옛 run = 평지
    assert code_drift(tmp_path) == [] and code_drift(tmp_path, terrain=True) == ["--terrain"]


@pytest.fixture(scope="module")
def env():
    if not gs._initialized:  # 다른 테스트 파일이 먼저 돌았을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")
    return Go2BoxEnv(2, *cfgs.get_cfgs(cfgs.load_sim2sim(), dr=False))


def new_runner(env, scramble=False):  # 새 runner = 새 Adam. scramble이면 가중치를 섞어 ckpt를 불러왔는지 구별되게
    runner = OnPolicyRunner(env, cfgs.get_train_cfg("x", 1), device=gs.device)
    if scramble:
        with torch.no_grad():
            for p in runner.alg.optimizer.param_groups[0]["params"]:
                p.add_(1.0)
    return runner


@pytest.fixture
def ckpt(env, tmp_path):
    """옛 run의 체크포인트: Adam 한 걸음으로 optimizer state를 채우고 lr 3e-4(기본 1e-3·INIT_FROM_LR과 구별)·iter 7로 저장."""
    runner = new_runner(env)
    opt = runner.alg.optimizer
    sum((p ** 2).sum() for p in opt.param_groups[0]["params"]).backward()
    opt.step()
    opt.param_groups[0]["lr"] = 3e-4
    runner.current_learning_iteration = 7
    (tmp_path / "rough_v1").mkdir()
    runner.save(str(path := tmp_path / "rough_v1" / "model_7.pt"))
    return path


def same(a: dict, b: dict) -> bool:  # state_dict 텐서별 비교
    return a.keys() == b.keys() and all(torch.equal(a[k], b[k]) for k in a)


def lrs(runner) -> set:  # PPO가 쓰는 lr과 optimizer param_group들의 lr
    return {runner.alg.learning_rate} | {g["lr"] for g in runner.alg.optimizer.param_groups}


def test_init_from_weights_only(env, ckpt):  # 보상이 바뀐 파인튜닝: 옛 보상의 Adam 모멘텀·반복 수는 가져오지 않는다
    saved = torch.load(ckpt, weights_only=False)
    runner = new_runner(env, scramble=True)
    assert not same(runner.alg.actor.state_dict(), saved["actor_state_dict"])
    info = train.init_from(runner, ckpt)
    assert same(runner.alg.actor.state_dict(), saved["actor_state_dict"])
    assert same(runner.alg.critic.state_dict(), saved["critic_state_dict"])
    assert runner.alg.optimizer.state_dict()["state"] == {} and runner.current_learning_iteration == 0
    assert lrs(runner) == {1e-5}  # 새 Adam의 첫 걸음(≈ lr·sign(g))이 KL을 튀기지 않게 낮게 시작 — adaptive가 올린다
    assert info == {"run": "rough_v1", "ckpt": "model_7.pt", "sha256": hashlib.sha256(ckpt.read_bytes()).hexdigest(),
                    "optimizer": False}


def test_init_from_with_optimizer(env, ckpt):
    saved = torch.load(ckpt, weights_only=False)["optimizer_state_dict"]["state"]
    runner = new_runner(env, scramble=True)
    assert train.init_from(runner, ckpt, load_optimizer=True)["optimizer"] is True
    state = runner.alg.optimizer.state_dict()["state"]
    assert saved and state.keys() == saved.keys() and all(same(state[i], saved[i]) for i in saved)
    assert lrs(runner) == {3e-4}  # 불러온 optimizer의 lr을 그대로 쓴다


def test_init_from_resume_exclusive(tmp_path, capsys):  # --resume은 같은 run을 잇고 --init-from은 새 run — Genesis 빌드 전에 거부
    with pytest.raises(SystemExit):
        train.main(["-e", "x", "--log-dir", str(tmp_path), "--resume", "--init-from", "x"])
    assert "같이 쓸 수 없다" in capsys.readouterr().err
    assert not any(tmp_path.iterdir())


def test_export_meta_init_from(env, tmp_path):  # 파인튜닝 run을 내보내면 npz에도 출발 체크포인트가 남는다
    run = tmp_path / "rough_v2"
    run.mkdir()
    src = {"run": "rough_v1", "ckpt": "model_7.pt", "sha256": "0" * 64, "optimizer": False}
    (run / "sim2sim.json").write_text(json.dumps(cfgs.load_sim2sim()))
    (run / "provenance.json").write_text(json.dumps({"git": "abc123", "dirty": False, "code": {}, "init_from": src}))
    new_runner(env).save(str(run / "model_0.pt"))
    out = tmp_path / "p.npz"
    assert export.main(["--run-dir", str(run), "--ckpt", "model_0.pt", "--out", str(out)]) == 0
    assert rl_policy.load_meta(out)["init_from"] == src
