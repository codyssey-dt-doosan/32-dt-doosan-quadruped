"""train.py 로그 폴더 처리: 같은 실험명은 지우지 않고 거부, --resume는 마지막 체크포인트, 학습 코드가 바뀌면 resume 거부."""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

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
