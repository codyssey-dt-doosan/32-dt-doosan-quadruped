"""train.py 로그 폴더 처리: 같은 실험명은 지우지 않고 거부, --resume는 마지막 체크포인트."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from train import prepare_log_dir  # noqa: E402


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
