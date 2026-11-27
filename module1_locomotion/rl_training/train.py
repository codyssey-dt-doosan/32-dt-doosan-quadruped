"""PPO 학습. 사용: python train.py -e EXP [-b 4096] [--max-iterations N] [--log-dir DIR] [--resume] [--save-interval 50] [--cpu]
실행 폴더 = <log-dir>/<EXP>. 같은 실험명이 있으면 지우지 않고 거부한다(--resume로 이어 학습)."""
import argparse
import json
import os
import pickle
import re
from pathlib import Path

import cfgs
from export import drift, make_runner


def default_log_dir() -> Path:
    """$RL_LOG_DIR, 없으면 이 폴더의 logs(.gitignore) — 실행 위치와 무관."""
    return Path(os.environ.get("RL_LOG_DIR") or Path(__file__).resolve().parent / "logs")


def prepare_log_dir(path: Path, resume: bool) -> Path | None:
    """resume면 반복 수가 가장 큰 model_*.pt, 새 폴더면 만들고 None. 있는 폴더를 resume 없이 쓰면 FileExistsError."""
    path = Path(path)
    if path.exists() and not resume:
        raise FileExistsError(f"{path} exists — use --resume or a new experiment name")
    if path.exists():
        ckpts = [p for p in path.glob("model_*.pt") if re.fullmatch(r"model_\d+\.pt", p.name)]
        if not ckpts:
            raise FileNotFoundError(f"no model_*.pt in {path}")
        return max(ckpts, key=lambda p: int(p.stem[6:]))
    path.mkdir(parents=True)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-e", "--exp-name", required=True)
    ap.add_argument("-b", "--num-envs", type=int, default=4096)
    ap.add_argument("--max-iterations", type=int, default=1500, help="--resume면 이어서 더 돌 반복 수")
    ap.add_argument("--log-dir", type=Path, default=default_log_dir())
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--save-interval", type=int, default=50)
    ap.add_argument("--cpu", action="store_true")
    a = ap.parse_args()

    run_dir = a.log_dir / a.exp_name
    ckpt = prepare_log_dir(run_dir, a.resume)
    if ckpt is not None and (keys := drift(run_dir)):
        raise SystemExit(f"sim2sim.json이 학습 때와 다르다: {', '.join(keys)}")

    train_cfg = cfgs.get_train_cfg(a.exp_name, a.max_iterations)
    train_cfg["save_interval"] = a.save_interval
    runner = make_runner(a.num_envs, a.cpu, dr=True, log_dir=str(run_dir), train_cfg=train_cfg)
    if ckpt is None:
        env = runner.env
        with open(run_dir / "cfgs.pkl", "wb") as f:
            pickle.dump([env.env_cfg, env.obs_cfg, env.reward_cfg, env.command_cfg, runner.cfg], f)
        (run_dir / "sim2sim.json").write_text(json.dumps(cfgs.load_sim2sim(), indent=2))
    else:
        print(f"resume {ckpt}")
        runner.load(str(ckpt), map_location=runner.device)  # Colab(CUDA) 체크포인트를 맥에서 이어도 되게
        runner.current_learning_iteration += 1  # rsl_rl은 저장된 반복부터 다시 돈다 → 재개한 체크포인트를 덮어쓰지 않게
    runner.learn(num_learning_iterations=a.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
