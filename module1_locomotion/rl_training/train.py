"""PPO 학습. 사용: python train.py -e EXP [-b 4096] [--max-iterations N] [--log-dir DIR] [--resume] [--save-interval 50] [--cpu] [--terrain]
실행 폴더 = <log-dir>/<EXP>. 같은 실험명이 있으면 지우지 않고 거부한다(--resume로 이어 학습).
--resume은 sim2sim.json과 학습 코드(provenance.json)가 run 시작 때와 같을 때만 이어 간다."""
import argparse
import hashlib
import json
import os
import pickle
import re
import subprocess
from pathlib import Path

import cfgs
from export import drift, git_head, make_runner

HERE = Path(__file__).resolve().parent
# 바뀌면 같은 run을 이어 학습하지 않는다(보상·PPO·env 식·동역학, make_runner의 gs.init, Genesis·rsl_rl 버전)
CODE = ("go2_box_env.py", "cfgs.py", "terrain.py", "train.py", "export.py", "assets/go2_box.urdf", "requirements.txt")


def provenance() -> dict:
    """학습 출처: 저장소 HEAD, 추적 파일 미커밋 변경 여부, 학습 코드 sha256."""
    r = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=HERE, capture_output=True, text=True)
    return {"git": git_head(), "dirty": r.returncode != 0 or bool(r.stdout.strip()),
            "code": {f: hashlib.sha256((HERE / f).read_bytes()).hexdigest() for f in CODE}}


def code_drift(run_dir, terrain: bool = False) -> list:
    """run 시작 때 남긴 provenance.json과 지금 학습 코드가 다른 파일, 지형 모드가 다르면 "--terrain". 기록이 없으면 그 사실을 돌려준다."""
    p = Path(run_dir) / "provenance.json"
    if not p.exists():
        return ["provenance.json 없음"]
    saved = json.loads(p.read_text())
    now = provenance()["code"]
    keys = sorted(f for f in saved["code"].keys() | now.keys() if saved["code"].get(f) != now.get(f))
    return keys + (["--terrain"] if saved.get("terrain", False) != terrain else [])  # 옛 run은 평지


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
    ap.add_argument("--terrain", action="store_true", help="지형 커리큘럼(cfgs.get_terrain_cfg), 없으면 평지")
    a = ap.parse_args()

    prov, s2s = provenance(), cfgs.load_sim2sim()  # Genesis 빌드(수십 초) 전, 이 프로세스가 import한 코드 기준
    run_dir = a.log_dir / a.exp_name
    ckpt = prepare_log_dir(run_dir, a.resume)
    prov["terrain"] = a.terrain  # 재연결 때 --terrain을 빠뜨리면 험지 run을 평지로 이어 학습한다
    if ckpt is not None and (keys := drift(run_dir) + code_drift(run_dir, a.terrain)):
        raise SystemExit(f"run 시작 때와 다르다(sim2sim.json·학습 코드): {', '.join(keys)} — "
                         f"provenance.json의 git 커밋으로 맞추거나 새 -e로 시작")

    train_cfg = cfgs.get_train_cfg(a.exp_name, a.max_iterations)
    train_cfg["save_interval"] = a.save_interval
    runner = make_runner(a.num_envs, a.cpu, dr=True, log_dir=str(run_dir), train_cfg=train_cfg,
                         terrain=cfgs.get_terrain_cfg() if a.terrain else None)  # env_cfg["terrain"]
    if ckpt is None:
        env = runner.env
        with open(run_dir / "cfgs.pkl", "wb") as f:
            pickle.dump([env.env_cfg, env.obs_cfg, env.reward_cfg, env.command_cfg, runner.cfg], f)
        (run_dir / "sim2sim.json").write_text(json.dumps(s2s, indent=2))
        (run_dir / "provenance.json").write_text(json.dumps(prov, indent=2))
    else:
        print(f"resume {ckpt}")
        runner.load(str(ckpt), map_location=runner.device)  # Colab(CUDA) 체크포인트를 맥에서 이어도 되게
        runner.current_learning_iteration += 1  # rsl_rl은 저장된 반복부터 다시 돈다 → 재개한 체크포인트를 덮어쓰지 않게
    runner.learn(num_learning_iterations=a.max_iterations, init_at_random_ep_len=True)


if __name__ == "__main__":
    main()
