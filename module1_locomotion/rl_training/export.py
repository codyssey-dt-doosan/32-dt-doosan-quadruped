"""체크포인트 actor → numpy npz(rl_policy 형식). 사용: python export.py --run-dir DIR --ckpt model_N.pt --out PATH.npz"""
import argparse
import subprocess
import sys
from pathlib import Path

import torch
from rsl_rl.runners import OnPolicyRunner
from tensordict import TensorDict

import genesis as gs

import cfgs
from go2_box_env import Go2BoxEnv
from mpc_controller import rl_policy, sim2sim


def make_runner(num_envs, cpu=False, dr=True, log_dir=None, train_cfg=None, **env_overrides):
    """cfgs + Go2BoxEnv + OnPolicyRunner. env_overrides는 env_cfg 키를 덮어쓴다(eval의 에피소드 길이 등)."""
    if not gs._initialized:  # 테스트는 한 프로세스에서 Genesis를 한 번만 초기화한다
        gs.init(backend=gs.cpu if cpu else gs.gpu, precision="32", logging_level="warning", seed=1)
    env_cfg, obs_cfg, reward_cfg, command_cfg = cfgs.get_cfgs(cfgs.load_sim2sim(), dr=dr)
    env_cfg.update(env_overrides)
    env = Go2BoxEnv(num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg)
    return OnPolicyRunner(env, train_cfg or cfgs.get_train_cfg("export", 0), log_dir, device=gs.device)


def drift(run_dir) -> list:
    """학습 때 run_dir에 남긴 sim2sim.json 사본과 저장소 JSON의 다른 키."""
    return sim2sim.diff(sim2sim.load(Path(run_dir) / "sim2sim.json"), cfgs.load_sim2sim())


def torch_actor_mean(runner, obs: torch.Tensor) -> torch.Tensor:
    """rsl_rl actor의 결정적 출력(Gaussian 평균). obs (N, 45)."""
    actor = runner.alg.get_policy()
    with torch.inference_mode():
        td = TensorDict({actor.obs_groups[0]: obs.to(runner.device)}, batch_size=[len(obs)])
        return actor(td).cpu()


def git_head() -> str:
    """내보낼 때 저장소 HEAD, 실패하면 "unknown"."""
    try:
        r = subprocess.run(["git", "rev-parse", "HEAD"], cwd=Path(__file__).resolve().parent, capture_output=True, text=True)
    except OSError:
        return "unknown"
    return r.stdout.strip() if r.returncode == 0 else "unknown"


def export_actor(runner, cfg: dict, out, meta: dict | None = None) -> None:
    actor = runner.alg.get_policy()
    if actor.obs_normalization:  # numpy 정책은 관측 정규화를 재현하지 않는다
        raise ValueError("actor obs_normalization is not supported by rl_policy")
    layers = [(m.weight.detach().cpu().numpy(), m.bias.detach().cpu().numpy())
              for m in actor.mlp if isinstance(m, torch.nn.Linear)]
    rl_policy.save(out, layers, cfg, runner.cfg["actor"]["activation"], meta)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--ckpt", required=True, help="run-dir 안의 model_N.pt")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)

    keys = drift(a.run_dir)
    if keys:
        print("sim2sim.json이 학습 때와 다르다:", ", ".join(keys))
        return 1
    runner = make_runner(num_envs=1, cpu=True, dr=False)
    runner.load(str(a.run_dir / a.ckpt), map_location="cpu")
    a.out.parent.mkdir(parents=True, exist_ok=True)
    meta = {"run": a.run_dir.resolve().name, "ckpt": a.ckpt, "iteration": runner.current_learning_iteration, "git": git_head()}
    export_actor(runner, cfgs.load_sim2sim(), a.out, meta)
    print(f"{a.out} {a.out.stat().st_size / 1e6:.2f} MB {meta}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
