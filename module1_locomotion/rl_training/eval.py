"""G1 판정: 평지·DR 없음, 고정 명령 4조합 × 15 s × 32 env. 사용: python eval.py --run-dir DIR --ckpt model_N.pt [--cpu]
v = 몸체 헤딩 방향 속도, w = yaw 속도(walk_metrics.velocity_track과 같은 정의), 앞 2 s는 버린다. 오차 <10%·전도 0이면 PASS."""
import argparse
import math
import sys
from pathlib import Path

import torch

import genesis as gs

from export import drift, make_runner

COMBOS = [(0.3, 0.0), (0.0, 0.5), (0.5, 1.0), (-0.2, 0.0)]
EVAL_S, SETTLE_S, NUM_ENVS, TOL = 15.0, 2.0, 32, 0.1


def yaw(q):  # Genesis 쿼터니언 (w, x, y, z)
    w, x, y, z = q.unbind(-1)
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rel_err(meas, cmd):  # 명령 0이면 절대 오차(walk_metrics와 같다)
    return abs(meas - cmd) / abs(cmd) if abs(cmd) > 1e-9 else abs(meas)


def track(env, policy, v, w):
    env.commands[:] = torch.tensor([v, 0.0, w], device=gs.device)
    obs = env.reset()
    fell = torch.zeros(env.num_envs, dtype=torch.bool, device=gs.device)
    dist, dyaw = torch.zeros(env.num_envs, device=gs.device), torch.zeros(env.num_envs, device=gs.device)
    pos, h = env.robot.get_pos(), yaw(env.robot.get_quat())
    settle, steps = round(SETTLE_S / env.dt), round(EVAL_S / env.dt)
    for i in range(steps):
        obs, _, done, _ = env.step(policy(obs))
        fell |= done  # 시간 초과는 없으니 종료 = 전도(전도한 env는 리셋돼 위치가 튄다 → 평균에서 뺀다)
        p, hn = env.base_pos, yaw(env.base_quat)
        if i >= settle:
            d = p - pos
            dist += d[:, 0] * torch.cos(h) + d[:, 1] * torch.sin(h)
            dyaw += torch.remainder(hn - h + math.pi, 2 * math.pi) - math.pi
        pos, h = p, hn
    t = (steps - settle) * env.dt
    ok = ~fell
    return (dist[ok] / t).mean().item(), (dyaw[ok] / t).mean().item(), int(fell.sum())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--ckpt", required=True, help="run-dir 안의 model_N.pt")
    ap.add_argument("--cpu", action="store_true")
    a = ap.parse_args(argv)

    keys = drift(a.run_dir)
    if keys:
        print("sim2sim.json이 학습 때와 다르다:", ", ".join(keys))
        return 1
    runner = make_runner(NUM_ENVS, a.cpu, dr=False, episode_length_s=EVAL_S + 1.0)  # 시간 초과 리셋 없음
    runner.load(str(a.run_dir / a.ckpt), map_location=runner.device)
    policy = runner.get_inference_policy(device=runner.device)
    env = runner.env
    env._resample_commands = lambda envs_idx: None  # 명령 고정

    ok = True
    with torch.inference_mode():
        for v, w in COMBOS:
            v_meas, w_meas, falls = track(env, policy, v, w)
            err_v, err_w = rel_err(v_meas, v), rel_err(w_meas, w)
            ok &= err_v < TOL and err_w < TOL and falls == 0
            print(f"cmd=({v:+.1f},{w:+.1f}) v_meas={v_meas:+.3f} w_meas={w_meas:+.3f} "
                  f"err_v={err_v:.3f} err_w={err_w:.3f} falls={falls}")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
