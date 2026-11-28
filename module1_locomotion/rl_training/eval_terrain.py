"""P3 시험값 난이도 Genesis 평가(정보용, 판정 아님): 학습 타일 생성기로 계단 0.10 m·파이프 0.08 m·경사 20°를 고정해 걷게 한다.
사용: python eval_terrain.py --run-dir DIR --ckpt model_N.pt [--cpu] [--per-type 8] [--seconds 20]
DR 없음, 명령 (0.3, 0) 고정, 타일 가운데 플랫폼에서 +x로 출발(파이프는 축 y라 가로지른다). 출발 xy·yaw는 env마다 흔든다(시드 고정). 가운데에서의 체비셰프 거리로 진행을 잰다.
r3 = 플랫폼 끝 + 디딤면 3개(Gazebo 계단 3단 상당), r_full = 타일 끝(계단 6단·경사 2 m·파이프 3개). 먼저 닿은 쪽을 latch한다 —
r_full에 닿은 뒤 타일 밖(border)으로 나가 리셋되는 것은 전도로 세지 않는다. Gazebo 코스(§5)와 형상이 달라 G3를 대신하지 않는다."""
import argparse
import math
import sys
from pathlib import Path

import torch

import genesis as gs

import cfgs
from export import drift, make_runner

TYPES = ["stairs_up", "stairs_down", "slope_up", "slope_down", "pipes"]
TEST = {"step_h_max": 0.10, "pipe_d_max": 0.08, "slope_deg_max": 20.0}  # §5 시험값 = rows 2의 난이도 1.0 행
CMD_V = 0.3
XY_OFF, YAW_OFF = 0.3, 10.0  # 출발 흔들기 [m]·[°]: 플랫폼 반폭 0.5 m 안, 파이프를 비스듬히 건너도 축 y와 80° 이상


def terrain_cfg() -> dict:
    return {**cfgs.get_terrain_cfg(), **TEST, "rows": 2, "cols": TYPES}


def radii(t: dict) -> tuple[float, float]:
    return t["platform"] / 2 + 3 * t["tread"], t["tile"] / 2 - t["hs"]


def latch(reached, fell, prog, r, done):
    """한 스텝 갱신(제자리). reached·fell은 먼저 일어난 쪽만 남는다. done = 이번 스텝 리셋(이 스텝의 위치는 리셋 후라 진행에 넣지 않는다)."""
    live = ~(reached | fell)
    fell |= live & done
    live &= ~done
    prog[live] = torch.maximum(prog[live], r[live])
    return live


def perturb(env, seed=0):
    """DR이 없으면 같은 타일의 env가 모두 똑같이 움직여 n이 사실상 1이다 → 출발 xy ±XY_OFF(플랫폼 안)·yaw ±YAW_OFF를 env마다 흔든다."""
    g = torch.Generator().manual_seed(seed)
    u = lambda *s: (2 * torch.rand(*s, generator=g) - 1).to(gs.device)
    pos = env.robot.get_pos()
    pos[:, :2] += XY_OFF * u(env.num_envs, 2)
    half = math.radians(YAW_OFF) * u(env.num_envs) / 2
    quat = torch.stack([half.cos(), 0 * half, 0 * half, half.sin()], dim=-1)  # (w, x, y, z), z축 회전
    env.robot.set_pos(pos, zero_velocity=True)
    env.robot.set_quat(quat, zero_velocity=True)


def run(env, policy, seconds):
    t = env.terrain
    r3, rf = radii(t)
    env.levels[:] = 1  # 시험값 행만
    obs = env.reset()
    perturb(env)
    n = env.num_envs
    fell = torch.zeros(n, dtype=torch.bool, device=gs.device)
    reached = torch.zeros_like(fell)
    prog = torch.zeros(n, device=gs.device)
    t_reach = torch.full((n,), float("nan"), device=gs.device)
    steps = round(seconds / env.dt)
    for i in range(steps):
        obs, _, done, _ = env.step(policy(obs))
        r = (env.base_pos[:, :2] - env.origins[:, :2]).abs().max(dim=1).values
        latch(reached, fell, prog, r, done)
        new = ~reached & ~fell & (prog >= rf)
        t_reach[new] = (i + 1) * env.dt
        reached |= new
    return prog, reached, fell, t_reach, r3, rf


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--ckpt", required=True, help="run-dir 안의 model_N.pt")
    ap.add_argument("--cpu", action="store_true")
    ap.add_argument("--per-type", type=int, default=8)
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args(argv)

    keys = drift(a.run_dir)
    if keys:
        print("sim2sim.json이 학습 때와 다르다:", ", ".join(keys))
        return 1
    runner = make_runner(a.per_type * len(TYPES), a.cpu, dr=False, terrain=terrain_cfg(),
                         episode_length_s=a.seconds + 1.0)  # 시간 초과 리셋 없음
    runner.load(str(a.run_dir / a.ckpt), map_location=runner.device)
    policy = runner.get_inference_policy(device=runner.device)
    env = runner.env
    env._resample_commands = lambda envs_idx: None
    env.commands[:] = torch.tensor([CMD_V, 0.0, 0.0], device=gs.device)

    with torch.inference_mode():
        prog, reached, fell, t_reach, r3, rf = run(env, policy, a.seconds)
    print(f"{a.ckpt} | 명령 v={CMD_V} {a.seconds:.0f} s | r3={r3:.2f} m(3단 상당) r_full={rf:.2f} m | "
          f"계단 {TEST['step_h_max']} m·파이프 {TEST['pipe_d_max']} m·경사 {TEST['slope_deg_max']:.0f}°")
    for k, kind in enumerate(TYPES):
        m = env.types == k
        tr = t_reach[m & reached]
        print(f"{kind:12s} n={int(m.sum())} 전도 {int(fell[m].sum())} | r3 도달 {int((prog[m] >= r3).sum())} "
              f"| 완주 {int(reached[m].sum())} (평균 {tr.mean().item() if len(tr) else float('nan'):.1f} s) "
              f"| 진행 평균 {prog[m].mean().item():.2f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
