"""G0: 행동 0(=Q_NOM 목표)으로 서 있기 시험. 사용: python stand_check.py --kp 20 --kd 0.5 --seconds 10 -b 64"""
import argparse
import math

import torch

import genesis as gs

import cfgs
from go2_box_env import Go2BoxEnv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kp", type=float, required=True)
    ap.add_argument("--kd", type=float, required=True)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("-b", "--num_envs", type=int, default=64)
    ap.add_argument("--cpu", action="store_true")
    a = ap.parse_args()

    gs.init(backend=gs.cpu if a.cpu else gs.gpu, logging_level="warning")
    s2s = cfgs.load_sim2sim()
    env_cfg, obs_cfg, reward_cfg, command_cfg = cfgs.get_cfgs(s2s, dr=False)
    env_cfg["kp"], env_cfg["kd"] = a.kp, a.kd
    env_cfg["episode_length_s"] = a.seconds + 10.0  # 시간 초과 리셋이 시험에 끼지 않게
    env = Go2BoxEnv(a.num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg)
    env.reset()

    steps = round(a.seconds / env.dt)
    zero = torch.zeros((a.num_envs, env.num_actions), device=gs.device)
    fell = torch.zeros(a.num_envs, dtype=torch.bool, device=gs.device)
    tilt_max, z = 0.0, []
    for _ in range(steps):
        _, _, done, _ = env.step(zero)
        fell |= done  # 이 시험에서 종료는 곧 전도(기울기·몸체 접촉)
        tilt_max = max(tilt_max, math.degrees(torch.acos((-env.projected_gravity[:, 2]).clamp(-1, 1)).max().item()))
        z.append(env.base_pos[:, 2])
    z = torch.stack(z)
    z_mean, z_std = z.mean().item(), z.std().item()
    falls = int(fell.sum())
    print(f"kp={a.kp} kd={a.kd} falls={falls} tilt_max={tilt_max:.2f} z_mean={z_mean:.4f} z_std={z_std:.4f}")
    ok = falls == 0 and tilt_max < 5.0 and abs(z_mean - env_cfg["base_init_pos"][2]) < 0.03
    print("PASS" if ok else "FAIL")


if __name__ == "__main__":
    main()
