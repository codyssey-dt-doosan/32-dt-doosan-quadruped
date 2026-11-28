"""Genesis 경사 사전 진단(설계 §4.7 관문, Gazebo 전): 경사 타일에서 헤딩 회전·횡 이동·피치를 잰다. 원형은 일회용 진단 slope_drift.py.
사용: python eval_slope.py --run-dir DIR --ckpt model_N.pt [--deg 20 15]
Gazebo 경사 길이(L cos20 = 2.82 m)에 맞춰 tile 6.65 m. DR·잡음 없음, 명령 (0.3, 0, 0) 고정. 각도마다 scene 하나(rows 2의 1행 = 그 각도).
출발 yaw는 거울쌍(0, ±2.5, ±5, ±10°) — 정책이 좌우 대칭이면 y(+ψ0) = −y(−ψ0). 경사 위 = 타일 원점 체비셰프 거리 lo..hi(양끝 0.2 m 제외).
관문: 경사 위 |mean ω| ≤ 0.004·max |ω| ≤ 0.008 rad/s(오르막·내리막), 내리막 max |평균 피치 − 지형각| ≤ 3°, 평지 max |ω| ≤ 0.004.
마지막 줄은 GATE PASS 또는 GATE FAIL: 사유, 종료 코드 0/1."""
import argparse
import math
import sys
from pathlib import Path

import torch

import genesis as gs

import cfgs
from export import drift, make_runner

TYPES = ["flat", "slope_up", "slope_down"]
YAWS = [0.0, 2.5, -2.5, 5.0, -5.0, 10.0, -10.0, 0.0]  # 출발 yaw [°]
TILE, CMD_V, SECONDS = 6.65, 0.3, 30.0
W_MEAN_MAX, W_ABS_MAX, PITCH_ERR_MAX, FLAT_W_MAX = 0.004, 0.008, 3.0, 0.004  # 관문 [rad/s]·[rad/s]·[°]·[rad/s]
SEED = 0  # 롤아웃 직전 전역 torch 시드 — env 생성 때 RNG를 몇 번 쓰든 같은 ckpt면 같은 수치(리스폰 타일 randint 등)


def yaw_pitch(q):
    """쿼터니언 (n, 4) (w, x, y, z) → yaw, pitch [rad]. pitch +는 머리 숙임."""
    w, x, y, z = q.unbind(-1)
    yaw = torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = torch.asin(torch.clamp(2 * (w * y - z * x), -1, 1))
    return yaw, pitch


def wrap_pi(x):
    return torch.remainder(x + math.pi, 2 * math.pi) - math.pi


def classify_reset(done, r, half):
    """리셋을 (전도, 이탈)로 나눈다. r = 리셋 직전 타일 원점 체비셰프 거리 — 타일 끝(half − 0.1 이상)에서 border로 나가 리셋된 것은 전도가 아니다."""
    exited = done & (r >= half - 0.1)
    return done & ~exited, exited


def gate(stats: dict) -> tuple[bool, list[str]]:
    """stats[(deg, kind)] = {"w": run마다 경사 위 평균 yaw 속도 [rad/s], "pitch_err": [°]}. 경사 위에 못 들어간 run은 NaN —
    기록이 비었거나 NaN이 하나라도 있으면 실패(fail-closed). 없는 키는 판정하지 않는다. (통과, 실패 사유)."""
    why = []
    for (deg, kind), s in stats.items():
        w = s["w"]
        if not w:
            why.append(f"{deg:g}° {kind} 경사 위 기록 없음")
            continue
        if bad := sum(not math.isfinite(v) for v in w):
            why.append(f"{deg:g}° {kind} 경사 위에 못 들어간 run {bad}/{len(w)}")
            continue
        w_abs = max(abs(v) for v in w)
        if kind == "flat":
            if w_abs > FLAT_W_MAX:
                why.append(f"{deg:g}° {kind} max |ω| {w_abs:.4f} > {FLAT_W_MAX}")
            continue
        w_mean = sum(w) / len(w)
        if abs(w_mean) > W_MEAN_MAX:
            why.append(f"{deg:g}° {kind} mean ω {w_mean:+.4f} > ±{W_MEAN_MAX}")
        if w_abs > W_ABS_MAX:
            why.append(f"{deg:g}° {kind} max |ω| {w_abs:.4f} > {W_ABS_MAX}")
        err = max(s["pitch_err"], key=abs)  # 오르막 피치는 출력만 한다
        if kind == "slope_down" and abs(err) > PITCH_ERR_MAX:
            why.append(f"{deg:g}° {kind} 피치 오차 {err:+.1f}° > ±{PITCH_ERR_MAX}°")
    return not why, why


def measure(run_dir, ckpt, deg):
    """경사 deg° scene에서 TYPES × YAWS를 SECONDS 걷게 한다. +x 끝이 border라 타일 끝을 넘으면 리셋된다(이탈, 그 뒤는 세지 않는다).
    env마다 타입·경사 위 시간 [s]·Δψ [rad]·Δy [m]·평균 피치 [rad](경사 위에 못 들어갔으면 NaN)·전도·이탈을 dict로 돌려준다."""
    t = {**cfgs.get_terrain_cfg(), "rows": 2, "cols": TYPES, "slope_deg_max": deg, "tile": TILE}
    runner = make_runner(len(YAWS) * len(TYPES), False, dr=False, terrain=t, episode_length_s=SECONDS + 1.0)  # 시간 초과 리셋 없음
    runner.load(str(run_dir / ckpt), map_location=runner.device)
    policy = runner.get_inference_policy(device=runner.device)
    env = runner.env
    env._resample_commands = lambda envs_idx: None
    env.commands[:] = torch.tensor([CMD_V, 0.0, 0.0], device=gs.device)
    env.levels[:] = 1
    torch.manual_seed(SEED)
    obs = env.reset()
    n = env.num_envs
    psi0 = torch.tensor(YAWS, device=gs.device)[torch.arange(n, device=gs.device) % len(YAWS)] * math.pi / 180
    env.robot.set_quat(torch.stack([torch.cos(psi0 / 2), 0 * psi0, 0 * psi0, torch.sin(psi0 / 2)], -1), zero_velocity=True)

    org = env.origins[:, :2].clone()
    yaw_prev, _ = yaw_pitch(env.robot.get_quat())
    unwrap = yaw_prev.clone()
    s = {k: torch.zeros(n, device=gs.device) for k in ("cnt", "dpsi", "dy", "pitch_sum", "psi_in", "y_in")}
    live = torch.ones(n, dtype=torch.bool, device=gs.device)
    fell, exited = torch.zeros_like(live), torch.zeros_like(live)
    r = torch.zeros(n, device=gs.device)
    lo, hi = t["platform"] / 2 + 0.2, TILE / 2 - 0.2
    with torch.inference_mode():
        for _ in range(round(SECONDS / env.dt)):
            obs, _, done, _ = env.step(policy(obs))
            f, e = classify_reset(done & live, r, TILE / 2)  # r은 직전 스텝 값: 리셋된 env의 base_pos는 이미 스폰 위치다
            fell |= f
            exited |= e
            live &= ~done
            yaw, pitch = yaw_pitch(env.base_quat)
            unwrap += wrap_pi(yaw - yaw_prev)
            yaw_prev = yaw
            rel = env.base_pos[:, :2] - org
            r = rel.abs().max(dim=1).values
            on = live & (r > lo) & (r < hi)
            first = on & (s["cnt"] == 0)
            s["y_in"][first] = rel[first, 1]
            s["psi_in"][first] = unwrap[first]
            s["dy"][on] = rel[on, 1] - s["y_in"][on]
            s["dpsi"][on] = unwrap[on] - s["psi_in"][on]
            s["pitch_sum"][on] += pitch[on]
            s["cnt"][on] += 1
    return {"type": env.types, "t_on": s["cnt"] * env.dt, "dpsi": s["dpsi"], "dy": s["dy"],
            "pitch": s["pitch_sum"] / s["cnt"], "fell": fell, "exited": exited}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--ckpt", required=True, help="run-dir 안의 model_N.pt")
    ap.add_argument("--deg", type=float, nargs="+", default=[20.0, 15.0])
    a = ap.parse_args(argv)

    keys = drift(a.run_dir)
    if keys:
        print("sim2sim.json이 학습 때와 다르다:", ", ".join(keys))
        return 1
    stats = {}
    for deg in a.deg:
        m = measure(a.run_dir, a.ckpt, deg)
        n = len(m["type"])
        print(f"{a.ckpt} 경사 {deg:g}° tile {TILE} | 명령 ({CMD_V}, 0, 0) {SECONDS:.0f} s | "
              f"전도 {int(m['fell'].sum())}/{n} 타일 끝 이탈 {int(m['exited'].sum())}/{n}")
        for k, kind in enumerate(TYPES):
            st = stats[(deg, kind)] = {"w": [], "pitch_err": []}
            for j in torch.nonzero(m["type"] == k).flatten().tolist():
                t_on = m["t_on"][j].item()
                w = m["dpsi"][j].item() / t_on if t_on > 0 else math.nan  # 못 들어간 run도 남겨 gate가 실패로 센다
                pit = math.degrees(m["pitch"][j].item())
                err = pit + {"slope_up": deg, "slope_down": -deg}.get(kind, 0.0)  # pitch +는 머리 숙임: 내리막 = 피치 − deg
                st["w"].append(w)
                st["pitch_err"].append(err)
                tag = " 전도" if m["fell"][j] else " 이탈" if m["exited"][j] else ""
                print(f"  {kind:10s} yaw0 {YAWS[j % len(YAWS)]:+5.1f}° | 경사 위 {t_on:5.1f} s  Δψ {math.degrees(m['dpsi'][j].item()):+6.1f}°  "
                      f"ω {w:+.4f} rad/s  Δy {m['dy'][j].item():+.3f} m  피치 {pit:+5.1f}° (오차 {err:+5.1f}°){tag}")
            if st["w"]:
                print(f"  → {kind} n={len(st['w'])} mean ω {sum(st['w']) / len(st['w']):+.4f}  max |ω| {max(map(abs, st['w'])):.4f}  "
                      f"피치 오차 {min(st['pitch_err']):+.1f}~{max(st['pitch_err']):+.1f}°")
    passed, why = gate(stats)
    print("GATE PASS" if passed else "GATE FAIL: " + "; ".join(why))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
