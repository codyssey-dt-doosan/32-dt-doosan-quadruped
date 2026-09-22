#!/usr/bin/env python3
"""힘 제어 기립 지표. balance.launch.py(gui:=false)가 뜨고 기동(≈6 s)이 끝난 뒤 실행.

포즈는 gz `/world/<w>/pose/info`(월드 프레임), 힘·QP 시간·모드는 노드 로그의 1 Hz status 줄에서 읽는다.
기준(스펙 §4, 창 = 실행 시점부터 duration s):
  기립: z 표준편차 < 1 mm, 기울기 < 1°, xy 드리프트 < 1 cm, Σf_z = mg ±5%, QP 평균 < 2 ms·p99 < 5 ms, fail 0, hold 0, sat 0
  --push N S: 시작 후 2 s에 +y N을 S s → 전도 0, 최대 기울기 < 10°, 최대 xy 이탈 < 5 cm, clear 후 2 s 안 복귀, 발 이동 < 2 cm
  --expect-pitch D: 몸체 pitch 평균 = D ±2°, 발 이동 < 2 cm, Σnᵀf = mg cos D ±5%, Σtᵀf = mg sin D ±10%
"""
import argparse
import math
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import trot_metrics as tm  # noqa: E402

LEGS = tm.LEGS
CALF_TO_FOOT = (0.0, 0.0, -0.10)  # 구 중심
MG = 13.2 * 9.8


def parse_feet(text: str) -> list:
    """메시지마다 (x, y, z, tilt_deg, pitch_deg, yaw_deg, {leg: 발 월드 xyz}, sim_t)."""
    rows = []
    for msg in text.split("header {")[1:]:
        stamp = re.search(r"stamp \{\s*(?:sec: (\d+))?\s*(?:nsec: (\d+))?", msg)
        ts = int(stamp.group(1) or 0) + int(stamp.group(2) or 0) * 1e-9
        poses = {m.group(1): tm._pose(m.group(2)) for m in re.finditer(r'pose \{\s*name: "(\w+)"(.*?)\n\}', msg, re.S)}
        if "go2" not in poses or any(f"{leg}_calf" not in poses for leg in LEGS):
            continue
        (bx, by, bz), bq = poses["go2"]
        w, x, y, z = bq
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        feet = {}
        for leg in LEGS:
            cp, cq = poses[f"{leg}_calf"]
            local = tuple(c + f for c, f in zip(cp, tm._rotate(cq, CALF_TO_FOOT)))
            feet[leg] = tuple(b + r for b, r in zip((bx, by, bz), tm._rotate(bq, local)))
        rows.append((bx, by, bz, tilt, pitch, yaw, feet, ts))
    return rows


def log_stats(path: str) -> dict:
    all_lines = Path(path).read_text(encoding="utf-8", errors="ignore").splitlines()
    hold = sum("hold:" in l for l in all_lines)
    lines = [l for l in all_lines if "status mode=" in l]
    if not lines:
        return {}
    m = re.search(r"mode=(\w+) qp_ms=([\d.]+)/([\d.]+) fail=(\d+) sat=(\d+) sum_fz=([-\d.]+)", lines[-1])
    return {
        "mode": m.group(1), "qp_ms": float(m.group(2)), "qp_p99": float(m.group(3)), "fail": int(m.group(4)),
        "sat": int(m.group(5)), "sum_fz": float(m.group(6)), "hold": hold,
    }


def _push(world: str, newton: float, seconds: float) -> None:
    time.sleep(2.0)
    ent = 'entity: {name: "go2::base_link", type: LINK}'  # Task 1 Step 3에서 확인한 형태로
    subprocess.run(["gz", "topic", "-t", f"/world/{world}/wrench/persistent", "-m", "gz.msgs.EntityWrench",
                    "-p", f"{ent}, wrench: {{force: {{y: {newton}}}}}"], capture_output=True)
    time.sleep(seconds)
    subprocess.run(["gz", "topic", "-t", f"/world/{world}/wrench/clear", "-m", "gz.msgs.Entity",
                    "-p", 'name: "go2::base_link", type: LINK'], capture_output=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="corridor")
    ap.add_argument("--log", required=True)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--push", nargs=2, type=float, metavar=("N", "S"))
    ap.add_argument("--expect-pitch", type=float)
    a = ap.parse_args()
    if a.push:
        threading.Thread(target=_push, args=(a.world, *a.push), daemon=True).start()
    rows = parse_feet(tm.capture(a.world, a.duration))
    if len(rows) < 10:
        print(f"포즈 {len(rows)}건 — 시뮬·GZ_IP 확인")
        return 1
    st = log_stats(a.log)
    z = [r[2] for r in rows]
    z_std = (sum((v - sum(z) / len(z)) ** 2 for v in z) / len(z)) ** 0.5
    tilt = max(r[3] for r in rows)
    pitch = sum(r[4] for r in rows) / len(rows)
    drift = math.hypot(rows[-1][0] - rows[0][0], rows[-1][1] - rows[0][1])
    slip = max(math.hypot(r[6][l][0] - rows[0][6][l][0], r[6][l][1] - rows[0][6][l][1]) for r in rows for l in LEGS)
    out = (f"n={len(rows)} z_std_mm={z_std * 1e3:.2f} tilt_max={tilt:.2f} pitch_mean={pitch:.2f} xy_drift_cm={drift * 100:.1f} "
           f"foot_slip_cm={slip * 100:.1f} " + " ".join(f"{k}={v}" for k, v in st.items()))
    ok = st.get("mode") == "run" and st.get("hold", 1) == 0 and st.get("fail", 1) == 0 and st.get("sat", 1) == 0
    ok &= st.get("qp_ms", 9) < 2.0 and st.get("qp_p99", 9) < 5.0
    if a.push:
        t0 = rows[0][7]
        pre = [r for r in rows if r[7] - t0 < 2.0]
        cx, cy = sum(r[0] for r in pre) / len(pre), sum(r[1] for r in pre) / len(pre)
        exc = max(math.hypot(r[0] - cx, r[1] - cy) for r in rows)
        t_clear = t0 + 2.0 + a.push[1]
        after = [r for r in rows if r[7] > t_clear]
        rec = next((r[7] - t_clear for i, r in enumerate(after[:-5]) if r[3] < 1.0
                    and math.hypot(after[i + 5][0] - r[0], after[i + 5][1] - r[1]) / max(1e-6, after[i + 5][7] - r[7]) < 0.01), None)
        out += f" xy_exc_cm={exc * 100:.1f} recover_s={rec}"
        ok &= tilt < 10.0 and exc < 0.05 and rec is not None and rec < 2.0 and slip < 0.02
    elif a.expect_pitch is not None:
        ok &= abs(pitch - a.expect_pitch) < 2.0 and slip < 0.02 and abs(st.get("sum_fz", 0) - MG) < 0.05 * MG
        out += f" expect_pitch={a.expect_pitch}"
    else:
        ok &= z_std < 1e-3 and tilt < 1.0 and drift < 0.01 and abs(st.get("sum_fz", 0) - MG) < 0.05 * MG
    print(out)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
