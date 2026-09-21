#!/usr/bin/env python3
"""제자리 trot 지표. legged.launch.py(gui:=false trot:=true)가 뜨고 6 s 넘게 지난 뒤 실행.

gz 포즈 토픽을 duration 동안 받아 몸체 기울기·z·xy/yaw 드리프트와 발끝 이격(발끝 z의 max-min)을 낸다.
3D 포즈는 ROS로 브리지되지 않아 `gz topic -e` 텍스트를 파싱한다. 맥은 GZ_IP=127.0.0.1 필요.
통과 기준: 기울기 < 10°, xy 드리프트 < 10 cm, 네 발 이격 > 1 cm.
"""
import argparse
import math
import re
import subprocess
import sys
import tempfile
import time

LEGS = ("FL", "FR", "RL", "RR")
CALF_TO_FOOT = (0.0, 0.0, -0.10)  # calf 링크 원점(박스 중심) → 발끝


def capture(world: str, seconds: float) -> str:
    with tempfile.TemporaryFile("w+") as f:
        proc = subprocess.Popen(["gz", "topic", "-e", "-t", f"/world/{world}/pose/info"], stdout=f)
        time.sleep(seconds)
        proc.terminate()
        proc.wait()
        f.seek(0)
        return f.read()


def _block(body: str, name: str) -> dict:
    m = re.search(name + r" \{(.*?)\}", body, flags=re.S)
    return {k: float(v) for k, v in re.findall(r"(\w): ([-\de.+]+)", m.group(1))} if m else {}


def _pose(body: str) -> tuple:
    p, q = _block(body, "position"), _block(body, "orientation")  # proto 텍스트는 0인 필드를 생략한다
    return (p.get("x", 0.0), p.get("y", 0.0), p.get("z", 0.0)), (
        q.get("w", 1.0),
        q.get("x", 0.0),
        q.get("y", 0.0),
        q.get("z", 0.0),
    )


def _rotate(q: tuple, v: tuple) -> tuple:
    w, x, y, z = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + y * tz - z * ty, v[1] + w * ty + z * tx - x * tz, v[2] + w * tz + x * ty - y * tx)


def parse(text: str) -> list:
    """메시지마다 (x, y, z, tilt_deg, yaw_deg, {leg: 발끝 월드 z})."""
    rows = []
    for msg in text.split("header {")[1:]:
        poses = {m.group(1): _pose(m.group(2)) for m in re.finditer(r'pose \{\s*name: "(\w+)"(.*?)\n\}', msg, re.S)}
        if "go2" not in poses or any(f"{leg}_calf" not in poses for leg in LEGS):
            continue
        (bx, by, bz), bq = poses["go2"]
        w, x, y, z = bq
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
        feet = {}
        for leg in LEGS:
            cp, cq = poses[f"{leg}_calf"]  # 링크 포즈는 모델 기준
            foot = tuple(c + f for c, f in zip(cp, _rotate(cq, CALF_TO_FOOT)))
            feet[leg] = bz + _rotate(bq, foot)[2]
        rows.append((bx, by, bz, tilt, yaw, feet))
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="corridor")
    ap.add_argument("--duration", type=float, default=12.0)
    args = ap.parse_args()
    rows = parse(capture(args.world, args.duration))
    if len(rows) < 10:
        print(f"포즈 {len(rows)}건 — 시뮬이 떠 있는지, GZ_IP가 설정됐는지 확인")
        return 1
    tilt = max(r[3] for r in rows)
    drift = math.hypot(rows[-1][0] - rows[0][0], rows[-1][1] - rows[0][1])
    lift = {leg: max(r[5][leg] for r in rows) - min(r[5][leg] for r in rows) for leg in LEGS}
    print(
        f"n={len(rows)} tilt_max_deg={tilt:.2f} z={min(r[2] for r in rows):.3f}~{max(r[2] for r in rows):.3f} "
        f"xy_drift_cm={drift * 100:.1f} yaw_drift_deg={rows[-1][4] - rows[0][4]:.1f} "
        + " ".join(f"lift_{leg}_cm={v * 100:.1f}" for leg, v in lift.items())
    )
    ok = tilt < 10.0 and drift < 0.10 and min(lift.values()) > 0.01
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
