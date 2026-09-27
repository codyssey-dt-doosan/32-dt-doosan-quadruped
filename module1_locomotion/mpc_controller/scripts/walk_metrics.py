#!/usr/bin/env python3
"""지평 MPC 보행 지표. walk.launch.py(gui:=false)가 뜨고 기동(≈6 s)이 끝난 뒤 실행.

  --cmd V W [S]     : (v, w)를 20 Hz로 S초 발행하며 pose/info 캡처 → 속도 추종·기울기·z 표준편차 (기준 1)
  --record F.csv S  : /cmd_vel을 S초 기록 (sim 시각 기준, 기본 구동 full_system 순찰 중 실행)
  --replay F.csv    : 기록을 sim 시각에 맞춰 재발행 → 추종 RMS·종점 오차 (기준 2)
  --slope DEG LEN   : x 구간(오르막/평지/내리막)별 속도·pitch·최대 기울기·횡 드리프트 (기준 3~5, --cmd와 같이)
판정 공통: hold 0, fail(첫 3틱 제외) 0, sat ≤ 50, mpc 평균 < 5 ms·p99 < 10 ms, 전도(tilt > 45°) 0.
"""
import argparse
import csv
import math
import re
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import balance_metrics as bm  # noqa: E402
import trot_metrics as tm  # noqa: E402

SPAWN_X = -20.0
MPC_WARMUP_FAILS = 3
SAT_MAX = 50  # 1 kHz 틱 0.25%: 복합 최대 명령(0.5, 1.0)에서 27틱, 낙상·hold 무관(실측)


def parse_rows(text: str) -> list:
    return bm.parse_feet(text)


def _unwrap_deg(a: list) -> list:
    out = [a[0]]
    for v in a[1:]:
        d = v - out[-1]
        out.append(out[-1] + (d + 180.0) % 360.0 - 180.0)
    return out


def velocity_track(rows: list, v: float, w: float, after: float) -> dict:
    """행 (x, y, z, tilt, pitch, yaw, feet, t). after초 이후 몸체 헤딩 방향 속도·yaw 속도 평균."""
    t0 = rows[0][7]
    r = [q for q in rows if q[7] - t0 >= after]
    yaw = _unwrap_deg([q[5] for q in r])
    vs, ws = [], []
    for a, b, ya, yb in zip(r[:-1], r[1:], yaw[:-1], yaw[1:]):
        dt = b[7] - a[7]
        if dt <= 0:
            continue
        h = math.radians(ya)
        vs.append(((b[0] - a[0]) * math.cos(h) + (b[1] - a[1]) * math.sin(h)) / dt)
        ws.append(math.radians(yb - ya) / dt)
    z = [q[2] for q in r]
    zm = sum(z) / len(z)
    v_meas, w_meas = sum(vs) / len(vs), sum(ws) / len(ws)
    return {
        "v_meas": v_meas, "w_meas": w_meas,
        "err_v": abs(v_meas - v) / max(abs(v), 1e-9) if abs(v) > 1e-9 else abs(v_meas),
        "err_w": abs(w_meas - w) / max(abs(w), 1e-9) if abs(w) > 1e-9 else abs(w_meas),
        "tilt_max": max(q[3] for q in r), "z_std": (sum((a - zm) ** 2 for a in z) / len(z)) ** 0.5,
    }


def log_stats(path: str) -> dict:
    st = bm.log_stats(path)
    lines = [l for l in Path(path).read_text(encoding="utf-8", errors="ignore").splitlines() if "status mode=" in l]
    if lines:
        m = re.search(r"ik_fail=(\d+) ik_clamp=(\d+) phase=([\d.]+) active=(\d)", lines[-1])
        st.update({"ik_fail": int(m.group(1)), "ik_clamp": int(m.group(2)), "active": int(m.group(4))})
    return st


def _publish(v: float, w: float, seconds: float) -> None:
    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("walk_cmd")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    msg = Twist()
    msg.linear.x, msg.angular.z = float(v), float(w)
    end = time.time() + seconds
    while time.time() < end:
        pub.publish(msg)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()


def _record(path: str, seconds: float) -> None:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter

    rclpy.init()
    node = rclpy.create_node("walk_rec")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    rows, t0 = [], [None]

    def cb(m):
        t = node.get_clock().now().nanoseconds * 1e-9
        t0[0] = t0[0] if t0[0] is not None else t
        rows.append((t - t0[0], m.linear.x, m.angular.z))

    node.create_subscription(Twist, "/cmd_vel", cb, 10)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    with open(path, "w", newline="") as f:
        csv.writer(f).writerows(rows)
    print(f"recorded {len(rows)} rows → {path}")
    node.destroy_node()
    rclpy.shutdown()


def _replay(path: str, start_sim: list) -> None:
    """CSV (t, v, w)를 sim 시각에 맞춰 재발행. start_sim[0]에 첫 발행 sim 시각을 남긴다."""
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter

    rows = [(float(t), float(v), float(w)) for t, v, w in csv.reader(open(path))]
    rclpy.init()
    node = rclpy.create_node("walk_replay")
    node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    while node.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(node, timeout_sec=0.05)
    t0 = node.get_clock().now().nanoseconds * 1e-9
    start_sim[0] = t0
    for t, v, w in rows:
        while node.get_clock().now().nanoseconds * 1e-9 < t0 + t:
            rclpy.spin_once(node, timeout_sec=0.005)
        m = Twist()
        m.linear.x, m.angular.z = v, w
        pub.publish(m)
    node.destroy_node()
    rclpy.shutdown()


def replay_errors(rows: list, cmd_path: str, t_start: float) -> dict:
    """추종 RMS(v, w)·종점 오차/경로 길이. 명령은 sim 시각 t_start + t."""
    cmd = [(float(t) + t_start, float(v), float(w)) for t, v, w in csv.reader(open(cmd_path))]
    r = [q for q in rows if q[7] >= t_start]
    yaw = _unwrap_deg([q[5] for q in r])

    def cmd_at(t):
        prev = (0.0, 0.0)
        for ct, v, w in cmd:
            if ct > t:
                break
            prev = (v, w)
        return prev

    ev, ew, n = 0.0, 0.0, 0
    for a, b, ya, yb in zip(r[:-1], r[1:], yaw[:-1], yaw[1:]):
        dt = b[7] - a[7]
        if dt <= 0:
            continue
        h = math.radians(ya)
        vm = ((b[0] - a[0]) * math.cos(h) + (b[1] - a[1]) * math.sin(h)) / dt
        wm = math.radians(yb - ya) / dt
        v, w = cmd_at(a[7])
        ev, ew, n = ev + (vm - v) ** 2, ew + (wm - w) ** 2, n + 1
    x, y, th, length = r[0][0], r[0][1], math.radians(r[0][5]), 0.0
    for (ta, v, w), (tb, _, _) in zip(cmd[:-1], cmd[1:]):
        dt = tb - ta
        x, y, th, length = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt, length + abs(v) * dt
    end_err = math.hypot(r[-1][0] - x, r[-1][1] - y)
    return {"rms_v": (ev / n) ** 0.5, "rms_w": (ew / n) ** 0.5, "end_err": end_err, "path_len": length}


def slope_segments(rows: list, deg: float, L: float) -> dict:
    th = math.radians(deg)
    x0 = SPAWN_X + 2.0
    x1, x2, x3 = x0 + L * math.cos(th), x0 + L * math.cos(th) + 2.0, x0 + 2 * L * math.cos(th) + 2.0
    out = {"completed": max(q[0] for q in rows) >= x3 + 0.3, "tilt_max": max(q[3] for q in rows), "y_max": max(abs(q[1]) for q in rows)}
    for name, lo, hi, exp_pitch in (("up", x0, x1, -deg), ("flat", x1, x2, 0.0), ("down", x2, x3, deg)):
        seg = [q for q in rows if lo + 0.2 <= q[0] < hi - 0.2]
        if len(seg) < 2:
            out[name] = None
            continue
        dt = seg[-1][7] - seg[0][7]
        out[name] = {"v": (seg[-1][0] - seg[0][0]) / max(dt, 1e-6), "pitch_err": sum(q[4] for q in seg) / len(seg) - exp_pitch,
                     "tilt_max": max(q[3] for q in seg), "v_max": max((b[0] - a[0]) / max(b[7] - a[7], 1e-6) for a, b in zip(seg[:-1], seg[1:]))}
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="corridor")
    ap.add_argument("--log", required=True)
    ap.add_argument("--after", type=float, default=2.0)
    ap.add_argument("--cmd", nargs="+", type=float, metavar="V W [S]")
    ap.add_argument("--record", nargs=2, metavar=("F", "S"))
    ap.add_argument("--replay", metavar="F")
    ap.add_argument("--slope", nargs=2, type=float, metavar=("DEG", "LEN"))
    a = ap.parse_args()
    if a.record:
        _record(a.record[0], float(a.record[1]))
        return 0
    start_sim = [None]
    if a.cmd:
        v, w = a.cmd[0], a.cmd[1]
        dur = a.cmd[2] if len(a.cmd) > 2 else 15.0
        threading.Thread(target=_publish, args=(v, w, dur + 1.0), daemon=True).start()
    elif a.replay:
        rows_cmd = list(csv.reader(open(a.replay)))
        dur = float(rows_cmd[-1][0]) + 2.0
        threading.Thread(target=_replay, args=(a.replay, start_sim), daemon=True).start()
    else:
        dur = 15.0
    rows = parse_rows(tm.capture(a.world, dur))
    if len(rows) < 10:
        print(f"포즈 {len(rows)}건 — 시뮬·GZ_IP 확인")
        return 1
    st = log_stats(a.log)
    fell = any(q[3] > 45.0 for q in rows)
    ok = st.get("mode") == "run" and st.get("hold", 1) == 0 and st.get("fail", 99) <= MPC_WARMUP_FAILS and st.get("sat", 99) <= SAT_MAX
    ok &= st.get("qp_ms", 9) < 5.0 and st.get("qp_p99", 99) < 10.0 and not fell
    out = f"n={len(rows)} fell={fell} " + " ".join(f"{k}={v}" for k, v in st.items())
    if a.cmd:
        m = velocity_track(rows, v, w, a.after)
        out += " " + " ".join(f"{k}={val:.3f}" for k, val in m.items())
        if a.slope:
            s = slope_segments(rows, *a.slope)
            out += f" slope={s}"
            ok &= s["completed"] and s["y_max"] < 0.3
            ok &= all(seg is None or abs(seg["pitch_err"]) < 5.0 for seg in (s["up"], s["down"]))
            min_up = 0.15 if a.slope[0] >= 20 else 0.05  # 기준 3은 20°에서만 속도 하한
            ok &= s["up"] is not None and s["up"]["v"] >= min_up
            ok &= s["down"] is None or (s["down"]["tilt_max"] < 20.0 and s["down"]["v_max"] <= 0.4)
        else:
            ok &= m["err_v"] < 0.10 and m["err_w"] < 0.10 and m["tilt_max"] < 3.0 and m["z_std"] < 0.01
    elif a.replay:
        while start_sim[0] is None:
            time.sleep(0.1)
        e = replay_errors(rows, a.replay, start_sim[0])
        out += " " + " ".join(f"{k}={val:.3f}" for k, val in e.items())
        ok &= e["rms_v"] < 0.05 and e["rms_w"] < 0.10 and e["end_err"] < 0.10 * e["path_len"]
    print(out)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
