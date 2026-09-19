#!/usr/bin/env python3
"""순찰 한 바퀴 지표. full_system.launch.py(gui:=false)가 떠 있는 상태에서 실행.

home 출발 → 다음 home 도착(= /mission/status가 goto:home으로 두 번째 진입)까지:
lap_s(완주 시간), mean_abs_w(평균 |w|, 요동), min_obst_m(장애물 중심까지 최소 거리 − 반경).
막힘 횟수는 런치 로그에서 `grep -c '경로 막힘'`.
"""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# (x, y, 반경). obstacle 박스 0.6×0.4 → 외접 반경 0.36, gas_tank 실린더 0.35
OBSTACLES = {
    "corridor": [(8.0, 0.6, 0.36), (-6.0, -0.5, 0.36)],
    "factory": [(6.0, 4.0, 0.36), (-5.0, -3.0, 0.36), (8.0, -6.0, 0.35), (10.0, -6.0, 0.35)],
}


class Metrics(Node):
    def __init__(self, world: str) -> None:
        super().__init__("patrol_metrics")
        self.obst = OBSTACLES[world]
        self.status = None
        self.pos = None
        self.w_abs: list[float] = []
        self.min_obst = math.inf
        self.recording = False
        self.create_subscription(String, "/mission/status", self.status_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 10)

    def status_cb(self, m: String) -> None:
        self.status = m.data

    def odom_cb(self, m: Odometry) -> None:
        p = m.pose.pose.position
        self.pos = (round(p.x, 2), round(p.y, 2))
        if self.recording:
            for ox, oy, r in self.obst:
                self.min_obst = min(self.min_obst, math.hypot(p.x - ox, p.y - oy) - r)

    def cmd_cb(self, m: Twist) -> None:
        if self.recording:
            self.w_abs.append(abs(m.angular.z))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default="corridor")
    ap.add_argument("--timeout", type=float, default=600.0)
    a = ap.parse_args()

    rclpy.init()
    n = Metrics(a.world)
    t_start = time.monotonic()
    # 0) home 이외 상태 대기(출발 확인) → 1) goto:home 진입 = 랩 시작 → 2) home 벗어남 → 3) 다시 goto:home = 랩 끝
    phase = 0
    lap_t0 = 0.0
    while time.monotonic() - t_start < a.timeout:
        rclpy.spin_once(n, timeout_sec=0.1)
        s = n.status
        if s is None:
            continue
        if phase == 0 and s != "goto:home":
            phase = 1
        elif phase == 1 and s == "goto:home":
            phase = 2
            lap_t0 = time.monotonic()
            n.recording = True
            print(f"lap start at {n.pos}", flush=True)
        elif phase == 2 and s != "goto:home":
            phase = 3
        elif phase == 3 and s == "goto:home":
            lap = time.monotonic() - lap_t0
            mean_w = sum(n.w_abs) / len(n.w_abs) if n.w_abs else float("nan")
            print(
                f"world={a.world} lap_s={lap:.1f} mean_abs_w={mean_w:.3f} min_obst_m={n.min_obst:.2f}",
                flush=True,
            )
            break
    else:
        print(f"timeout: phase={phase} status={n.status} pos={n.pos}", flush=True)
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
