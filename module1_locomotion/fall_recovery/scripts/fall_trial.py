#!/usr/bin/env python3
"""전도 회복 성공률 측정. full_system.launch.py(gui:=false)가 떠 있는 상태에서 실행.

각 trial: 로봇을 현재 위치에서 roll 100°로 눕힘(set_pose) → status가 fallen → recovered 되는지 대기.
"""

import argparse
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from fall_recovery.fall_recovery_node import set_pose_cmd


class Trial(Node):
    def __init__(self) -> None:
        super().__init__("fall_trial")
        self.status = None
        self.odom = None
        self.create_subscription(String, "/fall_recovery/status", lambda m: setattr(self, "status", m.data), 10)
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), 10)

    def wait_status(self, want: set[str], timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status in want:
                return True
        return False

    def tip_over(self, world: str, roll_deg: float) -> bool:
        p = self.odom.pose.pose.position
        h = math.radians(roll_deg) / 2
        req = (
            f'name: "go2", position: {{x: {p.x}, y: {p.y}, z: 0.5}}, '
            f"orientation: {{x: {math.sin(h):.4f}, y: 0.0, z: 0.0, w: {math.cos(h):.4f}}}"
        )
        r = subprocess.run(set_pose_cmd(world, req), capture_output=True, text=True, timeout=2.0)
        return r.returncode == 0 and "true" in r.stdout.lower()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=5)
    ap.add_argument("--world", default="corridor")
    ap.add_argument("--roll", type=float, default=100.0)
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--gap", type=float, default=5.0, help="trial 사이 대기(s), recovered→idle 2 s 포함")
    a = ap.parse_args()

    rclpy.init()
    n = Trial()
    assert n.wait_status({"idle"}, 30.0), "fall_recovery idle 대기 실패 (노드 안 떠 있음?)"
    while n.odom is None:
        rclpy.spin_once(n, timeout_sec=0.1)

    ok, times = 0, []
    for i in range(1, a.trials + 1):
        assert n.tip_over(a.world, a.roll), "set_pose 실패"
        t0 = time.monotonic()
        fell = n.wait_status({"fallen"}, 5.0)
        rec = fell and n.wait_status({"recovered"}, a.timeout)
        dt = time.monotonic() - t0
        print(f"trial {i}: fallen={fell} recovered={rec} {dt:.1f}s status={n.status}")
        if rec:
            ok += 1
            times.append(dt)
        n.wait_status({"idle"}, a.gap + 5.0)
        time.sleep(1.0)
    mean = sum(times) / len(times) if times else float("nan")
    print(f"recovered {ok}/{a.trials}, mean {mean:.1f} s")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
