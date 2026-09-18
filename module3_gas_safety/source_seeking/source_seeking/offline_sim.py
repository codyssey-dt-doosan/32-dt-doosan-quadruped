"""Gazebo 없이 module3 전체(플룸 + 탐색 + 배터리 + 복귀)를 2D로 돌려 보는 시뮬레이터.

ROS 노드들과 같은 코어(GasField, SeekerCore, ReturnManager, GoalDriver)를 쓰고, 월드의 벽·장애물은
world_<world>.yaml의 박스로 근사한다. 결과는 <out>.csv / .json / .png.

  ros2 run source_seeking offline_sim --world factory
  ros2 run source_seeking offline_sim --world factory --mode gradient     # 국소 최대에 갇혔다가 탈출
  ros2 run source_seeking offline_sim --world factory --battery 30         # 배터리 부족 복귀
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass, field, replace

from plume_sim.plume import GasField, GasFieldParams, load_params
from return_to_home.battery import BatteryModel, BatteryParams
from return_to_home.drive import DriveParams, GoalDriver
from return_to_home.home_path import ReturnManager, ReturnParams
from source_seeking.run_log import RunLog
from source_seeking.seeker import SeekerCore, SeekerParams


@dataclass
class WorldParams:
    start_x: float = -10.0
    start_y: float = 0.0
    start_yaw: float = 0.0
    robot_radius: float = 0.35
    dt: float = 0.1
    duration: float = 1500.0
    obstacles: list = field(default_factory=list)   # [xmin, ymin, xmax, ymax, ...]


def config_path(pkg: str, name: str) -> str:
    """설치된 share 디렉터리, 없으면 소스 트리에서 찾는다."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory(pkg), "config", name)
        if os.path.exists(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(__file__), "..", "..", pkg, "config", name)


def boxes(flat):
    return [tuple(flat[i:i + 4]) for i in range(0, len(flat) - len(flat) % 4, 4)]


def collides(x, y, r, obstacles):
    for (x0, y0, x1, y1) in obstacles:
        cx, cy = min(max(x, x0), x1), min(max(y, y0), y1)
        if (x - cx) ** 2 + (y - cy) ** 2 < r * r:
            return True
    return False


def load_all(world: str):
    plume = config_path("plume_sim", f"plume_{world}.yaml")
    seek = config_path("source_seeking", "source_seeking.yaml")
    wfile = config_path("source_seeking", f"world_{world}.yaml")
    ret = config_path("return_to_home", "return_to_home.yaml")
    sp = load_params(seek, "source_seeking", SeekerParams)
    sp = replace(sp, bounds=load_params(wfile, "source_seeking", SeekerParams).bounds)
    return dict(
        gas=load_params(plume, "plume_sim", GasFieldParams),
        seeker=sp,
        drive=load_params(seek, "source_seeking", DriveParams),
        ret=load_params(ret, "return_to_home", ReturnParams),
        battery=load_params(ret, "battery_sim", BatteryParams),
        world=load_params(wfile, "offline_sim", WorldParams),
    )


def run(world="factory", mode=None, battery=None, seed=None, out="offline_run"):
    cfg = load_all(world)
    sp, gp, bp = cfg["seeker"], cfg["gas"], cfg["battery"]
    if mode:
        sp = replace(sp, mode=mode)
    if battery is not None:
        bp = replace(bp, initial_percent=battery)
    if seed is not None:
        gp = replace(gp, seed=seed)
    wp = cfg["world"]
    gas, batt = GasField(gp), BatteryModel(bp)
    seeker, ret, drv = SeekerCore(sp), ReturnManager(cfg["ret"]), GoalDriver(cfg["drive"])
    obst = boxes(wp.obstacles)
    log = RunLog(out)

    x, y, yaw, v, w = wp.start_x, wp.start_y, wp.start_yaw, 0.0, 0.0
    t = 0.0
    for _ in range(int(wp.duration / wp.dt)):
        b = batt.update(t, x, y)
        c = gas.sample(x, y)
        # 노드 간 토픽 흐름과 같은 순서: return_to_home이 복귀를 시작하면 source_seeking은 멈춘다
        ret.update(t, x, y, b, c, seeker_state=seeker.state)
        if ret.state == "IDLE":
            target = seeker.update(t, x, y, yaw, c, gas.wind())
            owner = seeker
        else:
            target = ret.target()
            owner = ret
        log.row(t, x, y, yaw, c, seeker, ret, b)
        if ret.state == "HOME" or b <= 0.0:
            break
        if drv.blocked(t, x, y, yaw, target):
            owner.on_blocked()
            drv.start_recovery(t)
        cmd_v, cmd_w = drv.command(t, x, y, yaw, target)
        a = wp.dt / 0.4                 # 1차 속도 응답
        v += a * (cmd_v - v)
        w += a * (cmd_w - w)
        yaw = math.atan2(math.sin(yaw + w * wp.dt), math.cos(yaw + w * wp.dt))
        nx, ny = x + v * math.cos(yaw) * wp.dt, y + v * math.sin(yaw) * wp.dt
        if not collides(nx, ny, wp.robot_radius, obst):
            x, y = nx, ny
        t += wp.dt
    log.close(seeker, ret, extra={"world": world, "mode": sp.mode, "initial_battery": bp.initial_percent,
                                  "true_source": [gp.source_x, gp.source_y]})
    return seeker, ret, log


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="factory", choices=["factory", "corridor"])
    ap.add_argument("--mode", choices=["hybrid", "gradient"])
    ap.add_argument("--battery", type=float, help="시작 배터리 %%")
    ap.add_argument("--seed", type=int)
    ap.add_argument("--out", default="offline_run", help="출력 접두사 (.csv/.json/.png)")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    seeker, ret, log = run(a.world, a.mode, a.battery, a.seed, a.out)
    for (t, s, r) in log.events(seeker, ret):
        print("%7.1f s  %-13s %s" % (t, s, r))
    if not a.no_plot:
        from source_seeking.plot_run import plot
        print("plot:", plot(a.out, a.world))


if __name__ == "__main__":
    main()
