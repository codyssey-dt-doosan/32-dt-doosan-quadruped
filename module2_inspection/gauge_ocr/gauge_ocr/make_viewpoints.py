"""게이지 점검 위치를 계산해 순찰 waypoints YAML 형식으로 출력한다.

    ros2 run gauge_ocr make_viewpoints --world factory
    python -m gauge_ocr.make_viewpoints --world factory --sim-share <저장소>/simulation   # ROS 없이
"""

import argparse
import sys

from inspection_common.viewpoint import gauge_viewpoint
from inspection_common.world_map import body_height_for, load_world, resolve_world


def compute_viewpoints(wm, body_height: float, **kw) -> list:
    return [gauge_viewpoint(g, wm.camera, wm.obstacles, body_height, **kw) for g in wm.gauges]


def to_yaml(vps: list, dwell_s: float) -> str:
    lines = ["  # make_viewpoints 결과 (순찰 waypoints에 붙여 넣기 전 태우 님 검토)"]
    for v in vps:
        if v.ok:
            lines.append(
                f"  - {{name: {v.name}, x: {v.x:.2f}, y: {v.y:.2f}, yaw: {v.yaw:.2f}, dwell_s: {dwell_s:.1f}}}"
                f"  # 원판 {v.dial_px:.0f}~{v.dial_px_far:.0f} px"
            )
        else:
            lines.append(f"  # {v.name}: 불가 ({v.reason})")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="게이지 점검 위치 계산")
    ap.add_argument("--world", default="factory")
    ap.add_argument("--sim-share", default="", help="simulation 폴더 (없으면 설치된 패키지)")
    ap.add_argument("--locomotion", default="velocity", choices=("velocity", "legged"))
    ap.add_argument("--margin-deg", type=float, default=5.0)
    ap.add_argument("--arrive-tol", type=float, default=0.4)
    ap.add_argument("--robot-radius", type=float, default=0.35)
    ap.add_argument("--min-dial-px", type=float, default=20.0)
    ap.add_argument("--dwell", type=float, default=5.0)
    a = ap.parse_args(argv)
    wm = load_world(*resolve_world(a.world, sim_share=a.sim_share))
    vps = compute_viewpoints(
        wm, body_height_for(a.locomotion), margin_deg=a.margin_deg, arrive_tol=a.arrive_tol,
        robot_radius=a.robot_radius, min_dial_px=a.min_dial_px,
    )
    sys.stdout.write(to_yaml(vps, a.dwell))
    return 0 if all(v.ok for v in vps) else 1


if __name__ == "__main__":
    sys.exit(main())
