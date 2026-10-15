"""지금 판독 중인 게이지 이름: 순찰 상태 → 가까운 점검 위치 → unknown."""

import math


def current_gauge_name(status: str, robot_xy, viewpoints, radius: float = 1.0) -> str:
    if status.startswith("dwell:"):
        return status[len("dwell:"):]
    if robot_xy is not None:
        near = [(math.hypot(v.x - robot_xy[0], v.y - robot_xy[1]), v.name) for v in viewpoints if v.ok]
        if near and min(near)[0] <= radius:
            return min(near)[1]
    return "unknown"
