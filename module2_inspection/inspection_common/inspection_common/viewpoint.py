"""점검 위치 계산: 게이지를 화면에 담는 정차 위치, 과열 지점에 다가갈 위치."""

from dataclasses import dataclass
import math

from inspection_common.geometry import camera_xyz, circle_collision, segment_blocked, wrap
from inspection_common.world_map import CameraSpec, Gauge

APPROACH_OFFSETS_DEG = (0, 20, -20, 40, -40, 60, -60)


@dataclass(frozen=True)
class Viewpoint:
    name: str
    x: float
    y: float
    yaw: float
    distance: float             # 원판 앞면 → base_link 수평 거리
    dial_px: float              # 정위치에서 원판 지름(px)
    dial_px_far: float          # arrive_tol만큼 멀리 섰을 때
    ok: bool
    reason: str                 # "" | no_space:<장애물> | blocked:<장애물> | too_small


def gauge_viewpoint(g: Gauge, cam: CameraSpec, obstacles, body_height: float,
                    margin_deg: float = 5.0, arrive_tol: float = 0.4,
                    robot_radius: float = 0.35, min_dial_px: float = 20.0) -> Viewpoint:
    """원판 윗끝이 화면 위 가장자리(여유각 margin) 안에 드는 가장 가까운 정면 위치.

    arrive_tol만큼 더 가까이 서도 화면에 들어오도록 그만큼 물러선다.
    """
    cz = body_height + cam.z
    limit = cam.half_vfov - math.radians(margin_deg) - cam.pitch
    if limit <= 0:
        raise ValueError("여유각이 반화각보다 크다")
    cam_d = max(g.z + g.dial_r - cz, 0.0) / math.tan(limit) + arrive_tol
    dist = cam_d + cam.x
    ux, uy = math.cos(g.yaw), math.sin(g.yaw)
    face_x, face_y = g.x + ux * g.face, g.y + uy * g.face
    x, y = face_x + ux * dist, face_y + uy * dist
    yaw = wrap(g.yaw + math.pi)

    def px(d: float) -> float:
        return 2 * cam.fx * g.dial_r / math.hypot(d, g.z - cz)

    near, far = px(cam_d), px(cam_d + arrive_tol)
    others = [o for o in obstacles if not o.name.startswith(g.name + "/")]
    reason = ""
    hit = circle_collision(x, y, robot_radius, obstacles)
    if hit:
        reason = f"no_space:{hit}"
    else:
        cx, cy, _ = camera_xyz(x, y, yaw, cam, body_height)
        blk = segment_blocked(cx, cy, face_x, face_y, others, z_from=cz)
        if blk:
            reason = f"blocked:{blk}"
        elif far < min_dial_px:
            reason = "too_small"
    return Viewpoint(g.name, x, y, yaw, dist, near, far, reason == "", reason)


def approach_point(tx: float, ty: float, from_x: float, from_y: float, obstacles,
                   dist: float = 1.5, robot_radius: float = 0.35) -> tuple:
    """과열 지점 (tx, ty)를 처음 본 쪽에서 dist 떨어진 위치와 정면 yaw.

    막히면 ±20°, ±40°, ±60° 돌려 본다. 모두 안 되면 (None, 마지막 사유).
    """
    base = math.atan2(from_y - ty, from_x - tx)
    last = "no_space"
    for off in APPROACH_OFFSETS_DEG:
        a = base + math.radians(off)
        x, y = tx + dist * math.cos(a), ty + dist * math.sin(a)
        hit = circle_collision(x, y, robot_radius, obstacles)
        if hit:
            last = f"no_space:{hit}"
            continue
        ex, ey = tx + 0.05 * math.cos(a), ty + 0.05 * math.sin(a)   # 지점은 표면 위라 살짝 앞까지만 본다
        blk = segment_blocked(x, y, ex, ey, obstacles)
        if blk:
            last = f"blocked:{blk}"
            continue
        return (x, y, wrap(a + math.pi)), ""
    return None, last
