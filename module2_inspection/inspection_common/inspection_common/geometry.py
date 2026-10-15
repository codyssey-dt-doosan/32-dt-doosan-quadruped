"""2D 기하: 로봇 몸체 충돌, 시야 가림, 광선 쏘기, 픽셀 → 각도. 거리는 모두 수평(m)."""

import math

from inspection_common.world_map import CameraSpec, Obstacle


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _to_local(o: Obstacle, x: float, y: float) -> tuple:
    dx, dy = x - o.x, y - o.y
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    return c * dx + s * dy, -s * dx + c * dy


def point_distance(o: Obstacle, x: float, y: float) -> float:
    """점에서 장애물 바닥 모양까지 거리. 안이면 0."""
    if o.kind == "cylinder":
        return max(0.0, math.hypot(x - o.x, y - o.y) - o.r)
    lx, ly = _to_local(o, x, y)
    return math.hypot(max(abs(lx) - o.sx / 2, 0.0), max(abs(ly) - o.sy / 2, 0.0))


def circle_collision(x: float, y: float, radius: float, obstacles) -> str | None:
    for o in obstacles:
        if point_distance(o, x, y) < radius:
            return o.name
    return None


def _ray_enter(o: Obstacle, x: float, y: float, dx: float, dy: float) -> float | None:
    """반직선이 장애물에 처음 들어가는 거리. 시작점이 안이면 0."""
    if o.kind == "cylinder":
        fx, fy = x - o.x, y - o.y
        b = fx * dx + fy * dy
        disc = b * b - (fx * fx + fy * fy - o.r * o.r)
        if disc < 0:
            return None
        t0, t1 = -b - math.sqrt(disc), -b + math.sqrt(disc)
        return None if t1 < 0 else max(t0, 0.0)
    lx, ly = _to_local(o, x, y)
    c, s = math.cos(o.yaw), math.sin(o.yaw)
    ldx, ldy = c * dx + s * dy, -s * dx + c * dy
    t_near, t_far = -math.inf, math.inf
    for p, d, half in ((lx, ldx, o.sx / 2), (ly, ldy, o.sy / 2)):          # slab 방식
        if abs(d) < 1e-12:
            if abs(p) > half:
                return None
            continue
        ta, tb = (-half - p) / d, (half - p) / d
        t_near, t_far = max(t_near, min(ta, tb)), min(t_far, max(ta, tb))
    if t_near > t_far or t_far < 0:
        return None
    return max(t_near, 0.0)


def ray_hits(x: float, y: float, angle: float, obstacles, max_range: float = 50.0) -> list:
    dx, dy = math.cos(angle), math.sin(angle)
    hits = []
    for o in obstacles:
        t = _ray_enter(o, x, y, dx, dy)
        if t is not None and t <= max_range:
            hits.append((t, o))
    hits.sort(key=lambda h: h[0])
    return hits


def segment_blocked(x0: float, y0: float, x1: float, y1: float, obstacles,
                    z_from: float = -math.inf) -> str | None:
    """두 점 사이를 가리는 장애물 이름. 윗면이 z_from 이하인 낮은 장애물은 무시."""
    length = math.hypot(x1 - x0, y1 - y0)
    if length == 0:
        return None
    for _, o in ray_hits(x0, y0, math.atan2(y1 - y0, x1 - x0), obstacles, length):
        if o.z_max > z_from:
            return o.name
    return None


def camera_xyz(rx: float, ry: float, ryaw: float, cam: CameraSpec, body_height: float) -> tuple:
    c, s = math.cos(ryaw), math.sin(ryaw)
    return rx + c * cam.x - s * cam.y, ry + s * cam.x + c * cam.y, body_height + cam.z


def pixel_angles(u: float, v: float, cam: CameraSpec) -> tuple:
    """픽셀 → (수평각 왼쪽 +, 올려본각 위 +) rad. 주점 = 이미지 중앙."""
    du, dv = cam.width / 2 - u, cam.height / 2 - v
    return math.atan2(du, cam.fx), math.atan2(dv, math.hypot(cam.fx, du)) - cam.pitch
