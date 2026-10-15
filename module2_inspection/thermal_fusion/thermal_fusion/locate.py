"""열화상 픽셀 → 맵 위 과열 지점. 2D 광선 + 높이 확인 + 바닥 제외.

LiDAR(/points)로 바꿀 때는 같은 인자·반환으로 함수를 하나 더 만들어 노드에서 바꿔 부른다.
"""

from dataclasses import dataclass
import math

from inspection_common.geometry import camera_xyz, pixel_angles, ray_hits


@dataclass(frozen=True)
class Hit:
    x: float
    y: float
    z: float
    dist: float                 # 카메라 → 지점 수평 거리
    depth: float                # 광축 방향 거리 (정렬 호모그래피용)
    name: str


def locate(u: float, v: float, rx: float, ry: float, ryaw: float, cam, body_height: float,
           obstacles, max_range: float = 15.0) -> tuple:
    az, el = pixel_angles(u, v, cam)
    cx, cy, cz = camera_xyz(rx, ry, ryaw, cam, body_height)
    ang = ryaw + az
    ground = cz / math.tan(-el) if el < 0 else math.inf     # 수평선 아래면 바닥에 닿는 거리
    for d, o in ray_hits(cx, cy, ang, obstacles, max_range):
        if d > ground:
            break
        z = cz + d * math.tan(el)
        if o.z_min <= z <= o.z_max:                          # 장애물 위·아래로 지나가면 다음 것
            return Hit(cx + d * math.cos(ang), cy + d * math.sin(ang), z, d, d * math.cos(az), o.name), ""
    if ground <= max_range:
        return None, "floor"
    return None, "no_hit"
