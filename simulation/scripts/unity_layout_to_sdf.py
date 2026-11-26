#!/usr/bin/env python3
"""Unity 발전소 배치 격자(DoosanGazeboLayoutExport) → Gazebo 월드 unity_plant.sdf + 순찰 웨이포인트.

좌표는 Unity Spot 시작점 기준 /odom(ROS) 프레임 — Gazebo 원점 = Unity 시작점이라 미러가 /odom을 그대로 쓴다.
바닥 높이(경사로·계단·낮은 적재물)는 높이 기둥 박스로, 벽·기계(바닥+0.3~2.3 m 점유)는 벽 박스로 재현한다.

사용: python3 simulation/scripts/unity_layout_to_sdf.py unity/config/gazebo_layout.json
"""
import argparse
import base64
import json
import sys
from pathlib import Path

import numpy as np

STEP = 0.02      # 높이 양자화(m) — 12° 경사로가 0.1 m 셀마다 약 2 cm 오르므로 그 정도의 계단으로 근사
FLAT = 0.02      # 이 높이 이하는 바닥판으로 충분
WALL_H = 3.0     # 벽·기계 박스 높이(m, 바닥 기준)
SPAWN_Z = 0.4    # 다른 월드와 같은 스폰 높이(바닥 위)
RISE = 0.03      # 얕은 구멍 판정: 양쪽 바닥이 이만큼 이상 높을 때
SIMILAR = 0.1    # 그 양쪽 바닥끼리의 높이 차 허용
MAX_GAP = 2      # 메울 구멍 폭(셀)


def load_grid(layout: dict):
    """JSON → (heights m 배열 [ny,nx], wall 마스크, void 마스크)."""
    raw = np.frombuffer(base64.b64decode(layout["data"]), dtype="<i2")
    ny, nx = layout["ny"], layout["nx"]
    if raw.size != nx * ny:
        raise ValueError(f"셀 수 불일치: data {raw.size} != nx*ny {nx * ny}")
    cells = raw.reshape(ny, nx)
    wall = cells == layout["wall"]
    void = cells == layout["void"]
    heights = np.where(wall | void, 0, cells).astype(float) / 1000.0
    return heights, wall, void


def _shift(a, k, axis):
    """out[i] = a[i+k] (axis 방향), 범위 밖은 NaN."""
    out = np.full(a.shape, np.nan)
    n = a.shape[axis]
    src, dst = [slice(None)] * a.ndim, [slice(None)] * a.ndim
    src[axis], dst[axis] = (slice(k, n), slice(0, n - k)) if k > 0 else (slice(0, n + k), slice(-k, n))
    out[tuple(dst)] = a[tuple(src)]
    return out


def fill_pits(heights, floor):
    """바닥 셀 중 x 또는 y 양쪽 MAX_GAP칸 안에 RISE 이상 높고 서로 비슷한 바닥이 있으면 그 평균으로 메운다.
    경사로 그레이팅 구멍 — 내보내기(DoosanGazeboLayoutExport.Bridge)는 판이 바닥+0.1 m 이상일 때만 메워서
    경사로 들머리(판 < 0.1 m)의 구멍이 0.1 m 셀 구덩이로 남는다. 한쪽만 높은 계단·적재물 모서리는 그대로."""
    h = np.where(floor, heights, np.nan)
    out = heights.copy()
    for axis in (0, 1):
        sides = []
        for s in (1, -1):
            v = np.full(h.shape, np.nan)
            for d in range(1, MAX_GAP + 1):
                cand = _shift(h, s * d, axis)
                take = np.isnan(v) & (cand >= h + RISE)
                v[take] = cand[take]
            sides.append(v)
        a, b = sides
        ok = floor & ~np.isnan(a) & ~np.isnan(b) & (np.abs(a - b) <= SIMILAR)
        out = np.where(ok, np.maximum(out, (a + b) / 2), out)
    return out


def classify(heights, wall, void):
    """(셀 클래스, 바닥판 높이 base). -1 = 벽(벽·빈 곳), 0 = 바닥판, k>0 = base 위 k*STEP 기둥.
    base는 가장 낮은 바닥(원점 0보다 낮은 구덩이도 평지로 뭉개지 않게)."""
    floor = ~(wall | void)
    base = min(0.0, float(heights[floor].min())) if floor.any() else 0.0
    cls = np.rint((heights - base) / STEP).astype(int)
    cls[cls * STEP <= FLAT] = 0
    cls[~floor] = -1
    return cls, base


def merge_rects(cls, value):
    """같은 클래스 셀을 축정렬 직사각형으로 탐욕 병합. (i0, j0, w, h) 목록 — 합집합이 정확히 그 셀들."""
    todo = cls == value
    rects = []
    ny, nx = todo.shape
    for j in range(ny):
        i = 0
        while i < nx:
            if not todo[j, i]:
                i += 1
                continue
            w = 1
            while i + w < nx and todo[j, i + w]:
                w += 1
            h = 1
            while j + h < ny and todo[j + h, i:i + w].all():
                h += 1
            todo[j:j + h, i:i + w] = False
            rects.append((i, j, w, h))
            i += w
    return rects


def _box(name, x, y, z, sx, sy, sz, rgb):
    """링크 하나에 들어갈 collision+visual 한 쌍. 박스마다 모델을 따로 두면 1천여 개 엔티티로 RTF가 0.3까지 떨어진다."""
    color = " ".join(f"{c:.2f}" for c in rgb)
    pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
    geo = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    return (f'        <collision name="{name}">{pose}{geo}</collision>'
            f'<visual name="{name}">{pose}{geo}<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>')


def world_sdf(layout: dict, heights, cls, base: float = 0.0) -> str:
    res, ox, oy = layout["resolution"], layout["origin_x"], layout["origin_y"]
    ny, nx = cls.shape

    def center(i, j, w, h):
        return ox + (i + w / 2) * res, oy + (j + h / 2) * res, w * res, h * res

    parts = []
    x, y, sx, sy = center(0, 0, nx, ny)
    parts.append(_box("floor", x, y, base - 0.05, sx, sy, 0.1, (0.45, 0.45, 0.48)))
    for n, (i, j, w, h) in enumerate(merge_rects(cls, -1)):
        x, y, sx, sy = center(i, j, w, h)
        parts.append(_box(f"wall_{n}", x, y, base + WALL_H / 2, sx, sy, WALL_H, (0.72, 0.74, 0.78)))
    n = 0
    for k in sorted(set(cls[cls > 0].tolist())):
        hz = k * STEP
        for i, j, w, h in merge_rects(cls, k):
            x, y, sx, sy = center(i, j, w, h)
            parts.append(_box(f"step_{n}", x, y, base + hz / 2, sx, sy, hz, (0.55, 0.52, 0.48)))
            n += 1

    i0, j0 = int((0 - ox) / res), int((0 - oy) / res)
    spawn_z = (heights[j0, i0] if 0 <= i0 < nx and 0 <= j0 < ny else 0.0) + SPAWN_Z
    body = "\n".join(parts)
    return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <!-- Unity 발전소(Plant_GazeboMirror) 배치에서 생성 — simulation/scripts/unity_layout_to_sdf.py. 손으로 고치지 않는다. -->
  <!-- 원점 = Unity Spot 시작점, x = 시작 방향(/odom). 격자 {nx}×{ny} @ {res} m -->
  <world name="unity_plant">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <atmosphere type="adiabatic"/>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>

    <model name="plant">
      <static>true</static>
      <link name="link">
{body}
      </link>
    </model>

    <include>
      <uri>model://go2</uri>
      <name>go2</name>
      <pose>0 0 {spawn_z:.3f} 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def waypoints_yaml(route: dict) -> str:
    """unity/config/inspection_route.json(odom 프레임) → patrol_path waypoints yaml."""
    lines = ["# Unity 발전소 순찰 경로(unity/config/inspection_route.json, /odom = Unity Spot 시작점) — unity_layout_to_sdf.py 생성",
             "waypoints:"]
    for w in route["waypoints"]:
        lines.append(f"  - {{name: {w['name']}, x: {w['x']:.2f}, y: {w['y']:.2f}, "
                     f"yaw: {float(w.get('yaw', 0.0)):.3f}, dwell_s: {float(w.get('dwell_s', 0.0)):.1f}}}")
    return "\n".join(lines) + "\n"


def main(argv=None) -> int:
    root = Path(__file__).resolve().parents[2]
    ap = argparse.ArgumentParser()
    ap.add_argument("layout", type=Path)
    ap.add_argument("--route", type=Path, default=root / "unity/config/inspection_route.json")
    ap.add_argument("--world-out", type=Path, default=root / "simulation/worlds/unity_plant.sdf")
    ap.add_argument("--waypoints-out", type=Path, default=root / "module2_inspection/patrol_path/config/waypoints_unity_plant.yaml")
    a = ap.parse_args(argv)

    layout = json.loads(a.layout.read_text())
    heights, wall, void = load_grid(layout)
    filled = fill_pits(heights, ~(wall | void))
    pits = int((filled > heights).sum())
    heights = filled
    cls, base = classify(heights, wall, void)
    a.world_out.write_text(world_sdf(layout, heights, cls, base))
    a.waypoints_out.write_text(waypoints_yaml(json.loads(a.route.read_text())))
    raised = cls > 0
    print(f"grid {layout['nx']}x{layout['ny']} @ {layout['resolution']} m | 벽 {int((cls == -1).sum())} 셀 "
          f"| 높이 셀 {int(raised.sum())} (최대 {heights[raised].max() if raised.any() else 0:.2f} m, 바닥 최저 {heights[~(wall | void)].min():.2f} m) "
          f"| 메운 구멍 {pits} 셀 | 박스 {a.world_out.read_text().count('<collision ')} → {a.world_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
