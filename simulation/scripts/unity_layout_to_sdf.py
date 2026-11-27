#!/usr/bin/env python3
"""Unity 발전소 배치 격자(DoosanGazeboLayoutExport) → Gazebo 월드 unity_plant.sdf + 순찰 웨이포인트.

좌표는 Unity Spot 시작점 기준 /odom(ROS) 프레임 — Gazebo 원점 = Unity 시작점이라 미러가 /odom을 그대로 쓴다.
바닥 높이(계단·낮은 적재물)는 높이 기둥 박스로, 경사로(5~25° 직사각형 평면)는 기울인 판 하나로, 벽·기계(바닥+0.3~2.3 m 점유)는 벽 박스로 재현한다.

사용: python3 simulation/scripts/unity_layout_to_sdf.py unity/config/gazebo_layout.json
"""
import argparse
import base64
import json
import sys
from pathlib import Path

import numpy as np

STEP = 0.02      # 기둥 높이 양자화(m) — 경사로는 find_ramps가 판으로 바꿔 이 양자화를 타지 않는다
FLAT = 0.02      # 이 높이 이하는 바닥판으로 충분
WALL_H = 3.0     # 벽·기계 박스 높이(m, 바닥 기준)
SPAWN_Z = 0.4    # 다른 월드와 같은 스폰 높이(바닥 위)
RISE = 0.03      # 얕은 구멍 판정: 양쪽 바닥이 이만큼 이상 높을 때
SIMILAR = 0.1    # 그 양쪽 바닥끼리의 높이 차 허용
MAX_GAP = 2      # 메울 구멍 폭(셀)
RAMP_MIN, RAMP_MAX = np.tan(np.radians(5)), np.tan(np.radians(25))  # 경사로로 볼 기울기 범위
RAMP_CELLS = 30  # 이보다 작은 경사 영역은 기둥 그대로
RAMP_TOL = 0.04  # 경사로 셀이 맞춘 평면에서 벗어나도 되는 높이(m) — 들머리의 메우지 못한 그레이팅 구멍(0 mm)이 평면보다 2.4 cm 낮다


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


def find_ramps(heights, floor, res):
    """x 방향으로 기우는 직사각형 경사 영역 → 기울인 판 하나씩. 2 cm 기둥 계단은 4 cm 턱(양자화)에 velocity 몸체가
    들려 넘어져서(2026-11-27 실측), Unity 원본처럼 매끈한 면으로 바꾼다. 조건(직사각형·평면·x축 기울기)이 안 맞으면 [] — 기둥 그대로.

    반환: [{"x0", "x1", "j0", "j1", "a", "c", "cells"}] — x는 격자 왼쪽 끝 기준(m), 윗면 평면 z = a·x + c가
    x0~x1에서 양끝 바닥과 만난다. j0~j1은 판이 덮는 행, cells는 기둥 대신 판이 덮는 셀 마스크.
    ponytail: x축 경사만 — y축 경사로가 생기면 전치해서 같은 함수를 쓴다.
    """
    from scipy import ndimage

    h = np.where(floor, heights, np.nan)
    gx = np.nan_to_num((_shift(h, 1, 1) - _shift(h, -1, 1)) / (2 * res))
    gy = np.nan_to_num((_shift(h, 1, 0) - _shift(h, -1, 0)) / (2 * res))
    seed = floor & (np.abs(gx) >= RAMP_MIN) & (np.abs(gx) <= RAMP_MAX) & (np.abs(gy) <= 0.1 * np.abs(gx))
    lab, n = ndimage.label(seed)
    ramps = []
    for k in range(1, n + 1):
        jj, ii = np.nonzero(lab == k)
        if ii.size < RAMP_CELLS:
            continue
        xs = (ii + 0.5) * res
        a, c = np.polyfit(xs, heights[jj, ii], 1)
        j0, j1, i0, i1 = jj.min(), jj.max(), ii.min(), ii.max()
        lo_i, hi_i = (i0 - 1, i1 + 1) if a > 0 else (i1 + 1, i0 - 1)
        ends = []
        for i_out, i_in in ((lo_i, i0 if a > 0 else i1), (hi_i, i1 if a > 0 else i0)):
            col = floor[j0:j1 + 1, i_out] if 0 <= i_out < h.shape[1] else np.zeros(0, bool)
            z_end = np.median(heights[j0:j1 + 1, i_out][col]) if col.any() else a * (i_in + 0.5) * res + c
            ends.append((z_end - c) / a)  # 평면이 이웃 바닥 높이와 만나는 x
        x0, x1 = sorted(ends)
        if x0 < (i0 - 1) * res or x1 > (i1 + 2) * res:
            print(f"경사 영역 {k}: 끝이 바닥과 안 만남 → 기둥 유지", file=sys.stderr)
            continue
        cells = np.zeros_like(floor)
        cx = (np.arange(h.shape[1]) + 0.5) * res
        cells[j0:j1 + 1, (cx > x0) & (cx < x1)] = True
        on_plane = np.abs(heights - (a * cx[None, :] + c)) <= RAMP_TOL
        if not (floor & on_plane)[cells].all():
            print(f"경사 영역 {k}: 직사각형 평면이 아님 → 기둥 유지", file=sys.stderr)
            continue
        ramps.append({"x0": x0, "x1": x1, "j0": int(j0), "j1": int(j1), "a": float(a), "c": float(c), "cells": cells})
    return ramps


def classify(heights, wall, void, ramps=()):
    """(셀 클래스, 바닥판 높이 base). -1 = 벽(벽·빈 곳), 0 = 바닥판, k>0 = base 위 k*STEP 기둥.
    base는 가장 낮은 바닥(원점 0보다 낮은 구덩이도 평지로 뭉개지 않게). 경사로 판이 덮는 셀은 0."""
    floor = ~(wall | void)
    base = min(0.0, float(heights[floor].min())) if floor.any() else 0.0
    cls = np.rint((heights - base) / STEP).astype(int)
    cls[cls * STEP <= FLAT] = 0
    for r in ramps:
        cls[r["cells"]] = 0
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


def _box(name, x, y, z, sx, sy, sz, rgb, pitch=0.0):
    """링크 하나에 들어갈 collision+visual 한 쌍. 박스마다 모델을 따로 두면 1천여 개 엔티티로 RTF가 0.3까지 떨어진다."""
    color = " ".join(f"{c:.2f}" for c in rgb)
    pose = f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 {pitch:.4f} 0</pose>" if pitch else f"<pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>"
    geo = f"<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>"
    return (f'        <collision name="{name}">{pose}{geo}</collision>'
            f'<visual name="{name}">{pose}{geo}<material><ambient>{color} 1</ambient><diffuse>{color} 1</diffuse></material></visual>')


def _ramp_box(n, r, ox, oy, res, base):
    """윗면이 평면 z = a·x + c를 x0~x1에서 따르고, 아랫면은 바닥 아래까지 내려가 속이 찬 기울인 판."""
    xa, xb = r["x0"], r["x1"]
    za, zb = r["a"] * xa + r["c"], r["a"] * xb + r["c"]
    th = np.arctan2(zb - za, xb - xa)
    length, thick = np.hypot(xb - xa, zb - za), max(za, zb) - base + 0.2
    top = np.array([ox + (xa + xb) / 2, (za + zb) / 2])
    cx, cz = top - thick / 2 * np.array([-np.sin(th), np.cos(th)])  # 윗면 중심에서 법선 반대로 두께 절반
    y0, y1 = oy + r["j0"] * res, oy + (r["j1"] + 1) * res
    return _box(f"ramp_{n}", cx, (y0 + y1) / 2, cz, length, y1 - y0, thick, (0.55, 0.52, 0.48), pitch=-th)


def world_sdf(layout: dict, heights, cls, base: float = 0.0, ramps=()) -> str:
    res, ox, oy = layout["resolution"], layout["origin_x"], layout["origin_y"]
    ny, nx = cls.shape

    def center(i, j, w, h):
        return ox + (i + w / 2) * res, oy + (j + h / 2) * res, w * res, h * res

    parts = []
    x, y, sx, sy = center(0, 0, nx, ny)
    parts.append(_box("floor", x, y, base - 0.05, sx, sy, 0.1, (0.45, 0.45, 0.48)))
    parts += [_ramp_box(n, r, ox, oy, res, base) for n, r in enumerate(ramps)]
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
    ramps = find_ramps(heights, ~(wall | void), layout["resolution"])
    cls, base = classify(heights, wall, void, ramps)
    a.world_out.write_text(world_sdf(layout, heights, cls, base, ramps))
    a.waypoints_out.write_text(waypoints_yaml(json.loads(a.route.read_text())))
    raised = cls > 0
    print(f"grid {layout['nx']}x{layout['ny']} @ {layout['resolution']} m | 벽 {int((cls == -1).sum())} 셀 "
          f"| 높이 셀 {int(raised.sum())} (최대 {heights[raised].max() if raised.any() else 0:.2f} m, 바닥 최저 {heights[~(wall | void)].min():.2f} m) "
          f"| 메운 구멍 {pits} 셀 | 경사로 판 {len(ramps)} | 박스 {a.world_out.read_text().count('<collision ')} → {a.world_out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
