"""2D 기하 함수."""

import math
from pathlib import Path

import pytest

from inspection_common.geometry import (
    camera_xyz, circle_collision, pixel_angles, point_distance, ray_hits, segment_blocked, wrap,
)
from inspection_common.world_map import CameraSpec, Obstacle, load_world, resolve_world

SIM = Path(__file__).resolve().parents[3] / "simulation"
CAM = CameraSpec(0.28, 0.0, 0.05, 0.0, 1.396, 640, 480)
TANK = Obstacle("tank/link", "cylinder", 2.0, 0.0, 0.0, 0.0, 0.0, 0.35, 0.0, 1.5)
# 가로 2 m × 세로 0.2 m 판을 90° 돌림 → x ∈ [2.9, 3.1], y ∈ [-1, 1]
PLATE = Obstacle("plate/link", "box", 3.0, 0.0, math.pi / 2, 2.0, 0.2, 0.0, 0.0, 1.0)
LOW = Obstacle("low/link", "box", 1.0, 0.0, 0.0, 0.2, 0.2, 0.0, 0.0, 0.3)


def test_wrap():
    assert wrap(3 * math.pi / 2) == pytest.approx(-math.pi / 2)


def test_point_distance_rotated_box():
    assert point_distance(PLATE, 2.5, 0.0) == pytest.approx(0.4)
    assert point_distance(PLATE, 3.0, 0.5) == 0.0
    assert point_distance(PLATE, 3.0, 1.5) == pytest.approx(0.5)


def test_circle_collision():
    assert circle_collision(1.5, 0.0, 0.35, [TANK]) == "tank/link"   # 표면까지 0.15 m
    assert circle_collision(1.0, 0.0, 0.35, [TANK]) is None          # 표면까지 0.65 m


def test_ray_hits_sorted():
    hits = ray_hits(0.0, 0.0, 0.0, [PLATE, TANK])
    assert [o.name for _, o in hits] == ["tank/link", "plate/link"]
    assert hits[0][0] == pytest.approx(1.65)
    assert hits[1][0] == pytest.approx(2.9)


def test_ray_miss_and_range():
    assert ray_hits(0.0, 0.0, math.pi / 2, [PLATE, TANK]) == []
    assert ray_hits(0.0, 0.0, 0.0, [PLATE], max_range=2.0) == []


def test_ray_from_inside_is_zero():
    assert ray_hits(2.0, 0.0, 0.0, [TANK])[0][0] == 0.0


def test_segment_blocked_ignores_low_obstacle():
    assert segment_blocked(0.0, 0.0, 2.5, 0.0, [LOW]) == "low/link"
    assert segment_blocked(0.0, 0.0, 2.5, 0.0, [LOW], z_from=0.45) is None


def test_camera_xyz():
    x, y, z = camera_xyz(1.0, 2.0, math.pi / 2, CAM, 0.4)
    assert (x, y, z) == pytest.approx((1.0, 2.28, 0.45))


def test_pixel_angles():
    assert pixel_angles(320, 240, CAM) == pytest.approx((0.0, 0.0))
    az, _ = pixel_angles(0, 240, CAM)
    assert az == pytest.approx(1.396 / 2)                         # 왼쪽 끝 = +hfov/2
    _, el = pixel_angles(320, 0, CAM)
    assert el == pytest.approx(CAM.half_vfov)


def test_ray_hits_real_gas_tank():
    wm = load_world(*resolve_world("factory", sim_share=str(SIM)))
    d, o = ray_hits(8.0, -4.5, -math.pi / 2, wm.obstacles)[0]
    assert o.name.startswith("gas_tank_1/")
    assert d == pytest.approx(1.15)
