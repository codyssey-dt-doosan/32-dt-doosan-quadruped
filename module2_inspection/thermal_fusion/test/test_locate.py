"""픽셀 → 맵 위치. factory 실제 맵으로 확인."""

import math
from pathlib import Path

import pytest

from inspection_common.world_map import load_world, resolve_world
from thermal_fusion.locate import locate

SIM = Path(__file__).resolve().parents[3] / "simulation"
WM = load_world(*resolve_world("factory", sim_share=str(SIM)))
TH = WM.thermal                                          # 320×240, hfov 1.0, z 0.02


def _loc(u, v, rx, ry, ryaw):
    return locate(u, v, rx, ry, ryaw, TH, 0.4, WM.obstacles)


def test_center_pixel_hits_tank_west_surface():
    hit, reason = _loc(160, 120, 5.0, -6.0, 0.0)
    assert reason == ""
    assert hit.name.startswith("gas_tank_1/")
    assert (hit.x, hit.y) == pytest.approx((7.65, -6.0), abs=1e-6)
    assert hit.dist == pytest.approx(7.65 - 5.28)
    assert hit.z == pytest.approx(0.42)


def test_pixel_below_horizon_hits_floor_first():
    hit, reason = _loc(160, 239, 5.0, -6.0, 0.0)          # 바닥 1.0 m 앞 < 탱크 2.37 m
    assert hit is None and reason == "floor"


def test_upper_pixel_hits_north_wall():
    hit, reason = _loc(160, 0, 0.0, 0.0, math.pi / 2)
    assert reason == ""
    assert hit.name.startswith("wall_north/")
    assert hit.y == pytest.approx(9.0)
    assert hit.z == pytest.approx(0.42 + (9.0 - 0.28) * 120 / math.hypot(TH.fx, 0), rel=1e-3)


def test_left_pixel_turns_left():
    hit, _ = _loc(0, 120, 0.0, 0.0, math.pi / 2)          # 북쪽을 보며 왼쪽 끝 → 서쪽으로 0.5 rad
    assert hit.x < 0.0
    assert math.atan2(hit.y - 0.28, hit.x) == pytest.approx(math.pi / 2 + 0.5, abs=1e-6)


def test_no_hit_when_out_of_range():
    hit, reason = locate(160, 100, 0.0, 0.0, math.pi / 2, TH, 0.4, WM.obstacles, max_range=3.0)
    assert hit is None and reason == "no_hit"
