"""점검 위치 계산. 숫자는 설계 문서 4-2절 표."""

import math
from pathlib import Path

import pytest

from inspection_common.viewpoint import approach_point, gauge_viewpoint
from inspection_common.world_map import Obstacle, body_height_for, load_world, resolve_world

SIM = Path(__file__).resolve().parents[3] / "simulation"


def _vp(world, locomotion, index=0, **kw):
    wm = load_world(*resolve_world(world, sim_share=str(SIM)))
    return gauge_viewpoint(wm.gauges[index], wm.camera, wm.obstacles, body_height_for(locomotion), **kw)


def test_factory_velocity():
    v = _vp("factory", "velocity")
    assert v.ok and v.reason == ""
    assert v.distance == pytest.approx(2.959, abs=0.005)
    assert (v.x, v.y) == pytest.approx((10.0, 8.7 - 0.04 - 2.959), abs=0.005)
    assert v.yaw == pytest.approx(math.pi / 2, abs=1e-4)   # SDF gauge yaw -1.5708 (반올림) → +π 후 3.7e-6 차이
    assert v.dial_px == pytest.approx(31.8, abs=0.2)
    assert v.dial_px_far == pytest.approx(28.1, abs=0.2)


def test_factory_legged_is_farther():
    v = _vp("factory", "legged")
    assert v.ok
    assert v.distance == pytest.approx(3.163, abs=0.005)


def test_factory_second_gauge():
    v = _vp("factory", "velocity", index=1)
    assert v.ok and (v.x, v.y) == pytest.approx((-8.0, 5.70), abs=0.01)


def test_corridor_velocity_fits():
    v = _vp("corridor", "velocity")
    assert v.ok
    assert v.distance == pytest.approx(2.764, abs=0.005)


def test_corridor_legged_is_no_space():
    v = _vp("corridor", "legged")
    assert not v.ok
    assert v.reason.startswith("no_space:wall_south")


def test_too_small_when_threshold_high():
    v = _vp("factory", "velocity", min_dial_px=40)
    assert not v.ok and v.reason == "too_small"


def test_blocked_by_obstacle_in_between():
    wm = load_world(*resolve_world("factory", sim_share=str(SIM)))
    wall = Obstacle("screen/link", "box", 10.0, 7.5, 0.0, 1.0, 0.1, 0.0, 0.0, 2.0)
    v = gauge_viewpoint(wm.gauges[0], wm.camera, wm.obstacles + [wall], 0.4)
    assert not v.ok and v.reason == "blocked:screen/link"


TANK = Obstacle("tank/link", "cylinder", 8.0, -6.0, 0.0, 0.0, 0.0, 0.35, 0.0, 1.5)


def test_approach_straight():
    goal, reason = approach_point(7.65, -6.0, 5.0, -6.0, [TANK])
    assert reason == ""
    assert goal == pytest.approx((6.15, -6.0, 0.0))


def test_approach_rotates_when_blocked():
    rack = Obstacle("rack/link", "box", 6.15, -6.0, 0.0, 0.4, 0.4, 0.0, 0.0, 0.8)
    goal, reason = approach_point(7.65, -6.0, 5.0, -6.0, [TANK, rack])
    assert reason == ""
    x, y, yaw = goal
    assert math.hypot(x - 7.65, y + 6.0) == pytest.approx(1.5)
    assert abs(math.degrees(math.atan2(y + 6.0, x - 7.65))) == pytest.approx(140.0)   # ±20°는 rack에 걸리고 ±40°에서 성공
    assert math.hypot(math.cos(yaw) - (7.65 - x) / 1.5, math.sin(yaw) - (-6.0 - y) / 1.5) < 1e-6


def test_approach_impossible():
    box = Obstacle("cage/link", "box", 7.65, -6.0, 0.0, 6.0, 6.0, 0.0, 0.0, 2.0)
    goal, reason = approach_point(7.65, -6.0, 5.0, -6.0, [box])
    assert goal is None and reason.startswith("no_space:cage")
