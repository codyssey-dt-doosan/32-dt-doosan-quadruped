"""저장소의 실제 시뮬 맵을 읽는다."""

import math
from pathlib import Path

import pytest

from inspection_common.world_map import (
    body_height_for, load_world, resolve_world,
)

SIM = Path(__file__).resolve().parents[3] / "simulation"


def _load(world):
    return load_world(*resolve_world(world, sim_share=str(SIM)))


def test_factory_gauges():
    wm = _load("factory")
    names = [g.name for g in wm.gauges]
    assert names == ["gauge_line1", "gauge_line2"]
    g = wm.gauges[0]
    assert (g.x, g.y, g.z) == pytest.approx((10.0, 8.7, 1.5))
    assert g.yaw == pytest.approx(-1.5708)
    assert g.dial_r == pytest.approx(0.12)
    assert g.face == pytest.approx(0.04)


def test_corridor_gauge():
    wm = _load("corridor")
    assert [(g.name, g.z) for g in wm.gauges] == [("gauge_corridor", 1.4)]


def test_factory_obstacles_without_floor_and_robot():
    wm = _load("factory")
    models = sorted({o.name.split("/")[0] for o in wm.obstacles})
    assert models == sorted([
        "wall_north", "wall_south", "wall_west", "wall_east",
        "rack_a", "rack_b", "gauge_line1", "gauge_line2", "gas_tank_1", "gas_tank_2",
    ])


def test_gas_tank_is_cylinder():
    tank = next(o for o in _load("factory").obstacles if o.name.startswith("gas_tank_1/"))
    assert tank.kind == "cylinder"
    assert (tank.x, tank.y, tank.r) == pytest.approx((8.0, -6.0, 0.35))
    assert (tank.z_min, tank.z_max) == pytest.approx((0.0, 1.5))


def test_rotated_rack_box():
    rack = next(o for o in _load("factory").obstacles if o.name.startswith("rack_a/"))
    assert rack.kind == "box"
    assert (rack.x, rack.y, rack.yaw, rack.sx, rack.sy) == pytest.approx((6, 4, 0.3, 0.6, 0.4))
    assert (rack.z_min, rack.z_max) == pytest.approx((0.0, 0.8))


def test_camera_specs():
    wm = _load("factory")
    cam, th = wm.camera, wm.thermal
    assert (cam.x, cam.y, cam.z, cam.pitch) == pytest.approx((0.28, 0.0, 0.05, 0.0))
    assert (cam.hfov, cam.width, cam.height) == (pytest.approx(1.396), 640, 480)
    assert cam.fx == pytest.approx(381.46, abs=0.01)
    assert math.degrees(cam.half_vfov) == pytest.approx(32.18, abs=0.01)
    assert (th.z, th.hfov, th.width, th.height) == (pytest.approx(0.02), pytest.approx(1.0), 320, 240)


def test_resolve_missing_world_raises():
    with pytest.raises(FileNotFoundError):
        resolve_world("no_such_world", sim_share=str(SIM))


def test_missing_model_dir_raises(tmp_path):
    world = tmp_path / "w.sdf"
    world.write_text(
        '<sdf version="1.10"><world name="w"><include><uri>model://missing</uri>'
        "<name>m</name></include></world></sdf>"
    )
    with pytest.raises(FileNotFoundError, match="missing"):
        load_world(str(world), str(SIM / "models"))


def test_world_without_gauges(tmp_path):
    world = tmp_path / "w.sdf"
    world.write_text('<sdf version="1.10"><world name="w"></world></sdf>')
    wm = load_world(str(world), str(SIM / "models"))
    assert wm.gauges == [] and wm.obstacles == []


def test_body_height_for():
    assert body_height_for("velocity") == 0.4
    assert body_height_for("legged") == 0.295
    with pytest.raises(ValueError):
        body_height_for("wheels")
