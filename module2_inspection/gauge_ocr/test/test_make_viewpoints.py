"""make_viewpoints 도구. 출력은 순찰 waypoints YAML에 그대로 붙일 수 있어야 한다."""

from pathlib import Path

import yaml

from gauge_ocr.make_viewpoints import compute_viewpoints, main, to_yaml
from inspection_common.viewpoint import Viewpoint
from inspection_common.world_map import load_world, resolve_world

SIM = str(Path(__file__).resolve().parents[3] / "simulation")


def test_compute_factory():
    wm = load_world(*resolve_world("factory", sim_share=SIM))
    vps = compute_viewpoints(wm, 0.4)
    assert [v.name for v in vps] == ["gauge_line1", "gauge_line2"]
    assert all(v.ok for v in vps)


def test_yaml_is_patrol_format():
    vps = [
        Viewpoint("g1", 10.0, 5.701, 1.5708, 2.96, 31.8, 28.1, True, ""),
        Viewpoint("g2", 0.0, 0.0, 0.0, 2.97, 0.0, 0.0, False, "no_space:wall_south/link"),
    ]
    text = to_yaml(vps, 5.0)
    assert "# g2: 불가 (no_space:wall_south/link)" in text
    data = yaml.safe_load("waypoints:\n" + text)
    assert data["waypoints"] == [{"name": "g1", "x": 10.0, "y": 5.7, "yaw": 1.57, "dwell_s": 5.0}]


def test_main_factory(capsys):
    assert main(["--world", "factory", "--sim-share", SIM]) == 0
    out = capsys.readouterr().out
    assert "{name: gauge_line1, x: 10.00, y: 5.70, yaw: 1.57, dwell_s: 5.0}" in out


def test_main_exit_code_when_impossible(capsys):
    assert main(["--world", "corridor", "--locomotion", "legged", "--sim-share", SIM]) == 1
    assert "gauge_corridor: 불가 (no_space:wall_south" in capsys.readouterr().out
