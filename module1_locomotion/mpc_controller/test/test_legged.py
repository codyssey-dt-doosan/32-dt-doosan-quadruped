import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from mpc_controller.legged_model import (
    JOINTS,
    POS_CTRL,
    VEL_CTRL,
    bridge_entries,
    make_legged_model,
    make_legged_world,
)

REPO = Path(__file__).resolve().parents[3]
GO2 = (REPO / "simulation/models/go2/model.sdf").read_text(encoding="utf-8")


def test_model_has_one_controller_per_joint():
    model = ET.fromstring(make_legged_model(GO2)).find("model")
    ctrls = [p for p in model.findall("plugin") if p.get("filename") == POS_CTRL]
    assert len(ctrls) == 12
    # 다중 <joint_name>은 첫 관절만 피드백한다 → 플러그인마다 관절 하나
    assert all(len(c.findall("joint_name")) == 1 for c in ctrls)
    assert sorted(c.find("joint_name").text for c in ctrls) == sorted(JOINTS)
    assert len({c.find("topic").text for c in ctrls}) == 12


def test_model_drops_velocity_control_and_springs_restores_friction():
    model = ET.fromstring(make_legged_model(GO2, mu=0.8)).find("model")
    assert not [p for p in model.findall("plugin") if p.get("filename") == VEL_CTRL]
    assert not list(model.iter("spring_stiffness")) and not list(model.iter("spring_reference"))
    assert len(list(model.iter("damping"))) == 12
    mus = [
        e.text
        for link in model.findall("link")
        if link.get("name").endswith(("_thigh", "_calf"))
        for e in link.iter("mu")
    ]
    assert mus == ["0.8"] * 8


def test_model_keeps_sensors_name_and_other_plugins():
    model = ET.fromstring(make_legged_model(GO2)).find("model")
    assert model.get("name") == "go2"  # /model/go2/* 토픽명 유지
    assert len(list(model.iter("sensor"))) == 4
    assert len(model.findall("plugin")) == 14  # odometry + joint_state + 컨트롤러 12


def test_model_gains_are_passed_through():
    model = ET.fromstring(make_legged_model(GO2, p_gain=55.0, d_gain=3.5)).find("model")
    ctrl = next(p for p in model.findall("plugin") if p.get("filename") == POS_CTRL)
    assert ctrl.find("p_gain").text == "55.0" and ctrl.find("d_gain").text == "3.5"


def test_model_fails_loudly_when_source_changes():
    with pytest.raises(ValueError, match="VelocityControl"):
        make_legged_model(GO2.replace(VEL_CTRL, "gz-sim-gone"))
    with pytest.raises(ValueError, match="revolute"):
        make_legged_model(
            GO2.replace('<joint name="RR_calf_joint" type="revolute">', '<joint name="RR_calf_joint" type="fixed">')
        )


@pytest.mark.parametrize("world", ["corridor", "factory"])
def test_world_swaps_only_the_go2_include(world):
    text = (REPO / f"simulation/worlds/{world}.sdf").read_text(encoding="utf-8")
    out = make_legged_world(text)
    assert out.count("model://go2_legged") == 1
    assert "model://go2<" not in out
    assert "<name>go2</name>" in out
    with pytest.raises(ValueError, match="include"):
        make_legged_world(text.replace("model://go2", "model://none"))


def test_bridge_entries_cover_all_joints():
    entries = bridge_entries()
    assert [e["ros_topic_name"] for e in entries] == [f"/legged/{j}" for j in JOINTS]
    assert all(e["gz_topic_name"] == f"/model/go2{e['ros_topic_name']}" for e in entries)
    assert all(e["direction"] == "ROS_TO_GZ" and e["gz_type_name"] == "gz.msgs.Double" for e in entries)
