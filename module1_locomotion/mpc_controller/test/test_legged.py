import math
import os
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from mpc_controller.gait_node import (
    CALF_MIN,
    L1,
    L2,
    R_MAX,
    R_MIN,
    SIDE,
    foot_x,
    ik,
    leg_depth,
    stride_half,
    swing_height,
)
from mpc_controller.legged_model import (
    JOINTS,
    POS_CTRL,
    VEL_CTRL,
    bridge_entries,
    make_legged_model,
    make_legged_world,
    write_legged_assets,
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


def fk(thigh: float, calf: float) -> tuple:
    """순기구학: thigh 축 기준 발끝 (앞 +x, 아래 d). thigh 양수 = 다리 뒤로."""
    shank = thigh + calf
    return -(L1 * math.sin(thigh) + L2 * math.sin(shank)), L1 * math.cos(thigh) + L2 * math.cos(shank)


def test_ik_matches_spike_postures():
    thigh, calf = ik(0.0, 0.2242)  # 기립 스파이크 자세
    assert abs(thigh - 0.45) < 0.01 and abs(calf + 1.35) < 0.01
    thigh, calf = ik(0.0, 0.25)  # trot 명목 자세
    assert abs(thigh - 0.338) < 0.005 and abs(calf + 0.978) < 0.005


def test_ik_round_trips_through_fk():
    for x in (-0.05, 0.0, 0.06):
        for d in (0.21, 0.25, 0.27):
            fx, fd = fk(*ik(x, d))
            assert math.isclose(fx, x, abs_tol=1e-9) and math.isclose(fd, d, abs_tol=1e-9)


def test_ik_clamps_instead_of_raising():
    for x, d in ((0.0, 0.05), (0.0, 0.0), (0.0, 0.5), (0.3, 0.3)):
        thigh, calf = ik(x, d)
        assert CALF_MIN <= calf <= 0.0
        assert R_MIN - 1e-9 <= math.hypot(*fk(thigh, calf)) <= R_MAX + 1e-9


def test_swing_height_profile():
    assert swing_height(0.0, 0.5, 0.04) == 0.0 and swing_height(0.49, 0.5, 0.04) == 0.0  # 지지
    assert math.isclose(swing_height(0.75, 0.5, 0.04), 0.04)  # 스윙 중앙
    assert swing_height(0.5, 0.5, 0.04) < 1e-9 and swing_height(0.999, 0.5, 0.04) < 1e-3  # 경계 연속
    assert swing_height(0.9, 1.0, 0.04) == 0.0  # duty 1 = 항상 지지


def test_leg_depth_ramps_from_straight_to_nominal():
    assert leg_depth(0.0, 0.0, 0.25) == R_MAX
    assert leg_depth(1.0, 0.0, 0.25) == 0.25
    assert leg_depth(5.0, 0.04, 0.25) == pytest.approx(0.21)


def test_stride_half_straight_is_same_on_both_sides():
    left = stride_half(0.3, 0.0, SIDE["FL"], 0.25, 0.33, 0.10)
    right = stride_half(0.3, 0.0, SIDE["FR"], 0.25, 0.33, 0.10)
    assert left == right == 0.3 * 0.25 / 2


def test_stride_half_turn_is_antisymmetric():
    # w > 0(좌회전): 좌측 발은 반대로, 우측 발은 더 쓸어야 한다
    left = stride_half(0.0, 0.5, SIDE["RL"], 0.25, 0.33, 0.10)
    right = stride_half(0.0, 0.5, SIDE["RR"], 0.25, 0.33, 0.10)
    assert left < 0.0 < right and math.isclose(left, -right)


def test_stride_half_reverse_and_clamp():
    assert stride_half(-0.2, 0.0, 1.0, 0.25, 0.33, 0.10) == -stride_half(0.2, 0.0, 1.0, 0.25, 0.33, 0.10)
    assert stride_half(5.0, 0.0, 1.0, 0.25, 0.33, 0.10) == 0.10
    assert stride_half(-5.0, 0.0, 1.0, 0.25, 0.33, 0.10) == -0.10


def test_foot_x_sweeps_back_in_stance_and_returns_in_swing():
    x0 = 0.05
    assert foot_x(0.0, 0.5, x0) == x0  # 지지 시작: 앞
    assert math.isclose(foot_x(0.25, 0.5, x0), 0.0, abs_tol=1e-12)
    assert math.isclose(foot_x(0.5, 0.5, x0), -x0)  # 지지 끝 = 스윙 시작: 뒤
    assert math.isclose(foot_x(0.75, 0.5, x0), 0.0, abs_tol=1e-12)
    assert math.isclose(foot_x(0.999999, 0.5, x0), x0, abs_tol=1e-6)  # 주기 경계 연속
    assert math.isclose(foot_x(0.4999, 0.5, x0), -x0, abs_tol=1e-3)  # 지지→스윙 경계 연속


def test_foot_x_zero_cases():
    for phase in (0.0, 0.3, 0.7):
        assert foot_x(phase, 0.5, 0.0) == 0.0
        assert foot_x(phase, 1.0, 0.05) == 0.0


def test_write_legged_assets_writes_converted_files():
    world_file, bridge_yaml, models_dir = write_legged_assets(str(REPO / "simulation"), "corridor")
    try:
        model = ET.parse(os.path.join(models_dir, "go2_legged", "model.sdf")).getroot().find("model")
        plugins = [p.get("filename") for p in model.findall("plugin")]
        assert plugins.count(POS_CTRL) == 12 and VEL_CTRL not in plugins
        assert os.path.isfile(os.path.join(models_dir, "go2_legged", "model.config"))
        assert os.path.basename(world_file) == "corridor.sdf"
        assert Path(world_file).read_text(encoding="utf-8").count("model://go2_legged") == 1
        assert len(yaml.safe_load(Path(bridge_yaml).read_text(encoding="utf-8"))) == 12
    finally:
        shutil.rmtree(os.path.dirname(world_file))


def test_write_legged_assets_unknown_world():
    with pytest.raises(FileNotFoundError):
        write_legged_assets(str(REPO / "simulation"), "no_such_world")


from mpc_controller.legged_model import (  # noqa: E402
    FORCE_CTRL,
    RAMP_NAME,
    WRENCH_SYS,
    force_topic,
    ramp_spawn_z,
    ros_force_topic,
)

CORRIDOR = (REPO / "simulation/worlds/corridor.sdf").read_text(encoding="utf-8")


def test_defaults_add_nothing_new():
    m = make_legged_model(GO2)
    assert FORCE_CTRL not in m and "<sphere>" not in m
    w = make_legged_world(CORRIDOR)
    assert WRENCH_SYS not in w and RAMP_NAME not in w
    assert len(bridge_entries()) == 12


def test_force_plugins_come_after_position_controllers():
    model = ET.fromstring(make_legged_model(GO2, force_ctrl=True)).find("model")
    names = [p.get("filename") for p in model.findall("plugin")]
    assert names.count(FORCE_CTRL) == 12
    assert max(i for i, n in enumerate(names) if n == POS_CTRL) < min(i for i, n in enumerate(names) if n == FORCE_CTRL)
    joints = sorted(p.find("joint_name").text for p in model.findall("plugin") if p.get("filename") == FORCE_CTRL)
    assert joints == sorted(JOINTS)


def test_sphere_feet_on_four_calves_with_leg_friction():
    model = ET.fromstring(make_legged_model(GO2, mu=0.7, foot_r=0.02)).find("model")
    feet = [c for link in model.findall("link") for c in link.findall("collision") if c.get("name") == "foot"]
    assert len(feet) == 4
    for c in feet:
        assert c.find("pose").text == "0 0 -0.1 0 0 0"
        assert c.find("geometry/sphere/radius").text == "0.02"
        assert c.find("surface/friction/ode/mu").text == "0.7"


def test_world_wrench_plugin_and_ramp():
    root = ET.fromstring(make_legged_world(CORRIDOR, wrench=True, ramp_deg=15.0))
    world = root.find("world")
    assert any(p.get("filename") == WRENCH_SYS for p in world.findall("plugin"))
    ramp = next(m for m in world.findall("model") if m.get("name") == RAMP_NAME)
    x, y, z, roll, pitch, yaw = (float(v) for v in ramp.find("pose").text.split())
    assert (x, y) == (-20.0, 0.0) and pitch == pytest.approx(-math.radians(15))
    go2 = next(i for i in world.findall("include") if i.find("uri").text == "model://go2_legged")
    assert float(go2.find("pose").text.split()[2]) == pytest.approx(ramp_spawn_z(15.0))
    assert ramp_spawn_z(0.0) == pytest.approx(0.5)  # 판 두께 0.1 위 0.4


def test_world_ramp_zero_is_identical_to_default():
    assert make_legged_world(CORRIDOR, ramp_deg=0.0) == make_legged_world(CORRIDOR)


def test_bridge_force_entries():
    entries = bridge_entries(force=True)
    assert len(entries) == 24
    f = [e for e in entries if e["ros_topic_name"].startswith("/legged_force/")]
    assert len(f) == 12 and f[0]["gz_topic_name"] == force_topic(JOINTS[0]) and f[0]["direction"] == "ROS_TO_GZ"
    assert ros_force_topic("FL_hip_joint") == "/legged_force/FL_hip_joint"


def test_write_legged_assets_passes_options_through():
    world_file, bridge_yaml, models_dir = write_legged_assets(
        str(REPO / "simulation"), "corridor", force_ctrl=True, foot_r=0.02, wrench=True, ramp_deg=10.0
    )
    try:
        sdf = Path(models_dir, "go2_legged", "model.sdf").read_text(encoding="utf-8")
        assert sdf.count(FORCE_CTRL) == 12 and sdf.count("<sphere>") == 4
        w = Path(world_file).read_text(encoding="utf-8")
        assert WRENCH_SYS in w and RAMP_NAME in w
        assert len(yaml.safe_load(Path(bridge_yaml).read_text(encoding="utf-8"))) == 24
    finally:
        shutil.rmtree(os.path.dirname(world_file))
