import copy
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from mpc_controller import sim2sim
from mpc_controller.balance_node import TAU_MAX
from mpc_controller.leg_kinematics import Q_NOM
from mpc_controller.legged_model import JOINTS, make_legged_model

REPO = Path(__file__).resolve().parents[3]
MC = REPO / "module1_locomotion/mpc_controller"
GO2_SDF = REPO / "simulation/models/go2/model.sdf"
WALK_LAUNCH = (MC / "launch/walk.launch.py").read_text()
CFG = sim2sim.load(MC / "config/sim2sim.json")


def sdf_imu_rate(path):
    root = ET.parse(path).getroot()
    return float(root.find(".//sensor[@type='imu']/update_rate").text)


def sdf_total_mass(path):
    return sum(float(m.text) for m in ET.parse(path).getroot().iter("mass"))


def launch_default(src, name):
    return float(re.search(rf'DeclareLaunchArgument\("{name}", default_value="([\d.]+)"', src)[1])


def test_joint_order_and_q_nom():
    assert CFG["joint_names"] == list(JOINTS)
    assert np.allclose(CFG["q_nom"], Q_NOM * 4)


def test_torque_and_timing():
    assert CFG["tau_max"] == TAU_MAX
    assert abs(CFG["sim_dt"] * CFG["decimation"] - 1.0 / CFG["policy_hz"]) < 1e-12
    assert CFG["imu_hz"] == 4 * CFG["policy_hz"]
    assert sdf_imu_rate(GO2_SDF) == CFG["imu_hz"]


def test_mass_friction_foot_nominal():
    assert abs(sdf_total_mass(GO2_SDF) - CFG["nominal"]["total_mass"]) < 1e-9
    assert launch_default(WALK_LAUNCH, "mu") == CFG["nominal"]["mu"]
    assert re.search(r"foot_r=([\d.]+)", WALK_LAUNCH)[1] == str(CFG["foot_r"])
    lo, hi = CFG["dr"]["friction"]
    assert all(lo <= m <= hi for m in [CFG["nominal"]["mu"], *CFG["nominal"]["test_mu"]])
    lo, hi = CFG["dr"]["added_mass"]
    assert lo <= 0.0 <= hi


def test_armature_matches_gazebo():  # SDF·walk 변환본에 armature 요소가 없다 → Gazebo/DART 로터 관성 0
    assert CFG["armature"] == 0.0
    walk = make_legged_model(GO2_SDF.read_text(encoding="utf-8"), CFG["nominal"]["mu"], 0.0, 0.0,
                             force_ctrl=True, foot_r=CFG["foot_r"], damping=0.0)
    for root in (ET.parse(GO2_SDF).getroot(), ET.fromstring(walk)):
        assert root.find(".//armature") is None


def test_obs_dim():
    assert sim2sim.obs_dim(CFG) == CFG["obs"]["dim"] == 45


def test_diff():
    assert sim2sim.diff(CFG, CFG) == []
    b = copy.deepcopy(CFG)
    b["kp"] += 1.0
    b["obs"]["scales"]["ang_vel"] = 0.3
    assert sorted(sim2sim.diff(CFG, b)) == ["kp", "obs.scales.ang_vel"]


def test_split_diff_contract_vs_training():
    assert set(sim2sim.CONTRACT) <= CFG.keys()
    b = copy.deepcopy(CFG)
    b["kp"] += 1.0
    b["dr"]["friction"] = [0.1, 1.2]
    b["gravity"] = 9.8  # 정책에 없던 학습 전용 키가 생겨도 계약은 그대로
    assert sim2sim.split_diff(CFG, b) == (["kp"], ["dr.friction", "gravity"])
