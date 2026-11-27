import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from mpc_controller import sim2sim
from mpc_controller.leg_kinematics import LEGS, LIMITS, foot_pos
from mpc_controller.legged_model import JOINTS
from mpc_controller.rl_asset import make_go2_box_urdf, sdf_to_urdf

REPO = Path(__file__).resolve().parents[3]
MC = REPO / "module1_locomotion/mpc_controller"
GO2_SDF = (REPO / "simulation/models/go2/model.sdf").read_text(encoding="utf-8")
CFG = sim2sim.load(MC / "config/sim2sim.json")
URDF = make_go2_box_urdf(GO2_SDF, CFG)


def _floats(s):
    return [float(v) for v in s.split()]


def urdf_joint(urdf, name):
    return next(j for j in ET.fromstring(urdf).iter("joint") if j.get("name") == name)


def urdf_axis(urdf, name):
    return tuple(_floats(urdf_joint(urdf, name).find("axis").get("xyz")))


def urdf_limits(urdf, name):
    lim = urdf_joint(urdf, name).find("limit")
    return float(lim.get("lower")), float(lim.get("upper"))


def urdf_effort(urdf, name):
    return float(urdf_joint(urdf, name).find("limit").get("effort"))


def urdf_damping(urdf, name):
    return float(urdf_joint(urdf, name).find("dynamics").get("damping"))


def urdf_total_mass(urdf):
    return sum(float(m.get("value")) for m in ET.fromstring(urdf).iter("mass"))


def sdf_axis(sdf, name):
    j = next(j for j in ET.fromstring(sdf).iter("joint") if j.get("name") == name)
    return tuple(_floats(j.find("axis/xyz").text))


def sdf_limits(sdf, name):
    j = next(j for j in ET.fromstring(sdf).iter("joint") if j.get("name") == name)
    return float(j.find("axis/limit/lower").text), float(j.find("axis/limit/upper").text)


def _rot(axis, q):
    a = np.array(axis, float)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


def urdf_fk(urdf, leg, q):
    """base → hip → thigh → calf를 origin 이동 + axis 회전으로 잇고 calf 구 충돌 중심을 반환."""
    root = ET.fromstring(urdf)
    R, p = np.eye(3), np.zeros(3)
    for part, qi in zip(("hip", "thigh", "calf"), q):
        j = urdf_joint(urdf, f"{leg}_{part}_joint")
        p = p + R @ np.array(_floats(j.find("origin").get("xyz")))
        R = R @ _rot(urdf_axis(urdf, j.get("name")), qi)
    calf = next(l for l in root.iter("link") if l.get("name") == f"{leg}_calf")
    sph = next(c for c in calf.iter("collision") if c.find("geometry/sphere") is not None)
    return p + R @ np.array(_floats(sph.find("origin").get("xyz")))


def test_fk_matches_leg_kinematics():  # 부호·축 오류를 잡는다
    rng = np.random.default_rng(0)
    for leg in LEGS:
        for _ in range(50):
            q = [rng.uniform(lo, hi) for lo, hi in LIMITS]
            assert np.linalg.norm(urdf_fk(URDF, leg, q) - foot_pos(leg, q)) < 1e-3


def test_mass_axes_limits_effort_damping():
    assert abs(urdf_total_mass(URDF) - 13.2) < 1e-9
    for i, j in enumerate(JOINTS):
        assert urdf_axis(URDF, j) == sdf_axis(GO2_SDF, j)
        assert urdf_limits(URDF, j) == sdf_limits(GO2_SDF, j)
        assert urdf_limits(URDF, j) == LIMITS[i % 3]  # rl_node 행동 클립(Q_LO·Q_HI) = Genesis 클립(URDF 한계)
        assert urdf_effort(URDF, j) == 30.0 and urdf_damping(URDF, j) == 0.0


def test_rejects_rotated_pose():
    with pytest.raises(ValueError):
        sdf_to_urdf(GO2_SDF.replace("<pose>0.19 0.10 0 0 0 0</pose>", "<pose>0.19 0.10 0 0.1 0 0</pose>", 1), 30.0)


def test_committed_urdf_up_to_date():
    assert (REPO / "module1_locomotion/rl_training/assets/go2_box.urdf").read_text(encoding="utf-8") == URDF
