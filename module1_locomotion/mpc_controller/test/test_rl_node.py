"""rl_node 순수 함수: 관절 재정렬, 투영 중력, 관측 배치, 명령 끊김, 기동 램프, hold, 정책 경로, 서보 명령 배치."""
import copy
import math
from pathlib import Path

import numpy as np
import pytest

from mpc_controller import sim2sim
from mpc_controller.balance_node import blend
from mpc_controller.joint_servo import pack_cmd, unpack_cmd
from mpc_controller.leg_kinematics import LIMITS
from mpc_controller.legged_model import JOINTS
from mpc_controller.rl_node import (action_to_q, build_obs, cmd_or_zero, joints_by_name, projected_gravity,
                                    resolve_policy, rl_supervise, startup_gains)

CFG = sim2sim.load(Path(__file__).resolve().parents[1] / "config/sim2sim.json")


def test_joints_by_name_reorders_and_rejects_missing():
    names = list(reversed(JOINTS)); pos = np.arange(12.0)
    q, qd = joints_by_name(names, pos, [])
    assert np.allclose(q, pos[::-1]) and np.allclose(qd, 0.0)
    assert joints_by_name(names[:-1], pos[:-1], []) is None


def test_projected_gravity():
    assert np.allclose(projected_gravity([0, 0, 0, 1]), [0, 0, -1])
    s = math.sqrt(0.5)
    assert np.allclose(projected_gravity([0, s, 0, s]), [1, 0, 0], atol=1e-9)   # y축 +90°: 몸체 x가 아래


def test_zero_quat_not_ready():                  # Review Focus 3
    assert projected_gravity([0, 0, 0, 0]) is None and projected_gravity([float("nan"), 0, 0, 1]) is None


def test_build_obs_layout():
    obs = build_obs(CFG, [1, 2, 3], [0, 0, -1], (0.3, 0.5), np.array(CFG["q_nom"]) + 0.1, np.ones(12), np.full(12, 0.2))
    assert obs.shape == (45,)
    assert np.allclose(obs[0:3], np.array([1, 2, 3]) * 0.25) and np.allclose(obs[6:9], [0.3 * 2.0, 0.0, 0.5 * 0.25])
    assert np.allclose(obs[9:21], 0.1) and np.allclose(obs[21:33], 0.05) and np.allclose(obs[33:45], 0.2)


def test_cmd_timeout_and_clip():                 # Review Focus 2
    assert cmd_or_zero((0.3, 0.5), 0.6, CFG) == (0.0, 0.0)
    assert cmd_or_zero((0.9, -2.0), 0.1, CFG) == (0.5, -1.0)


def test_startup_gains_phases():
    q, kp, kd = startup_gains(0.0, CFG); assert np.allclose(q, 0.0) and (kp, kd) == (120.0, 2.0)
    q, kp, kd = startup_gains(2.5, CFG); assert np.allclose(q, CFG["q_nom"]) and kp == pytest.approx(120 + (CFG["kp"] - 120) * blend(2.5))
    assert startup_gains(3.0, CFG) is None


def test_nonfinite_holds():                      # Review Focus 3
    assert rl_supervise(1.0, 0.0, 0.0, False, CFG) == "hold"
    assert rl_supervise(1.0, 0.0, 0.0, True, CFG) == "run" and rl_supervise(46.0, 0.0, 0.0, True, CFG) == "hold"


def test_policy_mismatch_lists_keys():           # Review Focus 4
    other = copy.deepcopy(CFG); other["kp"] += 5.0
    assert sim2sim.diff(other, CFG) == ["kp"]


def test_resolve_policy_missing(tmp_path):       # Review Focus 5
    with pytest.raises(FileNotFoundError, match="policy:="):
        resolve_policy("", tmp_path)


def test_cmd_roundtrip_layout():
    a = np.linspace(-1, 1, 12)
    t, q, kp, kd, tau = unpack_cmd(pack_cmd(1.0, action_to_q(CFG, a), CFG["kp"], CFG["kd"], 0.0))
    assert np.allclose(q, np.array(CFG["q_nom"]) + 0.25 * a) and np.allclose(tau, 0.0)


def test_action_to_q_clips_to_limits():
    q = action_to_q(CFG, np.full(12, 100.0)).reshape(4, 3)
    assert np.allclose(q, [hi for _, hi in LIMITS])
