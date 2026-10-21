import numpy as np
import pytest

from mpc_controller.balance_node import KD_START, KP_START, TAU_MAX
from mpc_controller.joint_servo import (CMD_LEN, gap_summary, hold_cmd, is_stale, pack_cmd, servo_step, servo_torque,
                                        unpack_cmd)
from mpc_controller.leg_kinematics import Q_NOM

Q_NOM12 = np.array(Q_NOM * 4)


def test_pack_unpack_roundtrip_and_broadcast():
    q = np.arange(12) * 0.1
    tau = -np.arange(12) * 0.5
    data = pack_cmd(1.25, q, 120.0, 2.0, tau)
    assert len(data) == CMD_LEN == 49 and all(isinstance(x, float) for x in data)
    t, q2, kp, kd, tau2 = unpack_cmd(data)
    assert t == 1.25 and np.allclose(q2, q) and np.allclose(kp, 120.0) and np.allclose(kd, 2.0) and np.allclose(tau2, tau)


def test_unpack_rejects_bad():
    good = pack_cmd(0.0, np.zeros(12), 0.0, 1.0, np.zeros(12))
    assert unpack_cmd(good[:-1]) is None
    assert unpack_cmd(good + [0.0]) is None
    for bad in (float("nan"), float("inf")):
        d = list(good)
        d[7] = bad
        assert unpack_cmd(d) is None


def test_servo_torque_formula_and_clamp():
    q, qd = np.full(12, 0.1), np.full(12, 0.5)
    cmd = unpack_cmd(pack_cmd(0.0, np.full(12, 0.2), 100.0, 2.0, np.full(12, 3.0)))
    assert np.allclose(servo_torque(cmd, q, qd), 100.0 * 0.1 - 2.0 * 0.5 + 3.0)
    big = unpack_cmd(pack_cmd(0.0, np.full(12, 5.0), 120.0, 0.0, np.zeros(12)))
    assert np.allclose(servo_torque(big, np.zeros(12), np.zeros(12)), TAU_MAX)


def test_is_stale_boundary():
    assert not is_stale(1.0, 1.05)
    assert is_stale(1.0, 1.0501)


def test_no_cmd_means_no_torque():
    tau, stale = servo_step(None, 5.0, np.zeros(12), np.zeros(12))
    assert tau is None and stale is False


def test_stale_cmd_falls_back_to_hold():
    q, qd = Q_NOM12 + 0.05, np.full(12, 0.2)
    cmd = unpack_cmd(pack_cmd(1.0, np.zeros(12), 0.0, 1.0, np.full(12, 7.0)))
    tau, stale = servo_step(cmd, 1.2, q, qd)
    assert stale and np.allclose(tau, np.clip(KP_START * (Q_NOM12 - q) - KD_START * qd, -TAU_MAX, TAU_MAX))
    tau2, stale2 = servo_step(cmd, 1.03, q, qd)
    assert not stale2 and np.allclose(tau2, 7.0 - 1.0 * 0.2)
    h = hold_cmd(3.0)
    assert h[0] == 3.0 and np.allclose(h[1], Q_NOM12) and np.allclose(h[2], KP_START) and np.allclose(h[4], 0.0)


def test_gap_summary():
    assert gap_summary([]) == (0.0, 0.0, 0.0)
    p50, p99, mx = gap_summary([0.001] * 99 + [0.010])
    assert p50 == pytest.approx(1.0) and mx == pytest.approx(10.0) and 1.0 <= p99 <= 10.0
