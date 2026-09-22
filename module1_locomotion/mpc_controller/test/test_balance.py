import numpy as np
import pytest

from mpc_controller.balance_node import KD_RUN, KD_START, KP_START, blend, gains, joint_torque, q_startup, supervise
from mpc_controller.leg_kinematics import Q_NOM


def test_blend_is_zero_then_smooth_then_one():
    assert blend(0.0) == 0.0 and blend(1.99) == 0.0 and blend(3.0) == 1.0 and blend(10.0) == 1.0
    assert blend(2.5) == pytest.approx(0.5)
    ts = np.linspace(1.9, 3.1, 200)
    assert np.all(np.diff([blend(t) for t in ts]) >= -1e-12)  # 단조


def test_gains_go_from_start_to_run():
    assert gains(0.0) == (KP_START, KD_START) and gains(1.0) == (0.0, KD_RUN)
    assert gains(0.5) == pytest.approx((KP_START / 2, (KD_START + KD_RUN) / 2))


def test_q_startup_ramps_straight_to_nominal():
    assert np.allclose(q_startup(0.0), 0.0)
    assert np.allclose(q_startup(1.0), np.array(Q_NOM * 4) / 2)
    assert np.allclose(q_startup(2.0), Q_NOM * 4) and np.allclose(q_startup(50.0), Q_NOM * 4)


def test_torque_blend_is_continuous_and_clipped():
    q, qd = np.array(Q_NOM * 4), np.zeros(12)
    tau_ff = np.full(12, 5.0)
    t0 = joint_torque(0.0, *gains(0.0), q, q, qd, tau_ff)
    t1 = joint_torque(1e-6, *gains(1e-6), q, q, qd, tau_ff)
    assert np.allclose(t0, 0.0) and np.allclose(t1, 0.0, atol=1e-3)
    assert np.allclose(joint_torque(1.0, *gains(1.0), q, q, qd, tau_ff), 5.0)
    assert np.all(np.abs(joint_torque(1.0, 0.0, 0.0, q, q, qd, np.full(12, 99.0))) <= 30.0)


def test_feedforward_torque_virtual_work_sign():
    """τ_ff = −Jᵀf: 지면힘의 가상일 + 관절 토크의 가상일 = 0 (스파이크 실측 부호와 같은 규약)."""
    from mpc_controller.leg_kinematics import jacobian

    rng = np.random.default_rng(2)
    J = jacobian("FL", Q_NOM)
    f, dq = rng.normal(size=3), rng.normal(size=3) * 1e-3
    tau_ff = -J.T @ f
    assert abs(f @ (J @ dq) + tau_ff @ dq) < 1e-12
    assert np.allclose(-J.T @ np.zeros(3), 0.0)
    up = -J.T @ np.array([0.0, 0.0, 32.3])
    assert up[2] > 0 and abs(up[1]) < 0.1  # 기립: calf +2.5 N·m, thigh ≈ 0 (스파이크)


def test_supervise():
    assert supervise(0.0, 1.0, 0.01, 0.001) == "run"
    assert supervise(0.04, 1.0, 0.01, 0.001) == "run"  # 50 ms까지는 직전 f 유지
    assert supervise(0.06, 1.0, 0.01, 0.001) == "hold"
    assert supervise(0.0, 46.0, 0.01, 0.001) == "hold"
    assert supervise(0.0, 1.0, 0.2, 0.001) == "hold"
    assert supervise(0.0, 1.0, 0.01, 0.2) == "hold"
