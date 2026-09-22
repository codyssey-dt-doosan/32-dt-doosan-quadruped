import math

import numpy as np
import pytest

osqp = pytest.importorskip("osqp")

from mpc_controller.force_qp import ForceQP, tangents, wrench_matrix  # noqa: E402

M, G = 13.2, 9.8
R_FLAT = np.array([[0.19, 0.14, -0.28], [0.19, -0.14, -0.28], [-0.19, 0.14, -0.28], [-0.19, -0.14, -0.28]])
Z = np.array([0.0, 0.0, 1.0])
ALL = (True, True, True, True)


def _b(a=(0, 0, 0), alpha=(0, 0, 0)):
    return np.concatenate([M * (np.asarray(a, float) + G * Z), np.asarray(alpha, float)])


def _slope(deg):
    th = math.radians(deg)
    n = np.array([math.sin(th), 0.0, math.cos(th)])  # 오르막이 +x
    r = R_FLAT.copy()
    r[:, 2] += -np.tan(th) * r[:, 0]  # 발 평면을 기울임
    return n, r


def test_static_flat_splits_weight_evenly():
    f = ForceQP().solve(R_FLAT, Z, _b(), ALL, np.zeros(12))
    fz = f.reshape(4, 3)[:, 2]
    assert fz == pytest.approx([M * G / 4] * 4, abs=0.05)
    assert np.abs(f.reshape(4, 3)[:, :2]).max() < 1e-3


def test_slope15_sums_and_statics():
    n, r = _slope(15)
    f = ForceQP().solve(r, n, _b(), ALL, np.zeros(12)).reshape(4, 3)
    t1, _ = tangents(n)
    assert (f @ n).sum() == pytest.approx(M * G * math.cos(math.radians(15)), rel=0.01)
    assert (f @ t1).sum() == pytest.approx(-M * G * math.sin(math.radians(15)), rel=0.02)  # 지면이 오르막(−t1) 쪽으로 밈
    assert np.allclose(wrench_matrix(r) @ f.ravel(), _b(), atol=0.5)  # 힘·모멘트 잔차 (N, N·m)


def test_friction_saturates_on_pyramid_edge():
    qp = ForceQP(mu=0.5)
    f = qp.solve(R_FLAT, Z, _b(a=(20.0, 0, 0)), ALL, np.zeros(12)).reshape(4, 3)  # 2 g 요구 → 불가
    assert np.allclose(f[:, 0], 0.5 * f[:, 2], atol=1e-3)


def test_three_leg_contact_zeroes_swing_foot():
    f = ForceQP().solve(R_FLAT, Z, _b(), (True, True, True, False), np.zeros(12)).reshape(4, 3)
    assert np.allclose(f[3], 0.0, atol=1e-9)
    assert f[:3, 2].sum() == pytest.approx(M * G, rel=0.01)


def test_no_contact_returns_none():
    assert ForceQP().solve(R_FLAT, Z, _b(), (False,) * 4, np.zeros(12)) is None


def test_fmax_caps_each_foot_and_reports_residual():
    """소프트 등식이라 불능은 없다 — f_max에 걸리면 각 발 f_z = f_max로 포화하고 A f ≠ b 잔차가 남는다."""
    qp = ForceQP(f_max=20.0)
    f = qp.solve(R_FLAT, Z, _b(), ALL, np.zeros(12))
    assert f is not None and np.allclose(f.reshape(4, 3)[:, 2], 20.0, atol=1e-3)
    assert (wrench_matrix(R_FLAT) @ f)[2] < M * G * 0.7


def test_same_inputs_are_deterministic():
    """워밍스타트가 같은 입력의 답을 바꾸지 않는다(f_prev=a로 다시 풀면 β 항 때문에 달라지는 게 정상)."""
    qp = ForceQP()
    a = qp.solve(R_FLAT, Z, _b(), ALL, np.zeros(12))
    b = qp.solve(R_FLAT, Z, _b(), ALL, np.zeros(12))
    assert np.allclose(a, b, atol=1e-6)


def test_pattern_update_equals_fresh_setup():
    """스펙 M4: 수평으로 setup한 뒤 15°로 update한 결과가 15°로 새로 만든 것과 같다(희소 패턴 고정)."""
    n, r = _slope(15)
    qp = ForceQP()
    qp.solve(R_FLAT, Z, _b(), ALL, np.zeros(12))
    updated = qp.solve(r, n, _b(), ALL, np.zeros(12))
    fresh = ForceQP().solve(r, n, _b(), ALL, np.zeros(12))
    assert np.allclose(updated, fresh, atol=1e-4)


def test_beta_pulls_toward_previous():
    prev = np.zeros(12)
    prev[2::3] = [40.0, 40.0, 24.7, 24.7]
    f_small = ForceQP(beta=1e-2).solve(R_FLAT, Z, _b(), ALL, prev).reshape(4, 3)[:, 2]
    f_big = ForceQP(beta=10.0).solve(R_FLAT, Z, _b(), ALL, prev).reshape(4, 3)[:, 2]
    assert abs(f_big[0] - 40.0) < abs(f_small[0] - 40.0)


def test_wrench_matrix_moment_sign():
    A = wrench_matrix(R_FLAT)
    f = np.zeros(12)
    f[2] = 10.0  # FL(+x,+y)에 +z 힘 → 모멘트 r×f = (y·fz, −x·fz, 0)
    assert np.allclose(A @ f, [0, 0, 10, 0.14 * 10, -0.19 * 10, 0])
