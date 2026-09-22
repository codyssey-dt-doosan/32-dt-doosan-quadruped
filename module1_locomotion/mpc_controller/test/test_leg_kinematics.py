import math

import numpy as np
import pytest

from mpc_controller.leg_kinematics import (
    FOOT_Z_NOM,
    HIP,
    LIMITS,
    Q_NOM,
    SIDE,
    foot_pos,
    ik,
    jacobian,
)
from mpc_controller.legged_model import LEGS


def _fd_jacobian(leg, q, eps=1e-6):
    j = np.zeros((3, 3))
    for i in range(3):
        d = np.zeros(3)
        d[i] = eps
        j[:, i] = (foot_pos(leg, np.add(q, d)) - foot_pos(leg, np.subtract(q, d))) / (2 * eps)
    return j


@pytest.mark.parametrize("leg", LEGS)
def test_fk_stand_matches_spike(leg):
    p = foot_pos(leg, Q_NOM)
    assert p == pytest.approx((HIP[leg][0], SIDE[leg] * 0.14, FOOT_Z_NOM), abs=1e-4)


def test_fk_straight_leg_hangs_below_hip():
    p = foot_pos("FL", (0.0, 0.0, 0.0))
    assert p == pytest.approx((0.19, 0.14, -0.08 - 0.18 - 0.10), abs=1e-9)


def test_hip_roll_rotates_about_x():
    p = foot_pos("FL", (0.3, 0.45, -1.35))
    p0 = foot_pos("FL", Q_NOM)
    assert p[0] == pytest.approx(p0[0])  # x는 hip 롤에 안 변함
    assert math.hypot(p[1] - 0.10, p[2]) == pytest.approx(math.hypot(p0[1] - 0.10, p0[2]))


@pytest.mark.parametrize("leg", LEGS)
def test_jacobian_matches_finite_difference(leg):
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = [rng.uniform(lo, hi) for lo, hi in LIMITS]
        assert np.allclose(jacobian(leg, q), _fd_jacobian(leg, q), atol=1e-6)


def test_straight_leg_is_singular():
    assert np.linalg.cond(jacobian("FL", (0.0, 0.0, 0.0))) > 1e3
    assert np.linalg.cond(jacobian("FL", Q_NOM)) < 50


def test_symmetry():
    fl, fr, rl, rr = (foot_pos(leg, Q_NOM) for leg in ("FL", "FR", "RL", "RR"))
    assert np.allclose(fl * [1, -1, 1], fr)  # 좌우는 거울
    assert np.allclose(fl - [0.38, 0, 0], rl) and np.allclose(fr - [0.38, 0, 0], rr)  # 앞뒤는 평행 이동


@pytest.mark.parametrize("leg", LEGS)
def test_ik_round_trip(leg):
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = (rng.uniform(-0.8, 0.8), rng.uniform(-0.3, 1.5), rng.uniform(-1.5, -0.3))
        sol = ik(leg, foot_pos(leg, q))
        assert sol is not None and np.allclose(sol, q, atol=1e-6)


def test_ik_returns_none_outside_reach_or_limits():
    assert ik("FL", (0.19, 0.14, -0.60)) is None  # 다리 길이 0.36 밖
    assert ik("FL", (0.19, 0.14, -0.05)) is None  # calf 한계 −1.57 밖으로 접어야 함


def test_ik_reach_on_15deg_slope_with_com_shift():
    """스펙 M7: 15°에서 CoM을 중력 방향 발 중심 위로 옮기면 앞발은 몸체 기준 8.6 cm 뒤, 뒷발은 앞으로 간다."""
    shift = 0.28 * math.tan(math.radians(15))
    for leg in LEGS:
        p = foot_pos(leg, Q_NOM) - np.array([shift, 0.0, 0.0])  # 몸체가 경사 위쪽(+x)으로 이동 = 발은 −x
        assert ik(leg, p) is not None, leg
