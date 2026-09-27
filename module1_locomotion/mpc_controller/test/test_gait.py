import numpy as np
import pytest

from mpc_controller.gait import Gait, OFFSETS, contact_at, crossed_half, raibert, swing_foot, swing_s

Z = np.array([0.0, 0.0, 1.0])
O = np.array([0.0, 0.0, -0.3])


def test_contact_at_diagonal_pairs():
    assert np.array_equal(OFFSETS, [0.0, 0.5, 0.5, 0.0])  # FL FR RL RR
    assert np.array_equal(contact_at(0.1, 0.5), [True, False, False, True])
    assert np.array_equal(contact_at(0.6, 0.5), [False, True, True, False])
    assert contact_at(0.3, 1.0).all()


def test_swing_s():
    assert swing_s(0.2, 0.5) is None and swing_s(0.5, 0.5) == 0.0 and swing_s(0.75, 0.5) == pytest.approx(0.5)


def test_crossed_half():
    assert crossed_half(0.45, 0.55) and crossed_half(0.95, 0.02) and not crossed_half(0.1, 0.2) and not crossed_half(0.6, 0.9)


def test_gait_inactive_is_all_stance_and_stops_only_at_boundary():
    g = Gait(3.0, 0.5)
    assert g.contact().all() and g.schedule(5, 0.03).all() and not g.active
    g.tick(0.005, True)
    assert g.active and g.phase == pytest.approx(0.015)
    g.tick(0.1, False)  # 0.015 → 0.315: 경계 안 지남 → 계속
    assert g.active
    g.tick(0.1, False)  # 0.315 → 0.615: 0.5 지남 → 정지, 위상 0.5로 스냅
    assert not g.active and g.phase == pytest.approx(0.5) and g.contact().all()
    g.tick(0.01, True)
    assert g.active and g.phase == pytest.approx(0.53)


def test_trusted_excludes_stance_edges():
    g = Gait(3.0, 0.5)
    assert g.trusted().all() and not g.active  # 비활성 → 전부 True

    g.tick(0.02, True)  # 위상 0.06 → FL·RR 진행 0.12(<0.15), FR·RL 스윙(진행 >1)
    assert not g.trusted().any()

    g.tick(0.08, True)  # 위상 0.30 → FL·RR 진행 0.6(지지 중간), FR·RL 여전히 스윙
    assert np.array_equal(g.trusted(), [True, False, False, True])


def test_schedule_and_timing():
    g = Gait(3.0, 0.5)
    g.tick(0.1, True)  # phase 0.3
    s = g.schedule(10, 0.03)
    assert s.shape == (10, 4) and np.array_equal(s[0], contact_at(0.3, 0.5))
    assert np.array_equal(s[3], contact_at(0.3 + 3 * 0.09, 0.5))
    assert g.t_stance == pytest.approx(0.5 / 3.0)
    assert g.t_remaining(1) == pytest.approx((1.0 - 0.8) / 3.0)  # FR 위상 0.8 스윙
    assert g.t_remaining(0) == 0.0  # FL 지지


def test_raibert_under_hip_when_still_and_forward_when_moving():
    hip = np.array([0.19, 0.10, 0.0])
    p = raibert(hip, O, Z, np.zeros(3), np.zeros(3), 0.0, 0.05, 0.167, 0.03, 0.12)
    assert np.allclose(p[:2], hip[:2]) and p[2] == pytest.approx(-0.3)
    v = np.array([0.3, 0.0, 0.0])
    p = raibert(hip, O, Z, v, v, 0.0, 0.05, 0.167, 0.03, 0.12)
    assert p[0] == pytest.approx(0.19 + 0.3 * 0.05 + 0.3 * 0.167 / 2) and p[2] == pytest.approx(-0.3)
    p = raibert(hip, O, Z, np.zeros(3), v, 0.0, 0.05, 0.167, 0.03, 0.12)
    assert p[0] < 0.19 + 0.3 * 0.05  # v_est < v_ref → 뒤로 당김


def test_raibert_clamps_and_projects_on_slope():
    hip = np.array([0.19, 0.10, 0.0])
    v = np.array([3.0, 0.0, 0.0])
    p = raibert(hip, O, Z, v, v, 0.0, 0.05, 0.167, 0.03, 0.12)
    assert np.hypot(*(p[:2] - (hip[:2] + v[:2] * 0.05))) == pytest.approx(0.12)
    n = np.array([-np.sin(0.3), 0, np.cos(0.3)])
    p = raibert(hip, O, n, np.zeros(3), np.zeros(3), 0.0, 0.0, 0.167, 0.03, 0.12)
    assert (p - O) @ n == pytest.approx(0.0, abs=1e-12)


def test_raibert_turning_rotates_hip():
    hip = np.array([0.19, 0.10, 0.0])
    p = raibert(hip, O, Z, np.zeros(3), np.zeros(3), 1.0, 0.1, 0.167, 0.03, 0.12)
    exp = np.array([[np.cos(0.1), -np.sin(0.1)], [np.sin(0.1), np.cos(0.1)]]) @ hip[:2]
    assert np.allclose(p[:2], exp)


def test_swing_foot_endpoints_and_apex():
    a, b = np.array([0.0, 0.0, -0.3]), np.array([0.1, 0.0, -0.3])
    assert np.allclose(swing_foot(a, b, 0.0, 0.06, Z), a)
    assert np.allclose(swing_foot(a, b, 0.85, 0.06, Z), b)  # land_at: 정확히 착지
    assert np.allclose(swing_foot(a, b, 1.0, 0.06, Z), b - 0.01 * Z)  # 착지 후 push만큼 눌러 내림
    mid = swing_foot(a, b, 0.425, 0.06, Z)  # land_at의 절반 위상 → 궤적 중간(정점)
    assert mid[0] == pytest.approx(0.05) and mid[2] == pytest.approx(-0.3 + 0.06)
    assert swing_foot(a, b, 0.95, 0.06, Z)[2] < b[2]
