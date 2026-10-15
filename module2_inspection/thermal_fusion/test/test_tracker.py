"""과열 지점 추적 상태 전이."""

import pytest

from thermal_fusion.tracker import (
    CANDIDATE, CONFIRMED, FALSE_ALARM, INSPECTING, PENDING, Tracker, UNREACHABLE,
)


def planner(tx, ty, fx, fy):
    return (tx - 1.5, ty, 0.0), ""


def make(**kw):
    return Tracker(plan_goal=planner, t_confirm=60.0, **kw)


def see(tr, n, x=7.65, y=-6.0, t=85.0, start=0.0):
    for i in range(n):
        tr.observe(x + 0.05 * (i % 2), y, t, 5.0, -6.0, start + 0.1 * i)


def test_repeated_detections_merge_and_become_pending():
    tr = make()
    see(tr, 2)
    assert len(tr.spots) == 1 and tr.spots[0].state == CANDIDATE
    see(tr, 1, start=0.2)
    s = tr.spots[0]
    assert s.state == PENDING and s.n_obs == 3
    assert tr.goal() is None                              # update 전에는 활성 지점 없음
    tr.update(5.0, -6.0, 0.3, 0.3)
    assert tr.goal() == pytest.approx((s.x - 1.5, -6.0, 0.0))


def test_far_detection_is_new_spot():
    tr = make()
    tr.observe(7.65, -6.0, 85.0, 5.0, -6.0, 0.0)
    tr.observe(0.0, 9.0, 85.0, 0.0, 0.0, 0.0)
    assert len(tr.spots) == 2


def test_candidate_forgotten():
    tr = make()
    see(tr, 2)
    tr.update(5.0, -6.0, 0.0, 3.5)
    assert tr.spots == []


def test_candidate_forgotten_without_odom():
    tr = make()
    see(tr, 2)
    tr.forget(3.5)                                        # 위치를 몰라도 오래된 후보는 지운다
    assert tr.spots == []


def _pending(tr):
    see(tr, 3)
    tr.update(0.0, 0.0, 0.5, 1.0)
    return tr.spots[0]


def test_inspect_confirms():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx + 0.2, gy, 0.01, 10.0)
    assert s.state == INSPECTING
    tr.observe(s.x, s.y, 80.0, gx, gy, 12.0)
    tr.update(gx, gy, 0.0, 14.9)
    assert s.state == INSPECTING
    tr.update(gx, gy, 0.0, 15.0)
    assert s.state == CONFIRMED
    assert tr.alert() is True and tr.goal() is None


def test_inspect_without_heat_is_false_alarm():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 10.0)
    tr.update(gx, gy, 0.0, 15.0)
    assert s.state == FALSE_ALARM and tr.alert() is False


def test_moving_robot_does_not_start_inspection():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.3, 10.0)
    assert s.state == PENDING


def test_timeout_retries_once_then_unreachable():
    tr = make()
    s = _pending(tr)                                      # goal_since = 1.0
    tr.update(0.0, 0.0, 0.5, 61.5)
    assert s.state == PENDING and s.reason == "unreachable" and s.attempts == 1
    tr.update(0.0, 0.0, 0.5, 62.0)                        # 재시도 시작
    tr.update(0.0, 0.0, 0.5, 122.5)
    assert s.state == UNREACHABLE and tr.goal() is None and tr.alert() is False


def test_reason_cleared_after_later_arrival():
    tr = make()
    s = _pending(tr)
    tr.update(0.0, 0.0, 0.5, 61.5)                        # 첫 시간 초과 → "unreachable"
    assert s.reason == "unreachable"
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 70.0)                          # 이후 도착·정차
    assert s.state == INSPECTING and s.reason == ""
    tr.update(gx, gy, 0.0, 75.0)
    assert s.state == FALSE_ALARM and s.reason == ""


def test_planner_failure_is_unreachable():
    tr = Tracker(plan_goal=lambda *a: (None, "no_space:wall"), t_confirm=60.0)
    see(tr, 3)
    s = tr.spots[0]
    assert s.state == UNREACHABLE and s.reason == "no_space:wall"


def test_decided_spot_is_not_recreated():
    tr = make()
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 10.0)
    tr.observe(s.x, s.y, 80.0, gx, gy, 11.0)
    tr.update(gx, gy, 0.0, 15.0)
    see(tr, 5, start=100.0)
    assert len(tr.spots) == 1 and s.state == CONFIRMED
    tr.update(gx, gy, 0.0, 101.0)
    assert tr.goal() is None


def test_recheck_resets_after_time():
    tr = make(recheck_s=600.0)
    s = _pending(tr)
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 10.0)
    tr.update(gx, gy, 0.0, 15.0)                          # FALSE_ALARM at 15
    tr.observe(s.x, s.y, 85.0, 5.0, -6.0, 700.0)
    assert s.state == CANDIDATE and s.n_obs == 1


def test_recheck_does_not_time_out_immediately():
    """이전 주기의 goal_since가 남아 재점검 goal이 바로 시간 초과되던 버그의 회귀 테스트."""
    tr = make(recheck_s=600.0)
    s = _pending(tr)                                      # goal_since = 1.0
    gx, gy, _ = s.goal
    tr.update(gx, gy, 0.0, 10.0)
    tr.update(gx, gy, 0.0, 15.0)                          # FALSE_ALARM, goal_since는 1.0으로 남아 있음
    see(tr, 3, start=800.0)                               # 재점검 주기 → 다시 PENDING
    assert s.state == PENDING
    tr.update(0.0, 0.0, 0.5, 801.0)
    assert s.state == PENDING and s.attempts == 0 and s.reason == ""


def test_active_spot_is_sticky():
    tr = make()
    see(tr, 3, x=7.65)                                    # goal (6.15, -6)
    see(tr, 3, x=0.0, y=9.0)                              # goal (-1.5, 9)
    tr.update(6.0, -6.0, 0.5, 1.0)
    first = tr.goal()
    tr.update(-1.0, 9.0, 0.5, 2.0)                        # 로봇이 다른 지점 쪽으로 밀려도
    assert tr.goal() == first


def test_headline_prefers_active_then_confirmed():
    tr = make()
    assert tr.headline() is None
    see(tr, 2)
    assert tr.headline().state == CANDIDATE
    see(tr, 1, start=0.2)
    tr.update(5.0, -6.0, 0.5, 0.3)
    assert tr.headline().state == PENDING
