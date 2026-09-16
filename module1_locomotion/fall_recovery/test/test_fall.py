import math

from fall_recovery.fall_recovery_node import (
    FAILED,
    FALLEN,
    IDLE,
    RECOVERED,
    RECOVERING,
    FallStateMachine,
    tilt_deg,
)


def _q_roll(deg: float):
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def _q_pitch(deg: float):
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def _sm(**kw):
    base = dict(fall_deg=60.0, upright_deg=20.0, debounce=0.3, recover_delay=1.5, max_retries=3)
    base.update(kw)
    return FallStateMachine(**base)


def test_tilt_deg_upright_is_zero():
    assert tilt_deg(0.0, 0.0, 0.0, 1.0) == 0.0


def test_tilt_deg_roll_90():
    assert abs(tilt_deg(*_q_roll(90.0)) - 90.0) < 1e-6


def test_tilt_deg_pitch_negative_uses_abs():
    assert abs(tilt_deg(*_q_pitch(-70.0)) - 70.0) < 1e-6


def test_tilt_deg_yaw_only_is_zero():
    h = math.radians(120.0) / 2
    assert abs(tilt_deg(0.0, 0.0, math.sin(h), math.cos(h))) < 1e-6


def test_idle_stays_idle_when_upright():
    sm = _sm()
    for t in (0.0, 0.1, 0.2, 5.0):
        assert sm.update(t, 5.0) is False
    assert sm.state == IDLE


def test_fall_requires_debounce():
    sm = _sm()
    sm.update(0.0, 90.0)
    assert sm.state == IDLE
    sm.update(0.2, 90.0)
    assert sm.state == IDLE
    sm.update(0.3, 90.0)
    assert sm.state == FALLEN


def test_short_tilt_spike_does_not_trigger():
    sm = _sm()
    sm.update(0.0, 90.0)
    sm.update(0.1, 5.0)
    sm.update(0.5, 90.0)  # 새로 카운트 시작
    assert sm.state == IDLE


def test_full_cycle_fallen_recovering_recovered_idle():
    sm = _sm()
    sm.update(0.0, 90.0)
    assert sm.update(0.3, 90.0) is False
    assert sm.state == FALLEN
    assert sm.update(1.0, 90.0) is False  # recover_delay 전
    assert sm.update(1.8, 90.0) is True  # 1.5 s 경과 → 리셋 요청
    assert sm.state == RECOVERING
    assert sm.retries == 1
    assert sm.update(2.0, 3.0) is False  # 바로 서긴 했지만 upright_hold 전
    assert sm.state == RECOVERING
    sm.update(2.5, 3.0)
    assert sm.state == RECOVERED
    sm.update(4.0, 3.0)
    assert sm.state == RECOVERED
    sm.update(4.5, 3.0)
    assert sm.state == IDLE


def test_recovering_retries_on_timeout_then_fails():
    sm = _sm()
    sm.update(0.0, 90.0)
    sm.update(0.3, 90.0)
    assert sm.update(1.8, 90.0) is True  # retry 1
    assert sm.update(6.7, 90.0) is False  # 4.9 s, 아직
    assert sm.update(6.8, 90.0) is True  # 5 s → retry 2
    assert sm.update(11.8, 90.0) is True  # retry 3
    assert sm.update(16.8, 90.0) is False  # retries == max → failed
    assert sm.state == FAILED
    assert sm.update(100.0, 3.0) is False  # failed는 고정
    assert sm.state == FAILED


def test_reset_failed_triggers_immediate_retry():
    sm = _sm()
    sm.update(0.0, 90.0)
    sm.update(0.3, 90.0)
    assert sm.update(1.8, 90.0) is True
    sm.reset_failed()
    assert sm.update(1.9, 90.0) is True
    assert sm.retries == 2


def test_upright_must_hold_continuously():
    sm = _sm()
    sm.update(0.0, 90.0)
    sm.update(0.3, 90.0)
    sm.update(1.8, 90.0)
    sm.update(2.0, 3.0)
    sm.update(2.3, 50.0)  # 흔들림 → 카운트 리셋
    sm.update(2.4, 3.0)
    sm.update(2.8, 3.0)
    assert sm.state == RECOVERING
    sm.update(2.9, 3.0)
    assert sm.state == RECOVERED
