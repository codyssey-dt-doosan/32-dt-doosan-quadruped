import math

from fall_recovery.fall_recovery_node import (
    FAILED,
    FALLEN,
    IDLE,
    RECOVERED,
    RECOVERING,
    FallStateMachine,
    model_z,
    reset_z,
    set_pose_cmd,
    set_pose_req,
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


def test_set_pose_req_keeps_xy_yaw_and_levels_body():
    req = set_pose_req(1.5, -2.0, math.pi / 2, 0.4)
    assert 'name: "go2"' in req
    assert "x: 1.5" in req and "y: -2.0" in req and "z: 0.4" in req
    # yaw 90° → qz = sin(45°), qw = cos(45°), qx = qy = 0
    assert "orientation: {x: 0.0, y: 0.0, z: 0.7071" in req
    assert "w: 0.7071" in req


def test_set_pose_cmd_targets_world_service():
    cmd = set_pose_cmd("factory", 'name: "go2"')
    assert cmd[:3] == ["gz", "service", "-s"]
    assert "/world/factory/set_pose" in cmd
    assert "gz.msgs.Pose" in cmd and "gz.msgs.Boolean" in cmd
    assert cmd[-1] == 'name: "go2"'


GZ_MODEL_OUT = """Requesting state for world [unity_plant]...

Model: [2332]
  - Name: go2
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [7.876220 0.076232 1.012000]
    [0.005089 0.000194 0.156080]
"""


def test_model_z_parses_gz_model_pose():
    assert model_z(GZ_MODEL_OUT) == 1.012
    assert model_z("Service call timed out") is None


def test_reset_z_keeps_stand_z_on_flat_floor():
    assert reset_z(0.4, None) == 0.4      # gz 조회 실패
    assert reset_z(0.4, 0.15) == 0.4     # 평지에 누운 몸체 — 기존 동작 그대로
    assert reset_z(0.4, 0.45) == 0.4


def test_reset_z_lifts_above_raised_floor():
    # unity_plant 경사로 위 플랫폼(바닥 0.875 m)에 누운 몸체 z ≈ 0.975 → 바닥 위 stand_z
    assert math.isclose(reset_z(0.4, 0.975), 0.875 + 0.4)
