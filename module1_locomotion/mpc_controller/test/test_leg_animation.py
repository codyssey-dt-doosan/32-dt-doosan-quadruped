import math

from mpc_controller.leg_animation_node import gait_speed, leg_targets

A_T, A_C = 0.3, 0.5


def test_stopped_legs_straight():
    for phase in (0.0, 1.0, math.pi, 5.0):
        assert leg_targets(phase, 0.0, A_T, A_C) == (0.0, 0.0, 0.0, 0.0)


def test_diagonal_pairs_antiphase():
    for phase in (0.3, 1.7, 4.0):
        thigh_a, thigh_b, _, _ = leg_targets(phase, 1.0, A_T, A_C)
        assert math.isclose(thigh_a, -thigh_b)


def test_calf_bends_only_in_swing():
    # thigh 양수 = 다리 뒤로. 스윙(앞으로 되돌아옴) = cos(phase) < 0 구간에서만 무릎 굽힘
    for k in range(40):
        phase = k * math.tau / 40
        thigh_a, thigh_b, calf_a, calf_b = leg_targets(phase, 1.0, A_T, A_C)
        assert abs(thigh_a) <= A_T + 1e-9 and abs(thigh_b) <= A_T + 1e-9
        assert 0.0 <= calf_a <= A_C + 1e-9 and 0.0 <= calf_b <= A_C + 1e-9
        assert (calf_a > 1e-9) == (math.cos(phase) < -1e-9)
        assert calf_a == 0.0 or calf_b == 0.0  # 두 쌍이 동시에 발을 들지 않음


def test_gait_speed_counts_turning():
    assert gait_speed(0.0, 0.0, 0.3) == 0.0
    assert gait_speed(-0.4, 0.0, 0.3) == 0.4
    assert math.isclose(gait_speed(0.0, 1.0, 0.3), 0.3)
