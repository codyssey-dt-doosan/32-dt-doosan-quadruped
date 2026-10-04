import math
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("osqp")

from mpc_controller.balance_node import KD_RUN, KD_START, KP_START, blend, gains, joint_torque, q_startup, supervise
from mpc_controller.leg_kinematics import Q_NOM

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))


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


def test_log_stats_parses_status_line(tmp_path):
    from balance_metrics import log_stats  # scripts/ 경로는 conftest 없이 sys.path에 추가

    p = tmp_path / "x.log"
    p.write_text(
        "[INFO] status mode=run qp_ms=0.31/0.80 fail=0 sat=0 sum_fz=129.1 sum_fn=125.0 sum_ft=-33.5 calf_min=-1.351 "
        "tilt=15.02\n"
    )
    st = log_stats(str(p))
    assert st["mode"] == "run" and st["qp_ms"] == 0.31 and st["sum_fn"] == 125.0 and st["hold"] == 0
    assert st["calf_min"] == -1.351


def test_supervise():
    assert supervise(0.0, 1.0, 0.01, 0.001) == "run"
    assert supervise(0.04, 1.0, 0.01, 0.001) == "run"  # 50 ms까지는 직전 f 유지
    assert supervise(0.06, 1.0, 0.01, 0.001) == "hold"
    assert supervise(0.0, 46.0, 0.01, 0.001) == "hold"
    assert supervise(0.0, 1.0, 0.2, 0.001) == "hold"
    assert supervise(0.0, 1.0, 0.01, 0.2) == "hold"


def test_gz_unpause_returns_error_string_when_gz_missing(monkeypatch):
    from mpc_controller import balance_node

    monkeypatch.setattr(balance_node.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(OSError("no gz")))
    assert "no gz" in balance_node.gz_unpause("corridor")


def test_walk_metrics_velocity_track_and_log(tmp_path):
    from walk_metrics import log_stats, velocity_track

    rows = [(0.3 * t, 0.0, 0.3, 1.0, 0.0, 0.0, {}, t) for t in np.arange(0, 10, 0.1)]  # +x 0.3 m/s, yaw 0
    m = velocity_track(rows, 0.3, 0.0, after=2.0)
    assert m["v_meas"] == pytest.approx(0.3, abs=1e-6) and m["w_meas"] == pytest.approx(0.0, abs=1e-6)
    # z_std: 80개 동일값 0.3의 분산 계산도 이 환경(파이썬 3.11) 부동소수 누적합이 정확히 24.0이 아니어서(24.000000000000036)
    # 결과가 정확히 0.0이 아니라 ~4.4e-16으로 나온다 — 브리프의 `== 0.0`은 부동소수 노이즈를 못 견뎌 근사로 고침(진짜 결함 시엔 여전히 실패)
    assert m["err_v"] == pytest.approx(0.0, abs=1e-6) and m["tilt_max"] == 1.0 and m["z_std"] == pytest.approx(0.0, abs=1e-9)
    rows = [(0.0, 0.0, 0.3, 0.0, 0.0, math.degrees(0.5 * t), {}, t) for t in np.arange(0, 10, 0.1)]
    assert velocity_track(rows, 0.0, 0.5, after=2.0)["w_meas"] == pytest.approx(0.5, abs=1e-6)

    p = tmp_path / "walk.log"
    p.write_text(
        "[INFO] status mode=run qp_ms=1.7/2.3 fail=3 sat=0 sum_fz=134.8 sum_fn=0.0 sum_ft=0.0 calf_min=-1.124 "
        "tilt=0.24 ik_fail=0 ik_clamp=5 phase=0.96 active=1 v_cmd=0.30 w_cmd=0.00 v_est_max=0.41\n"
    )
    st = log_stats(str(p))
    assert st["ik_fail"] == 0 and st["ik_clamp"] == 5 and st["active"] == 1 and st["hold"] == 0


def test_swing_targets_reachable_on_slope():
    """스펙 §7: 15° 오르막·v 0.3·height 0.32에서 스윙 발 IK 목표가 관절 한계 안(실패 0). 보행 중 접촉 침투로 1.5 cm 가라앉아도(z −0.325) 도달."""
    from scipy.spatial.transform import Rotation

    from mpc_controller.body_estimate import estimate
    from mpc_controller.gait import Gait
    from mpc_controller.leg_kinematics import HIP, HIP_Y, SIDE
    from mpc_controller.leg_kinematics import ik as leg_ik
    from mpc_controller.legged_model import LEGS
    from mpc_controller.walk_node import swing_targets

    zero12 = np.zeros(12)
    q12 = np.concatenate([leg_ik(leg, [HIP[leg][0], HIP[leg][1] + SIDE[leg] * HIP_Y, -0.34]) for leg in LEGS])
    R = Rotation.from_euler("y", -math.radians(15)).as_matrix()  # 오르막 지형 평행(코 들기)
    e = estimate(q12, zero12, R, np.zeros(3), (True,) * 4, 0.02, [0.0, 0.0, -0.04])
    g = Gait(3.0, 0.5)
    g.tick(0.25, True)  # 위상 0.75 → FL·RR 스윙 중간
    q_sw, p_land, fails, _, _ = swing_targets(g, e, R, 0.3 * e.R_des[:, 0], 0.0, e.p, 0.02, 0.025, 0.03, 0.08, q12, g.contact())
    assert fails == 0 and not np.allclose(q_sw[:3], q12[:3]) and np.allclose(q_sw[3:6], q12[3:6])
    assert p_land[0][0] > e.p[0][0]  # 전진 → 앞 착지

    q12_sunk = np.concatenate([leg_ik(leg, [HIP[leg][0], HIP[leg][1] + SIDE[leg] * HIP_Y, -0.325]) for leg in LEGS])
    e_sunk = estimate(q12_sunk, zero12, R, np.zeros(3), (True,) * 4, 0.02, [0.0, 0.0, -0.04])
    _, _, fails_sunk, _, _ = swing_targets(g, e_sunk, R, 0.3 * e_sunk.R_des[:, 0], 0.0, e_sunk.p, 0.02, 0.025, 0.03, 0.08, q12_sunk, g.contact())
    assert fails_sunk == 0

    # 5차 실측 진단: lift 0.15(도달 불가)는 이제 hold 대신 ik_clamped가 흡수 — fails 0·clamps > 0이어야 한다
    _, _, fails_bad, clamps_bad, _ = swing_targets(g, e, R, 0.3 * e.R_des[:, 0], 0.0, e.p, 0.02, 0.15, 0.03, 0.08, q12, g.contact())
    assert fails_bad == 0 and clamps_bad > 0


def test_swing_targets_reachable_flat_fast():
    """평지·빠른 이동(v 0.5·w 1.0)에서도 스윙 발 IK 목표가 관절 한계 안(실패 0). 1.5 cm 가라앉아도(z −0.325) 도달."""
    from mpc_controller.body_estimate import estimate
    from mpc_controller.gait import Gait
    from mpc_controller.leg_kinematics import HIP, HIP_Y, SIDE
    from mpc_controller.leg_kinematics import ik as leg_ik
    from mpc_controller.legged_model import LEGS
    from mpc_controller.walk_node import swing_targets

    zero12 = np.zeros(12)
    q12 = np.concatenate([leg_ik(leg, [HIP[leg][0], HIP[leg][1] + SIDE[leg] * HIP_Y, -0.34]) for leg in LEGS])
    R = np.eye(3)
    e = estimate(q12, zero12, R, np.zeros(3), (True,) * 4, 0.02, [0.0, 0.0, -0.04])
    g = Gait(3.0, 0.5)
    g.tick(0.25, True)  # 위상 0.75 → FL·RR 스윙 중간
    _, _, fails, _, _ = swing_targets(g, e, R, 0.5 * e.R_des[:, 0], 1.0, e.p, 0.02, 0.025, 0.03, 0.08, q12, g.contact())
    assert fails == 0

    q12_sunk = np.concatenate([leg_ik(leg, [HIP[leg][0], HIP[leg][1] + SIDE[leg] * HIP_Y, -0.325]) for leg in LEGS])
    e_sunk = estimate(q12_sunk, zero12, R, np.zeros(3), (True,) * 4, 0.02, [0.0, 0.0, -0.04])
    _, _, fails_sunk, _, _ = swing_targets(g, e_sunk, R, 0.5 * e_sunk.R_des[:, 0], 1.0, e_sunk.p, 0.02, 0.025, 0.03, 0.08, q12_sunk, g.contact())
    assert fails_sunk == 0


def test_swing_targets_land_at_neutral_y_when_still():
    """실측 결함: 착지 기준이 hip 롤 축(y ±0.10)이면 중립 발 위치(y ±0.14)보다 4 cm 안쪽에 착지 → 무릎 과굴곡. 정지(v=w=0)면 착지점이 곧 기준점이어야 한다."""
    from mpc_controller.body_estimate import estimate
    from mpc_controller.gait import Gait
    from mpc_controller.leg_kinematics import HIP, HIP_Y, SIDE
    from mpc_controller.leg_kinematics import ik as leg_ik
    from mpc_controller.legged_model import LEGS
    from mpc_controller.walk_node import swing_targets

    zero12 = np.zeros(12)
    q12 = np.concatenate([leg_ik(leg, [HIP[leg][0], HIP[leg][1] + SIDE[leg] * HIP_Y, -0.34]) for leg in LEGS])
    R = np.eye(3)
    e = estimate(q12, zero12, R, np.zeros(3), (True,) * 4, 0.02, [0.0, 0.0, -0.04])
    g = Gait(3.0, 0.5)
    g.tick(0.25, True)  # 위상 0.75 → FL·RR 스윙 중간
    _, p_land, _, _, _ = swing_targets(g, e, R, np.zeros(3), 0.0, e.p, 0.02, 0.025, 0.03, 0.08, q12, g.contact())
    assert p_land[0][1] == pytest.approx(0.14, abs=1e-6)
    assert p_land[0][0] == pytest.approx(0.19, abs=1e-6)
    assert p_land[3][1] == pytest.approx(-0.14, abs=1e-6)


def test_window_rows_slices_and_rezeroes():
    """--replay-window: s ≤ t < s+d만 골라 t -= s로 재영점, v·w 값은 보존."""
    from walk_metrics import window_rows

    rows = [(float(t), t * 0.1, t * 0.01) for t in range(6)]  # t=0..5
    out = window_rows(rows, 2.0, 3.0)
    assert len(out) == 3
    for i, (t, v, w) in enumerate(out):
        assert t == pytest.approx(i)
        assert v == pytest.approx((i + 2) * 0.1)
        assert w == pytest.approx((i + 2) * 0.01)


def test_replay_errors_stride_window_ignores_trot_sway_and_accel(tmp_path):
    """기준 2: 3 Hz 전후 흔들림은 보폭 창 평균에서 사라지고, 명령 변화 후 0.5 s 창은 채점 안 함."""
    import math

    from walk_metrics import replay_errors

    cmd = tmp_path / "c.csv"
    cmd.write_text("".join(f"{i / 10:.1f},{0.3 if i < 30 else 0.0},0.0\n" for i in range(60)))  # 3 s 전진 → 3 s 정지
    rows = []
    for k in range(1201):  # 200 Hz 6 s, 명령 변화 즉시 반응 + 진폭 1 cm 3 Hz 흔들림
        t = k / 200
        x = 0.3 * min(t, 3.0) + 0.01 * math.sin(2 * math.pi * 3 * t)
        rows.append((x, 0.0, 0.3, 0.0, 0.0, 0.0, None, 100.0 + t))
    e = replay_errors(rows, str(cmd), 100.0)
    assert e["rms_v"] < 0.01 and e["rms_w"] < 1e-9
    assert 10 <= e["n_win"] < 18  # 18창 중 시작·정지 직후 창 제외
    assert e["path_len"] == pytest.approx(0.9, abs=1e-6)


def test_replay_errors_ignores_capture_tail_after_last_command(tmp_path):
    """캡처는 마지막 명령 뒤 여유(로봇은 cmd_timeout으로 정지)까지 이어짐 — 그 구간을 마지막 명령으로 채점하면 안 됨."""
    from walk_metrics import replay_errors

    cmd = tmp_path / "c.csv"
    cmd.write_text("".join(f"{i / 10:.1f},0.3,0.0\n" for i in range(30)))  # 0~2.9 s v 0.3, 이후 명령 없음
    rows = [(0.3 * min(k / 200, 3.0), 0.0, 0.3, 0.0, 0.0, 0.0, None, 100.0 + k / 200) for k in range(1001)]  # 3 s 뒤 정지, 5 s까지
    e = replay_errors(rows, str(cmd), 100.0)
    assert e["rms_v"] < 0.01


def test_sensor_touching_uses_freshness():
    """gz contact 센서는 닿아 있을 때만 발행 — 마지막 수신이 hold 안이면 접촉, None이면 공중."""
    from mpc_controller.walk_node import sensor_touching

    out = sensor_touching([None, 10.0, 9.995, 9.98], 10.0, 0.01)
    assert out.tolist() == [False, True, True, False]



def test_touchdown_delay_sign_and_scuff():
    """지지 구간 착지 = +지연, 스윙 후반 착지 = −(이름), 스윙 전반 접촉 = 긁힘(None). 3 Hz·duty 0.5."""
    from mpc_controller.walk_node import td_summary, touchdown_delay

    assert touchdown_delay(0.069, 0.5, 3.0) == pytest.approx(0.023)
    assert touchdown_delay(0.97, 0.5, 3.0) == pytest.approx(-0.01)
    assert touchdown_delay(0.6, 0.5, 3.0) is None  # s 0.2
    assert td_summary([]) == "-"
    assert td_summary([0.02, 0.023, 0.026]).endswith("/3") and td_summary([0.02, 0.023, 0.026]).startswith("23/")



def test_ik_clamped_respects_knee_margin():
    """r_max = reach(knee)면 도달 밖 목표에서 calf가 −knee로 멈춤(완전히 펴지지 않음). 안쪽 목표는 그대로."""
    from mpc_controller.leg_kinematics import HIP, HIP_Y, L1, L2, SIDE, ik_clamped, reach

    assert reach(0.0) == pytest.approx(L1 + L2)
    far = [HIP["FL"][0], HIP["FL"][1] + SIDE["FL"] * HIP_Y, -0.45]  # 다리 길이 밖, 바로 아래
    q_full, c_full = ik_clamped("FL", far)
    q_marg, c_marg = ik_clamped("FL", far, reach(0.7))
    assert c_full and c_marg
    assert q_full[2] > -0.2  # 기본 상한: 거의 다 펴짐
    assert q_marg[2] == pytest.approx(-0.7, abs=1e-6)
    near = [HIP["FL"][0], HIP["FL"][1] + SIDE["FL"] * HIP_Y, -0.30]
    assert ik_clamped("FL", near, reach(0.7)) == ik_clamped("FL", near)
