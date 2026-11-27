"""학습 venv 정합 테스트: Genesis 환경이 sim2sim.json 값으로 dof 순서·PD·주기·부호·관측·DR을 설정했는지."""
import sys
from pathlib import Path

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import genesis as gs  # noqa: E402
from genesis.utils.geom import inv_quat, transform_by_quat  # noqa: E402

import cfgs  # noqa: E402
from go2_box_env import Go2BoxEnv  # noqa: E402
from mpc_controller.leg_kinematics import foot_pos  # noqa: E402

S2S = cfgs.load_sim2sim()
DR_RANGE = {**{k: S2S["dr"][k] for k in ("friction", "added_mass", "kp_scale", "kd_scale")},
            "latency_k": S2S["dr"]["latency_steps"]}
CALF_FOOT = (0.0, 0.0, -0.1)  # calf 링크 원점 → 발 구 중심(URDF sphere origin)


def make_env(num_envs, dr):
    return Go2BoxEnv(num_envs, *cfgs.get_cfgs(S2S, dr=dr))


@pytest.fixture(scope="module")
def env():
    if not gs._initialized:  # 다른 테스트 파일이 먼저 돌았을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")
    return make_env(2, dr=False)


@pytest.fixture(scope="module")
def env_dr(env):  # env가 gs.init을 먼저 부른다
    return make_env(64, dr=True)


def set_joint_positions_by_name(env, names, q):
    idx = [env.robot.get_joint(n).dofs_idx_local[0] for n in names]
    env.robot.set_dofs_position(torch.tensor(q, dtype=gs.tc_float).repeat(env.num_envs, 1), idx, zero_velocity=True)


def foot_in_base(env, leg, offset):
    q = np.array(S2S["q_nom"]) + [offset.get(n, 0.0) for n in S2S["joint_names"]]
    set_joint_positions_by_name(env, S2S["joint_names"], q)
    calf = env.robot.get_link(f"{leg}_calf")
    foot_w = calf.get_pos() + transform_by_quat(torch.tensor(CALF_FOOT, dtype=gs.tc_float), calf.get_quat())
    foot_b = transform_by_quat(foot_w - env.robot.get_pos(), inv_quat(env.robot.get_quat()))
    return foot_b[0].cpu().numpy()


def sampled_dr(env):
    """Genesis에 실제로 들어간 DR 값(지연만 env 버퍼)."""
    robot = env.robot
    ratio = robot.solver.get_geoms_friction_ratio(range(robot.geom_start, robot.geom_end))
    assert torch.allclose(ratio, ratio[:, :1])  # 한 env의 모든 geom이 같은 마찰
    return {
        # Genesis 접촉 마찰 = max(두 geom 마찰 × 비율)
        "friction": torch.clamp(ratio[:, 0] * robot.geoms[0].get_friction(), min=env.plane.geoms[0].get_friction()),
        "added_mass": robot.get_mass() - S2S["nominal"]["total_mass"],
        "kp_scale": robot.get_dofs_kp(env.motors_dof_idx) / S2S["kp"],
        "kd_scale": robot.get_dofs_kv(env.motors_dof_idx) / S2S["kd"],
        "latency_k": env.latency_k.float(),
    }


def test_dof_order_by_name(env):  # JSON 순서 = env.dof_pos 순서
    q = np.array(S2S["q_nom"]) + 0.01 * np.arange(12)
    set_joint_positions_by_name(env, S2S["joint_names"], q)
    assert np.allclose(env.robot.get_dofs_position(env.motors_dof_idx)[0].cpu(), q, atol=1e-6)


def test_pd_force_timing(env, monkeypatch):
    assert np.allclose(env.robot.get_dofs_kp(env.motors_dof_idx).cpu(), S2S["kp"])
    assert np.allclose(env.robot.get_dofs_kv(env.motors_dof_idx).cpu(), S2S["kd"])
    lo, hi = env.robot.get_dofs_force_range(env.motors_dof_idx)
    assert np.allclose(hi.cpu(), 30.0) and np.allclose(lo.cpu(), -30.0)
    assert env.dt == 0.02 and env.scene.dt == 0.005 and env.scene.substeps == 1 and env.decimation == 4
    assert np.allclose(env.robot.solver.get_gravity().cpu(), [0.0, 0.0, -S2S["gravity"]])  # Genesis 기본 9.81이 아니라 Gazebo 9.8
    # 지연: latency_k=k면 물리 4스텝 중 앞 k번은 직전 목표(q_nom), 나머지는 새 목표 q_nom + s·a
    env.reset()
    env.latency_k[:] = torch.tensor([1, 3])
    targets, control = [], env.robot.control_dofs_position
    monkeypatch.setattr(env.robot, "control_dofs_position", lambda t, idx: (targets.append(t[:, 1].clone()), control(t, idx)))
    a = torch.zeros((env.num_envs, env.num_actions), dtype=gs.tc_float)
    a[:, 1] = 1.0  # FL thigh
    env.step(a)
    q0, q1 = S2S["q_nom"][1], S2S["q_nom"][1] + S2S["action_scale"]
    assert np.allclose(torch.stack(targets).cpu(), [[q0, q0], [q1, q0], [q1, q0], [q1, q1]])


def test_joint_sign_matches_fk(env):  # 관절 하나씩 +0.1 rad → 발 이동 방향이 leg_kinematics와 같다
    for i, name in enumerate(S2S["joint_names"]):
        leg = name[:2]
        d_gen = foot_in_base(env, leg, offset={name: 0.1}) - foot_in_base(env, leg, offset={})
        j = i % 3
        q0 = np.array(S2S["q_nom"][3 * (i // 3):3 * (i // 3) + 3])
        q1 = q0.copy()
        q1[j] += 0.1
        d_ref = foot_pos(leg, q1) - foot_pos(leg, q0)
        assert d_gen @ d_ref / (np.linalg.norm(d_gen) * np.linalg.norm(d_ref)) > 0.99, name


def test_obs_layout(env):
    env.reset()
    obs, *_ = env.step(torch.zeros((env.num_envs, env.num_actions), dtype=gs.tc_float))
    policy = obs["policy"][0].cpu().numpy()
    assert policy.shape == (45,) and np.allclose(policy[3:6], [0, 0, -1], atol=1e-3)  # 수평 초기 자세의 중력
    assert obs["privileged"].shape == (env.num_envs, 5)


def test_armature_matches_json(env):  # Genesis URDF 기본 armature 0.1이 아니라 JSON 값(Gazebo 0)
    assert np.array_equal(env.robot.get_dofs_armature(env.motors_dof_idx).cpu(), np.full((env.num_envs, 12), S2S["armature"]))
    # calf: 축이 링크 질량 중심이라 질량 행렬 대각 = URDF Iyy 0.003 + armature (기본값이면 0.103)
    calf = env.motors_dof_idx[2::3]
    diag = env.robot.get_mass_mat()[0].diagonal()[calf].cpu().numpy()
    assert np.allclose(diag, 0.003 + S2S["armature"], atol=1e-6)


def test_dr_ranges(env_dr):  # dr=True, 64 env
    for key, values in sampled_dr(env_dr).items():
        lo, hi = DR_RANGE[key]
        assert lo <= values.min() and values.max() <= hi and values.std() > 0, key
        q = 0.25 * (hi - lo)  # 범위 양 끝 25%에 표본이 있다(잘린 범위를 잡는다, 64개면 실패 확률 ~1e-8)
        assert values.min() < lo + q and values.max() > hi - q, key


def test_dr_push_and_noise_ranges(env, env_dr, monkeypatch):
    p, zero = S2S["dr"]["push_vel"], torch.zeros((env_dr.num_envs, env_dr.num_actions), dtype=gs.tc_float)
    env.reset(), env_dr.reset()
    env_dr.step(zero)
    for e, lim in ((env_dr, p), (env, 0.0)):  # 밀기 직후 base 월드 xy 속도 변화 ∈ [−push_vel, push_vel], z 그대로. dr=False면 0
        v0 = e.robot.get_vel()
        e._push(torch.ones(e.num_envs, dtype=torch.bool))
        dv = (e.robot.get_vel() - v0).abs()
        assert dv[:, :2].max() <= lim + 1e-6 and dv[:, :2].max() >= 0.75 * lim and dv[:, 2].max() < 1e-6, lim
    pushed = []  # step은 push_interval_s마다 민다
    monkeypatch.setattr(env_dr, "_push", lambda m: pushed.append(m.clone()))
    env_dr.episode_length_buf[:] = round(S2S["dr"]["push_interval_s"] / env_dr.dt) - 1
    env_dr.step(zero)
    assert len(pushed) == 1 and pushed[0].all()
    # IMU 잡음: actor 관측 ang_vel에만 N(0, ang_vel_noise)·스케일, critic(policy_clean)은 참값. dr=False면 0
    d = env_dr.get_observations()["policy"] - env_dr.get_observations()["policy_clean"]
    assert S2S["obs"]["order"][0] == "ang_vel" and torch.all(d[:, 3:] == 0)
    sigma = S2S["dr"]["ang_vel_noise"] * S2S["obs"]["scales"]["ang_vel"]
    assert abs(d[:, :3].std().item() / sigma - 1.0) < 0.3
    obs = env.get_observations()
    assert torch.equal(obs["policy"], obs["policy_clean"])
