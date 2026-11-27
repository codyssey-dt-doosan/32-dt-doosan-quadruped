"""I-4: Gazebo rl_node의 관측·행동 식(mpc_controller.rl_obs, numpy)이 Genesis go2_box_env 식(torch)과 같은지."""
import sys
from pathlib import Path

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import genesis as gs  # noqa: E402

import cfgs  # noqa: E402
from go2_box_env import Go2BoxEnv  # noqa: E402
from mpc_controller.rl_obs import Q_HI, Q_LO, action_to_q, build_obs, projected_gravity  # noqa: E402

S2S = cfgs.load_sim2sim()


@pytest.fixture(scope="module")
def env():
    if not gs._initialized:  # 같은 프로세스의 다른 테스트 파일이 먼저 초기화했을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")
    return Go2BoxEnv(8, *cfgs.get_cfgs(S2S, dr=True))


def test_obs_matches_env(env):
    torch.manual_seed(0)
    env.reset()
    for _ in range(10):  # 기울기·각속도·관절 속도가 0이 아닌 상태에서 비교
        obs, *_ = env.step(0.5 * torch.randn((env.num_envs, env.num_actions), dtype=gs.tc_float))
    ref = obs["policy"].cpu().numpy()
    assert np.abs(ref[:, 0:3]).max() > 1e-3 and np.abs(ref[:, 21:33]).max() > 1e-3  # 정지 상태가 아니다
    for i in range(env.num_envs):
        w, x, y, z = env.base_quat[i].cpu().numpy()  # Genesis wxyz → IMU xyzw
        v, vy, wz = env.commands[i].cpu().numpy()
        assert vy == 0.0  # rl_node는 v_y를 0으로 넣는다(명령 범위 [0, 0])
        o = build_obs(S2S, env.imu_ang_vel[i].cpu().numpy(),projected_gravity([x, y, z, w]), (v, wz),
                      env.dof_pos[i].cpu().numpy(), env.dof_vel[i].cpu().numpy(), env.actions[i].cpu().numpy())
        assert np.abs(o - ref[i]).max() < 1e-5, i


def test_action_to_q_matches_env(env):
    a = np.random.default_rng(0).uniform(-8.0, 8.0, (env.num_envs, env.num_actions))  # ±2 rad: 관절 한계에 걸린다
    ref = env._q_des(torch.tensor(np.clip(a, -S2S["clip_actions"], S2S["clip_actions"]), dtype=gs.tc_float)).cpu().numpy()
    assert np.isclose(ref, Q_LO).any() and np.isclose(ref, Q_HI).any()
    for i in range(env.num_envs):
        assert np.abs(action_to_q(S2S, a[i]) - ref[i]).max() < 1e-5, i
