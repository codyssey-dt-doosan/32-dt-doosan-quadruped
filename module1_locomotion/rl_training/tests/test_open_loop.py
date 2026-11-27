"""I-5 동역학 회귀: Gazebo가 정책 틱마다 보낸 q_des를 Genesis에서 개루프로 재생하면 관절 응답이 Gazebo와 가까워야 한다.
FK·관측 테스트가 통과해도 동역학은 다를 수 있다(armature 0.1 사례: Gazebo에서 25 Hz 진동 → 전도).

data/gz_open_loop_z400.npz = SSD 증거 sim2sim_debug/z400_ticks.npz(2026-11-27, Gazebo corridor, flat_v1 model_400, 명령 0)의
정책 틱 0~9: 몸체 투영 중력 g·각속도 w(body), 관절 q·q̇(JSON 순서), 그 틱에 보낸 q_des.
"""
import math
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
D = np.load(Path(__file__).resolve().parent / "data" / "gz_open_loop_z400.npz")
WINDOWS = ((0, 5), (5, 10))
# rad. 실측(CPU, 2026-11-27): armature 0(JSON) 0.011·0.038, armature 0.1(Genesis 기본) 0.029·0.126 — 앞은 통과, 뒤는 실패해야 한다
RMS_MAX = (0.02, 0.08)


def base_rotation(g):
    """yaw 0으로 투영 중력 g를 만드는 몸체→월드 회전(R, 쿼터니언 wxyz). 평지라 yaw는 동역학에 영향이 없다."""
    pitch, roll = math.asin(float(np.clip(g[0], -1, 1))), math.atan2(-g[1], -g[2])
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    R = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    cx, sx, cy, sy = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return R, (cy * cx, cy * sx, cx * sy, -sy * sx)  # q_pitch ⊗ q_roll


def make_env():
    if not gs._initialized:  # 같은 프로세스의 다른 테스트 파일이 먼저 초기화했을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")
    return Go2BoxEnv(1, *cfgs.get_cfgs(S2S, dr=False))  # 공칭 마찰 0.8·kp 20·kd 0.5·지연 0 = Gazebo 시험 조건


def replay_rms(env, armature=None):
    """Gazebo 틱 0 상태에서 q_des를 개루프로 재생하고 구간별 관절 위치 RMS(Genesis − Gazebo)."""
    r, idx = env.robot, env.motors_dof_idx
    if armature is not None:
        r.set_dofs_armature(torch.full((len(idx),), armature, dtype=gs.tc_float, device=gs.device), idx)
    R, quat = base_rotation(D["g"][0])
    q0, qd0 = D["q"][0], D["qd"][0]
    feet = [R @ foot_pos(leg, q0[3 * k:3 * k + 3]) for k, leg in enumerate(("FL", "FR", "RL", "RR"))]
    gidx = {j.name: j.dofs_idx_local[0] for j in r.joints[1:]}  # Genesis dof 순서(0~5는 base)
    qpos, vel = np.zeros(7 + len(idx)), np.zeros(6 + len(idx))
    qpos[:3] = (0.0, 0.0, S2S["foot_r"] - min(f[2] for f in feet))  # 가장 낮은 발이 바닥에 닿는 높이
    qpos[3:7] = quat
    vel[3:6] = R @ D["w"][0]
    for k, name in enumerate(S2S["joint_names"]):
        qpos[7 + gidx[name] - 6], vel[gidx[name]] = q0[k], qd0[k]
    r.set_qpos(torch.tensor(qpos, dtype=gs.tc_float, device=gs.device)[None], zero_velocity=True)
    r.set_dofs_velocity(torch.tensor(vel, dtype=gs.tc_float, device=gs.device)[None])
    g_gen = transform_by_quat(env.global_gravity[None], inv_quat(r.get_quat()))[0].cpu().numpy()
    assert np.abs(g_gen - D["g"][0]).max() < 1e-4  # 초기 자세 재구성 확인

    q = []
    for q_des in D["q_des"]:
        q.append(r.get_dofs_position(idx)[0].cpu().numpy())
        target = torch.clamp(torch.tensor(q_des, dtype=gs.tc_float, device=gs.device)[None], env.dof_lower, env.dof_upper)
        for _ in range(env.decimation):
            r.control_dofs_position(target, idx)
            env.scene.step()
    err = np.array(q) - D["q"]
    return [float(np.sqrt((err[lo:hi] ** 2).mean())) for lo, hi in WINDOWS]


@pytest.fixture(scope="module")
def rms_json():
    return replay_rms(make_env())


def test_open_loop_matches_gazebo(rms_json):
    assert all(e < m for e, m in zip(rms_json, RMS_MAX)), rms_json


def test_negative_control_armature(rms_json):  # 같은 재생을 armature 0.1로 하면 문턱을 넘어야 이 테스트가 의미 있다
    rms_def = replay_rms(make_env(), armature=0.1)
    assert any(e >= m for e, m in zip(rms_def, RMS_MAX)), (rms_json, rms_def)
