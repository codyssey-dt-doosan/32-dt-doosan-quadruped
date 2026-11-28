"""좌우 거울(y → −y) mirror.py: 인덱스·부호·대합(mirror∘mirror = id)·증강 모양, Genesis 물리 대칭(rough_v2 학습 전 관문)."""
import sys
from pathlib import Path

import pytest
import torch
from tensordict import TensorDict

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import genesis as gs  # noqa: E402

import cfgs  # noqa: E402
import mirror  # noqa: E402
from go2_box_env import Go2BoxEnv  # noqa: E402

S2S = cfgs.load_sim2sim()
# 관측 45 성분 이름(실패 메시지용)
NAMES = [f"{k}.{c}" for k in S2S["obs"]["order"] for c in (S2S["joint_names"] if k in mirror.JOINT_BLOCKS else "xyz")]


@pytest.fixture(scope="module")
def env():  # dr=False: 공칭 한 점·IMU 잡음 0·밀기 없음, 초기 자세는 q_nom·yaw 0으로 좌우 대칭
    if not gs._initialized:  # 다른 테스트 파일이 먼저 돌았을 수 있다
        gs.init(backend=gs.cpu, logging_level="warning")
    return Go2BoxEnv(2, *cfgs.get_cfgs(S2S, dr=False))


def test_joint_perm_sign():
    perm, sign = mirror.joint_perm_sign(S2S["joint_names"])
    assert perm.tolist() == [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
    assert sign.tolist() == [-1, 1, 1] * 4


def test_involution():  # mirror(mirror(x)) == x
    for g, d in (("policy", 45), ("policy_clean", 45), ("privileged", 5)):
        x = torch.randn(7, d)
        assert torch.equal(mirror.mirror_obs(g, mirror.mirror_obs(g, x)), x)
    a = torch.randn(7, 12)
    assert torch.equal(mirror.mirror_actions(mirror.mirror_actions(a)), a)


def test_obs_signs():  # 관측 45 = ang_vel 0:3, gravity 3:6, commands 6:9, dof_pos 9:21, dof_vel 21:33, last_action 33:45
    x = torch.arange(1., 46.)[None]
    m = mirror.mirror_obs("policy", x)[0]
    assert m[0:3].tolist() == [-1., 2., -3.]        # roll·yaw 각속도 반전, pitch 유지
    assert m[3:6].tolist() == [4., -5., 6.]         # 중력 y(=roll 기울기) 반전
    assert m[6:9].tolist() == [7., -8., -9.]        # vx 유지, vy·wz 반전
    assert m[9:12].tolist() == [-13., 14., 15.]     # FL ← FR(hip 반전)
    assert m[12:15].tolist() == [-10., 11., 12.]    # FR ← FL


def test_privileged_signs():  # 선속도 3 + 마찰 + 추가 질량
    m = mirror.mirror_obs("privileged", torch.tensor([[1., 2., 3., 4., 5.]]))[0]
    assert m.tolist() == [1., -2., 3., 4., 5.]


def test_data_augmentation_shape():
    obs = TensorDict({"policy": torch.randn(4, 45), "policy_clean": torch.randn(4, 45),
                      "privileged": torch.randn(4, 5)}, batch_size=[4])
    act = torch.randn(4, 12)
    o, a = mirror.data_augmentation(None, obs, act)
    assert o.batch_size[0] == 8 and a.shape == (8, 12)
    assert torch.equal(a[:4], act) and torch.equal(a[4:], mirror.mirror_actions(act))  # Symmetry 거울 손실의 정답
    assert torch.equal(o["policy"][:4], obs["policy"])
    assert torch.equal(o["policy"][4:], mirror.mirror_obs("policy", obs["policy"]))
    assert mirror.data_augmentation(None, None, None) == (None, None)  # Symmetry는 obs·actions 한쪽만 넘기기도 한다


def test_physical_mirror(env):  # env0에 a, env1에 mirror(a)를 10스텝 → env1 관측 = env0 관측의 거울
    torch.manual_seed(0)
    env.commands[:] = torch.tensor([[0.3, 0.0, 0.4], [0.3, 0.0, -0.4]])
    for _ in range(10):
        a0 = 0.3 * torch.randn(1, 12)
        obs, *_ = env.step(torch.cat([a0, mirror.mirror_actions(a0)]))
    p = obs["policy"]
    m = mirror.mirror_obs("policy", p[:1])[0]
    d = (p[1] - m).abs()
    worst = ", ".join(f"{NAMES[i]}={d[i]:.2e}" for i in d.argsort(descending=True)[:5].tolist())
    assert torch.allclose(p[1], m, atol=1e-3), worst
    q = obs["privileged"]  # 선속도 3·마찰·추가 질량 구성 전제
    assert torch.allclose(q[1], mirror.mirror_obs("privileged", q[:1])[0], atol=1e-3)
