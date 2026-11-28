"""학습 설정. 정합 항목은 모두 sim2sim.json에서 가져오고, 보상·PPO 값만 Genesis 예제(go2_train.py)를 따른다."""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "module1_locomotion" / "mpc_controller"))  # ROS 없이 순수 Python 모듈만 쓴다

from mpc_controller import sim2sim  # noqa: E402


def load_sim2sim() -> dict:
    return sim2sim.load(REPO / "module1_locomotion" / "mpc_controller" / "config" / "sim2sim.json")


def get_cfgs(s2s: dict, dr: bool = True, terrain: dict | None = None) -> tuple[dict, dict, dict, dict]:
    # dr=False: 범위를 공칭값 한 점으로 접는다(지연·밀기·IMU 잡음 0). terrain: None = 평지 plane, get_terrain_cfg() = 지형 커리큘럼
    nominal = {"friction": [s2s["nominal"]["mu"]] * 2, "added_mass": [0.0, 0.0], "kp_scale": [1.0, 1.0],
               "kd_scale": [1.0, 1.0], "latency_steps": [0, 0], "push_vel": 0.0,
               "push_interval_s": s2s["dr"]["push_interval_s"], "ang_vel_noise": 0.0}
    env_cfg = {
        "num_actions": len(s2s["joint_names"]),
        "joint_names": s2s["joint_names"],
        "q_nom": s2s["q_nom"],
        "kp": s2s["kp"],
        "kd": s2s["kd"],
        "armature": s2s["armature"],
        "tau_max": s2s["tau_max"],
        "sim_dt": s2s["sim_dt"],
        "gravity": s2s["gravity"],
        "decimation": s2s["decimation"],
        "action_scale": s2s["action_scale"],
        "clip_actions": s2s["clip_actions"],
        "tilt_max_deg": s2s["tilt_max_deg"],
        "foot_r": s2s["foot_r"],
        "base_init_pos": [0.0, 0.0, s2s["base_init_z"]],
        "base_init_quat": [1.0, 0.0, 0.0, 0.0],
        "episode_length_s": 20.0,
        "resampling_time_s": 4.0,
        "dr": s2s["dr"] if dr else nominal,
        "terrain": terrain,
    }
    obs_cfg = {"order": s2s["obs"]["order"], "obs_scales": dict(s2s["obs"]["scales"])}
    reward_cfg = {
        # 명령 최대 0.5 m/s에 맞춰 0.25×0.5²; 0.25면 3 cm/s 오차가 보상 0.4%라 후진이 13~19% 느림
        "tracking_sigma": 0.0625,
        "base_height_target": s2s["base_init_z"],
        "feet_air_time_target": 0.25,
        "normal_baseline": 0.25,  # rough_v2: 지형 법선 중앙차분 ±기준 길이 [m](몸통 반 길이 0.30 근처)
        "reward_scales": {
            "tracking_lin_vel": 1.0,
            "tracking_ang_vel": 0.5,  # rough_v2: 0.2 → 0.5, G3 경사에서 늘 + 방향으로 돌던 회전 편향 대책
            "lin_vel_z": -1.0,
            "base_height": -50.0,
            "action_rate": -0.005,
            "similar_to_default": -0.1,
            # 험지(스펙 §4.2, 시작값): 블라인드 계단엔 feet air time이 사실상 필요
            "feet_air_time": 1.0,
            "collision": -1.0,
            "torques": -1e-4,
            "dof_power": -2e-5,
            "terrain_orientation": -20.0,  # 지형 법선 기준 자세(rough_v2 −5 → rough_v3 −20: −5는 후반에 효과가 거의 없었다)
        },
    }
    command_cfg = {
        "num_commands": 3,
        "lin_vel_x_range": s2s["commands"]["lin_vel_x"],
        "lin_vel_y_range": s2s["commands"]["lin_vel_y"],
        "ang_vel_range": s2s["commands"]["ang_vel"],
        "zero_ang_vel_prob": 0.3,  # rough_v2: 재표본 명령의 30%는 wz = 0(직진 유지를 따로 배운다). 범위는 그대로
    }
    return env_cfg, obs_cfg, reward_cfg, command_cfg


def get_terrain_cfg() -> dict:
    # 지형 커리큘럼(스펙 §4.2, 학습 전용). 행 = 난이도, 열 = 타입. 상한은 §5 시험값(계단 0.10·파이프 0.08)보다 크게
    return {"rows": 10, "cols": ["flat", "rough", "slope_up", "slope_down", "stairs_up", "stairs_down", "pipes", "pipes"],
            "tile": 5.0, "hs": 0.05, "vs": 0.005, "border": 2.5, "platform": 1.0, "slope_deg_max": 25.0,
            "step_h_max": 0.12, "tread": 0.30, "pipe_d_max": 0.10, "pipe_gap": 0.6, "rough_max": 0.04,
            "max_init_level": 6}


def get_train_cfg(exp_name: str, max_iterations: int) -> dict:
    import mirror  # mirror가 모듈 로드 때 cfgs를 import하므로 순환을 피해 여기서

    return {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
            # rough_v2: 좌우 거울 증강 + 거울 손실(rsl-rl Symmetry). env는 rsl-rl이 복사본에만 넣어 runner.cfg는 pickle된다
            "symmetry_cfg": {"data_augmentation_func": mirror.data_augmentation, "use_data_augmentation": True,
                             "use_mirror_loss": True, "mirror_loss_coeff": 10.0},
        },
        "actor": {
            "class_name": "MLPModel",
            "hidden_dims": [512, 256, 128],
            "activation": "elu",
            "distribution_cfg": {"class_name": "GaussianDistribution", "init_std": 1.0, "std_type": "scalar"},
        },
        "critic": {"class_name": "MLPModel", "hidden_dims": [512, 256, 128], "activation": "elu"},
        # 비대칭: actor는 고유수용감각만(IMU 잡음 포함), critic은 잡음 없는 같은 관측에 참값 선속도·마찰·질량을 더 본다
        "obs_groups": {"actor": ["policy"], "critic": ["policy_clean", "privileged"]},
        "num_steps_per_env": 24,
        "save_interval": 100,
        "max_iterations": max_iterations,
        "run_name": exp_name,
        "logger": "tensorboard",
    }
