"""rl_node의 관측·행동 식(numpy만). 학습 venv 테스트가 Genesis go2_box_env와 같은 식인지 교차 비교하려고 ROS 없이 둔다."""
import numpy as np

from mpc_controller.leg_kinematics import LIMITS, Q_NOM
from mpc_controller.legged_model import JOINTS

Q_NOM12 = np.array(Q_NOM * 4)
Q_LO = np.array([lo for lo, _ in LIMITS] * 4)
Q_HI = np.array([hi for _, hi in LIMITS] * 4)


def joints_by_name(names, pos, vel):
    """JOINTS 순서로 재정렬. 하나라도 없거나 속도 길이가 0·len(names) 둘 다 아니면 None, 속도가 없으면 0."""
    if len(vel) not in (0, len(names)):
        return None
    p, v = dict(zip(names, pos)), dict(zip(names, vel))
    if any(j not in p for j in JOINTS):
        return None
    return np.array([p[j] for j in JOINTS], float), np.array([v.get(j, 0.0) for j in JOINTS], float)


def projected_gravity(quat_xyzw):
    """Rᵀ·(0, 0, −1) = −(R의 셋째 행). 노름 < 0.5이거나 비유한값이면 None."""
    q = np.asarray(quat_xyzw, float)
    n = np.linalg.norm(q)
    if not np.all(np.isfinite(q)) or n < 0.5:
        return None
    x, y, z, w = q / n
    return -np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])


def build_obs(cfg, ang_vel_B, grav_B, cmd_vw, q, qd, last_action) -> np.ndarray:
    """Genesis go2_box_env._update_observation과 같은 순서·스케일."""
    s = cfg["obs"]["scales"]
    v, w = cmd_vw
    parts = {
        "ang_vel": np.asarray(ang_vel_B, float) * s["ang_vel"],
        "projected_gravity": np.asarray(grav_B, float),
        "commands": np.array([v * s["lin_vel"], 0.0, w * s["ang_vel"]]),
        "dof_pos": (np.asarray(q, float) - np.asarray(cfg["q_nom"])) * s["dof_pos"],
        "dof_vel": np.asarray(qd, float) * s["dof_vel"],
        "last_action": np.asarray(last_action, float),
    }
    return np.concatenate([parts[k] for k in cfg["obs"]["order"]])


def cmd_or_zero(cmd_vw, age_s, cfg) -> tuple:
    c = cfg["commands"]
    if age_s > c["timeout_s"]:
        return 0.0, 0.0
    v, w = cmd_vw
    return float(np.clip(v, *c["lin_vel_x"])), float(np.clip(w, *c["ang_vel"]))


def action_to_q(cfg, action) -> np.ndarray:
    """Genesis _q_des와 같은 식: q_nom + s·clip(a)를 관절 한계로 자른다."""
    a = np.clip(action, -cfg["clip_actions"], cfg["clip_actions"])
    return np.clip(np.asarray(cfg["q_nom"]) + cfg["action_scale"] * a, Q_LO, Q_HI)
