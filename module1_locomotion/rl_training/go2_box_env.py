# Adapted from Genesis examples/locomotion/go2_env.py (v1.5.0), Apache License 2.0.
# 바꾼 점: 박스 모델 URDF, dt 0.005 × decimation 4, 물리 스텝 단위 행동 지연, DR(마찰·질량·kp·kd), 목표 관절 한계 클립,
# 기울기·몸체 접촉 종료, critic용 privileged 관측, 지형 모드(heightfield·terrain_levels_vel), 험지 보상(feet air time·다리 충돌·토크·일률),
# 밀기·IMU 잡음 DR. 정합 값은 cfgs.get_cfgs(sim2sim.json)에서만 받는다.
import math
from pathlib import Path

import torch
from tensordict import TensorDict

import genesis as gs
from genesis.utils.geom import inv_quat, transform_by_quat

from terrain import make_heightfield

URDF = Path(__file__).resolve().parent / "assets" / "go2_box.urdf"
CALF_FOOT = (0.0, 0.0, -0.1)  # calf 링크 원점 → 발 구 중심(URDF sphere origin)


def gs_rand(lower, upper, batch_shape):
    assert lower.shape == upper.shape
    return (upper - lower) * torch.rand(size=(*batch_shape, *lower.shape), dtype=gs.tc_float, device=gs.device) + lower


class Go2BoxEnv:
    def __init__(self, num_envs, env_cfg, obs_cfg, reward_cfg, command_cfg, show_viewer=False):
        self.num_envs: int = num_envs
        self.num_actions = env_cfg["num_actions"]
        self.cfg = env_cfg
        self.num_commands = command_cfg["num_commands"]
        self.device = gs.device

        self.decimation: int = env_cfg["decimation"]
        self.dt = env_cfg["sim_dt"] * self.decimation  # 정책 50 Hz
        self.max_episode_length = math.ceil(env_cfg["episode_length_s"] / self.dt)

        self.env_cfg = env_cfg
        self.obs_cfg = obs_cfg
        self.reward_cfg = reward_cfg
        self.command_cfg = command_cfg
        self.dr = env_cfg["dr"]

        self.obs_scales: dict[str, float] = obs_cfg["obs_scales"]
        self.reward_scales = {k: v * self.dt for k, v in reward_cfg["reward_scales"].items()}

        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(dt=env_cfg["sim_dt"], substeps=1, gravity=(0.0, 0.0, -env_cfg["gravity"])),
            rigid_options=gs.options.RigidOptions(
                enable_self_collision=False,
                max_collision_pairs=20,
                tolerance=1e-5,
                batch_dofs_info=True,  # env별 kp·kv
                batch_links_info=True,  # env별 base 질량
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=[0]),
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
            show_viewer=show_viewer,
        )

        # 접촉 마찰 = max(두 geom) → 바닥은 하한 0.01, 로봇 geom 1.0 × env별 비율이 실효 마찰이 된다
        self.terrain = env_cfg["terrain"]
        if self.terrain is None:
            self.ground = self.plane = self.scene.add_entity(
                gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True), material=gs.materials.Rigid(friction=0.01)
            )
        else:  # heightfield 하나(행 = 난이도 x, 열 = 타입 y, 꼭짓점 (i, j) = 월드 (i·hs, j·hs))
            t = self.terrain
            hf, tile_origins = make_heightfield(t)
            self.ground = self.scene.add_entity(
                gs.morphs.Terrain(height_field=hf / t["vs"], horizontal_scale=t["hs"], vertical_scale=t["vs"], pos=(0, 0, 0)),
                material=gs.materials.Rigid(friction=0.01),
            )
        # armature: Genesis URDF 기본값 0.1(로터 관성) 대신 JSON 값 — Gazebo/DART에는 로터 관성이 없다
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=str(URDF),
                pos=env_cfg["base_init_pos"],
                quat=env_cfg["base_init_quat"],
                default_armature=env_cfg["armature"],
            ),
            material=gs.materials.Rigid(friction=1.0),
        )

        self.scene.build(n_envs=num_envs)

        # JSON 관절 이름 → dof 인덱스(Genesis 내부 순서를 쓰지 않는다)
        self.motors_dof_idx = torch.tensor(
            [self.robot.get_joint(name).dofs_idx_local[0] for name in env_cfg["joint_names"]],
            dtype=gs.tc_int,
            device=gs.device,
        )
        base_link = self.robot.get_link("base_link")
        self.base_link_idx = base_link.idx  # 접촉 조회용 전역 인덱스
        self.base_link_idx_local = base_link.idx_local
        self.base_mass = float(base_link.get_mass()[0])
        self.calf_links_idx = [self.robot.get_link(f"{leg}_calf").idx_local for leg in ("FL", "FR", "RL", "RR")]
        self.calf_foot = torch.tensor(CALF_FOOT, dtype=gs.tc_float, device=gs.device)
        # 다리 충돌 벌점 대상: thigh·calf 박스 geom(전역 인덱스). 발 구·base 박스(접촉하면 종료)는 뺀다
        self.leg_box_geoms = torch.tensor(
            [g.idx for g in self.robot.geoms if g.type == gs.GEOM_TYPE.BOX and g.link.idx != self.base_link_idx],
            dtype=gs.tc_int, device=gs.device,
        )
        self.push_steps = round(self.dr["push_interval_s"] / self.dt)

        # PD: τ = kp(q_des − q) − kd·q̇를 ±tau_max로 자른다(Gazebo joint_servo와 같은 식). kp·kd는 _randomize에서 env별로
        tau = torch.full((self.num_actions,), env_cfg["tau_max"], dtype=gs.tc_float, device=gs.device)
        self.robot.set_dofs_force_range(-tau, tau, self.motors_dof_idx)
        self.dof_lower, self.dof_upper = self.robot.get_dofs_limit(self.motors_dof_idx)  # URDF 한계
        self.cos_tilt_max = math.cos(math.radians(env_cfg["tilt_max_deg"]))

        self.global_gravity = torch.tensor([0.0, 0.0, -1.0], dtype=gs.tc_float, device=gs.device)

        # Initial state
        self.init_base_pos = torch.tensor(env_cfg["base_init_pos"], dtype=gs.tc_float, device=gs.device)
        self.init_base_quat = torch.tensor(env_cfg["base_init_quat"], dtype=gs.tc_float, device=gs.device)
        q_nom = dict(zip(env_cfg["joint_names"], env_cfg["q_nom"]))
        self.init_dof_pos = torch.tensor(
            [q_nom[joint.name] for joint in self.robot.joints[1:]], dtype=gs.tc_float, device=gs.device
        )  # qpos 순서
        self.init_qpos = torch.concatenate((self.init_base_pos, self.init_base_quat, self.init_dof_pos))
        self.init_projected_gravity = transform_by_quat(self.global_gravity, inv_quat(self.init_base_quat))
        self.default_dof_pos = torch.tensor(env_cfg["q_nom"], dtype=gs.tc_float, device=gs.device)  # JSON 순서

        # initialize buffers
        self.base_lin_vel = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_ang_vel = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.projected_gravity = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.rew_buf = torch.empty((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.reset_buf = torch.ones((self.num_envs,), dtype=gs.tc_bool, device=gs.device)
        self.episode_length_buf = torch.empty((self.num_envs,), dtype=gs.tc_int, device=gs.device)
        self.commands = torch.empty((self.num_envs, self.num_commands), dtype=gs.tc_float, device=gs.device)
        self.commands_scale = torch.tensor(
            [self.obs_scales["lin_vel"], self.obs_scales["lin_vel"], self.obs_scales["ang_vel"]],
            device=gs.device,
            dtype=gs.tc_float,
        )
        self.commands_limits: tuple[torch.Tensor, torch.Tensor] = tuple(
            torch.tensor(values, dtype=gs.tc_float, device=gs.device)
            for values in zip(
                self.command_cfg["lin_vel_x_range"],
                self.command_cfg["lin_vel_y_range"],
                self.command_cfg["ang_vel_range"],
            )
        )
        self.actions = torch.zeros((self.num_envs, self.num_actions), dtype=gs.tc_float, device=gs.device)
        self.last_actions = torch.zeros_like(self.actions)
        self.dof_pos = torch.empty_like(self.actions)
        self.dof_vel = torch.empty_like(self.actions)
        self.last_dof_vel = torch.zeros_like(self.actions)
        self.torques = torch.zeros_like(self.actions)
        self.feet_air_time = torch.zeros((self.num_envs, 4), dtype=gs.tc_float, device=gs.device)
        self.first_contact = torch.zeros((self.num_envs, 4), dtype=gs.tc_bool, device=gs.device)
        self.base_pos = torch.empty((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        self.base_quat = torch.empty((self.num_envs, 4), dtype=gs.tc_float, device=gs.device)
        # DR 표본(env별). latency_k = 4 물리 스텝 중 직전 목표를 유지하는 스텝 수
        self.friction = torch.empty((self.num_envs,), dtype=gs.tc_float, device=gs.device)
        self.added_mass = torch.empty_like(self.friction)
        self.kp_scale = torch.empty_like(self.friction)
        self.kd_scale = torch.empty_like(self.friction)
        self.latency_k = torch.empty((self.num_envs,), dtype=gs.tc_int, device=gs.device)
        # 에피소드 원점 기준 명령 유니사이클 적분 (x, y, yaw) — 명령대로 갔을 때의 위치(커리큘럼 내려가기 기준)
        self.cmd_pose = torch.zeros((self.num_envs, 3), dtype=gs.tc_float, device=gs.device)
        if self.terrain is not None:
            # 초기 레벨 0~max_init_level, 타입(열)은 env를 고르게 나눈다
            self.tile_origins = torch.tensor(tile_origins, dtype=gs.tc_float, device=gs.device)
            self.levels = torch.randint(0, min(t["max_init_level"], t["rows"] - 1) + 1, (self.num_envs,), dtype=gs.tc_int, device=gs.device)
            self.types = (torch.arange(self.num_envs, device=gs.device) * len(t["cols"]) // self.num_envs).to(gs.tc_int)
            self.origins = self.tile_origins[self.levels, self.types]
            self.hf_lo = torch.zeros(2, dtype=gs.tc_float, device=gs.device)
            self.hf_hi = torch.tensor([(s - 1) * t["hs"] for s in hf.shape], dtype=gs.tc_float, device=gs.device)
            self.inner_hi = torch.tensor([t["border"] + t["rows"] * t["tile"], t["border"] + len(t["cols"]) * t["tile"]],
                                         dtype=gs.tc_float, device=gs.device)  # border 안쪽 = 타일 영역
        self.extras = dict()  # extra information for logging

        self.reward_functions, self.episode_sums = dict(), dict()
        for name in self.reward_scales.keys():
            self.reward_functions[name] = getattr(self, "_reward_" + name)
            self.episode_sums[name] = torch.zeros((self.num_envs,), dtype=gs.tc_float, device=gs.device)

        self.reset()

    def _resample_commands(self, envs_idx):
        commands = gs_rand(*self.commands_limits, (self.num_envs,))
        # rough_v2: env별 zero_ang_vel_prob 확률로 wz = 0(직진 유지 명령). vx·vy는 그대로
        zero_wz = torch.rand(self.num_envs, dtype=gs.tc_float, device=gs.device) < self.command_cfg["zero_ang_vel_prob"]
        commands[:, 2].masked_fill_(zero_wz, 0.0)
        if envs_idx is None:
            self.commands.copy_(commands)
        else:
            torch.where(envs_idx[:, None], commands, self.commands, out=self.commands)

    def _randomize(self, envs_idx):
        """reset된 env에 DR 표본을 뽑아 Genesis에 넣는다(dr=False면 범위가 공칭값 한 점)."""
        idx = torch.arange(self.num_envs, device=gs.device) if envs_idx is None else envs_idx.nonzero()[:, 0]
        n = len(idx)
        if n == 0:
            return

        def rand(key):
            lo, hi = self.dr[key]
            return lo + (hi - lo) * torch.rand(n, dtype=gs.tc_float, device=gs.device)

        self.friction[idx] = rand("friction")
        self.added_mass[idx] = rand("added_mass")
        self.kp_scale[idx] = rand("kp_scale")
        self.kd_scale[idx] = rand("kd_scale")
        lo, hi = self.dr["latency_steps"]
        self.latency_k[idx] = torch.randint(lo, hi + 1, (n,), dtype=gs.tc_int, device=gs.device)

        self.robot.set_friction_ratio(self.friction[idx, None].expand(n, self.robot.n_links), envs_idx=idx)
        self.robot.set_links_mass((self.base_mass + self.added_mass[idx])[:, None], [self.base_link_idx_local], idx)
        ones = torch.ones((n, self.num_actions), dtype=gs.tc_float, device=gs.device)
        self.robot.set_dofs_kp(self.env_cfg["kp"] * self.kp_scale[idx, None] * ones, self.motors_dof_idx, idx)
        self.robot.set_dofs_kv(self.env_cfg["kd"] * self.kd_scale[idx, None] * ones, self.motors_dof_idx, idx)

    def _push(self, envs_idx):
        """밀기 DR: 고른 env의 base 월드 xy 속도(free joint dof 0·1)에 env별 U(−push_vel, push_vel)를 더한다."""
        idx = envs_idx.nonzero()[:, 0]
        if len(idx):
            p = self.dr["push_vel"]
            vel = self.robot.get_dofs_velocity([0, 1], idx)
            self.robot.set_dofs_velocity(vel + p * (2 * torch.rand_like(vel) - 1), [0, 1], idx)

    def _terrain_spawn(self, envs_idx):
        """지형 모드 reset: terrain_levels_vel(Isaac Lab)로 레벨을 옮기고 새 타일 origin 위 base_init_z에 세운다. 스폰 위치 (n, 3).
        타일 반 넘게 가면 +1, 명령대로 갔을 때의 직선 변위의 절반도 못 가면 −1(그 변위가 중앙 플랫폼 반경 이하면 지형을 시험하지 않았으니
        그대로). 경로 길이가 아니라 변위인 이유: 회전 명령이 4 s마다 바뀌어 명령을 잘 따라도 변위가 경로의 약 40%다(스모크 실측).
        최고 레벨을 넘기면 무작위 레벨, 0 아래는 0. 첫 reset은 초기 레벨 그대로."""
        if envs_idx is not None:
            dist = torch.norm(self.base_pos[:, :2] - self.origins[:, :2], dim=1)
            up = dist > self.terrain["tile"] / 2
            cmd_disp = torch.norm(self.cmd_pose[:, :2], dim=1)
            down = (dist < 0.5 * cmd_disp) & (cmd_disp > self.terrain["platform"] / 2) & ~up
            levels = self.levels + up.to(gs.tc_int) - down.to(gs.tc_int)
            rows = self.terrain["rows"]
            levels = torch.where(levels >= rows, torch.randint_like(levels, rows), levels.clamp(min=0))
            self.levels = torch.where(envs_idx, levels, self.levels)
        self.origins = self.tile_origins[self.levels, self.types]
        spawn = self.origins + self.init_base_pos
        idx = torch.arange(self.num_envs, device=gs.device) if envs_idx is None else envs_idx.nonzero()[:, 0]
        if len(idx):
            self.robot.set_pos(spawn[idx], envs_idx=idx, zero_velocity=True)
        return spawn

    def ground_height(self, xy):
        """월드 xy (n, k, 2)의 지면 높이 [m] (n, k). plane이면 0. 지형은 heightfield 범위로 잘라 조회한다(밖이면 NaN이라)."""
        if self.terrain is None:
            return torch.zeros(xy.shape[:-1], dtype=gs.tc_float, device=gs.device)
        return self.ground.get_terrain_height(torch.clamp(xy, self.hf_lo, self.hf_hi))

    def _feet_height(self):
        """발 구 중심(calf 링크 원점 + R·CALF_FOOT)의 지면 위 높이 [m] (n, 4), FL·FR·RL·RR."""
        foot = self.robot.get_links_pos(self.calf_links_idx) + transform_by_quat(
            self.calf_foot, self.robot.get_links_quat(self.calf_links_idx))
        return foot[..., 2] - self.ground_height(foot[..., :2])

    def _q_des(self, actions):
        # Gazebo rl_node action_to_q와 같은 식: q_nom + s·a를 관절 한계로 자른다
        return torch.clamp(self.default_dof_pos + self.env_cfg["action_scale"] * actions, self.dof_lower, self.dof_upper)

    def step(self, actions):
        self.actions = torch.clip(actions, -self.env_cfg["clip_actions"], self.env_cfg["clip_actions"])
        q_new, q_old = self._q_des(self.actions), self._q_des(self.last_actions)
        for i in range(self.decimation):
            # 지연: 앞 latency_k 물리 스텝은 직전 목표를 건다
            target = torch.where((i < self.latency_k)[:, None], q_old, q_new)
            self.robot.control_dofs_position(target, self.motors_dof_idx)
            self.scene.step()

        # update buffers
        self.episode_length_buf += 1
        yaw = self.cmd_pose[:, 2] + self.commands[:, 2] * self.dt
        self.cmd_pose[:, 0] += self.commands[:, 0] * torch.cos(yaw) * self.dt
        self.cmd_pose[:, 1] += self.commands[:, 0] * torch.sin(yaw) * self.dt
        self.cmd_pose[:, 2] = yaw
        self.base_pos = self.robot.get_pos()
        self.base_quat = self.robot.get_quat()
        inv_base_quat = inv_quat(self.base_quat)
        self.base_lin_vel = transform_by_quat(self.robot.get_vel(), inv_base_quat)
        self.base_ang_vel = transform_by_quat(self.robot.get_ang(), inv_base_quat)
        self.projected_gravity = transform_by_quat(self.global_gravity, inv_base_quat)
        self.dof_pos = self.robot.get_dofs_position(self.motors_dof_idx)
        self.dof_vel = self.robot.get_dofs_velocity(self.motors_dof_idx)
        self.torques = self.robot.get_dofs_control_force(self.motors_dof_idx)
        self.contacts = contacts = self.robot.get_contacts(with_entity=self.ground, is_padded=True)
        # 발 접지 = 발 구 중심이 지면 위 foot_r + 5 mm 안. first_contact = 공중에 있다 이번 스텝에 닿음(legged_gym)
        contact = self._feet_height() < self.env_cfg["foot_r"] + 0.005
        self.first_contact = (self.feet_air_time > 0) & contact
        self.feet_air_time += self.dt

        # compute reward
        self.rew_buf.zero_()
        for name, reward_func in self.reward_functions.items():
            rew = reward_func() * self.reward_scales[name]
            self.rew_buf += rew
            self.episode_sums[name] += rew
        self.feet_air_time *= ~contact

        self._resample_commands(self.episode_length_buf % int(self.env_cfg["resampling_time_s"] / self.dt) == 0)
        if self.dr["push_vel"] > 0:  # dr=False면 밀지 않는다
            self._push(self.episode_length_buf % self.push_steps == 0)

        # check termination and reset: 시간 초과, 기울기 > tilt_max(walk_node hold 기준), base_link 지면 접촉, 시뮬 오류,
        # 지형 모드면 타일 영역(border 안쪽) 이탈
        base_hit = (contacts["link_a"] == self.base_link_idx) | (contacts["link_b"] == self.base_link_idx)
        self.reset_buf = self.episode_length_buf > self.max_episode_length
        self.reset_buf |= -self.projected_gravity[:, 2] < self.cos_tilt_max
        self.reset_buf |= (base_hit & contacts["valid_mask"]).any(dim=-1)
        self.reset_buf |= self.scene.get_error_envs_mask()
        if self.terrain is not None:
            xy = self.base_pos[:, :2]
            self.reset_buf |= ((xy < self.terrain["border"]) | (xy > self.inner_hi)).any(dim=-1)

        # Compute timeout
        self.extras["time_outs"] = (self.episode_length_buf > self.max_episode_length).to(dtype=gs.tc_float)

        # Reset environment if necessary
        self._reset_idx(self.reset_buf)

        # update observations
        self._update_observation()

        self.last_actions.copy_(self.actions)
        self.last_dof_vel.copy_(self.dof_vel)

        return self.get_observations(), self.rew_buf, self.reset_buf, self.extras

    def get_observations(self):
        return TensorDict({"policy": self.obs_buf, "policy_clean": self.clean_obs_buf, "privileged": self.privileged_buf},
                          batch_size=[self.num_envs])

    def _reset_idx(self, envs_idx=None):
        # reset state
        self.robot.set_qpos(self.init_qpos, envs_idx=envs_idx, zero_velocity=True)
        spawn = self.init_base_pos if self.terrain is None else self._terrain_spawn(envs_idx)
        self._randomize(envs_idx)

        # reset buffers
        if envs_idx is None:
            self.base_pos.copy_(spawn)
            self.base_quat.copy_(self.init_base_quat)
            self.projected_gravity.copy_(self.init_projected_gravity)
            self.dof_pos.copy_(self.default_dof_pos)
            self.base_lin_vel.zero_()
            self.base_ang_vel.zero_()
            self.dof_vel.zero_()
            self.actions.zero_()
            self.last_actions.zero_()
            self.last_dof_vel.zero_()
            self.feet_air_time.zero_()
            self.cmd_pose.zero_()
            self.episode_length_buf.zero_()
            self.reset_buf.fill_(True)
        else:
            torch.where(envs_idx[:, None], spawn, self.base_pos, out=self.base_pos)
            torch.where(envs_idx[:, None], self.init_base_quat, self.base_quat, out=self.base_quat)
            torch.where(
                envs_idx[:, None], self.init_projected_gravity, self.projected_gravity, out=self.projected_gravity
            )
            torch.where(envs_idx[:, None], self.default_dof_pos, self.dof_pos, out=self.dof_pos)
            self.base_lin_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.base_ang_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.dof_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.actions.masked_fill_(envs_idx[:, None], 0.0)
            self.last_actions.masked_fill_(envs_idx[:, None], 0.0)
            self.last_dof_vel.masked_fill_(envs_idx[:, None], 0.0)
            self.feet_air_time.masked_fill_(envs_idx[:, None], 0.0)
            self.cmd_pose.masked_fill_(envs_idx[:, None], 0.0)
            self.episode_length_buf.masked_fill_(envs_idx, 0)
            self.reset_buf.masked_fill_(envs_idx, True)

        # fill extras
        n_envs = envs_idx.sum() if envs_idx is not None else self.num_envs
        self.extras["episode"] = {}
        for key, value in self.episode_sums.items():
            if envs_idx is None:
                mean = value.mean()
            else:
                mean = torch.where(n_envs > 0, value[envs_idx].sum() / n_envs, 0.0)
            self.extras["episode"]["rew_" + key] = mean / self.env_cfg["episode_length_s"]
            if envs_idx is None:
                value.zero_()
            else:
                value.masked_fill_(envs_idx, 0.0)
        if self.terrain is not None:
            self.extras["episode"]["terrain_level"] = self.levels.to(gs.tc_float).mean()

        # random sample command upon reset
        self._resample_commands(envs_idx)

    def _update_observation(self):
        # actor: sim2sim.json obs.order 순서(45). commands = (v_x, 0, w_z) × 스케일. IMU 각속도 = 참값 + N(0, ang_vel_noise)
        self.imu_ang_vel = self.base_ang_vel + self.dr["ang_vel_noise"] * torch.randn_like(self.base_ang_vel)
        parts = {
            "ang_vel": self.imu_ang_vel * self.obs_scales["ang_vel"],
            "projected_gravity": self.projected_gravity,
            "commands": self.commands * self.commands_scale,
            "dof_pos": (self.dof_pos - self.default_dof_pos) * self.obs_scales["dof_pos"],
            "dof_vel": self.dof_vel * self.obs_scales["dof_vel"],
            "last_action": self.actions,
        }
        self.obs_buf = torch.concatenate([parts[k] for k in self.obs_cfg["order"]], dim=-1)
        parts["ang_vel"] = self.base_ang_vel * self.obs_scales["ang_vel"]  # critic은 잡음 없는 같은 관측
        self.clean_obs_buf = torch.concatenate([parts[k] for k in self.obs_cfg["order"]], dim=-1)
        # critic 전용: 참값 선속도, 마찰, 추가 질량
        self.privileged_buf = torch.concatenate(
            (self.base_lin_vel * self.obs_scales["lin_vel"], self.friction[:, None], self.added_mass[:, None]), dim=-1
        )

    def reset(self):
        self._reset_idx()
        self._update_observation()
        return self.get_observations()

    # Reward functions.
    def _reward_tracking_lin_vel(self):
        # Tracking of linear velocity commands (xy axes)
        lin_vel_error = torch.sum(torch.square(self.commands[:, :2] - self.base_lin_vel[:, :2]), dim=1)
        return torch.exp(-lin_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_tracking_ang_vel(self):
        # Tracking of angular velocity commands (yaw)
        ang_vel_error = torch.square(self.commands[:, 2] - self.base_ang_vel[:, 2])
        return torch.exp(-ang_vel_error / self.reward_cfg["tracking_sigma"])

    def _reward_lin_vel_z(self):
        # Penalize z axis base linear velocity
        return torch.square(self.base_lin_vel[:, 2])

    def _reward_action_rate(self):
        # Penalize changes in actions
        return torch.sum(torch.square(self.last_actions - self.actions), dim=1)

    def _reward_similar_to_default(self):
        # Penalize joint poses far away from default pose
        return torch.sum(torch.abs(self.dof_pos - self.default_dof_pos), dim=1)

    def _reward_base_height(self):
        # Penalize base height above ground away from target
        ground = self.ground_height(self.base_pos[:, None, :2])[:, 0]
        return torch.square(self.base_pos[:, 2] - ground - self.reward_cfg["base_height_target"])

    def _reward_feet_air_time(self):
        # 착지한 발마다 (공중 시간 − 목표), 이동 명령(|v_cmd| > 0.1)일 때만
        air = torch.sum((self.feet_air_time - self.reward_cfg["feet_air_time_target"]) * self.first_contact, dim=1)
        return air * (torch.norm(self.commands[:, :2], dim=1) > 0.1)

    def _reward_collision(self):
        # 지면에 닿은 thigh·calf 박스 geom 수(발 구 제외)
        c = self.contacts
        hit = (c["geom_a"][..., None] == self.leg_box_geoms) | (c["geom_b"][..., None] == self.leg_box_geoms)
        return (hit & c["valid_mask"][..., None]).any(dim=1).sum(dim=1).to(gs.tc_float)

    def _reward_torques(self):
        return torch.sum(torch.square(self.torques), dim=1)

    def _reward_dof_power(self):
        return torch.sum(torch.abs(self.torques * self.dof_vel), dim=1)
