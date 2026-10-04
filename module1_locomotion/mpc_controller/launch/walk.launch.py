"""SRB 지평 MPC 보행 go2(C안 ④, 옵트인). balance.launch.py와 같은 변환본(ApplyJointForce·구형 발·damping 0·게인 0)에
walk_node를 붙인다. slope_deg>0이면 스폰 앞 2 m에 오르막·평지·내리막 경사로. 기본 구동·legged.launch.py·balance.launch.py와 별개.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mpc_controller.legged_model import write_legged_assets


def _launch_setup(context, *args, **kwargs):
    cfg = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    world, gui = cfg("world"), cfg("gui").lower() == "true"
    sim_share = get_package_share_directory("simulation")
    world_file, legged_yaml, legged_models = write_legged_assets(
        sim_share, world, float(cfg("mu")), 0.0, 0.0, force_ctrl=True, foot_r=0.02, wrench=True,
        ramp_deg=float(cfg("ramp_deg")), damping=0.0, slope_deg=float(cfg("slope_deg")), slope_len=float(cfg("slope_len")),
        contact=cfg("contact_sensors") == "true" or cfg("contact_source") == "sensor",
        real_time_factor=float(cfg("real_time_factor")),
    )
    split_gui = gui and sys.platform == "darwin"
    gz_args = f"{world_file}" if gui and not split_gui else f"-s {world_file}"  # -r 없음: 일시정지 시작
    gz_gui = [ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")] if split_gui else []
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    bridges = [
        Node(package="ros_gz_bridge", executable="parameter_bridge", name=name, parameters=[{"config_file": config}], output="screen")
        for name, config in (
            ("bridge_common", os.path.join(sim_share, "config", "ros_gz_bridge.yaml")),
            ("bridge_legged", legged_yaml),
        )
    ]
    walk_params = {
        "use_sim_time": True,
        "world": world,
        "stride_hz": float(cfg("stride_hz")),
        "lift": float(cfg("lift")),
        "k_raibert": float(cfg("k_raibert")),
        "v_gain": float(cfg("v_gain")),
        "touch_h": float(cfg("touch_h")),
        "contact_source": cfg("contact_source"),
        "swing_knee_min": float(cfg("swing_knee_min")),
    }
    def parse_q_weights(name):
        w = [float(v) for v in cfg(name).split(",")]
        if len(w) != 13:
            raise ValueError(f"{name}는 13개(rpy3 p3 ω3 v3 g)여야 함: {len(w)}개")
        return w

    if cfg("q_weights"):
        walk_params["q_weights"] = parse_q_weights("q_weights")
    if cfg("q_weights_walk"):
        walk_params["q_weights_walk"] = parse_q_weights("q_weights_walk")
    if cfg("alpha"):
        walk_params["alpha"] = float(cfg("alpha"))
    if cfg("height"):
        walk_params["height"] = float(cfg("height"))
    if cfg("height_walk"):
        walk_params["height_walk"] = float(cfg("height_walk"))
    walk = Node(
        package="mpc_controller",
        executable="walk_node",
        name="walk",
        output="screen",
        parameters=[walk_params],
    )
    resource_path = os.pathsep.join([legged_models, os.path.join(sim_share, "models")])
    return [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path), gz_sim, *gz_gui, *bridges, walk]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("mu", default_value="0.8", description="다리·발 충돌 마찰"),
            DeclareLaunchArgument("slope_deg", default_value="0.0", description="스폰 앞 2 m 오르막·평지·내리막 경사(0이면 없음)"),
            DeclareLaunchArgument("slope_len", default_value="3.0", description="오르막·내리막 길이 m"),
            DeclareLaunchArgument("ramp_deg", default_value="0.0", description="스폰 밑 경사 판(balance와 동일, slope_deg와 배타)"),
            DeclareLaunchArgument("stride_hz", default_value="3.0"),
            DeclareLaunchArgument("lift", default_value="0.025"),
            DeclareLaunchArgument("k_raibert", default_value="0.03"),
            DeclareLaunchArgument("v_gain", default_value="1.12", description="전진 보정 손잡이 — MPC 추종이 89%로 일관(실측)"),
            DeclareLaunchArgument("touch_h", default_value="0.008", description="조기 착지 판정 높이 m(스윙 후반 발이 평면 위 이 높이 미만이면 지지로)"),
            DeclareLaunchArgument("contact_source", default_value="sensor", description="조기 착지 판정: sensor(gz 발 contact 센서, 센서 자동 추가) | kinematic(발 높이 < touch_h — 추정 튐에 속아 가짜 착지)"),
            DeclareLaunchArgument("contact_sensors", default_value="false", description="발 contact 센서만 추가(계측: status td_ms 착지 지연·scuff), 판정은 contact_source"),
            DeclareLaunchArgument("swing_knee_min", default_value="0.7", description="스윙 목표 무릎 최소 굽힘 rad(펴진 다리 착지 방지, 0이면 완전히 펼 수 있음)"),
            DeclareLaunchArgument("real_time_factor", default_value="1.0", description="시뮬 실시간 비율(<1이면 느리게 — 머신 부하에 덜 흔들리는 실측용, walk_metrics --rtf 같이)"),
            DeclareLaunchArgument("q_weights", default_value="", description="13개 쉼표 구분(rpy3 p3 ω3 v3 g), 비우면 기본"),
            DeclareLaunchArgument("q_weights_walk", default_value="", description="보행 중(gait 활성) Q, 13개 쉼표 구분, 비우면 기본"),
            DeclareLaunchArgument("alpha", default_value="", description="비우면 기본"),
            DeclareLaunchArgument("height", default_value="", description="정지 목표 높이, 비우면 기본"),
            DeclareLaunchArgument("height_walk", default_value="", description="보행 목표 높이, 비우면 기본"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
