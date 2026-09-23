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
    walk = Node(
        package="mpc_controller",
        executable="walk_node",
        name="walk",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "world": world,
                "stride_hz": float(cfg("stride_hz")),
                "lift": float(cfg("lift")),
                "k_raibert": float(cfg("k_raibert")),
            }
        ],
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
            DeclareLaunchArgument("lift", default_value="0.06"),
            DeclareLaunchArgument("k_raibert", default_value="0.03"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
