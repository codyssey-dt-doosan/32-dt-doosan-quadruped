"""힘 제어 기립 go2(C안 ③, 옵트인). legged 변환본에 ApplyJointForce·구형 발·ApplyLinkWrench(+경사 판)를 더해
일시정지 상태로 띄우고, balance_node가 준비되면 재개한다. 기본 구동·legged.launch.py와 별개.
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
    mu, ramp_deg = float(cfg("mu")), float(cfg("ramp_deg"))
    sim_share = get_package_share_directory("simulation")
    # 위치 컨트롤러 게인 0 = 노드가 토크 전부 계산. 절충안(gz PD + τ_ff)으로 후퇴하려면 p_gain·d_gain만 올린다
    world_file, legged_yaml, legged_models = write_legged_assets(
        sim_share, world, mu, 0.0, 0.0, force_ctrl=True, foot_r=0.02, wrench=True, ramp_deg=ramp_deg
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
    balance = Node(
        package="mpc_controller",
        executable="balance_node",
        name="balance",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "world": world,
                "enable_qp": cfg("qp").lower() == "true",
                "com_shift": float(cfg("com_shift")),
                "pitch_offset": float(cfg("pitch_offset")),
                "roll_offset": float(cfg("roll_offset")),
            }
        ],
    )
    resource_path = os.pathsep.join([legged_models, os.path.join(sim_share, "models")])
    return [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path), gz_sim, *gz_gui, *bridges, balance]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("qp", default_value="true", description="false면 관절 PD 기립만(스파이크 기준선)"),
            DeclareLaunchArgument("mu", default_value="0.8", description="다리·발 충돌 마찰"),
            DeclareLaunchArgument("ramp_deg", default_value="0.0", description="스폰 밑 경사 판 각도(0이면 없음)"),
            DeclareLaunchArgument("com_shift", default_value="1.0", description="1 중력 방향 발 중심 위, 0 법선 투영"),
            DeclareLaunchArgument("pitch_offset", default_value="0.0", description="rad, 지형 평행 기준 추가 pitch"),
            DeclareLaunchArgument("roll_offset", default_value="0.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
