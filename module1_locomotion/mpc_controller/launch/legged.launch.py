"""다리 구동 go2(C안, 옵트인). go2/model.sdf·월드를 변환해 임시 dir에 쓰고 gz + 브리지 + gait_node만 띄운다.

기본 구동(full_system.launch.py, VelocityControl + mpc)과 별개 — 이 로봇은 아직 제자리 trot만 한다.
"""
import os
import sys
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mpc_controller.legged_model import MODEL_CONFIG, bridge_entries, make_legged_model, make_legged_world


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as f:
        return f.read()


def _write(path: str, text: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return path


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    mu, p_gain, d_gain = (float(LaunchConfiguration(k).perform(context)) for k in ("mu", "p_gain", "d_gain"))

    sim_share = get_package_share_directory("simulation")
    # ponytail: 실행마다 임시 dir 하나가 남는다(수십 KB). 거슬리면 런치 종료 핸들러에서 삭제
    out = tempfile.mkdtemp(prefix="go2_legged_")
    model_sdf = make_legged_model(_read(os.path.join(sim_share, "models", "go2", "model.sdf")), mu, p_gain, d_gain)
    _write(os.path.join(out, "models", "go2_legged", "model.sdf"), model_sdf)
    _write(os.path.join(out, "models", "go2_legged", "model.config"), MODEL_CONFIG)
    world_file = _write(
        os.path.join(out, f"{world}.sdf"), make_legged_world(_read(os.path.join(sim_share, "worlds", f"{world}.sdf")))
    )
    legged_yaml = _write(os.path.join(out, "bridge.yaml"), yaml.safe_dump(bridge_entries()))

    # macOS는 서버+GUI 한 프로세스를 지원하지 않음 → full_system과 같은 분리 기동
    split_gui = gui and sys.platform == "darwin"
    gz_args = f"-r {world_file}" if gui and not split_gui else f"-s -r {world_file}"
    gz_gui = [ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")] if split_gui else []

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )
    bridges = [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=name,
            parameters=[{"config_file": config}],
            output="screen",
        )
        for name, config in (
            ("bridge_common", os.path.join(sim_share, "config", "ros_gz_bridge.yaml")),  # /clock·센서
            ("bridge_legged", legged_yaml),
        )
    ]
    gait = Node(
        package="mpc_controller",
        executable="gait_node",
        name="gait",
        output="screen",
        parameters=[
            {"use_sim_time": True, "force_trot": ParameterValue(LaunchConfiguration("trot"), value_type=bool)}
        ],
    )
    resource_path = os.pathsep.join([os.path.join(out, "models"), os.path.join(sim_share, "models")])
    return [SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path), gz_sim, *gz_gui, *bridges, gait]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("trot", default_value="false", description="/cmd_vel 없이도 제자리 trot"),
            DeclareLaunchArgument("mu", default_value="0.8", description="다리 충돌 마찰(보정 손잡이)"),
            DeclareLaunchArgument("p_gain", default_value="120.0", description="관절 위치 P(보정 손잡이)"),
            DeclareLaunchArgument("d_gain", default_value="2.0", description="관절 위치 D(보정 손잡이)"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
