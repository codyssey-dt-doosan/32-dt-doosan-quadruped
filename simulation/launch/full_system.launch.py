"""통합 런치: Gazebo Harmonic + ros_gz_bridge + 모듈 1·2·3."""

import os
import sys

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
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    planner = LaunchConfiguration("planner").perform(context)
    map_resolution = LaunchConfiguration("map_resolution").perform(context)
    map_size = LaunchConfiguration("map_size").perform(context)
    leg_animation = LaunchConfiguration("leg_animation").perform(context)
    locomotion = LaunchConfiguration("locomotion").perform(context)

    sim_share = get_package_share_directory("simulation")
    world_file = os.path.join(sim_share, "worlds", f"{world}.sdf")
    bridge_yaml = os.path.join(sim_share, "config", "ros_gz_bridge.yaml")
    urdf_file = os.path.join(sim_share, "urdf", "go2.urdf")
    models_path = os.path.join(sim_share, "models")

    # 다리 구동 go2(C안, 옵트인): go2 모델·월드를 변환한 임시본으로 바꾸고 관절 명령 브리지를 하나 더 띄운다.
    # 기본(velocity)은 아래를 전혀 타지 않는다 — mpc_controller import도 이 분기 안에서만.
    legged_bridge = []
    if locomotion == "legged":
        from mpc_controller.legged_model import write_legged_assets

        world_file, legged_yaml, legged_models = write_legged_assets(sim_share, world)
        models_path = os.pathsep.join([legged_models, models_path])
        legged_bridge = [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="bridge_legged",
                parameters=[{"config_file": legged_yaml}],
                output="screen",
            )
        ]
    elif locomotion != "velocity":
        raise ValueError(f"locomotion은 velocity | legged (받은 값: {locomotion})")

    # macOS는 서버+GUI 한 프로세스(gz sim world.sdf)를 지원하지 않음 → 서버(-s)와 GUI(-g)를 따로 띄운다
    split_gui = gui and sys.platform == "darwin"
    gz_args = f"-r {world_file}" if gui and not split_gui else f"-s -r {world_file}"
    gz_gui = [ExecuteProcess(cmd=["gz", "sim", "-g"], output="screen")] if split_gui else []

    with open(urdf_file, "r", encoding="utf-8") as f:
        robot_description = f.read()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": bridge_yaml}],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        output="screen",
    )

    module_launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare(pkg), "launch", launch_file])
            ),
            launch_arguments={
                "world": world,
                "planner": planner,
                "map_resolution": map_resolution,
                "map_size": map_size,
                "leg_animation": leg_animation,
                "locomotion": locomotion,
            }.items(),
        )
        for pkg, launch_file in (
            ("elevation_map", "elevation_map.launch.py"),
            ("mpc_controller", "mpc_controller.launch.py"),
            ("fall_recovery", "fall_recovery.launch.py"),
            ("gauge_ocr", "gauge_ocr.launch.py"),
            ("thermal_fusion", "thermal_fusion.launch.py"),
            ("patrol_path", "patrol_path.launch.py"),
            ("plume_sim", "plume_sim.launch.py"),
            ("source_seeking", "source_seeking.launch.py"),
            ("return_to_home", "return_to_home.launch.py"),
        )
    ]

    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models_path),
        gz_sim,
        *gz_gui,
        bridge,
        *legged_bridge,
        robot_state_publisher,
        *module_launches,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="corridor",
                description="월드 이름: corridor | factory",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Gazebo GUI 사용 여부",
            ),
            DeclareLaunchArgument(
                "planner",
                default_value="mpc",
                description="mpc_controller 플래너: mpc | heading_scan",
            ),
            DeclareLaunchArgument(
                "map_resolution",
                default_value="0.1",
                description="elevation_map·mpc_controller 공용 그리드 셀 크기(m)",
            ),
            DeclareLaunchArgument(
                "map_size",
                default_value="4.0",
                description="elevation_map·mpc_controller 공용 그리드 한 변(m)",
            ),
            DeclareLaunchArgument(
                "leg_animation",
                default_value="true",
                description="다리 애니메이션(시각 효과) 사용 여부",
            ),
            DeclareLaunchArgument(
                "locomotion",
                default_value="velocity",
                description="구동 방식: velocity(몸체 속도 직접, 기본) | legged(다리 구동 trot, C안)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
