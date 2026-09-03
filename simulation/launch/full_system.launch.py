"""통합 런치: Gazebo Harmonic + ros_gz_bridge + 모듈 1·2·3."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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

    sim_share = get_package_share_directory("simulation")
    world_file = os.path.join(sim_share, "worlds", f"{world}.sdf")
    bridge_yaml = os.path.join(sim_share, "config", "ros_gz_bridge.yaml")
    urdf_file = os.path.join(sim_share, "urdf", "go2.urdf")
    models_path = os.path.join(sim_share, "models")

    gz_args = f"-r {world_file}" if gui else f"-s -r {world_file}"

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
            launch_arguments={"world": world}.items(),
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
        bridge,
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
