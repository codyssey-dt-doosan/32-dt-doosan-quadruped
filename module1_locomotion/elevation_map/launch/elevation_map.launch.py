from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("map_resolution", default_value="0.1", description="mpc_controller와 동일해야 함"),
            DeclareLaunchArgument("map_size", default_value="4.0", description="mpc_controller와 동일해야 함"),
            DeclareLaunchArgument(
                "traversability",
                default_value="false",
                description="true면 /elevation_map 값이 높이가 아니라 막힘 표시(1.0 막힘 / 0.0 통과 지면 / NaN 미관측)."
                " /imu로 자세를 보정해 경사로는 통과, 턱·낙차는 막힘(⑤-2). mpc_controller는 그대로 '> 0.15 = 막힘'으로 읽는다",
            ),
            Node(
                package="elevation_map",
                executable="elevation_map_node",
                name="elevation_map",
                output="screen",
                parameters=[
                    {
                        "world": LaunchConfiguration("world"),
                        "resolution": ParameterValue(LaunchConfiguration("map_resolution"), value_type=float),
                        "size": ParameterValue(LaunchConfiguration("map_size"), value_type=float),
                        "traversability": ParameterValue(LaunchConfiguration("traversability"), value_type=bool),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
