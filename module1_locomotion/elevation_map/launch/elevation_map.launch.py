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
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
