from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            Node(
                package="thermal_fusion",
                executable="thermal_fusion_node",
                name="thermal_fusion",
                output="screen",
                parameters=[{"world": LaunchConfiguration("world"), "use_sim_time": True}],
            ),
        ]
    )
