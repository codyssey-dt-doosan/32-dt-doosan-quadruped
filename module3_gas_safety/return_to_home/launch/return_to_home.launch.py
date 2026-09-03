from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            Node(
                package="return_to_home",
                executable="return_to_home_node",
                name="return_to_home",
                output="screen",
                parameters=[{"world": LaunchConfiguration("world"), "use_sim_time": True}],
            ),
        ]
    )
