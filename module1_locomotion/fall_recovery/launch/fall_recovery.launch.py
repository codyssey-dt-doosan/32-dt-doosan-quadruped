from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            Node(
                package="fall_recovery",
                executable="fall_recovery_node",
                name="fall_recovery",
                output="screen",
                parameters=[{"world": LaunchConfiguration("world"), "use_sim_time": True}],
            ),
        ]
    )
