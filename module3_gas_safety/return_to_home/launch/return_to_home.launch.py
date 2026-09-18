import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    cfg = os.path.join(get_package_share_directory("return_to_home"), "config", "return_to_home.yaml")
    common = {"world": LaunchConfiguration("world"), "use_sim_time": True}
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("drive_mode", default_value="goal", description="goal | cmd_vel"),
            DeclareLaunchArgument("battery", default_value="100.0", description="시작 배터리 %"),
            Node(
                package="return_to_home",
                executable="battery_sim_node",
                name="battery_sim",
                output="screen",
                parameters=[cfg, {"use_sim_time": True, "initial_percent": LaunchConfiguration("battery")}],
            ),
            Node(
                package="return_to_home",
                executable="return_to_home_node",
                name="return_to_home",
                output="screen",
                parameters=[cfg, common, {"drive_mode": LaunchConfiguration("drive_mode")}],
            ),
        ]
    )
