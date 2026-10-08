"""Gazebo + 장애물 회피 + 복도/공장 자동 순찰."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("simulation"), "launch", "full_system.launch.py"
            )),
            launch_arguments={"inspection": "false", "gas_safety": "false"}.items(),
        ),
    ])
