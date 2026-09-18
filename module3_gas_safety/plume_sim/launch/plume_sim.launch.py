import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    cfg = os.path.join(get_package_share_directory("plume_sim"), "config", f"plume_{world}.yaml")
    if not os.path.exists(cfg):
        cfg = os.path.join(get_package_share_directory("plume_sim"), "config", "plume_corridor.yaml")
    return [
        Node(
            package="plume_sim",
            executable="plume_sim_node",
            name="plume_sim",
            output="screen",
            parameters=[cfg, {"world": world, "use_sim_time": True}],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            OpaqueFunction(function=_setup),
        ]
    )
