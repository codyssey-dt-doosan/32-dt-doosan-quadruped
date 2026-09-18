import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    share = get_package_share_directory("source_seeking")
    world_cfg = os.path.join(share, "config", f"world_{world}.yaml")
    params = [os.path.join(share, "config", "source_seeking.yaml")]
    if os.path.exists(world_cfg):
        params.append(world_cfg)
    params.append({
        "world": world,
        "use_sim_time": True,
        "mode": LaunchConfiguration("mode").perform(context),
        "drive_mode": LaunchConfiguration("drive_mode").perform(context),
        "log_file": LaunchConfiguration("log_file").perform(context),
    })
    return [
        Node(
            package="source_seeking",
            executable="source_seeking_node",
            name="source_seeking",
            output="screen",
            parameters=params,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("mode", default_value="hybrid", description="hybrid | gradient"),
            DeclareLaunchArgument("drive_mode", default_value="goal", description="goal | cmd_vel"),
            DeclareLaunchArgument("log_file", default_value="", description="예: /tmp/gas_run"),
            OpaqueFunction(function=_setup),
        ]
    )
