from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("planner", default_value="mpc", description="mpc | heading_scan"),
            Node(
                package="mpc_controller",
                executable="mpc_controller_node",
                name="mpc_controller",
                output="screen",
                parameters=[
                    {
                        "world": LaunchConfiguration("world"),
                        "planner": LaunchConfiguration("planner"),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
