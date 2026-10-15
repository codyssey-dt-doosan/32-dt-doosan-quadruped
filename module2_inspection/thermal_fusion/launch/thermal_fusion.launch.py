from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    params_file = PathJoinSubstitution(
        [FindPackageShare("thermal_fusion"), "config", ["thermal_", world, ".yaml"]]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("locomotion", default_value="velocity"),  # full_system이 넘겨 줌
            Node(
                package="thermal_fusion",
                executable="thermal_fusion_node",
                name="thermal_fusion",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "world": world,
                        "locomotion": LaunchConfiguration("locomotion"),
                        "use_sim_time": True,
                    },
                ],
            ),
        ]
    )
