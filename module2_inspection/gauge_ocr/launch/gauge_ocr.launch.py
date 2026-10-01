from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    params_file = PathJoinSubstitution(
        [FindPackageShare("gauge_ocr"), "config", ["gauge_", world, ".yaml"]]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            Node(
                package="gauge_ocr",
                executable="gauge_ocr_node",
                name="gauge_ocr",
                output="screen",
                parameters=[params_file, {"world": world, "use_sim_time": True}],
            ),
        ]
    )
