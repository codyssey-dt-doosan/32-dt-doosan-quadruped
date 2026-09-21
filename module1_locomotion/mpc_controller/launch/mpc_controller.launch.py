from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="corridor"),
            DeclareLaunchArgument("planner", default_value="mpc", description="mpc | heading_scan"),
            DeclareLaunchArgument("map_resolution", default_value="0.1", description="elevation_map과 동일해야 함"),
            DeclareLaunchArgument("map_size", default_value="4.0", description="elevation_map과 동일해야 함"),
            DeclareLaunchArgument("leg_animation", default_value="true", description="다리 애니메이션(시각 효과) 켜기"),
            Node(
                package="mpc_controller",
                executable="leg_animation_node",
                name="leg_animation",
                output="screen",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("leg_animation")),
            ),
            Node(
                package="mpc_controller",
                executable="mpc_controller_node",
                name="mpc_controller",
                output="screen",
                parameters=[
                    {
                        "world": LaunchConfiguration("world"),
                        "planner": LaunchConfiguration("planner"),
                        "resolution": ParameterValue(LaunchConfiguration("map_resolution"), value_type=float),
                        "size": ParameterValue(LaunchConfiguration("map_size"), value_type=float),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
