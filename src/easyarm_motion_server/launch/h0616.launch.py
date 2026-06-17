from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    debug_enable = LaunchConfiguration("debug_enable", default="false")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="false")

    moveit_config = (
        MoveItConfigsBuilder("easyarm_a1", package_name="easyarm_a1_moveit_config")
        .robot_description(
            mappings={
                "debug_enable": debug_enable,
                "use_mock_hardware": use_mock_hardware,
            }
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )

    motion_server = Node(
        package="easyarm_motion_server",
        executable="easyarm_motion_server",
        name="easyarm_motion_server",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Use mock hardware mappings for robot_description consistency.",
            ),
            DeclareLaunchArgument(
                "debug_enable",
                default_value="false",
                description="Enable hardware debug mappings for robot_description consistency.",
            ),
            motion_server,
        ]
    )
