from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    debug_enable = LaunchConfiguration("debug_enable", default="false")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="false")
    moveit_config = (
        MoveItConfigsBuilder("easyarm_a1", package_name="easyarm_a1_h0616_moveit_config")
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
    launch_description = generate_demo_launch(moveit_config)
    launch_description.add_action(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use easyarm_hardware mock mode instead of connecting to CAN.",
        )
    )
    launch_description.add_action(
        DeclareLaunchArgument(
            "debug_enable",
            default_value="false",
            description="Enable easyarm_hardware binary debug logging.",
        )
    )
    return launch_description
