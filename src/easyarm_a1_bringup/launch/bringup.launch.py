import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="false")
    debug_enable = LaunchConfiguration("debug_enable", default="false")
    rviz = LaunchConfiguration("rviz", default="false")
    moveit_servo = LaunchConfiguration("moveit_servo", default="false")
    publish_frequency = LaunchConfiguration("publish_frequency", default="15.0")
    easyarm_urdf_path = LaunchConfiguration("easyarm_urdf_path")

    default_urdf_path = PathJoinSubstitution(
        [
            FindPackageShare("easyarm_description"),
            "urdf",
            "easyarm_a1_h0617.urdf.xacro",
        ]
    )

    moveit_config = (
        MoveItConfigsBuilder("easyarm_a1", package_name="easyarm_a1_moveit_config")
        .robot_description(
            mappings={
                "debug_enable": debug_enable,
                "use_mock_hardware": use_mock_hardware,
                "easyarm_urdf_path": easyarm_urdf_path,
            }
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    static_virtual_joint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="easyarm_world_to_base_tf",
        output="screen",
        arguments=[
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": publish_frequency},
        ],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("easyarm_a1_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    easyarm_servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "easyarm_servo_controller",
            "--inactive",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    easyarm_drag_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "easyarm_drag_controller",
            "--inactive",
            "--controller-manager-timeout",
            "30",
            "--service-call-timeout",
            "30",
        ],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
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

    servo_params = {
        "moveit_servo": load_yaml(
            "easyarm_a1_moveit_config",
            "config/moveit_servo.yaml",
        )
    }

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(moveit_servo),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("easyarm_a1_moveit_config"),
                "config",
                "moveit.rviz",
            ),
        ],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="Use easyarm_hardware mock mode instead of connecting to CAN.",
            ),
            DeclareLaunchArgument(
                "debug_enable",
                default_value="false",
                description="Enable easyarm_hardware binary debug logging.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start RViz with the MoveIt configuration.",
            ),
            DeclareLaunchArgument(
                "moveit_servo",
                default_value="false",
                description="Start MoveIt Servo for SpeedJ/SpeedL teleoperation.",
            ),
            DeclareLaunchArgument(
                "easyarm_urdf_path",
                default_value=default_urdf_path,
                description="URDF/Xacro file used by easyarm_a1.urdf.xacro and easyarm_hardware.",
            ),
            DeclareLaunchArgument(
                "publish_frequency",
                default_value="15.0",
                description="robot_state_publisher publish frequency.",
            ),
            static_virtual_joint_tf,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[
                        arm_controller_spawner,
                        easyarm_servo_controller_spawner,
                        easyarm_drag_controller_spawner,
                    ],
                )
            ),
            move_group,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=arm_controller_spawner,
                    on_exit=[motion_server, servo_node],
                )
            ),
            rviz_node,
        ]
    )
