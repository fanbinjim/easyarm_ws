from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    host = LaunchConfiguration("host")
    port = LaunchConfiguration("port")
    token = LaunchConfiguration("token")

    web_bridge = Node(
        package="easyarm_web_bridge",
        executable="easyarm_web_bridge",
        name="easyarm_web_bridge",
        output="screen",
        parameters=[
            {
                "host": host,
                "port": port,
                "token": token,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "host",
                default_value="0.0.0.0",
                description="Host address for the EasyArm web bridge backend.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="8000",
                description="Port for the EasyArm web bridge backend.",
            ),
            DeclareLaunchArgument(
                "token",
                default_value=EnvironmentVariable("EASYARM_WEB_TOKEN", default_value="easyarm"),
                description="Required token for browser/API access.",
            ),
            web_bridge,
        ]
    )
