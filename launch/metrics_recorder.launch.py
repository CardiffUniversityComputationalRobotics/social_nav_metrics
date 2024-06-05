import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ! PACKAGES DIR
    social_nav_metrics = FindPackageShare(package="social_nav_metrics").find(
        "social_nav_metrics"
    )

    # ! PARAMETERS

    config_file = LaunchDescription("metrics_config_file")

    config = os.path.join(
        get_package_share_directory("social_nav_metrics"),
        "config",
        "config_example.yaml",
    )

    config_file_param = DeclareLaunchArgument(
        "metrics_config_file",
        default_value=[social_nav_metrics, "/launch/config_example.yaml"],
        description="Configuration file for metrics recording.",
    )

    # ! NODES
    metrics_recorder_node = Node(
        package="social_nav_metrics",
        executable="metrics_recorder.py",
        name="metrics_recorder_node",
        output="screen",
        # parameters=[config],
        parameters=[
            "/home/sasm/ros/humble/system/src/social_nav_metrics/config/config_example.yaml"
        ],
    )

    collision_counter_node = Node(
        package="social_nav_metrics",
        executable="collision_counter",
        name="collision_counter_node",
        output="screen",
        parameters=[config],
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! PARAMETERS
    ld.add_action(config_file_param)

    # ! NODES
    ld.add_action(metrics_recorder_node)
    ld.add_action(collision_counter_node)

    return ld
