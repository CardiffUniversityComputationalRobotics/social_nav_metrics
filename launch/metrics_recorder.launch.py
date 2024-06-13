import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ! PACKAGES DIR
    social_nav_metrics_dir = FindPackageShare(package="social_nav_metrics").find(
        "social_nav_metrics"
    )

    # ! PARAMETERS

    config_file_param = DeclareLaunchArgument(
        "metrics_config_file",
        default_value=[social_nav_metrics_dir, "/config/config_example.yaml"],
        description="Configuration file for metrics recording.",
    )

    # ! NODES
    metrics_recorder_node = Node(
        package="social_nav_metrics",
        executable="metrics_recorder.py",
        name="metrics_recorder_node",
        output="screen",
        parameters=[LaunchConfiguration("metrics_config_file")],
    )

    collision_counter_node = Node(
        package="social_nav_metrics",
        executable="collision_counter",
        name="collision_counter_node",
        output="screen",
        parameters=[LaunchConfiguration("metrics_config_file")],
    )

    # ! LAUNCH DESCRIPTION DECLARATION
    ld = LaunchDescription()

    # ! PARAMETERS
    ld.add_action(config_file_param)

    # ! NODES
    ld.add_action(metrics_recorder_node)
    ld.add_action(collision_counter_node)

    return ld
