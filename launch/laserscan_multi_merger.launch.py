#!/usr/bin/env python3
################################################################################
# ROS2 Laser Scan Merger - Launch File
#
# This launch file starts the laser scan merger node and the pointcloud to
# laserscan conversion node with configurable parameters.
#
# The node now supports N laser scanners (configured via params file or args)
#
# Usage:
#   ros2 launch ros2_laser_scan_merger laserscan_multi_merger.launch.py
#
# Optional arguments:
#   params_file:=<path>        - Path to custom parameters file
#   num_lasers:=<N>            - Number of laser scanners to merge (default: 2)
#   use_sim_time:=<true/false> - Enable/disable simulation time
#   output_frame:=<frame_id>   - Output frame ID for merged cloud
#   cloud_topic:=<topic>       - Merged point cloud output topic
#
# Example with 3 lasers:
#   ros2 launch ros2_laser_scan_merger laserscan_multi_merger.launch.py num_lasers:=3
#
# Created by: Michael Jonathan (mich1342)
# Modified for ROS2 Jazzy best practices with N-laser support
################################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with configurable parameters for N lasers."""

    # Get package directory
    pkg_dir = get_package_share_directory("ros2_laser_scan_merger")

    # Default config file path
    default_config_path = os.path.join(pkg_dir, "config", "params.yaml")

    # =========================================================================
    # Launch Arguments
    # =========================================================================

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_config_path,
        description="Path to the ROS2 parameters YAML file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (set to true when playing back bag files)",
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="merged_lidar_cloud",
        description="Merged point cloud output topic",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="merged/scan",
        description="Merged laser scan output topic",
    )

    # =========================================================================
    # Launch Configurations
    # =========================================================================

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    cloud_topic = LaunchConfiguration("cloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    # =========================================================================
    # Nodes
    # =========================================================================

    # Laser Scan Merger Node
    merger_node = Node(
        package="ros2_laser_scan_merger",
        executable="ros2_laser_scan_merger",
        name="ros2_laser_scan_merger",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    # Pointcloud to LaserScan Conversion Node
    pc_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
            },
        ],
        remappings=[
            ("cloud_in", cloud_topic),
            ("scan", scan_topic),
        ],
    )

    # =========================================================================
    # Launch Description
    # =========================================================================

    return LaunchDescription(
        [
            # Launch arguments
            params_file_arg,
            use_sim_time_arg,
            pointcloud_topic_arg,
            scan_topic_arg,
            # Nodes
            merger_node,
            pc_to_scan_node,
        ]
    )
