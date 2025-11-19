#!/usr/bin/env python3
################################################################################
# ROS2 Laser Scan Merger - Launch File
#
# This launch file starts the laser scan merger node and the pointcloud to
# laserscan conversion node with configurable parameters.
#
# Usage:
#   ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py
#
# Optional arguments:
#   params_file:=<path>        - Path to custom parameters file
#   use_sim_time:=<true/false> - Enable/disable simulation time
#   output_frame:=<frame_id>   - Output frame ID for merged cloud
#   scan_topic_1:=<topic>      - First laser scan input topic
#   scan_topic_2:=<topic>      - Second laser scan input topic
#   cloud_topic:=<topic>       - Merged point cloud output topic
#   enable_respawn:=<true/false> - Enable node auto-restart on failure
#
# Created by: Michael Jonathan (mich1342)
# Modified for ROS2 Jazzy best practices
################################################################################

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with configurable parameters."""

    # Get package directory
    pkg_dir = get_package_share_directory('ros2_laser_scan_merger')

    # Default config file path
    default_config_path = os.path.join(pkg_dir, 'config', 'params.yaml')

    # =========================================================================
    # Launch Arguments
    # =========================================================================

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Path to the ROS2 parameters YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (set to true when playing back bag files)'
    )

    output_frame_arg = DeclareLaunchArgument(
        'output_frame',
        default_value='laser',
        description='Frame ID for the merged point cloud output'
    )

    scan_topic_1_arg = DeclareLaunchArgument(
        'scan_topic_1',
        default_value='/lidar_1/scan',
        description='First laser scan input topic'
    )

    scan_topic_2_arg = DeclareLaunchArgument(
        'scan_topic_2',
        default_value='/lidar_2/scan',
        description='Second laser scan input topic'
    )

    cloud_topic_arg = DeclareLaunchArgument(
        'cloud_topic',
        default_value='cloud_in',
        description='Output topic for merged point cloud'
    )

    enable_respawn_arg = DeclareLaunchArgument(
        'enable_respawn',
        default_value='true',
        description='Enable automatic node restart on failure'
    )

    respawn_delay_arg = DeclareLaunchArgument(
        'respawn_delay',
        default_value='2.0',
        description='Delay in seconds before restarting failed node'
    )

    # =========================================================================
    # Launch Configurations
    # =========================================================================

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    output_frame = LaunchConfiguration('output_frame')
    scan_topic_1 = LaunchConfiguration('scan_topic_1')
    scan_topic_2 = LaunchConfiguration('scan_topic_2')
    cloud_topic = LaunchConfiguration('cloud_topic')
    enable_respawn = LaunchConfiguration('enable_respawn')
    respawn_delay = LaunchConfiguration('respawn_delay')

    # =========================================================================
    # Nodes
    # =========================================================================

    # Laser Scan Merger Node
    merger_node = Node(
        package='ros2_laser_scan_merger',
        executable='ros2_laser_scan_merger',
        name='ros2_laser_scan_merger',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'pointCloutFrameId': output_frame,
                'scanTopic1': scan_topic_1,
                'scanTopic2': scan_topic_2,
                'pointCloudTopic': cloud_topic,
            }
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    # Pointcloud to LaserScan Conversion Node
    pc_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'target_frame': output_frame,
            }
        ],
    )

    # Optional: Static TF2 Transform Publisher
    # Uncomment and configure as needed for your robot setup
    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='laser_frame_publisher',
    #     arguments=[
    #         '--x', '0', '--y', '0', '--z', '0',
    #         '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
    #         '--frame-id', 'map',
    #         '--child-frame-id', output_frame
    #     ],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # =========================================================================
    # Launch Description
    # =========================================================================

    return LaunchDescription([
        # Launch arguments
        params_file_arg,
        use_sim_time_arg,
        output_frame_arg,
        scan_topic_1_arg,
        scan_topic_2_arg,
        cloud_topic_arg,
        enable_respawn_arg,
        respawn_delay_arg,

        # Nodes
        merger_node,
        pc_to_scan_node,
        # static_tf_node,  # Uncomment if needed
    ])
