#!/usr/bin/env python3
#
# Author: Lior Schwartz

import os

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    recorder_node_param_dir = LaunchConfiguration(
        'camera_node_param_dir',
        default=os.path.join(
            get_package_share_directory('mnt_surveillance_bringup'),
            'params',
            'recorder_node_params' + '.yaml'))

    return LaunchDescription([

        DeclareLaunchArgument(
            'recorder_node_param_dir',
            default_value=recorder_node_param_dir,
            description='Full path to camera_node parameter file to load'),

        Node(
            package='mnt_surveillance_master_node',
            executable='recorder_node',
            parameters=[recorder_node_param_dir],
            output='screen')
    ])
