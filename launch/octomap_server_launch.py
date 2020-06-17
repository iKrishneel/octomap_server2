#!/usr/bin/env python

import os.path as osp

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import \
    PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    params = {'resolution': 0.2,
              'frame_id': 'livox_frame',
              'sensor_model/max_range': 20.0,
              'base_frame_id': 'base_footprint',
              'height_map': True,
              'colored_map': False,
              'color_factor': 0.8,
    }
    
    remap = [('cloud_in', '/livox/lidar')]
    node = Node(package='octomap_server2',
                 executable='octomap_server',
                 output='screen',
                 remappings=remap,
                 parameters=[params])
    return LaunchDescription([node])
