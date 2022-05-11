#!/usr/bin/env python

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "octomap_server2"
    pkg_share_path = get_package_share_directory(package_name=pkg_name)

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("world_frame_id", default_value=str(DRONE_DEVICE_ID) + "/local_origin"))
    ld.add_action(launch.actions.DeclareLaunchArgument("robot_frame_id", default_value=str(DRONE_DEVICE_ID) + "/fcu"))
    
    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    namespace=DRONE_DEVICE_ID

    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_octomap_server',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                namespace=namespace,
                name='octomap_server',
                package=pkg_name,
                plugin='octomap_server::OctomapServer',
                remappings=[
                    # subscribers
                    ('laser_scan_in', '/' + str(DRONE_DEVICE_ID) + '/rplidar/scan'),
                    
                    # publishers
                    ('octomap_global_binary_out', '~/octomap/global/binary'),
                    ('octomap_global_full_out', '~/octomap/global/full'),
                    ('octomap_local_binary_out', '~/octomap/local/binary'),
                    ('octomap_local_full_out', '~/octomap/local/full'),
                    
                    # service servers
                    ('reset_map_in', '~/reset'),
                ],
                parameters=[
                    pkg_share_path + '/config/params.yaml',
                    {"world_frame_id": launch.substitutions.LaunchConfiguration("world_frame_id")},
                    {"robot_frame_id": launch.substitutions.LaunchConfiguration("robot_frame_id")},
                    {"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))
    return ld
