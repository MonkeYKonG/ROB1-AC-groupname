#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    turtlebot3_cartographer_launch_dir = os.path.join(turtlebot3_cartographer_prefix, 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_occupancy_grid_dir = LaunchConfiguration('cartographer_occupancy_grid',
                                                          default=turtlebot3_cartographer_launch_dir)
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
        turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), 'start_cartographer.launch.py']),
            launch_argument={
                'use_sim_time': use_sim_time,
                'cartographer_occupancy_grid_dir': cartographer_occupancy_grid_dir,
                'cartographer_config_dir': cartographer_config_dir,
                'configuration_basename': configuration_basename,
                'resolution': resolution,
                'publish_period_sec': publish_period_sec
            }.items()
        )
    ])
