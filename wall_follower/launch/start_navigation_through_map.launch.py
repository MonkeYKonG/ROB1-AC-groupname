#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir


TURTLEBOT3_MODEL = os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')


def generate_launch_description():
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    nav2_launch_file_dir_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    navigation_param_file_path = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'param', param_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav2_launch_file_dir = LaunchConfiguration('navigation_launch_dir', default=nav2_launch_file_dir_path)
    navigation_param_dir = LaunchConfiguration('params', default=navigation_param_file_path)
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'params': navigation_param_dir,
                              'map': map_file}.items(),
        ),
    ])
