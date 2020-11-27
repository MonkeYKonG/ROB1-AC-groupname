#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')


def generate_launch_description():
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    turtlebot3_cartographer_launch_dir = os.path.join(turtlebot3_cartographer_prefix, 'launch')
    cartographer_config_dir_path = os.path.join(turtlebot3_cartographer_prefix, 'config')
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    nav2_launch_file_dir_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    navigation_param_file_path = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'), 'param', param_file_name)
    wall_follower_pack_share_directory = get_package_share_directory("wall_follower")
    wall_follower_launch_dir = os.path.join(wall_follower_pack_share_directory, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    cartographer_occupancy_grid_dir = LaunchConfiguration('cartographer_occupancy_grid',
                                                          default=turtlebot3_cartographer_launch_dir)
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=cartographer_config_dir_path)
    configuration_basename = LaunchConfiguration('configuration_basename', default='turtlebot3_lds_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    nav2_launch_file_dir = LaunchConfiguration('navigation_launch_dir', default=nav2_launch_file_dir_path)
    navigation_param_dir = LaunchConfiguration('params', default=navigation_param_file_path)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([wall_follower_launch_dir, '/start_navigation.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'navigation_launch_dir': nav2_launch_file_dir,
                              'params': navigation_param_dir}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([wall_follower_launch_dir, '/start_cartographer.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time,
                              'resolution': resolution,
                              'publish_period_sec': publish_period_sec,
                              'cartographer_occupancy_grid': cartographer_occupancy_grid_dir,
                              'cartographer_config_dir': cartographer_config_dir,
                              'configuration_basename': configuration_basename}.items(),
        ),
        Node(
            package='wall_follower',
            executable='nav2_wall_follower',
            name='nav2_wall_follower'
        )
    ])
