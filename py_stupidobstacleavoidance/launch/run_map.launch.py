#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # world_file_name = 'turtlebot3_worlds/' + TURTLEBOT3_MODEL + '.model'
    world_file_name = 'custom_worlds/' + TURTLEBOT3_MODEL + '.world'
    world_file_name = 'custom_worlds/' + 'challenge_maze' + '.world'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
        # Node(
        #     package='py_stupidobstacleavoidance',
        #     executable='the_smartest_bot_i_ve_ever_seen',
        #     name='my_smart_bot'
        # )
    ])
