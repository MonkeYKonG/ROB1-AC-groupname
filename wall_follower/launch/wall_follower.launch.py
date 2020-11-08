#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_follower',
            name='wall_follower'
        )
    ])
