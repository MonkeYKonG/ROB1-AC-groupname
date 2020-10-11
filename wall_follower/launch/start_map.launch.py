from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    world = 'src/worlds/blackfoot_world/worlds/challenge_maze.burger.world'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen'),
    ])
