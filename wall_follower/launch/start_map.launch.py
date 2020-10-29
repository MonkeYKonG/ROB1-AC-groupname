from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    world = 'src/custom/smart_bot_driver/worlds/challenge_maze.burger.world'
    env = os.environ
    env['GAZEBO_MODEL_PATH'] = ':'.join(('src/custom/smart_bot_driver/models',
                                         'src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models',
                                         env.setdefault('GAZEBO_MODEL_PATH', '')))

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'],
            output='screen',
            env=env
        ),
    ])
