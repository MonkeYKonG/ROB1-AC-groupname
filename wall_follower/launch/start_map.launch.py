import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    env = os.environ
    turtlebot3_model = env.setdefault('TURTLEBOT3_MODEL', 'burger')
    world_dir_path = os.path.join(get_package_share_directory("wall_follower"), 'worlds')
    world_file_name = f'challenge_maze.{turtlebot3_model}.world'
    world_file_path = os.path.join(world_dir_path, world_file_name)
    turtlebot3_gazebo_launch_file_dir_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    env['GAZEBO_MODEL_PATH'] = ':'.join((f"{env['PWD']}/custom/smart_bot_driver/wall_follower/models", env.setdefault('GAZEBO_MODEL_PATH', '')))

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file_path)

    return LaunchDescription([
        ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'], output='screen', env=env),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_launch_file_dir_path, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
