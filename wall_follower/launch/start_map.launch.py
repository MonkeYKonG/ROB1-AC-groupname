import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    env = os.environ
    turtlebot3_model = env.setdefault('TURTLEBOT3_MODEL', 'burger')
    world_dir_path = os.path.join(get_package_share_directory("wall_follower"), 'worlds')
    world_file_name = f'challenge_maze.{turtlebot3_model}.world'
    world_file_path = os.path.join(world_dir_path, world_file_name)
    turtlebot3_gazebo_launch_file_dir_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file_path)
    models_path = LaunchConfiguration('models_path', default=os.path.join(get_package_share_directory('wall_follower'), 'models'))

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', models_path),
        ExecuteProcess(cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_init.so'], output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_launch_file_dir_path, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])
