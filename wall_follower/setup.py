from setuptools import setup
import os
import sys
from glob import glob


def get_all_file_paths(base_dir):
    results = []
    for file in os.listdir(base_dir):
        filepath = os.path.join(base_dir, file)
        if os.path.isfile(filepath):
            results.append(filepath)
        else:
            results += get_all_file_paths(filepath)
    return results


model_paths = get_all_file_paths('models')

package_name = 'wall_follower'
package_name_share_directory = os.path.join('share', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join(package_name_share_directory, 'launch'), glob('launch/*.launch.py')),
        (os.path.join(package_name_share_directory, 'config'), glob('config/*')),
        (os.path.join(package_name_share_directory, 'worlds'), glob('worlds/*')),
        *((os.path.join(package_name_share_directory, os.path.dirname(model)), [model]) for model in model_paths),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexis',
    maintainer_email='alexis1.craye@epitech.eu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = wall_follower.wall_follower:main',
            'nav2_wall_follower = wall_follower.nav2_wall_follower:main',
            'test_tools = wall_follower.test:main',
        ],
    },
)
