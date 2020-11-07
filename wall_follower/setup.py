from setuptools import setup
import os
from glob import glob

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
        (os.path.join(package_name_share_directory, 'config'), glob('config/*.yaml')),
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
        ],
    },
)
