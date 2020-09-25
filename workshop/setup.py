from setuptools import setup
import os
from glob import glob

package_name = 'workshop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexis',
    maintainer_email='alexis1.craye@epitech.eu',
    description='Node to move forward and turn if a wall is detected on the trajectory',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_smart_bot = workshop.my_smart_bot:main',
        ],
    },
)
