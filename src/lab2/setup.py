from setuptools import setup
import os
from glob import glob

package_name = 'lab2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed-jawed',
    maintainer_email='ahmed.jawed32@gmail.com',
    description='AI641 Lab2 Task2 turtlesim nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'chase = lab2.chase:main',
            'straight_line = lab2.straight_line:main',
            'go_to_goal = lab2.go_to_goal:main',
            'move_square = lab2.move_square:main',
            'move_circle = lab2.move_circle:main',
        ],
    },
)
