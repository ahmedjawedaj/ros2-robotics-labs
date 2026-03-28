from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'lab3_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed-jawed',
    maintainer_email='ahmed-jawed@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
            'console_scripts': [
                'turtlebot3_circle = lab3_perception.turtlebot3_circle:main',
                'turtlebot3_go2goal = lab3_perception.turtlebot3_go2goal:main',
                'turtlebot3_laserscan = lab3_perception.turtlebot3_laserscan:main',
                'dynamic_tf_broadcaster = lab3_perception.dynamic_tf_broadcaster:main',
                'turtlebot3_mapping = lab3_perception.turtlebot3_mapping:main',

            ],
        },
)
