from setuptools import setup

package_name = 'track_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed Jawed',
    maintainer_email='ahmedjawedaj@users.noreply.github.com',
    description='Camera-based track following for TurtleBot3 using ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = track_follower.line_follower:main',
        ],
    },
)
