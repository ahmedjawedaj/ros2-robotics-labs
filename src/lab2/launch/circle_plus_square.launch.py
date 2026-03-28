from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess

def _spawn_turtle2(context, *args, **kwargs):
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    theta = LaunchConfiguration('theta').perform(context)

    cmd = [
        'ros2', 'service', 'call',
        '/spawn', 'turtlesim/srv/Spawn',
        f'{{x: {x}, y: {y}, theta: {theta}, name: "turtle2"}}'
    ]
    subprocess.run(cmd, check=False)
    return []

def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen'
    )

    square = Node(
        package='lab2',
        executable='move_square',
        output='screen',
        parameters=[{'side': 3.0}]
    )

    circle = Node(
        package='lab2',
        executable='move_circle',
        output='screen',
        parameters=[{'turtle': 'turtle2', 'radius': 2.5, 'ang': 0.8}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='5.5'),
        DeclareLaunchArgument('y', default_value='5.5'),
        DeclareLaunchArgument('theta', default_value='0.0'),
        turtlesim,
        TimerAction(period=1.0, actions=[OpaqueFunction(function=_spawn_turtle2)]),
        square,
        circle,
    ])
