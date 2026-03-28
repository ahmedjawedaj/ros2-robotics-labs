from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen'
    )

    spawn2 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/spawn', 'turtlesim/srv/Spawn',
            '{x: 1.0, y: 1.0, theta: 0.0, name: "turtle2"}'
        ],
        output='screen'
    )

    chase = Node(
        package='lab2',
        executable='chase',
        output='screen',
        parameters=[{
            'target': 'turtle1',
            'chaser': 'turtle2',
            'k_lin': 1.2,
            'k_ang': 6.0,
            'max_lin': 2.0,
            'max_ang': 4.0,
            'catch_dist': 0.5,
            'manual_override': True,
            'manual_timeout': 0.35
        }]
    )

    return LaunchDescription([
        turtlesim,
        TimerAction(period=0.8, actions=[spawn2]),
        chase
    ])
