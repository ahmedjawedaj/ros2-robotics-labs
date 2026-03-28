from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    real_arg = DeclareLaunchArgument('real', default_value='0.0')
    imaginary_arg = DeclareLaunchArgument('imaginary', default_value='0.0')

    pub = Node(
        package='lab1',
        executable='publisher',
        output='screen',
        parameters=[{
            'real': LaunchConfiguration('real'),
            'imaginary': LaunchConfiguration('imaginary'),
        }],
    )

    sub = Node(
        package='lab1',
        executable='subscriber',
        output='screen',
    )

    return LaunchDescription([real_arg, imaginary_arg, pub, sub])
