from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tkg_autorobot_commander',
            executable='commander',
            name='commander',
            output='screen',
            parameters=[{'enemy': "RED"}],
        ),
    ])
