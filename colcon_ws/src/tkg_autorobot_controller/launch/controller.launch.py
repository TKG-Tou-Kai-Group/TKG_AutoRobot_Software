from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='corejp_sample_controller',
            executable='hammer',
            name='hammer',
            output='screen'
        ),
        Node(
            package='corejp_sample_controller',
            executable='mazemaze',
            name='mazemaze',
            output='screen'
        ),
        Node(
            package='corejp_sample_controller',
            executable='roller',
            name='roller',
            output='screen'
        ),
        Node(
            package='tkg_autorobot_controller',
            executable='turret',
            name='turret',
            output='screen',
        ),
    ])
