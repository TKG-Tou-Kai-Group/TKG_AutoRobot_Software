import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    rs_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('nakanoshima_robot_launcher'), 'launch'), '/rs_launch.py']),
             )
             
    nakanoshima_robot_description_path = os.path.join(
        get_package_share_directory('nakanoshima_robot_description'))

    xacro_file = os.path.join(nakanoshima_robot_description_path,
                              'robots',
                              'nakanoshima_robot.urdf.xacro')
    # xacroをロード
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'false'})
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    rviz_config_file = os.path.join(nakanoshima_robot_description_path, 'config', 'nakanoshima_robot_description.rviz')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("reasonable_outdoor_mobile_robot_bringup"),
            "config",
            "reasonable_outdoor_mobile_robot.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[params, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
        remappings=[
            ('/cmd_vel_stamped', '/diff_drive_controller/cmd_vel'),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        rs_launch,
        control_node,
        node_robot_state_publisher,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        velocity_converter,
        rviz,
    ])

