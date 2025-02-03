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
                    get_package_share_directory('tkg_autorobot_launcher'), 'launch'), '/rs_launch.py']),
            )

    urg_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tkg_autorobot_launcher'), 'launch'), '/urg_node2.launch.py']),
            )

    controller_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('tkg_autorobot_controller'),
                        'launch',
                        'controller.launch.py'
                    ])
                ]),
            )

    urg_object_detector = Node(
        package="tkg_urg_object_detector",
        executable="detector",
        name="detector",
        output="log",
    )

    #tkg_autorobot_description_path = os.path.join(
    #    get_package_share_directory('tkg_autorobot_description'))

    #xacro_file = os.path.join(nakanoshima_autorobot_description_path,
    #                          'robots',
    #                          'nakanoshima_robot.urdf.xacro')
    # xacroをロード
    #doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'false'})
    # xacroを展開してURDFを生成
    #robot_desc = doc.toprettyxml(indent='  ')

    #params = {'robot_description': robot_desc}

    #rviz_config_file = os.path.join(nakanoshima_robot_description_path, 'config', 'nakanoshima_robot_description.rviz')

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        #arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        rs_launch,
        urg_launch,
        controller_launch,
        urg_object_detector,
        #rviz,
    ])

