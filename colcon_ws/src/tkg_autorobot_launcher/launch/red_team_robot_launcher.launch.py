import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_launcher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tkg_autorobot_launcher'), 'launch'), '/robot_launcher.launch.py']),
            )

    commander_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tkg_autorobot_commander'), 'launch'), '/commander_for_red_team.launch.py']),
            )

    return LaunchDescription([
        robot_launcher,
        commander_launch,
    ])

