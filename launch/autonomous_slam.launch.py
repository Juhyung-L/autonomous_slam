import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_bot')

    autonomous_explorer_node = Node(
        package='mobile_bot',
        executable='autonomous_explorer',
        name='autonomous_explorer_node',
        output='screen'
    )

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_bringup.launch.py')
        )
    )

    return LaunchDescription([
        autonomous_explorer_node,
        robot_bringup
    ])