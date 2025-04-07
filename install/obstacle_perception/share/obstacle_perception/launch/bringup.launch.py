# ===========================
# File: launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    launch_perception = LaunchConfiguration('launch_perception')
    launch_nav2 = LaunchConfiguration('launch_nav2')

    return LaunchDescription([
        DeclareLaunchArgument('launch_perception', default_value='true'),
        DeclareLaunchArgument('launch_nav2', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            condition=IfCondition(launch_nav2)
        ),

        Node(
            package='obstacle_perception',
            executable='perception_node',
            name='obstacle_perception_node',
            output='screen',
            condition=IfCondition(launch_perception)
        )
    ])
