# costmap_only.launch.py
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='local_costmap',
            namespace='local_costmap',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('obstacle_perception'),
                    'config',
                    'local_costmap.yaml'
                ])
            ]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['local_costmap']
            }]
        )
    ])