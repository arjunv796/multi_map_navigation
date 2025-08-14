import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to your SQLite database file
    # You should create this db file or change the path to an existing one
    db_path = os.path.join(
        get_package_share_directory('multi_map_navigation'),
        'maps',
        'wormhole.db'
    )

    return LaunchDescription([
        Node(
            package='multi_map_navigation',
            executable='multi_map_server',
            name='multi_map_server',
            output='screen',
            parameters=[{
                'db_path': db_path
            }]
        )
    ])
