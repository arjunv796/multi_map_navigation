from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():

    # Package share directory
    pkg_share = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        'share', 'multi_map_navigation'
    )
    default_map_yaml = os.path.join(pkg_share, 'maps', 'map1.yaml')
    default_wormholes_db = os.path.join(pkg_share, 'config', 'wormholes.db')

    nav2_bringup_dir = '/opt/ros/humble/share/nav2_bringup/launch'

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'wormholes_db',
            default_value=default_wormholes_db,
            description='Full path to wormholes database'
        ),

        # Include the Nav2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'bringup_launch.py')
            ),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'use_sim_time': 'False'}.items()
        ),

        # Multi Map Navigation server node
        Node(
            package='multi_map_navigation',
            executable='multi_map_server_node',
            name='multi_map_server',
            output='screen',
            parameters=[{
                'wormholes_db_path': LaunchConfiguration('wormholes_db')
            }]
        )
    ])

