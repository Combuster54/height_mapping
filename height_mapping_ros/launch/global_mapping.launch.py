import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    debug_mode = LaunchConfiguration('debug_mode')

    hm_share = get_package_share_directory('height_mapping')
    global_yaml = os.path.join(hm_share, 'config', 'global_mapping_node.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Publicar info de depuración'
        ),

        Node(
            package='height_mapping',
            executable='global_mapping_node',
            #name='height_mapping_global',
            output='screen',
            parameters=[global_yaml, {'debug_mode': debug_mode}],
        ),
    ])
