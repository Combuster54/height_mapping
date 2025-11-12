import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    debug_mode = LaunchConfiguration('debug_mode')

    hm_share = get_package_share_directory('height_mapping')
    hm_yaml  = os.path.join(hm_share, 'config', 'height_mapping_node.yaml')
    vis_yaml = os.path.join(hm_share, 'config', 'include', 'heightmap_visualization.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Publicar info de depuración'
        ),

        # Mapping local
        Node(
            package='height_mapping',
            executable='height_mapping_node',
            #name='height_mapping_local',
            output='screen',
            parameters=[hm_yaml, {'debug_mode': debug_mode}],
        ),

        # Visualización con grid_map_visualization
        Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
           # name='height_mapping_vis',
            output='log',
            parameters=[vis_yaml],
        ),
    ])
