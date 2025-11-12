from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    debug_mode   = LaunchConfiguration('debug_mode')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Ruta a tu YAML (tal cual me diste)
    param_file = '/root/slam_ws/src/height_mapping/height_mapping_ros/config/sensor_processor_node.yaml'

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug_mode',
            default_value='false',
            description='Publicar info de depuración'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar reloj simulado (Gazebo)'
        ),

        Node(
            package='height_mapping',
            executable='sensor_processor_node',
            #name='sensor_processor',
            output='screen',
            parameters=[
                param_file,
                {'debug_mode':  debug_mode},
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])