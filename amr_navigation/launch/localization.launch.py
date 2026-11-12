"""Localización sobre un mapa guardado: map_server + AMCL (modelo omni).

    ros2 launch amr_navigation localization.launch.py map:=/ruta/mapa.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [FindPackageShare('amr_navigation'), 'maps', 'warehouse.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('amr_navigation'), 'config', 'nav2_params.yaml'])),
        # /scan sin el brazo -> /scan_filtered (lo consumen AMCL y Nav2).
        Node(package='amr_navigation', executable='scan_self_filter', name='scan_self_filter',
             output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server', 'amcl']}]),
    ])
