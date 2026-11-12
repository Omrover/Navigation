"""Mapeo con slam_toolbox (online async).

    ros2 launch amr_navigation slam.launch.py [use_sim_time:=true]
    ros2 run nav2_map_server map_saver_cli -f ~/maps/lab
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # /scan sin el brazo -> /scan_filtered (lo consumen SLAM y Nav2).
        Node(package='amr_navigation', executable='scan_self_filter', name='scan_self_filter',
             output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        DeclareLaunchArgument('slam_params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('amr_navigation'), 'config', 'slam_mapping.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'slam_params_file': LaunchConfiguration('slam_params_file'),
                              }.items()),
    ])
