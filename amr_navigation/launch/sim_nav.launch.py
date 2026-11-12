"""Gemelo digital completo: Gazebo + MCU simulado + bridge + SLAM/AMCL + Nav2 + dock.

    ros2 launch amr_navigation sim_nav.launch.py                 # SLAM (mapeo)
    ros2 launch amr_navigation sim_nav.launch.py mode:=amcl      # mapa guardado
    ros2 launch amr_navigation sim_nav.launch.py rviz:=true gui:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, launch_file, condition=None, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare(package), 'launch', launch_file])),
        launch_arguments=list(args.items()), condition=condition)


def generate_launch_description():
    nav = FindPackageShare('amr_navigation')
    mode = LaunchConfiguration('mode')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam', choices=['slam', 'amcl']),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution(
            [nav, 'maps', 'warehouse.yaml'])),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('battery_volt', default_value='25.0'),
        include('amr_gazebo', 'sim.launch.py', gui=LaunchConfiguration('gui'),
                battery_volt=LaunchConfiguration('battery_volt')),
        # La odometría del MCU simulado llega ~6 s después de lanzar Gazebo.
        TimerAction(period=8.0, actions=[
            include('amr_navigation', 'slam.launch.py',
                    condition=IfCondition(EqualsSubstitution(mode, 'slam')),
                    use_sim_time='true'),
            include('amr_navigation', 'localization.launch.py',
                    condition=IfCondition(EqualsSubstitution(mode, 'amcl')),
                    use_sim_time='true', map=LaunchConfiguration('map')),
            include('amr_navigation', 'navigation.launch.py', use_sim_time='true'),
            Node(package='amr_navigation', executable='dock_detector', name='dock_detector',
                 output='screen',
                 parameters=[PathJoinSubstitution([nav, 'config', 'dock_detector.yaml']),
                             {'use_sim_time': True}]),
        ]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', PathJoinSubstitution([nav, 'rviz', 'nav.rviz'])],
             parameters=[{'use_sim_time': True}]),
    ])
