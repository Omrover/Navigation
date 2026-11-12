"""Servidores de Nav2 + docking para la base omni (sin route_server ni map).

    ros2 launch amr_navigation navigation.launch.py [use_sim_time:=true]

Cadena de velocidad: controller_server -> cmd_vel_nav -> velocity_smoother ->
cmd_vel_smoothed -> collision_monitor -> cmd_vel (bridge MQTT -> MCU).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LIFECYCLE_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                   'velocity_smoother', 'collision_monitor', 'bt_navigator',
                   'waypoint_follower', 'docking_server']


def generate_launch_description():
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common = [params, {'use_sim_time': use_sim_time}]

    def server(package, executable, name=None, remappings=()):
        return Node(package=package, executable=executable, name=name or executable,
                    output='screen', parameters=common, remappings=list(remappings),
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution(
            [FindPackageShare('amr_navigation'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('log_level', default_value='info'),
        server('nav2_controller', 'controller_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
        server('nav2_smoother', 'smoother_server'),
        server('nav2_planner', 'planner_server'),
        server('nav2_behaviors', 'behavior_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
        server('nav2_velocity_smoother', 'velocity_smoother',
               remappings=[('cmd_vel', 'cmd_vel_nav')]),
        server('nav2_collision_monitor', 'collision_monitor'),
        server('nav2_bt_navigator', 'bt_navigator'),
        server('nav2_waypoint_follower', 'waypoint_follower'),
        server('opennav_docking', 'opennav_docking', name='docking_server'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': LIFECYCLE_NODES}]),
    ])
