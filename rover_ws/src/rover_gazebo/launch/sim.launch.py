import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('rover_gazebo')
    pkg_description = get_package_share_directory('rover_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_description, 'urdf', 'rover.urdf.xacro')
    world_file = PathJoinSubstitution(
        [pkg_gazebo, 'worlds', LaunchConfiguration('world')])
    bridge_config = os.path.join(pkg_gazebo, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(pkg_gazebo, 'rviz', 'sim.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=true',
                 ' articulated:=', LaunchConfiguration('articulated')]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world_file]}.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s --headless-rendering ', world_file]}.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server only, no GUI (for SLAM/testing)'),
        DeclareLaunchArgument(
            'world', default_value='obstacle_world.sdf',
            description='World file name in rover_gazebo/worlds '
                        '(obstacle_world.sdf, mars_world.sdf or '
                        'mars_rough_world.sdf suspension test track)'),
        DeclareLaunchArgument(
            'articulated',
            # passive rocker-bogie suspension needs the mimic (differential)
            # constraint, which only the bullet-featherstone engine enforces —
            # default on exactly for the world built on that engine
            default_value=PythonExpression(
                ["'true' if 'rough' in '", LaunchConfiguration('world'),
                 "' else 'false'"]),
            description='Unlock passive rocker/bogie joints (needs a '
                        'bullet-featherstone world, e.g. mars_rough_world.sdf)'),

        gz_sim,
        gz_sim_headless,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description',
                       '-name', 'rb_rover',
                       '-z', '0.02'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config,
                         'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
