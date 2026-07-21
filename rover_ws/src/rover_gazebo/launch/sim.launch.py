import os

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
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
                 ' locked_suspension:=',
                 LaunchConfiguration('locked_suspension')]),
        value_type=str)

    # let gz-sim find the RockerDifferential system plugin
    plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(get_package_prefix('rover_gz_plugins'), 'lib'))

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
                        '(obstacle_world.sdf, mars_world.sdf, '
                        'mars_rough_world.sdf suspension test track, or '
                        'mars_demo_world.sdf full-capability demo)'),
        DeclareLaunchArgument(
            'locked_suspension', default_value='false',
            description='Rigidify the rocker/bogie arms (debug only; the '
                        'suspension is fully articulated by default)'),

        plugin_path,
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
