import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('rover_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_nav, 'maps', 'sim_room.yaml')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'nav.rviz')

    # SLAM mode (default): slam_toolbox provides map -> odom while mapping.
    # Scoped group: the rviz:=false passed to slam.launch.py must not leak into
    # this file's own 'rviz' configuration (launch includes are unscoped).
    slam = GroupAction(
        [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'slam.launch.py')),
            launch_arguments={'rviz': 'false'}.items(),
        )],
        scoped=True, forwarding=True,
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # Localization mode: AMCL on a previously saved map
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
            'use_sim_time': 'true',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('slam')),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true: map while navigating (slam_toolbox); '
                        'false: localize on saved map (AMCL)'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Map yaml for localization mode'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz'),

        slam,
        localization,
        navigation,

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
