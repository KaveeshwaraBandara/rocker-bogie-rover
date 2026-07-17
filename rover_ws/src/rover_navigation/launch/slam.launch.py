import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('rover_navigation')
    pkg_slam = get_package_share_directory('slam_toolbox')
    slam_params = os.path.join(pkg_nav, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'slam.rviz')

    # slam_toolbox is a lifecycle node in Jazzy; its own launch file handles
    # the configure/activate transitions (autostart)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz with SLAM view'),

        slam,

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
