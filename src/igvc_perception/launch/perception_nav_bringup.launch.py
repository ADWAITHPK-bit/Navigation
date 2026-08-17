import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # TODO: replace 'YOUR_NAV_PACKAGE' with whatever package
    # bringup_launch.py actually lives in (e.g. your own
    # igvc_navigation package, or nav2_bringup if you're using
    # the stock one with a custom params file).
    nav_bringup_dir = get_package_share_directory('YOUR_NAV_PACKAGE')

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items()
    )

    lane_detector_node = Node(
        package='igvc_perception',
        executable='lane_detector',
        name='lane_detection_node',
        output='screen',
        parameters=[{'show_debug': False}],
    )

    goal_generator_node = Node(
        package='igvc_perception',
        executable='goal_generator',
        name='goal_generator_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        nav2_bringup,
        lane_detector_node,
        goal_generator_node,
    ])