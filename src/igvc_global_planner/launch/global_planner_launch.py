import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    igvc_global_planner_share = get_package_share_directory('igvc_global_planner')
    default_params = os.path.join(igvc_global_planner_share, 'config', 'global_planner_launch.yaml')

    costmap_params = os.path.join(
        get_package_share_directory('igvc_costmap'),
        'config',
        'costmap_params.yaml'
    )

    global_planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[default_params,
                    costmap_params]
    )

    return LaunchDescription([global_planner_node])