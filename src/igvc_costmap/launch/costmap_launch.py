import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    igvc_costmap_share = get_package_share_directory("igvc_costmap")
    cost_map_params = os.path.join(igvc_costmap_share, 'config', 'costmap_params.yaml')

    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace='global_costmap', # yaml
        output='screen',
        parameters=[cost_map_params]
    )

    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        namespace='local_costmap',
        output='screen',
        parameters=[cost_map_params]
    )

    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['global_costmap/global_costmap',
                           'local_costmap/local_costmap']
        }]
    )

    return LaunchDescription([
        global_costmap,
        local_costmap,
        nav2_lifecycle_manager
    ])
