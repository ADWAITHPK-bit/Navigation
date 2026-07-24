import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    igvc_local_planner_share = get_package_share_directory('igvc_local_planner')
    default_params = os.path.join(igvc_local_planner_share, 'config', 'local_planner_launch.yaml')
    costmap_params = os.path.join(get_package_share_directory('igvc_costmap'), 'config','costmap_params.yaml')

    local_planner_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[default_params,
                    costmap_params]


    )

    return LaunchDescription([local_planner_node])
