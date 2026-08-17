import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    igvc_bringup_share = get_package_share_directory('igvc_bringup')
    default_params = os.path.join(igvc_bringup_share, 'config', 'bt_navigator_params.yaml')
    default_xml = os.path.join(igvc_bringup_share, 'config', 'simple_nav.xml')
    bt_xml = os.path.join(
    get_package_share_directory('igvc_bringup'),
    'config',
    'navigate_to_pose_w_replanning_and_recovery.xml'
)
    global_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('igvc_global_planner'),
                'launch',
                'global_planner_launch.py'
            )
        )
    )

    local_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("igvc_local_planner"),
                'launch',
                'local_planner_launch.py'
            )
        )
    )

    bt_navigator_launch = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            default_params,
            {
                'use_sim_time': True,
                'default_nav_to_pose_bt_xml': bt_xml,
                'default_nav_through_poses_bt_xml': bt_xml,
            }
        ]

    )

    behavior_server_launch = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[
            default_params,
            {'use_sim_time': True}
        ]
    )

    lifecycle_manager_launch = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }
        ]
    )

    return LaunchDescription ([
        global_planner_launch,
        local_planner_launch,
        behavior_server_launch,
        bt_navigator_launch,
        lifecycle_manager_launch])