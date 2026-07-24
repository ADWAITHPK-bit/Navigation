import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    igvc_sim_share = get_package_share_directory('igvc_sim')
    world_path = os.path.join(igvc_sim_share, 'worlds', 'new.world')

    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # Stand-in robot: TurtleBot3 (swap for your real URDF later)
    os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle')
    tb3_description_share = get_package_share_directory('turtlebot3_gazebo')
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_description_share, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': '0.0', 'y_pose': '0.0'}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(tb3_description_share, 'launch', 'robot_state_publisher.launch.py')
    )
)

    return LaunchDescription([robot_state_publisher, gzserver, gzclient, spawn_tb3])
