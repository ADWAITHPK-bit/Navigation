import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fusioncore_share = get_package_share_directory('fusioncore_ros')
    igvc_loc_share = get_package_share_directory('igvc_localization')

    fusioncore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fusioncore_share, 'launch', 'fusioncore.launch.py')
        ),
        launch_arguments={
            'fusioncore_config': os.path.join(
                igvc_loc_share, 'config', 'fusioncore_waffle.yaml'
            ),
            'autoconfigure': 'true',
        }.items()
    )

    return LaunchDescription([fusioncore_launch])