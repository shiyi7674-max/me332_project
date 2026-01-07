from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')

    mode = DeclareLaunchArgument(
        'slam',
        default_value='false',
        description='Run SLAM or Nav2'
    )

    return LaunchDescription([
        mode,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'slam_sim.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('slam'))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav2_sim.launch.py')
            ),
            condition=UnlessCondition(LaunchConfiguration('slam'))
        )
    ])

