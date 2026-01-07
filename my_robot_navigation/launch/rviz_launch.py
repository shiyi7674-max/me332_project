import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('my_robot_navigation')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(pkg_nav, 'rviz', 'nav2_view.rviz')
            ],
            parameters=[{'use_sim_time': True}]
        )
    ])

