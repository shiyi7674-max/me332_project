import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_slam = get_package_share_directory('slam_toolbox')
    pkg_nav  = get_package_share_directory('my_robot_navigation')
    
    # === 新增：接收 world 参数 ===
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='complex_room.world',
        description='World file to load'
    )

    return LaunchDescription([
        # ▼▼▼▼▼▼ 2. 必须把参数加进这里，否则不生效 ▼▼▼▼▼▼
        world_arg,

        # 1. 仿真 / 机器人 / TF / 传感器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_desc, 'launch', 'sim_base.launch.py')
            ),
            # ▼▼▼▼▼▼ 3. 必须把参数传给下一层 ▼▼▼▼▼▼
            launch_arguments={
                'world': LaunchConfiguration('world')
            }.items()
        ),

        # 2. SLAM Toolbox（只做建图）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(
                    pkg_nav,
                    'config',
                    'maps',
                    'mapper_params_online_async.yaml'
                )
            }.items()
        ),

        # 3. RViz
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'rviz_launch.py')
            )
        )
    ])
