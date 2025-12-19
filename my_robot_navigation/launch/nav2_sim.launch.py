import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ========= 路径 =========
    pkg_desc = get_package_share_directory('my_robot_description')
    pkg_nav  = get_package_share_directory('my_robot_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # ========= 参数 =========
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            pkg_nav, 'maps', 'my_new_map.yaml'
        ),
        description='Full path to map yaml file to load'
    )

    # ========= 启动描述 =========
    return LaunchDescription([
        # --- 地图参数 ---
        map_arg,
        # --- 1. 仿真 / 机器人 / TF / 传感器 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_desc, 'launch', 'sim_base.launch.py')
            )
        ),

        # --- 2. Nav2（地图导航模式，非 SLAM） ---
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        #    ),
        #    launch_arguments={
        #        'use_sim_time': 'true',
        #        'map': LaunchConfiguration('map'),
        #        'params_file': os.path.join(
        #            pkg_nav, 'params', 'nav2_params.yaml'
        #        ),
        #        'autostart': 'true',
        #        #'use_composition': 'false'
        #    }.items()
        #),
       
        IncludeLaunchDescription(
    	    PythonLaunchDescriptionSource(
        	os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
    	    ),
    	    launch_arguments={
        	'use_sim_time': 'true',
        	'map': LaunchConfiguration('map'),
        	'params_file': os.path.join(
            	    pkg_nav,
            	'params',
            	'nav2_params.yaml'
        	),
        	'autostart': 'true',
        	'use_composition': 'False'
    	    }.items()
	),


        # --- 3. RViz（始终启动） ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'rviz_launch.py')
            )
        )
    ])

