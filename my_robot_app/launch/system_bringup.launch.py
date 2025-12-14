#!/usr/bin/env python3
"""
系统一键启动文件 - 启动所有AI交互节点
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_app = get_package_share_directory('my_robot_app')
    pkg_gazebo = get_package_share_directory('my_robot_gazebo')
    
    # 参数声明
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    enable_voice = LaunchConfiguration('enable_voice', default='true')
    enable_gesture = LaunchConfiguration('enable_gesture', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    declare_enable_voice = DeclareLaunchArgument(
        'enable_voice',
        default_value='true',
        description='Enable voice control'
    )
    
    declare_enable_gesture = DeclareLaunchArgument(
        'enable_gesture',
        default_value='true',
        description='Enable gesture control'
    )
    
    # AI检测节点
    ai_detector_node = Node(
        package='my_robot_app',
        executable='ai_detector_node',  # 需要创建
        name='ai_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_voice': enable_voice,
            'enable_gesture': enable_gesture,
            'control_mode': 'both',
        }]
    )
    
    # 语音控制节点
    voice_node = Node(
        package='my_robot_app',
        executable='voice_ctrl_node',
        name='voice_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'language': 'zh-CN',
            'energy_threshold': 300,
        }]
    )
    
    # 手势控制节点
    gesture_node = Node(
        package='my_robot_app',
        executable='gesture_ctrl_node',
        name='gesture_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_id': 0,
            'control_sensitivity': 0.5,
            'show_video': True,
        }]
    )
    
    # 机器人仿真启动（可选）
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'world': 'simple_room.world',
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_voice)
    ld.add_action(declare_enable_gesture)
    
    # 添加节点
    # ld.add_action(gazebo_launch)  # 如果需要同时启动仿真
    ld.add_action(ai_detector_node)
    ld.add_action(voice_node)
    ld.add_action(gesture_node)
    
    return ld
