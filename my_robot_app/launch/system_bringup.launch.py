#!/usr/bin/env python3
"""
系统一键启动文件 - 启动所有AI交互节点
"""

import os, launch_ros.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_app = get_package_share_directory('my_robot_app')
    
    # 参数声明
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_voice = LaunchConfiguration('enable_voice', default='true')
    enable_gesture = LaunchConfiguration('enable_gesture', default='true')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
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
    
    # AI检测节点 - 直接调用Python模块
    ai_detector_node = ExecuteProcess(
        cmd=['python3', '-m', 'my_robot_app.modules.ai_detector'],
        output='screen',
        name='ai_detector'
    )
    
    # 语音控制节点
    voice_node = ExecuteProcess(
        cmd=['python3', 'scripts/voice_ctrl_node.py'],
        output='screen',
        cwd=[pkg_app],  # 设置工作目录
        name='voice_control_node'
    )
    
    # 手势控制节点
    gesture_node = ExecuteProcess(
        cmd=['python3', 'scripts/gesture_ctrl_node.py'],
        output='screen',
        cwd=[pkg_app],
        name='gesture_control_node'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_voice)
    ld.add_action(declare_enable_gesture)
    
    # 添加节点
    ld.add_action(ai_detector_node)
    ld.add_action(voice_node)
    ld.add_action(gesture_node)
    
    return ld
