#!/usr/bin/env python3
"""
将机器人加载到 Gazebo 仿真环境中的启动文件
支持通过 world 参数指定不同的世界文件
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_gazebo_share = get_package_share_directory('my_robot_gazebo')
    pkg_description_share = get_package_share_directory('my_robot_description')
    
    # 启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_x = LaunchConfiguration('robot_x', default='0.0')
    robot_y = LaunchConfiguration('robot_y', default='0.0')
    robot_z = LaunchConfiguration('robot_z', default='0.0')
    robot_yaw = LaunchConfiguration('robot_yaw', default='0.0')
    world_name = LaunchConfiguration('world', default='simple_room.world')
    
    # XACRO 文件路径
    xacro_path = os.path.join(
        pkg_description_share,
        'urdf',
        'robot.urdf.xacro'
    )
    
    # 检查 XACRO 文件是否存在
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"XACRO文件不存在: {xacro_path}")
    
    print(f"使用 XACRO 文件: {xacro_path}")
    
    # 使用 Command 和 xacro 命令处理 XACRO 文件
    robot_description_content = Command([
        'xacro ', xacro_path,
        ' use_sim:=', 'true'
    ])
    
    # 方法1：使用字符串替换的方式构建世界文件路径（推荐）
    # 注意：这个路径检查会在运行时进行，而不是在launch文件解析时
    world_path = os.path.join(pkg_gazebo_share, 'worlds', '')
    # 实际的路径会在gazebo启动时由LaunchConfiguration提供
    
    # 方法2：使用PathJoinSubstitution（更优雅的ROS2方式）
    world_path_substitution = PathJoinSubstitution([
        FindPackageShare('my_robot_gazebo'),
        'worlds',
        world_name
    ])
    
    print(f"世界文件将来自: {world_path}[world参数值]")
    
    # 声明启动参数
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='使用仿真时间'
    )
    
    declare_robot_x = DeclareLaunchArgument(
        name='robot_x',
        default_value='0.0',
        description='机器人初始X坐标'
    )
    
    declare_robot_y = DeclareLaunchArgument(
        name='robot_y',
        default_value='0.0',
        description='机器人初始Y坐标'
    )
    
    declare_robot_z = DeclareLaunchArgument(
        name='robot_z',
        default_value='0.0',
        description='机器人初始Z坐标'
    )
    
    declare_robot_yaw = DeclareLaunchArgument(
        name='robot_yaw',
        default_value='0.0',
        description='机器人初始偏航角'
    )
    
    declare_world_name = DeclareLaunchArgument(
        name='world',
        default_value='simple_room.world',
        description='要加载的世界文件名称（位于 my_robot_gazebo/worlds/ 目录下）'
    )
    
    # 启动 Gazebo
    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments={
            'world': world_path_substitution,  # 使用PathJoinSubstitution
            'verbose': 'false',
            'pause': 'false'
        }.items()
    )
    
    # 发布机器人状态的节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )
    
    # 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', robot_x,
            '-y', robot_y,
            '-z', robot_z,
            '-Y', robot_yaw
        ],
        output='screen'
    )
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加声明的参数
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_x)
    ld.add_action(declare_robot_y)
    ld.add_action(declare_robot_z)
    ld.add_action(declare_robot_yaw)
    ld.add_action(declare_world_name)
    
    # 添加动作
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    
    return ld
