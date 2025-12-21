import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ros2_sim'
    
    # URDF文件路径
    urdf_file = os.path.join(
        get_package_share_directory(pkg_name), 
        'urdf', 
        'ros2_sim.urdf'
    )
    
    print(f"URDF文件: {urdf_file}")
    print(f"文件存在: {os.path.exists(urdf_file)}")
    
    # 直接读取文件内容
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 1. 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
       }]
    )

    # 2. Ignition Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # 3. 生成机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mobile_manipulator',
            '-z', '0.5'
        ],
        output='screen'
    )

    # 4. 关节状态发布器GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 5. RViz2
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'view_robot.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        joint_state_publisher_gui,
        spawn_entity,
        rviz_node,
    ])
