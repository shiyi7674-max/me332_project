import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    pkg_name = 'my_robot_description'
    
    # XACRO文件路径
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'test_base.xacro')
    
    # 使用正确的xacro命令
    robot_description_config = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    # 1. 启动机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 2. 启动 Ignition Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': 'empty.sdf -r'  # -r 参数让仿真自动运行
        }.items()
    )

    # 3. 在 Gazebo 中生成机器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mobile_manipulator',
            '-z', '0.5',  # 增加高度确保在地面之上
            '-x', '0.0',
            '-y', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # 4. 关节状态发布器（GUI界面控制机械臂）
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 5. ROS-Gazebo桥接（可选，用于通信）
    # 桥接器将Ignition话题转换为ROS2话题
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist'
        ],
        output='screen'
    )

    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_states_bridge',
        arguments=[
            '/world/default/model/mobile_manipulator/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen',
        remappings=[
            ('/world/default/model/mobile_manipulator/joint_state', '/joint_states')
        ]
    )

    # 6. 启动RViz2用于可视化
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
        # 首先启动Gazebo
        gazebo_launch,
        
        # 启动机器人状态发布器
        robot_state_publisher,
        
        # 关节状态发布器
        joint_state_publisher_gui,
        
        # 启动桥接器
        bridge_cmd_vel,
        bridge_joint_states,
        
        # 启动RViz2
        rviz_node,
        
        # 等待Gazebo启动后生成机器人
        RegisterEventHandler(
            OnProcessStart(
                target_action=gazebo_launch,
                on_start=[spawn_entity]
            )
        ),
    ])
