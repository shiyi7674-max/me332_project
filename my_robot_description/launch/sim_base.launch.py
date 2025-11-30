import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    
    # 指向 xacro 文件
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'test_base.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    # 1. 启动机器人状态发布者 (RSP)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 2. 启动新版 Gazebo (Ignition / Gazebo Sim)
    # 使用 empty.sdf (空环境)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. 在 Gazebo 中生成机器人 (Spawn)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_robot',
                   '-z', '0.1'], # 稍微抬高一点防止卡地里
        output='screen'
    )

    # 4. 关键：ROS-Gazebo 桥接 (Bridge)
    # 将 Gazebo 的 /cmd_vel, /odom, /tf 转换成 ROS 2 标准话题
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        #bridge,
    ])
