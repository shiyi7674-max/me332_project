import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 1. [修正] 指向总装文件 robot.urdf.xacro，而不是 test_base.xacro
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'robot.urdf.xacro')
    
    robot_description_config = Command(['xacro ', xacro_file])
    params = {'robot_description': robot_description_config, 'use_sim_time': True}

    # 2. 机器人状态发布 (RSP)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 3. 启动 Ignition Gazebo
    # -r 表示自动运行，empty.sdf 是空环境
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. 生成机器人 (Spawn)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_robot',
            '-z', '0.2' # 稍微抬高
        ],
        output='screen'
    )

    # ... (前面的代码保持不变)

    # 5. [修正] ROS-Gazebo 桥接 (Bridge)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # --- 1. 关键控制话题 ---
            # 告诉 Bridge 去 Ignition 里的 /model/my_robot/cmd_vel 这个地址送信
            '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # --- 2. 里程计 (Odom) ---
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # --- 3. TF 变换 ---
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # --- 4. 时钟 ---
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # --- [新增] 传感器话题桥接 ---
            
            # 1. 雷达 (Lidar)
            # 格式: /话题名@ROS消息类型[Ignition消息类型
            #'/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            
            # 2. 深度相机/RGB相机 (Camera)
            # 图像数据
            #'/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # 相机参数 (用于把图像投影到3D空间)
            #'/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            
            # (可选) 如果你有深度图
            #'/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            #'/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        
        # [关键重映射]
        # 把 Ignition 那边复杂的 "/model/my_robot/cmd_vel" 映射回 ROS 常用的 "/cmd_vel"
        # 这样 teleop_twist_keyboard 发出的指令才能被收到
        remappings=[
            ('/model/my_robot/cmd_vel', '/cmd_vel'),
            ('/model/my_robot/odometry', '/odom'),
            ('/model/my_robot/tf', '/tf'),
        ],
        output='screen'
    )
    
    # ... (后面的代码保持不变)

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn,
        bridge,
    ])
