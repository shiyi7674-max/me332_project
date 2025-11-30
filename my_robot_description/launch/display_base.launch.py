import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_description'
    
    # 这一行指向刚才那个“临时总装文件”
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'test_base.xacro')
    
    # 解析 xacro 得到 xml 字符串
    robot_description_config = Command(['xacro ', xacro_file])
    
    params = {'robot_description': robot_description_config}

    # 1. 启动 robot_state_publisher (发布 TF 树)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 2. 启动 joint_state_publisher_gui (给你一个滑块来转动轮子，检查轴向)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 3. 启动 Rviz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # 如果你有保存好的 .rviz 配置，可以在这里加 arguments=['-d', path_to_config]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz,
    ])
