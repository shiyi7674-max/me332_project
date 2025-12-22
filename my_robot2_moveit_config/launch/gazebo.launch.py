import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue 
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    moveit_config_package = "my_robot2_moveit_config"
    description_package = "my_robot_description"

    # 1. URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", "robot.urdf.xacro"]),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # 2. SRDF
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "my_robot.srdf"]),
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)
    }
    
    # 3. Kinematics
    kinematics_yaml = load_yaml(moveit_config_package, "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # 4. MoveIt Controllers
    moveit_controllers = load_yaml(moveit_config_package, "config/moveit_controllers.yaml")

    # 5. Trajectory Execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # --- 关键修复：定义 OMPL 规划管道及适配器 ---
    # 这段配置解决了 "Time between points 0 and 1" 的问题
    ompl_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            # 关键修改：去掉 [ ]，改为用空格分隔的长字符串
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixStartPositionBounds "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartPositionCollision"
            ),
            "response_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
        },
    }
    # 6. Ignition Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("ros_ign_gazebo"), "launch", "ign_gazebo.launch.py"])]
        ),
        launch_arguments={"ign_args": "-r --render-engine ogre empty.sdf"}.items(),
    )

    # 7. Bridge
    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ],
        output="screen",
        remappings=[
            ('/clock', '/clock'),
        ]
    )

    # 8. Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    # 9. Spawn Entity
    spawn_entity = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-z", "0.2", 
        ],
        output="screen",
    )

    # 10. Controllers Spawner
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    # 11. Move Group (核心修复节点)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            moveit_controllers,
            trajectory_execution,
            ompl_config, # 加载带适配器的 OMPL 配置
            {"use_sim_time": True}, 
        ],
    )

    # 12. RViz
    run_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            moveit_controllers,
            ompl_config, # RViz 也需要知道规划器配置
            {"use_sim_time": True},
        ],
        arguments=["-d", PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "moveit.rviz"])],
    )

    # ================= 启动逻辑重组 =================

    start_moveit_rviz = [run_move_group_node, run_rviz_node]

    cb_start_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_arm_controller,
            on_exit=start_moveit_rviz, 
        )
    )

    cb_start_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller, load_gripper_controller, cb_start_moveit],
        )
    )

    cb_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster, cb_start_controllers],
        )
    )

    delayed_ros_start = TimerAction(
        period=5.0,
        actions=[
            node_robot_state_publisher,
            spawn_entity,
            cb_spawn
        ]
    )

    return LaunchDescription([
        bridge,
        ign_gazebo,
        delayed_ros_start
    ])
