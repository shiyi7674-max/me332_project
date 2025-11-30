# ME332 机器人操作系统期末课程设计

## 🤖 项目简介
这是我们小组的 ROS 2 移动操作机器人项目。
基于 **ROS 2 Humble** 和 **Gazebo Fortress (Ignition)** 仿真环境开发。

- **M1 (底盘/导航)**: 负责底盘建模、SLAM 建图、Nav2 导航。
- **M2 (机械臂)**: 负责机械臂建模、MoveIt2 运动规划。
- **M3 (集成/AI)**: 负责传感器配置、环境搭建、人机交互 (AI)。

---

## 🛠️ 1. 环境依赖 (必读!)
在运行代码前，请确保你的电脑安装了以下包（可以直接复制命令运行）：

```bash
sudo apt update
# 安装构建工具
sudo apt install python3-colcon-common-extensions git

# 安装 ROS 2 核心依赖
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher

# 安装新版 Gazebo (Ignition) 及其桥接包 (关键!)
# 注意：如果你使用的是老师提供的虚拟机/环境，这一步可能已完成
sudo apt install ros-humble-ros-gz

# 安装导航与建图库
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# 安装机械臂控制库
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
