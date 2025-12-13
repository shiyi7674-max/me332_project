### 📄 README.md

````markdown
# ME332 机器人操作系统期末课程设计

## 🤖 项目简介
这是我们小组的 ROS 2 移动操作机器人项目。
基于 **ROS 2 Humble** 和 **Gazebo Fortress (Ignition)** 仿真环境开发。

- **M1 (底盘/导航)**: 负责底盘建模、SLAM 建图、Nav2 导航。
- **M2 (机械臂)**: 负责机械臂建模、MoveIt2 运动规划。
- **M3 (集成/AI)**: 负责传感器配置、环境搭建、人机交互 (AI)。

---

## 🛠️ 1. 环境依赖

```bash

# 安装导航与建图库
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

# 安装机械臂控制库（此为ai生成不确定真实性）
sudo apt install ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers
````

-----

## 🚀 2. 如何运行 (How to Run)

### 2.1 首次下载与编译

```bash
# 1. 建立工作空间 (如果已有 workspace 可跳过)
mkdir -p ~/me332_ws/src
cd ~/me332_ws/src

# 2. 克隆本项目
git clone [https://github.com/你的GitHub用户名/me332_project.git](https://github.com/你的GitHub用户名/me332_project.git)

# 3. 编译 (回到 ws 根目录)
cd ~/me332_ws
colcon build --packages-select my_robot_description

# 4. 刷新环境
source install/setup.bash
```

### 2.2 启动仿真与底盘

启动 Gazebo 仿真环境、加载机器人模型并启动 ROS-Gazebo 桥接：

```bash
ros2 launch my_robot_description sim_base.launch.py
```

> **注意**：如果 Gazebo 启动后黑屏或没反应，请点击窗口左下角的 **播放按钮 (▶️)** 开始仿真。

### 2.3 键盘控制测试

新开一个终端：

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

*按 `i` 前进，`u`/`o` 旋转。如果车动了，说明环境配置成功。*

### 2.4 运行建图 (SLAM)

（还没做）

保持仿真运行，新开一个终端：

```bash
ros2 launch slam_toolbox online_async_launch.py
```

*然后在 RViz 中添加 Map 组件即可看到建图过程。*

-----

## 📏 3. 硬件规格标准 (严格遵守\!)

为了防止模型合体时散架，请大家建模时严格遵守以下尺寸：

1.  **底盘 (Base)**:
      * 形状：圆柱体
      * 尺寸限制：**300mm x 300mm** (半径 `0.14m`, 高 `0.12m`)
      * 质量：**10.0 kg** (切勿随意改轻，否则抓取时会翻车)
2.  **机械臂 (Arm)**:
      * 安装位置：底盘顶面中心 `(x=0, y=0, z=0.12)`
      * 建议臂长：大臂 `0.20m`, 小臂 `0.20m` (因底盘变小，臂长已缩短以保平衡)
3.  **传感器**:
      * 雷达：安装在底盘顶部前方，注意不要被机械臂遮挡。

-----

## 🤝 4. 团队协作规范 (Git Guide for Beginners)

**为了不把代码搞炸，请大家严格遵守以下流程：**

### ✅ 每天开工第一步：拉取最新代码

在写任何代码之前，**必须**先同步队友的更新！

```bash
cd ~/me332_ws/src/me332_project
git pull
```

### 📤 提交代码三部曲

写完代码后，按顺序执行：

```bash
# 1. 添加文件 (注意：只添加你修改的文件，或者用 . 添加全部)
git add .

# 2. 提交并写清楚干了什么 (M1/M2/M3 是你的代号)
git commit -m "M1: 更新了底盘URDF，修复了摩擦力参数"

# 3. 推送到云端
git push
```

### 🚫 绝对禁止的操作

1.  **禁止上传 `build/`, `install/`, `log/` 文件夹**！(如果 `.gitignore` 生效了会自动忽略，但请留心)。
2.  **禁止直接修改别人的文件夹**。
      * M1 只改 `urdf/base` 和 `navigation`
      * M2 只改 `urdf/arm` 和 `moveit_config`
      * M3 只改 `urdf/sensors` 和 `gazebo`
3.  **遇到冲突 (Conflict) 怎么办？**
      * 不要慌，Git 会提示哪个文件冲突了。
      * 打开那个文件，搜索 `<<<<<<<`，保留正确的代码，删掉乱码。
      * 重新 `add`, `commit`, `push`。

<!-- end list -->

````
