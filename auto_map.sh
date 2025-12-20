#!/bin/bash

# 使用方法: ./auto_map.sh <world_name>
# 例如: ./auto_map.sh complex_room.world

WORLD_NAME=$1

# 如果没有输入参数，默认用 empty.sdf
if [ -z "$WORLD_NAME" ]; then
    WORLD_NAME="simple_room.world"
    echo "未指定 World，默认使用: $WORLD_NAME"
fi

# 提取不带后缀的文件名作为目录名 (complex_room.world -> complex_room)
MAP_DIR_NAME=$(basename "$WORLD_NAME" .world)
# 如果是 .sdf 也要处理
MAP_DIR_NAME=$(basename "$MAP_DIR_NAME" .sdf)

# 保存路径设为 src/my_robot_navigation/maps/<world_name>
# 注意：这里假设你在 workspace 根目录运行脚本
SAVE_PATH="./src/my_robot_navigation/maps/$MAP_DIR_NAME"

echo "========================================="
echo "  启动环境: $WORLD_NAME"
echo "  地图将保存到: $SAVE_PATH"
echo "========================================="

# 1. 创建保存目录
mkdir -p "$SAVE_PATH"

# 2. 后台启动仿真和 SLAM
ros2 launch my_robot_navigation slam_sim.launch.py world:=$WORLD_NAME &
SIM_PID=$!

# 3. 等待用户完成建图
echo ""
echo ">>> 系统已启动，请在 Rviz 中控制机器人建图 (需要另开终端运行键盘控制)"
echo ">>> ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ">>> 建图完成后，回到这里按下 [Enter] 键保存地图并退出..."
read -p ""

# 4. 保存地图
echo ">>> 正在保存地图..."
# 使用 nav2 map_saver 保存，文件名统一为 map
ros2 run nav2_map_server map_saver_cli -f "$SAVE_PATH/map"

# 5. 结束进程
echo ">>> 保存成功！正在关闭仿真..."
kill $SIM_PID

echo ">>> 完成！"
