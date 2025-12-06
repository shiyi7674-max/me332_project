cmake_minimum_required(VERSION 3.8)
project(my_robot_gazebo)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

# 安装世界文件
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 安装启动文件
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装模型文件
install(
  DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()
