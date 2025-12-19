cmake_minimum_required(VERSION 3.8)
project(my_robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)

install(DIRECTORY
  launch
  config
  scripts
  params
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

