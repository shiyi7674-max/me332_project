cmake_minimum_required(VERSION 3.8)
project(ros2_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

find_package(tf2_ros REQUIRED)

# 如果你有自定义的C++节点，取消注释并修改
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp)
# install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})

# 安装所有需要的目录
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}
)

#install(DIRECTORY config/
#  DESTINATION share/${PROJECT_NAME}
#)

#install(DIRECTORY meshes/
 # DESTINATION share/${PROJECT_NAME}
#)



# 如果你有Python脚本
# find_package(ament_cmake_python REQUIRED)
# install(PROGRAMS scripts/my_python_script.py
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
