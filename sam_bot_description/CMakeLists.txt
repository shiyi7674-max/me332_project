cmake_minimum_required(VERSION 3.8)
project(sam_bot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 明确安装每个目录
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*"  # 安装所有文件
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*"
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*"
)

# 或者如果只想安装特定文件
 install(
   FILES urdf/sam_bot_description.urdf
   DESTINATION share/${PROJECT_NAME}/urdf/
 )

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
