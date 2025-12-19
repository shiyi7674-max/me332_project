cmake_minimum_required(VERSION 3.8)
project(cpp_component_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(example_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)
# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(server_component SHARED
  src/server_component.cpp)
target_compile_definitions(server_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(server_component
  "example_interfaces"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(server_component "composition::Server")
set(node_plugins "${node_plugins}composition::Server;$<TARGET_FILE:server_component>\n")

add_library(client_component SHARED
  src/client_component.cpp)
target_compile_definitions(client_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(client_component
  "example_interfaces"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(client_component "composition::Client")
set(node_plugins "${node_plugins}composition::Client;$<TARGET_FILE:client_component>\n")

install(TARGETS
  server_component 
  client_component 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
 # Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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
