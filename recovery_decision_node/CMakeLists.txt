cmake_minimum_required(VERSION 3.8)
project(recovery_decision_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(recovery_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# BT Plugin
add_library(recovery_decision_node_bt_plugins SHARED
  src/topic_boolean_condition.cpp
)
target_include_directories(recovery_decision_node_bt_plugins PUBLIC include)
ament_target_dependencies(recovery_decision_node_bt_plugins
  rclcpp std_msgs behaviortree_cpp_v3
)

# ROS Node
add_executable(recovery_decision_node src/recovery_decision_node.cpp)
target_include_directories(recovery_decision_node PUBLIC include)
ament_target_dependencies(recovery_decision_node
  rclcpp std_msgs recovery_interfaces
)

# Plugin description
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

# Installation
install(TARGETS
  recovery_decision_node_bt_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
 # DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin.xml)

ament_package()