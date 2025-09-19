cmake_minimum_required(VERSION 3.8)
project(navi_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# MOC
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Merged source file libs
#  clicked_point_logger_panel.cpp
#  waypoint_navigator_panel.cpp
add_library(navi_plugin SHARED
  src/clicked_point_logger_panel.cpp
  src/clicked_pose_logger_panel.cpp
  src/waypoint_navigator_panel.cpp
  # src/follow_waypoints_from_csv_node.cpp
  src/pose2d_tool.cpp
  include/navi_plugin/clicked_point_logger_panel.hpp
  include/navi_plugin/clicked_pose_logger_panel.hpp
  include/navi_plugin/waypoint_navigator_panel.hpp
  # include/navi_plugin/pose_csv.hpp
  include/navi_plugin/pose2d_tool.hpp
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rviz_common
  rviz_rendering
  rviz_default_plugins
  Qt5           
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)
target_include_directories(navi_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
) 

# Marker publisher
add_executable(waypoint_marker_publisher src/waypoint_marker_publisher.cpp)
ament_target_dependencies(waypoint_marker_publisher
  rclcpp
  geometry_msgs
  visualization_msgs
)
add_executable(pose_marker_publisher src/pose_marker_publisher.cpp)
ament_target_dependencies(pose_marker_publisher
  tf2
  tf2_geometry_msgs
  tf2_ros
  rclcpp
  geometry_msgs
  visualization_msgs
)
target_include_directories(pose_marker_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# add_executable(follow_waypoints src/follow_waypoints_from_csv_node.cpp)
# ament_target_dependencies(follow_waypoints
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   nav2_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
# )
# target_include_directories(follow_waypoints PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# Install targets
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS waypoint_marker_publisher DESTINATION lib/${PROJECT_NAME})
install(TARGETS pose_marker_publisher DESTINATION lib/${PROJECT_NAME})
# install(Targets follow_waypoints DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
# Install the pluginlib export file
install(EXPORT ${PROJECT_NAME}
  FILE navi_pluginTargets.cmake
  NAMESPACE navi_plugin::
  DESTINATION lib/cmake/navi_plugin
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
