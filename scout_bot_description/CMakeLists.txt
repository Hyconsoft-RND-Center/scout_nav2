cmake_minimum_required(VERSION 3.8)
project(scout_bot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


add_executable(imu_no_orientation_node src/imu_no_orientation_node.cpp)
ament_target_dependencies(imu_no_orientation_node
  rclcpp
  sensor_msgs
)

add_executable(ultrasonic_obstacle_detection_node src/ultrasonic_obstacle_detection_node.cpp)
ament_target_dependencies(ultrasonic_obstacle_detection_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_include_directories(ultrasonic_obstacle_detection_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS imu_no_orientation_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS ultrasonic_obstacle_detection_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch rviz config meshes description
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY include 
  DESTINATION include
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
