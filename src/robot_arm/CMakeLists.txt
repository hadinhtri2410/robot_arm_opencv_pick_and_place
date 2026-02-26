cmake_minimum_required(VERSION 3.8)
project(robot_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(linkattacher_msgs REQUIRED)

add_executable(pick_and_place_node src/pick_and_place.cpp)
ament_target_dependencies(
  pick_and_place_node
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
  geometry_msgs
  moveit_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  my_robot_interfaces
)

add_executable(pick_and_place_moveit_node src/pick_and_place_moveit.cpp)
ament_target_dependencies(
  pick_and_place_moveit_node
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  shape_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  my_robot_interfaces
  linkattacher_msgs
)

install(TARGETS
  pick_and_place_node
  pick_and_place_moveit_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf meshes worlds rviz models
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
