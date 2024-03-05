cmake_minimum_required(VERSION 3.8)
project(robotname_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_itsrobocon_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(
  joy_node
  src/joy_node.cpp
  include/robotname_hardware/remote-controller/ds4.h)

ament_target_dependencies(joy_node rclcpp std_msgs sensor_msgs)

add_executable(
  robot_node 
  src/robot_node.cpp 
  src/udp.cpp
  
)

ament_target_dependencies(
  robot_node 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  std_srvs 
  robot_itsrobocon_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
)

install(TARGETS
  robot_node
  joy_node
  DESTINATION lib/${PROJECT_NAME})


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
