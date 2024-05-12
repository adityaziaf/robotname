cmake_minimum_required(VERSION 3.8)
project(robotname_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robotname_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(behaviortree_ros2)
find_package(btcpp_ros2_interfaces)
find_package(btcpp_ros2_samples)

# Install directories
install(DIRECTORY
  bt_xml launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install C++ nodes
set(AUTONOMY_SOURCES
    src/plugins/go_to_pose.cpp
    src/plugins/set_locations.cpp 
    src/plugins/go_through_poses.cpp
)

set(TARGET_DEPENDS
    ament_index_cpp rclcpp rclcpp_action
    nav2_msgs sensor_msgs behaviortree_cpp behaviortree_ros2
    tf2 tf2_ros tf2_geometry_msgs robotname_msgs yaml-cpp
)

set(THIS_PACKAGE_DEPS 
    behaviortree_ros2
    std_msgs
    ament_index_cpp
    btcpp_ros2_interfaces 
    tf2 tf2_ros tf2_geometry_msgs
    nav2_msgs
    rclcpp_action
    rclcpp 
    robotname_msgs
    std_srvs
    )

set(THIS_PACKAGE_SOURCES
    src/plugins/get_intake_color.cpp 
    src/plugins/set_intake_mechanism.cpp
    src/plugins/nav_to_pose.cpp 
    src/plugins/nav_through_poses.cpp
    src/plugins/find_nearest_ball.cpp
    src/plugins/follow_path.cpp
    src/plugins/ball_available.cpp
    src/plugins/set_speed.cpp
    src/plugins/get_intake_distance.cpp
    src/plugins/get_intake_proximity_array.cpp
    src/plugins/get_ball_grabbed.cpp
    src/plugins/flush_intake.cpp
    src/plugins/follow_ball.cpp
    src/plugins/get_nearest_ball.cpp
    src/plugins/grab_nearest_ball.cpp
    src/plugins/ball_grabbed.cpp
    src/plugins/amcl_update.cpp
    src/plugins/reset_ball_grabbed.cpp
    src/plugins/get_ball_grabbed_top.cpp
    )

include_directories(include)


#install server

add_executable(get_ball_grabbed_top_server src/action_server/get_ball_grabbed_top_server.cpp)
ament_target_dependencies(get_ball_grabbed_top_server rclcpp ${TARGET_DEPENDS})

install(TARGETS
get_ball_grabbed_top_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(reset_ball_grabbed_server src/server/reset_ball_grabbed_server.cpp)
ament_target_dependencies(reset_ball_grabbed_server rclcpp std_srvs std_msgs)

install(TARGETS
reset_ball_grabbed_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(set_intake_mechanism_server src/server/set_intake_mechanism_server.cpp)
ament_target_dependencies(set_intake_mechanism_server rclcpp robotname_msgs std_msgs)

install(TARGETS
set_intake_mechanism_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(get_intake_distance_server src/server/get_intake_distance_server.cpp)
ament_target_dependencies(get_intake_distance_server rclcpp robotname_msgs std_srvs)

install(TARGETS
get_intake_distance_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(set_speed_server src/server/set_speed_server.cpp)
ament_target_dependencies(set_speed_server rclcpp robotname_msgs geometry_msgs)

install(TARGETS
set_speed_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(find_nearest_ball_server src/action_server/find_nearest_ball_server.cpp)
ament_target_dependencies(find_nearest_ball_server rclcpp robotname_msgs ${TARGET_DEPENDS})

install(TARGETS
  find_nearest_ball_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(ball_available_server src/server/ball_available_server.cpp)
ament_target_dependencies(ball_available_server rclcpp robotname_msgs)

install(TARGETS
  ball_available_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(get_intake_color_server src/action_server/get_intake_color_server.cpp)
ament_target_dependencies(get_intake_color_server ${TARGET_DEPENDS})

add_executable(get_intake_proximity_array_server src/action_server/get_intake_proximity_array_server.cpp)
ament_target_dependencies(get_intake_proximity_array_server ${TARGET_DEPENDS})

install(TARGETS
get_intake_proximity_array_server
 DESTINATION lib/${PROJECT_NAME})


add_executable(get_ball_grabbed_server src/action_server/get_ball_grabbed_server.cpp)
ament_target_dependencies(get_ball_grabbed_server ${TARGET_DEPENDS})

install(TARGETS
get_ball_grabbed_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(flush_intake_server src/action_server/flush_intake_server.cpp)
ament_target_dependencies(flush_intake_server ${TARGET_DEPENDS})

install(TARGETS
flush_intake_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(follow_ball_server src/action_server/follow_ball_server.cpp)
ament_target_dependencies(follow_ball_server ${TARGET_DEPENDS})

install(TARGETS
follow_ball_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(follow_path_server src/server/follow_path_server.cpp)
ament_target_dependencies(follow_path_server ${TARGET_DEPENDS})

install(TARGETS
follow_path_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(get_nearest_ball_server src/server/find_nearest_ball_server.cpp)
ament_target_dependencies(get_nearest_ball_server rclcpp robotname_msgs std_msgs)

install(TARGETS
get_nearest_ball_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(auto src/auto2.cpp ${THIS_PACKAGE_SOURCES})
install(TARGETS
    auto
    DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(auto ${THIS_PACKAGE_DEPS})

install(
  DIRECTORY launch
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
