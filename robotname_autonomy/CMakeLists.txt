cmake_minimum_required(VERSION 3.8)
project(robotname_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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
    src/plugins/action/get_intake_color.cpp 
    src/plugins/service/set_intake_mechanism.cpp
    src/plugins/action/nav_to_pose.cpp 
    src/plugins/action/nav_through_poses.cpp
    src/plugins/action/find_nearest_ball.cpp
    src/plugins/service/follow_path.cpp
    src/plugins/service/ball_available.cpp
    src/plugins/action/get_intake_proximity_array.cpp
    src/plugins/action/get_ball_grabbed.cpp
    src/plugins/action/flush_intake.cpp
    src/plugins/action/follow_ball.cpp
    src/plugins/action/move_with_lidar_reference.cpp
    src/plugins/service/get_nearest_ball.cpp
    src/plugins/subscriber/grab_nearest_ball.cpp
    src/plugins/subscriber/ball_grabbed.cpp
    src/plugins/subscriber/ball_grabbed_top.cpp
    src/plugins/service/amcl_update.cpp
    src/plugins/service/reset_ball_grabbed.cpp
    src/plugins/action/get_ball_grabbed_top.cpp
    src/plugins/service/rotate.cpp
    src/plugins/subscriber/intake_color.cpp
    src/plugins/subscriber/id_found.cpp
    src/plugins/service/get_current_pose.cpp
    src/plugins/action/wait_button.cpp
    src/plugins/action/rotate_speed.cpp
    src/plugins/action/set_speed.cpp
    src/plugins/service/set_tail_position.cpp
    src/plugins/action/setup_menu.cpp
    src/plugins/service/set_pneumatic.cpp
    )

include_directories(include)

add_executable(set_pneumatic_server src/server/set_pneumatic_server.cpp)
  ament_target_dependencies(set_pneumatic_server rclcpp ${TARGET_DEPENDS})
  
install(TARGETS
    set_pneumatic_server
    DESTINATION lib/${PROJECT_NAME})

add_executable(setup_menu_server src/action_server/setup_menu_server.cpp)
  ament_target_dependencies(setup_menu_server rclcpp ${TARGET_DEPENDS})
  
install(TARGETS
    setup_menu_server
    DESTINATION lib/${PROJECT_NAME})

add_executable(set_speed_server src/action_server/set_speed_server.cpp)
ament_target_dependencies(set_speed_server rclcpp ${TARGET_DEPENDS})

install(TARGETS 
  set_speed_server
  DESTINATION lib/${PROJECT_NAME}) 

add_executable(rotate_speed_server src/action_server/rotate_speed_server.cpp)
  ament_target_dependencies(rotate_speed_server rclcpp ${TARGET_DEPENDS})
  
install(TARGETS
    rotate_speed_server
    DESTINATION lib/${PROJECT_NAME}) 

add_executable(move_with_lidar_reference_server src/action_server/move_with_lidar_reference.cpp)
  ament_target_dependencies(move_with_lidar_reference_server rclcpp ${TARGET_DEPENDS})
  
install(TARGETS
    move_with_lidar_reference_server
    DESTINATION lib/${PROJECT_NAME}) 

#install server
add_executable(set_tail_position_server src/server/set_tail_position_server.cpp)
ament_target_dependencies(set_tail_position_server rclcpp ${TARGET_DEPENDS})

install(TARGETS
set_tail_position_server
  DESTINATION lib/${PROJECT_NAME}) 


add_executable(wait_button_server src/action_server/wait_button_server.cpp)
ament_target_dependencies(wait_button_server rclcpp ${TARGET_DEPENDS})

install(TARGETS
wait_button_server
  DESTINATION lib/${PROJECT_NAME}) 

add_executable(rotate_server src/server/rotate_server.cpp)
ament_target_dependencies(rotate_server rclcpp ${TARGET_DEPENDS})

install(TARGETS
rotate_server
  DESTINATION lib/${PROJECT_NAME})

add_executable(get_current_pose_server src/server/get_current_pose_server.cpp)
ament_target_dependencies(get_current_pose_server rclcpp ${TARGET_DEPENDS})

install(TARGETS
get_current_pose_server
  DESTINATION lib/${PROJECT_NAME})


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

add_executable(ball_grabbed_server src/server/ball_grabbed_server.cpp)
ament_target_dependencies(ball_grabbed_server rclcpp robotname_msgs std_msgs)

install(TARGETS
ball_grabbed_server
DESTINATION lib/${PROJECT_NAME})  

add_executable(get_intake_distance_server src/server/get_intake_distance_server.cpp)
ament_target_dependencies(get_intake_distance_server rclcpp robotname_msgs std_srvs)

install(TARGETS
get_intake_distance_server
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

#install python node
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/numpad_node.py
  scripts/numpad_client_node.py
  DESTINATION lib/${PROJECT_NAME}
)

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
