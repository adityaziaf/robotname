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
find_package(rclcpp REQUIRED)
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
    nav2_msgs sensor_msgs behaviortree_cpp
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
    )

set(THIS_PACKAGE_SOURCES
    src/plugins/get_intake_color.cpp 
    src/plugins/set_intake_mechanism.cpp
    src/plugins/nav_to_pose.cpp 
    src/plugins/nav_through_poses.cpp
    )
include_directories(include)
add_executable(autonomy_node_cpp src/autonomy_node.cpp ${AUTONOMY_SOURCES})
install(TARGETS
    autonomy_node_cpp
    DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(autonomy_node_cpp ${TARGET_DEPENDS})
target_link_libraries(autonomy_node_cpp ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})


add_executable(auto src/auto2.cpp ${THIS_PACKAGE_SOURCES})
install(TARGETS
    auto
    DESTINATION lib/${PROJECT_NAME}
)
ament_target_dependencies(auto ${THIS_PACKAGE_DEPS})

# add_executable(follow_node src/follow_node.cpp)
# install(TARGETS
#     follow_node
#     DESTINATION lib/${PROJECT_NAME}
# )

# ament_target_dependencies(follow_node
#   rclcpp
#   robotname_msgs
#   geometry_msgs
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
