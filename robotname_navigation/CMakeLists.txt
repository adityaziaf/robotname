cmake_minimum_required(VERSION 3.8)
project(robotname_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp)
find_package(robotname_msgs)
find_package(nav_msgs)
find_package(geometry_msgs)
find_package(rclcpp_action)
find_package(rclcpp_components)

add_library(straightlineplanner SHARED
  src/straightLinePlanner.cpp)
target_include_directories(straightlineplanner PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(straightlineplanner
  PRIVATE "ROBOTNAME_NAVIGATION_CPP_BUILDING_DLL")
ament_target_dependencies(straightlineplanner
  "robotname_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  )
rclcpp_components_register_node(straightlineplanner PLUGIN "robotname_navigation::NavigateActionServer" EXECUTABLE navigate_action_server)
install(TARGETS
  straightlineplanner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(navigate_action_client SHARED
  src/straightlineclient.cpp)
target_include_directories(navigate_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(navigate_action_client
  PRIVATE "ROBOTNAME_NAVIGATION_CPP_BUILDING_DLL")
ament_target_dependencies(navigate_action_client
  "robotname_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(navigate_action_client PLUGIN "robotname_navigation::NavigateActionClient" EXECUTABLE action_client)
install(TARGETS
  navigate_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

include_directories(include)

add_executable(planner src/simpleplanner.cpp src/pid.cpp)
ament_target_dependencies(planner rclcpp std_msgs geometry_msgs nav_msgs)

install(TARGETS
  planner
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY maps launch config
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
