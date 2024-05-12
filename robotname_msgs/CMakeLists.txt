cmake_minimum_required(VERSION 3.8)
project(robotname_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "action/NavigateToPose.action"
  "srv/FindNearestBall.srv"
  "srv/BallAvailable.srv"
  "srv/SetSpeed.srv"
  "srv/GetIntakeDistance.srv"
  "action/GetIntakeColor.action"
  "action/GetIntakeProximityArray.action"
  "srv/SetIntakeMechanism.srv"
  "action/FindNearestBall.action"
  "action/GetBallGrabbed.action"
  "action/FlushIntake.action"
  "action/Follow.action"
  "srv/FollowPose.srv"
  DEPENDENCIES geometry_msgs std_msgs nav_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_export_dependencies(rosidl_default_runtime)

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
