cmake_minimum_required(VERSION 3.8)
project(robotname_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robotname_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(rclcpp_components REQUIRED)

find_package(OpenCV REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED)

include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    /usr/include/openvino/runtime/include
    /usr/include/openvino/runtime/include/ie
    /usr/include/openvino/runtime/include/ngraph
)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library (detector SHARED src/yolo_openvino.cpp)

target_include_directories(detector
      PUBLIC include/robotname_perception
      PUBLIC ${OpenCV_INCLUDE_DIRS}
      PUBLIC /usr/include/openvino/runtime/include
      PUBLIC /usr/include/openvino/runtime/include/ie
      PUBLIC /usr/include/openvino/runtime/include/ngraph
)

target_link_libraries(detector
      PUBLIC ${OpenCV_LIBS}
      PUBLIC /usr/lib/libopenvino.so
)

install(TARGETS detector
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_library(yolo_detector_component 
  SHARED src/yolo_detector_component.cpp)
target_compile_definitions(yolo_detector_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

target_link_libraries(yolo_detector_component detector)
ament_target_dependencies(yolo_detector_component
  rclcpp 
  rclcpp_components
  std_msgs 
  sensor_msgs 
  OpenCV 
  cv_bridge 
  message_filters
  robotname_msgs
  image_geometry
)
rclcpp_components_register_nodes(yolo_detector_component "robotname_perception::yoloDetectorComponent")
set(node_plugins "${node_plugins}robotname_perception::yoloDetectorComponent\n")

install(TARGETS
  yolo_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(transform_component 
  SHARED src/transform_component.cpp)
target_compile_definitions(transform_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(transform_component
  rclcpp 
  rclcpp_components
  robotname_msgs
  tf2
  tf2_ros
  geometry_msgs
)
rclcpp_components_register_nodes(transform_component "robotname_perception::transformComponent")
set(node_plugins "${node_plugins}robotname_perception::transformComponent\n")

install(TARGETS
  transform_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(visualize_component 
  SHARED src/visualize_component.cpp)
target_compile_definitions(visualize_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(visualize_component
  rclcpp 
  rclcpp_components
  visualization_msgs
  robotname_msgs
)
rclcpp_components_register_nodes(visualize_component "robotname_perception::visualizeComponent")
set(node_plugins "${node_plugins}robotname_perception::visualizeComponent\n")

install(TARGETS
  visualize_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/tracking_component.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
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
