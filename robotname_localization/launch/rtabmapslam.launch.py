# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_sync:=true
#
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_scan_cloud':True,
          'approx_sync':True,
          'subscribe_depth':False,
          'subscribe_rgb':False,
          'use_sim_time': True,

          'Reg/Force3DoF': 'true',
          'Icp/Force4DoF': 'true',
          'Reg/Strategy': '1',
          'Optimizer/Slam2D': 'true',
          'RGBD/OptimizeFromGraphEnd' : 'false',
          'Rtabmap/DetectionRate' : '5'
          }]

    # remappings=[
    #       ('scan_cloud', '/depth_camera/points')]
    remappings=[
          ('scan_cloud', '/camera/depth/color/points')]

    return LaunchDescription([

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']),
    ])