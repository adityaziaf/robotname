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
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    parameters=[{
          'frame_id':'base_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':False,
          'wait_imu_to_init':True,
          'Odom/Strategy':'1',
          'Vis/CorType':'1',
          'Odom/ResetCountdown':'1',
          'initial_pose' : "0.0 0.0 0.0 0.0 0. 0.0"
          }]

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', '/camera/infra1/image_rect_raw'),
          ('left/camera_info', '/camera/infra1/camera_info'),
          ('right/image_rect', '/camera/infra2/image_rect_raw'),
          ('right/camera_info', '/camera/infra2/camera_info')]
    
    # parameters=[{
    #       'frame_id':'base_link',
    #       'subscribe_depth':True,
    #       'subscribe_odom_info':False,
    #       'approx_sync':True,
    #       'wait_imu_to_init':True,

    #       'Odom/Strategy':'0',
    #       'Vis/CorType':'1',
    #       'Odom/ResetCountdown':'1'
    #       }]

    # remappings=[
    #       ('imu', '/imu/data'),
    #       ('rgb/image', '/camera/color/image_raw'),
    #       ('rgb/camera_info', '/camera/color/camera_info'),
    #       ('depth/image', '/camera/aligned_depth_to_color/image_raw')]
    
    # remappings=[
    #             ("scan_cloud", "/camera/depth/color/points"),
    #             ("imu", "/imu/data")]
    
    return LaunchDescription([

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'),'launch'),'/rs_launch.py']
        ),
        launch_arguments={
            'enable_gyro' : 'true',
            'enable_accel': 'true',
            'unite_imu_method' : '2',
            'enable_sync' : 'true',
            'rgb_camera.profile' : '640,480,60',
            'depth_module.profile' : '640,480,60',
            'align_depth.enable' : 'true',
            'enable_infra1' : 'true',
            'enable_infra2':'true',
            'depth_module.emitter_enabled' : '1',
            'pointcloud.enable':'true',
            'gyro_fps' : '200',
            'accel_fps': '250',
            'config_file': [os.path.join(get_package_share_directory('robotname_perception'),'config/realsense.json')]
        }.items(),
        
        ),
            
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]
            ),

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),
        
        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_node',
        arguments=[
                '0.12', '0', '0.42',  # Translation (x, y, z)
                #'0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w)
                '0','0.35','0',
                'base_link', 'camera_link'  # Parent and child frame IDs
            ],
        output='screen'
        ),

        Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_node',
        arguments=[
                '0.0', '0', '0.0',  # Translation (x, y, z)
                '0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w)
                'map', 'odom'  # Parent and child frame IDs
            ],
        output='screen'
        )
    ])