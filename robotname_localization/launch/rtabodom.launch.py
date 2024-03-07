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
    # parameters=[{
    #       'frame_id':'base_link',
    #       'subscribe_stereo':True,
    #       'subscribe_odom_info':True,
    #       'wait_imu_to_init':True}]

    # remappings=[
    #       ('imu', '/imu/data'),
    #       ('left/image_rect', '/camera/infra1/image_rect_raw'),
    #       ('left/camera_info', '/camera/infra1/camera_info'),
    #       ('right/image_rect', '/camera/infra2/image_rect_raw'),
    #       ('right/camera_info', '/camera/infra2/camera_info')]
    
    parameters=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True,
          
        #   'Reg/Strategy':'1',
        #   'Reg/Force3DoF':'true',
        #   'RGBD/NeighborLinkRefining':'True',
        #   'Grid/RangeMin':'0.2', # ignore laser scan points on the robot itself
        #   'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)

        'Icp/VoxelSize': '0.05',
        'Icp/PointToPlaneRadius': '0.0',
        'Icp/PointToPlaneK': '20.0',
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/PMOutlierRatio': '0.65',
        'Icp/Epsilon': '0.005', 
        'Icp/PointToPlaneMinComplexity': '0', 
        'Odom/ScanKeyFrameThr': '0.7', 
        'OdomF2M/ScanMaxSize': '15000',
        'Optimizer/GravitySigma': '0.3', 
        'RGBD/ProximityPathMaxNeighbors': '1', 
        'Reg/Strategy': '1'
          }]

    # remappings=[
    #       ('imu', '/imu/data'),
    #       ('rgb/image', '/camera/color/image_raw'),
    #       ('rgb/camera_info', '/camera/color/camera_info'),
    #       ('depth/image', '/camera/aligned_depth_to_color/image_raw')]
    
    remappings=[
                ("scan_cloud", "/camera/depth/color/points"),
                ("imu", "/imu/data")]
    
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
            'enable_infra1' : 'false',
            'enable_infra2':'false',
            'depth_module.emitter_enabled' : '1',
            'pointcloud.enable':'true',
            'gyro_fps' : '200',
            'accel_fps': '250'
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
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # Node(
        #     package='rtabmap_slam', executable='rtabmap', output='screen',
        #     parameters=parameters,
        #     remappings=remappings,
        #     arguments=['-d']),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),
        
        # Because of this issue: https://github.com/IntelRealSense/realsense-ros/issues/2564
        # Generate point cloud from not aligned depth
        # Node(
        #     package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        #     parameters=[{'approx_sync':False}],
        #     remappings=[('depth/image',       '/camera/depth/image_rect_raw'),
        #                 ('depth/camera_info', '/camera/depth/camera_info'),
        #                 ('cloud',             '/camera/cloud_from_depth')]),
        
        # # Generate aligned depth to color camera from the point cloud above       
        # Node(
        #     package='rtabmap_util', executable='pointcloud_to_depthimage', output='screen',
        #     parameters=[{ 'decimation':2,
        #                   'fixed_frame_id':'camera_link',
        #                   'fill_holes_size':1}],
        #     remappings=[('camera_info', '/camera/color/camera_info'),
        #                 ('cloud',       '/camera/cloud_from_depth'),
        #                 ('image_raw',   '/camera/realigned_depth_to_color/image_raw')]),
        
        # Compute quaternion of the IMU
        
        # The IMU frame is missing in TF tree, add it:
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ])