import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


from launch_ros.actions import Node


realsense_node_params = [{'name': 'serial_no',              'default': "''", 'description': 'choose device by serial number'},
                         {'name': 'usb_port_id',            'default': "''", 'description': 'choose device by usb port id'},
                         {'name': 'device_type',            'default': "''", 'description': 'choose device by type'},
                         {'name': 'log_level',              'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                         {'name': 'rgb_camera.profile',     'default': '640,480,30', 'description': 'color image width'},
                         {'name': 'depth_module.profile',   'default': '640,480,30', 'description': 'depth image width'},
                         {'name': 'enable_color',           'default': 'true', 'description': 'enable color stream'},
                         {'name': 'enable_depth',           'default': 'true', 'description': 'enable depth stream'},
                         {'name': 'enable_infra',           'default': 'false', 'description': 'enable infra stream'},
                         {'name': 'enable_infra1',          'default': 'false', 'description': 'enable infra1 stream'},
                         {'name': 'enable_infra2',          'default': 'false', 'description': 'enable infra2 stream'},
                         {'name': 'enable_gyro',            'default': 'false', 'description': "enable gyro stream"},
                         {'name': 'enable_accel',           'default': 'false', 'description': "enable accel stream"},
                         {'name': 'unite_imu_method',       'default': "1", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                         {'name': 'intra_process_comms',    'default': 'true', 'description': "enable intra-process communication"},
                         {'name': 'enable_sync',            'default': 'true', 'description': "'enable sync mode'"},
                         {'name': 'pointcloud.enable',      'default': 'false', 'description': ''},
                         {'name': 'enable_rgbd',            'default': 'false', 'description': "'enable rgbd topic'"},
                         {'name': 'align_depth.enable',     'default': 'true', 'description': "'enable align depth filter'"},
                         {'name': 'publish_tf',             'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                         {'name': 'tf_publish_rate',        'default': '1.0', 'description': '[double] rate in HZ for publishing dynamic TF'},
                        ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    plugin='realsense2_camera::RealSenseNodeFactory',
                    name='camera',
                    parameters=[set_configurable_parameters(realsense_node_params)],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='robotname_perception',
                    plugin='robotname_perception::yoloDetectorComponent',
                    name='yolodetector',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='robotname_perception',
                    plugin='robotname_perception::transformComponent',
                    name='transform',
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='robotname_perception',
                    plugin='robotname_perception::visualizeComponent',
                    name='visualize',
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='screen',
    )

    tracker = Node(
        package='robotname_perception',
        executable='tracking_component.py',
        output='screen'
    )
    
    trans = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_node',
        arguments=[
                '0', '0', '0',  # Translation (x, y, z)
                '0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w)
                'map', 'camera_link'  # Parent and child frame IDs
            ],
        output='screen'
    )

    return LaunchDescription(declare_configurable_parameters(realsense_node_params) + [
            container,
            trans,
            tracker
    ])