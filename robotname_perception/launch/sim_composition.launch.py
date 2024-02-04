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

remappings = [('/color/image_raw', '/depth_camera/image_raw'),
                ('/color/camera_info', '/depth_camera/camera_info'),
                ('/aligned_depth_to_color/image_raw', '/depth_camera/depth/image_raw')]

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robotname_perception',
                    plugin='robotname_perception::yoloDetectorComponent',
                    name='yolodetector',
                    remappings=remappings,
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
    
    # trans = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_node',
    #     arguments=[
    #             '0', '0', '0',  # Translation (x, y, z)
    #             '0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w)
    #             'map', 'camera_link'  # Parent and child frame IDs
    #         ],
    #     output='screen'
    # )

    return LaunchDescription([
            container,
            tracker
    ])