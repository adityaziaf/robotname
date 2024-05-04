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

def generate_launch_description():
   
    intakecam_config = os.path.join(
      get_package_share_directory('robotname_perception'),
      'config',
      'intakecam_params.yaml'
      )
    
    omni_config = os.path.join(
      get_package_share_directory('robotname_perception'),
      'config',
      'omni_params.yaml'
      )
    
    intakeprocess_config = os.path.join(
      get_package_share_directory('robotname_perception'),
      'config',
      'intakeprocess_params.yaml'
      )
    
    intakecam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='intake',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[intakecam_config]
    )
    
    omnicam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='omni',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[omni_config],
        
    )

    intakeprocessing = Node(
        package='robotname_perception',
        executable='intakeprocess.py',
        name='intakeprocess',
        output='screen',
        parameters=[intakeprocess_config]
        
    )
    
    container = ComposableNodeContainer(
            name='my_container',
            namespace='omni',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robotname_perception',
                    plugin='robotname_perception::yoloOnnxComponent',
                    name='yolodetector',
                    namespace='omni',
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

    return LaunchDescription([
        intakecam,
        intakeprocessing,
        omnicam,
        container,
        tracker
    ])