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
    
    top_config = os.path.join(
      get_package_share_directory('robotname_perception'),
      'config',
      'top_params.yaml'
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

    topcam= Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        namespace='intake2',
        name='usb_cam_node_exe',
        output='screen',
        parameters=[top_config],
        
    )


    intakeprocessing = Node(
        package='robotname_perception',
        executable='intakeprocess.py',
        name='intakeprocess',
        namespace='intake',
        output='screen',
        parameters=[intakeprocess_config],
        
        
    )
    
    topprocessing = Node(
        package='robotname_perception',
        executable='topprocess.py',
        name='intakeprocess',
        output='screen',
        namespace='intake2'
        #parameters=[intakeprocess_config]
        
    )

    container = ComposableNodeContainer(
            name='my_container',
            namespace='omni',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # ComposableNode(
                #     package='robotname_perception',
                #     plugin='robotname_perception::yoloOnnxComponent',
                #     name='yolodetector',
                #     namespace='omni',
                #     extra_arguments=[{'use_intra_process_comms': True}]
                # ),
                ComposableNode(
                    package='robotname_perception',
                    plugin='robotname_perception::visualizeComponent',
                    name='visualize',
                    extra_arguments=[{'use_intra_process_comms': False}]
                )
            ],
            output='screen',
    )

    tracker = Node(
        package='robotname_perception',
        executable='tracking_component.py',
        output='screen'
    )

    detector_node_cmd = Node(
        package="robotname_perception_py",
        executable="yolov5_node",
        name="yolov5_node",
        # parameters=[{
        #     "model": "home/itsrobocon3/robot_ws/omni.pt",
        #     # "device": device,
        #     # "enable": enable,
        #     # "threshold": threshold,
        #     # "image_reliability": image_reliability,
        # }],
        # remappings=[("image_raw", input_image_topic)]
    )

    return LaunchDescription([
        #intakecam,
        #intakeprocessing,
        #omnicam,
        #topcam,
        #topprocessing,
        detector_node_cmd,
        container,
        tracker
    ])