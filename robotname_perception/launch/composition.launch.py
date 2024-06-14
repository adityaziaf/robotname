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
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='my_container',
            namespace='camera',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robotname_perception',
                    plugin='robotname_perception::visualizeComponent',
                    name='visualize',
                    extra_arguments=[{'use_intra_process_comms': False}]
                )
            ],
            output='screen',
    )

    realsense= IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_perception'),'launch'),'/rs_dual.launch.py']
        )
    )

    tracker = Node(
        package='robotname_perception',
        executable='tracking_component.py',
        output='screen'
    )

    detector_node_cmd = Node(
        package="robotname_perception_py",
        executable="depth_camera_node",
        name="yolov5_node",
    )

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
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='intake',
        name='v4l2_camera_exe',
        output='screen',
        parameters=[intakecam_config]
    )

    topcam= Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='intake2',
        name='v4l2_camera_exe',
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

    tracker = Node(
        package='robotname_perception',
        executable='tracking_component.py',
        output='screen'
    )

    silo = Node(
        package='robotname_perception_py',
        executable='silo_camera_node',
        output='screen'
    )

    return LaunchDescription([
            realsense,
            container,
            intakecam,
            intakeprocessing,
            topcam,
            tracker,
            topprocessing,
            detector_node_cmd
        ])