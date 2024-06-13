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
   
    silo_config = os.path.join(
      get_package_share_directory('robotname_perception'),
      'config',
      'silo_camera.yaml'
      )
    
    silo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='silo',
        name='v4l2_camera_exe',
        output='screen',
        parameters=[ silo_config ]
    )

    return LaunchDescription([
        #intakecam,
        #intakeprocessing,
        #omnicam,
        #topcam,
        #topprocessing,
        silo
    ])