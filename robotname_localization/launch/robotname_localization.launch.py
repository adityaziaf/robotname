
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robotname_localization')
    slam_params_file = os.path.join(pkg_share,'config','online_async_slam.yaml')
    ekf_params_file = os.path.join(pkg_share,'config','ekf.yaml')
    
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share,'launch'),'/ekf.launch.py']
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'ekf_params_file' : [os.path.join(ekf_params_file)]
        }.items()
    )
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share,'launch'),'/online_async_slam.launch.py']
        ),
        launch_arguments={
            'slams_params_file': [os.path.join(slam_params_file)],
            'use_sim_time' : LaunchConfiguration('use_sim_time'
            )
        }.items()
    )

    trans = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_node',
        arguments=[
                '5.5', '5.7', '0.15',  # Translation (x, y, z)
                '0', '0', '-0.707', '0.707',  # Rotation (Quaternion: x, y, z, w)
                'map', 'odom'  # Parent and child frame IDs
            ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        ekf,
        trans
    ])