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
    pkg_share = get_package_share_directory('robotname_bringup')

    slamtool = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_localization'),'launch'),'/online_async_slam.launch.py']
        )
    )

    nav_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_navigation'),'launch'),'/localization.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
            'map': [os.path.join(get_package_share_directory('robotname_navigation'),'maps/mapku3.yaml')],
            'params_file': [os.path.join(get_package_share_directory('robotname_navigation'),'config/nav2_params_sim.yaml')]
        }.items()
    )

    nav_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_navigation'),'launch'),'/navigation.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
            'params_file': [os.path.join(get_package_share_directory('robotname_navigation'),'config/nav2_params_sim.yaml')]
        }.items()
    )
 
    trans = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_node',
        arguments=[
                '0', '0', '0',  # Translation (x, y, z)
                '0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w) or ypr
                'map', 'odom'  # Parent and child frame IDs
            ],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                            description='Flag to enable use_sim_time'),
        trans,
        nav_localization,
        #slamtool
        nav_navigation
        
    ])