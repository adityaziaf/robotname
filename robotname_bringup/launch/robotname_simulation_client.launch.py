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
    
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/navigation.rviz')

    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_localization'),'launch'),'/ekf.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'
            )
        }.items()
    )
    
    nav_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_navigation'),'launch'),'/localization.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
            'map': [os.path.join(get_package_share_directory('robotname_navigation'),'maps/mapkuh.yaml')]
        }.items()
    )

    nav_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_navigation'),'launch'),'/navigation.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time')
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
 
    trans = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_node',
        arguments=[
                '0', '0', '0',  # Translation (x, y, z)
                '0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w)
                'map', 'odom'  # Parent and child frame IDs
            ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        trans,
        nav_localization,
        rviz_node
    ])