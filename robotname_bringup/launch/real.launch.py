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

    hardware = Node(
        package='robotname_hardware',
        executable='robot_node',
        name='hardware',
        output='screen'
    )
    
    
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_perception'),'launch'),'/composition.launch.py']
        )
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
 
    odom_to_map = Node(
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
    
    odom_offset = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_node',
        arguments=[
                '0.25', '0', '0',  # Translation (x, y, z)
                '0', '0', '0', '1',  # Rotation (Quaternion: x, y, z, w)
                'base_odom', 'base_link'  # Parent and child frame IDs
            ],
        output='screen'
    )

    nav_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_navigation'),'launch'),'/localization.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
            'map': [os.path.join(get_package_share_directory('robotname_navigation'),'maps/mapku2.yaml')]
        }.items()
    )
    
    nav_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_navigation'),'launch'),'/navigation.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
            'params_file' : [os.path.join(get_package_share_directory('robotname_navigation'),'config/nav2_params.yaml')]
        }.items()
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        hardware,
        #perception,
        rviz_node,
        odom_offset,
        odom_to_map,
        nav_localization,
        nav_navigation
    ])