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
    
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/localization.rviz')

    default_model_path = os.path.join(
        get_package_share_directory('robotname_description'), 
        'urdf/realomni.urdf'
        )
    lidarconfig = os.path.join(
      get_package_share_directory('robotname_localization'),
      'config',
      'lidarserial.yaml'
      )
    
    hardware = Node(
        package='robotname_hardware',
        executable='robot_node',
        name='hardware',
        output='screen'
    )
    
    lidar = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[lidarconfig]
        
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
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

    nav_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robotname_navigation'),'launch'),'/localization.launch.py']
        ),
        launch_arguments={
            'use_sim_time' : LaunchConfiguration('use_sim_time'),
            'params_file' : [os.path.join(get_package_share_directory('robotname_navigation'),'config/nav2_params.yaml')],
            'map': [os.path.join(get_package_share_directory('robotname_navigation'),'maps/mapku3.yaml')]
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
        #lidar,
        robot_state_publisher_node,
        joint_state_publisher_node,
        #perception,
        rviz_node,
        odom_to_map,
        nav_localization,
        nav_navigation
    ])