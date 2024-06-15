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
    
    lidar_config_kiri = os.path.join(
      get_package_share_directory('robotname_localization'),
      'config',
      'lidarserial.yaml'
      )
    
    lidar_config_kanan = os.path.join(
      get_package_share_directory('robotname_localization'),
      'config',
      'lidarserial2.yaml'
    )

    hardware = Node(
        package='robotname_hardware',
        executable='robot_node',
        name='hardware',
        output='screen'
    )

    lidarkiri = Node(
        package='urg_node',
        namespace="lidarkiri",
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[lidar_config_kiri] 
    )

    lidarkanan = Node(
        package='urg_node',
        namespace="lidarkanan",
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[lidar_config_kanan] 
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

    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
 

    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
        hardware,
        lidarkiri,
        lidarkanan,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])