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
    pkg_share = get_package_share_directory('robotname_autonomy')
    
    get_intake_color = Node(
        package='robotname_autonomy',
        executable='get_intake_color_server',
        name='get_intake_color_server',
        output='screen'
    )
    
    find_nearest_ball = Node(
        package='robotname_autonomy',
        executable='find_nearest_ball_server',
        name='find_nearest_ball_server',
        output='screen'
    )

    ball_available = Node(
        package='robotname_autonomy',
        executable='ball_available_server',
        name='ball_available_server',
        output='screen'
    )

    set_speed = Node(
        package='robotname_autonomy',
        executable='set_speed_server',
        name='set_speed_server',
        output='screen'
    )
    
    return LaunchDescription([
        get_intake_color,
        find_nearest_ball,
        ball_available,
        set_speed
    ])