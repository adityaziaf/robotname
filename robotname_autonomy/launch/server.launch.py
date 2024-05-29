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
    
    intake_distance = Node(
        package='robotname_autonomy',
        executable='get_intake_distance_server',
        name='get_intake_distance',
        output='screen'
    )

    intake_proximity_array = Node(
        package='robotname_autonomy',
        executable='get_intake_proximity_array_server',
        name='get_intake_proximity_array',
        output='screen'
    )

    set_intake_mechanism = Node(
        package='robotname_autonomy',
        executable='set_intake_mechanism_server',
        name='set_intake_mechanism',
        output='screen'
    )

    ball_grabbed = Node(
        package='robotname_autonomy',
        executable='get_ball_grabbed_server',
        name='get_ball_grabbeds',
        output='screen'
    )

    ball_grabbed_top = Node(
        package='robotname_autonomy',
        executable='get_ball_grabbed_top_server',
        name='get_ball_grabbeds_top',
        output='screen'
    )

    flush = Node(
        package='robotname_autonomy',
        executable='flush_intake_server',
        name='flush_intake',
        output='screen'
    )

    follow_ball = Node(
        package='robotname_autonomy',
        executable='follow_ball_server',
        name='follow_ball',
        output='screen'
    )

    follow_path = Node(
        package='robotname_autonomy',
        executable='follow_path_server',
        name='follow_path',
        output='screen'
    )
    get_nearest_ball = Node(
        package='robotname_autonomy',
        executable='get_nearest_ball_server',
        name='get_nearest_ball',
        output='screen'
    )

    resetball = Node(
        package='robotname_autonomy',
        executable='reset_ball_grabbed_server',
        name='reset_ball_grabbed',
        output='screen'
    )

    get_current_pose_server = Node(
        package='robotname_autonomy',
        executable='get_current_pose_server',
        name='get_current_pose',
        output='screen'
    )

    rotate_server = Node(
        package='robotname_autonomy',
        executable='rotate_server',
        name='rotate',
        output='screen'
    )

    wait_button_server = Node(
        package='robotname_autonomy',
        executable='wait_button_server',
        name='wait_button',
        output='screen'
    )

    rotate_speed = Node(
        package='robotname_autonomy',
        executable='rotate_speed_server',
        name='rotate_speed',
        output='screen'
    )

    move_with_lidar_reference = Node(
        package='robotname_autonomy',
        executable='move_with_lidar_reference_server',
        name='move_with_lidar_reference',
        output='screen'
    )

    set_tail_position = Node(
        package='robotname_autonomy',
        executable='set_tail_position_server',
        name='set_tail_position',
        output='screen'
    )

    set_speed = Node(
        package='robotname_autonomy',
        executable='set_speed_server',
        name='set_speed_server',
        output='screen'
    )

    set_pneumatic = Node(
        package='robotname_autonomy',
        executable='set_pneumatic_server',
        name='set_pneumatic_server',
        output='screen'
    )



    return LaunchDescription([
        #get_intake_color,
        #find_nearest_ball,
        #ball_available,
        #intake_distance,
        intake_proximity_array,
        set_intake_mechanism,
        ball_grabbed,
        #flush,
        follow_ball,
        follow_path,
        #get_nearest_ball,
        #resetball,
        ball_grabbed_top,
        get_current_pose_server,
        rotate_server,
        wait_button_server,
        rotate_speed,
        move_with_lidar_reference,
        set_tail_position,
        set_speed,
        set_pneumatic
    ])