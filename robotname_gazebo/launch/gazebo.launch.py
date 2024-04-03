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
    pkg_share = get_package_share_directory('robotname_gazebo')
    
    default_model_path = os.path.join(get_package_share_directory('robotname_description'), 'urdf/robotname.urdf')
    world_path=os.path.join(get_package_share_directory('robotname_gazebo'), 'worlds/my_world.sdf'),

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path]
    )
 
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'bot', 
                   '-topic', 'robot_description', 
                   '-x', LaunchConfiguration('x'), 
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-R', LaunchConfiguration('R'),
                   '-P', LaunchConfiguration('P'),
                   '-Y', LaunchConfiguration('Y')
                  ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='x', default_value='0', #5.5
                                            description='x spawn coordinate'),
        DeclareLaunchArgument(name='y', default_value='0', #5.7
                                            description='y spawn coordinate'),
        DeclareLaunchArgument(name='z', default_value='0.15', #0.15
                                            description='z spawn coordinate'),
        DeclareLaunchArgument(name='R', default_value='0',
                                            description='R spawn coordinate'),
        DeclareLaunchArgument(name='P', default_value='0',
                                            description='P spawn coordinate'),
        DeclareLaunchArgument(name='Y', default_value='0', #-1.57
                                            description='Y spawn coordinate'),
        
        ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',world_path], output='screen'),
        #ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen')
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
    ])