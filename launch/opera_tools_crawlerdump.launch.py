from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='mst110cr')


    crawlerdump_swing_control_node = Node(
        package='opera_tools',
        executable='swing_control',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_vessel_control_node = Node(
        package='opera_tools',
        executable='vessel_control',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])


    # Build the launch description
    ld = LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_name_arg,

        crawlerdump_swing_control_node,
        crawlerdump_vessel_control_node
    ])

    return ld