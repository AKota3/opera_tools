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
    
    declare_command_interface_name = DeclareLaunchArgument(
        "command_interface_name", default_value="position")


    crawlerdump_swing_control_node = Node(
        package='opera_tools',
        executable='swing_control',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
        {'command_interface_name': LaunchConfiguration('command_interface_name')}
        ])
    
    # ベッセル角を指定した角度へ
    crawlerdump_vessel_control_node = Node(
        package='opera_tools',
        executable='vessel_control',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
        {'command_interface_name': LaunchConfiguration('command_interface_name')}
        ])

    # ベッセル角を指定した角度へ動かして、3秒経過後0度に戻す
    # crawlerdump_vessel_updown_control_node = Node(
    #     package='opera_tools',
    #     executable='vessel_control_updown',
    #     namespace=LaunchConfiguration('robot_name'),
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ])

    # crawlerdump_vessel_updown_control_node_2 = Node(
    #     package='opera_tools',
    #     executable='vessel_control_updown',
    #     namespace='mst110cr_2',
    #     parameters=[
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ])


    # Build the launch description
    ld = LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_name_arg,
        declare_command_interface_name,


        crawlerdump_swing_control_node,
        crawlerdump_vessel_control_node,
        # crawlerdump_vessel_updown_control_node,
        # crawlerdump_vessel_updown_control_node_2
    ])

    return ld