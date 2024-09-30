#! /usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch.substitutions import(
    LaunchConfiguration, EnvironmentVariable
)


def generate_launch_description():

    ### INPUT ###
    vikings_bot_name_arg = DeclareLaunchArgument("vikings_bot_name",
                default_value=EnvironmentVariable("ROBOT_NAME"),
                description="Namespace of robot - [vikings_bot_1 or vikings_bot_2]"
    )

   

    battery_monitor_service_node = Node(
        package='squad_robotics_pdu',
        executable='battery_service.py',
        namespace=LaunchConfiguration("vikings_bot_name"),
        name='battery_monitor',
        output='screen',
        parameters=[{
            'robot_name':LaunchConfiguration("vikings_bot_name"),
        }]
    )



    return LaunchDescription([
        # Arguments
        vikings_bot_name_arg,
        # Nodes
        battery_monitor_service_node
    ])

