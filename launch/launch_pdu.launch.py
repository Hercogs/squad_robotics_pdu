#! /usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                            TimerAction, LogInfo)

from launch.event_handlers import (
    OnProcessStart
)

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

    # robot_pcb_bridge node
    robot_pcb_bridge_node = Node(
        package="robot_pcb_bridge",
        executable="robot_pcb_bridge",
        namespace=LaunchConfiguration("vikings_bot_name"),
        output="both",
        arguments=["--ros-args", "--log-level", "error"]
    )

    # upload_config_node
    upload_config_node = Node(
        package="squad_robotics_pdu",
        executable="upload_config.py",
        namespace=LaunchConfiguration("vikings_bot_name"),
        output="both",
        arguments=["--ros-args", "--log-level", "error"]
    )

    upload_config_node_delayed = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_pcb_bridge_node,
            on_start=[
                LogInfo(msg="robot_pcb_bridge_node started, now uploading config..."),
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg="timer end"),
                        upload_config_node
                    ]
                )
            ]
        )
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

    input_state_publisher_node = Node(
        package='squad_robotics_pdu',
        executable='input_state_publisher.py',
        namespace=LaunchConfiguration("vikings_bot_name"),
        name='input_state_publisher',
        output='screen'
    )

    publish_input_state_delayed = RegisterEventHandler(
        OnProcessStart(
            target_action=upload_config_node,
            on_start=[
                LogInfo(msg="upload_config_node started, now publishing input status"),
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg="timer end"),
                        input_state_publisher_node
                    ]
                )
            ]
        )
    )



    return LaunchDescription([
        # Arguments
        vikings_bot_name_arg,

        # Nodes
        robot_pcb_bridge_node,
        upload_config_node_delayed,
        publish_input_state_delayed,
        battery_monitor_service_node
    ])

