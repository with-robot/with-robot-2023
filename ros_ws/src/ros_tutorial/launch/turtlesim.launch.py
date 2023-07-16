#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                parameters=[],
                arguments=[],
                output='screen'
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
                output="screen",
            )
        ]
    )
