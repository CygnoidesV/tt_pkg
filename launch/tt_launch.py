import launch
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tt_pkg',
            executable='detection1.py',
            output='screen',
        ),

        Node(
            package='tt_pkg',
            executable='detection2.py',
            output='screen',
        ),

        Node(
            package='tt_pkg',
            executable='display.py',
            output='screen',
        ),

        Node(
            package='tt_pkg',
            executable='communication',
            output='screen',
        ),

        Node(
            package='tt_pkg',
            executable='tracking.py',
            output='screen',
        ),

        # Node(
        #     package='tt_pkg',
        #     executable='policy.py',
        #     output='screen',
        # ),

        # Node(
        #     package='rosbridge_server',
        #     executable='rosbridge_websocket.py',
        #     parameters=[{
        #         'port':'9090'
        #         }],
        #     output='screen',
        # )
    ])
