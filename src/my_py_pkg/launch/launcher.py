#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_py_pkg',
            executable='number_publisher',
            name='number_publisher_node',
            output='screen'
        ),
        Node(
            package='my_py_pkg',
            executable='number_counter',
            name='number_counter_node',
            output='screen'
        ),
        Node(
            package='my_py_pkg',
            executable='robot_new_station',
            name='robot_news_node',
            output='screen'
        )
    ])