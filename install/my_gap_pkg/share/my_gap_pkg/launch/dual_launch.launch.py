from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gap_node = Node(
        package="my_gap_pkg",
        executable="gap_follower",
        name="gap_follower",
        parameters=[{"debug": False}]          # silenciar prints
    )

    timer_node = Node(
        package="my_gap_pkg",
        executable="lap_timer",
        name="lap_timer",
        parameters=[{
            "heading_tolerance_deg": 45.0,
            "cross_x_threshold": 0.0,
            "min_lap_time": 3.0,
        }]
    )

    return LaunchDescription([gap_node, timer_node])
