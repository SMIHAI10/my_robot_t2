import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():



    return LaunchDescription([

        Node(
            package='camera_ros',
            executable='camera_node',
            output='screen',
            parameters=[{
                'format': 'RGB888',
                'image_size': [320, 240],
                'time_per_frame': [1, 6],
                'camera_frame_id': 'camera_link_optical'
                }]
    )
    ])
