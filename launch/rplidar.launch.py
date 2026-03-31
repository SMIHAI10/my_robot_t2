import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
<<<<<<< HEAD
                'format': 'RGB888',
                'image_size': [800,600],
                'time_per_frame': [1, 30],
                'camera_frame_id': 'camera_link_optical'
                }]
=======
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'serial_baudrate': 256000
            }]
>>>>>>> c2ab477 (Camera working with flipped/compressed)
        )
    ])
