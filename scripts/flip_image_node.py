#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class FlipImageNode(Node):
    def __init__(self):
        super().__init__('flip_image_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            'input_image',
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            Image,
            'output_image',
            10
        )

        self.get_logger().info('Flip image node started')

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            flipped = cv2.rotate(frame, cv2.ROTATE_180)

            out = self.bridge.cv2_to_imgmsg(flipped, encoding=msg.encoding)
            out.header = msg.header
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().error(f'Image processing failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = FlipImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()