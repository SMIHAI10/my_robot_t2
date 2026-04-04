#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MotionCommander(Node):
    def __init__(self):
        super().__init__('motion_commander')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.15)    # m/s
        self.declare_parameter('angular_speed', 0.5)    # rad/s
        self.declare_parameter('publish_rate', 20.0)    # Hz
        self.declare_parameter('settle_time', 0.2)      # s
        self.declare_parameter('stop_burst_count', 10)  # publish stop several times

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.stop_burst_count = int(self.get_parameter('stop_burst_count').value)

        if self.linear_speed <= 0.0:
            raise ValueError('linear_speed must be > 0')
        if self.angular_speed <= 0.0:
            raise ValueError('angular_speed must be > 0')
        if self.publish_rate <= 0.0:
            raise ValueError('publish_rate must be > 0')

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.get_logger().info(f'Publishing motion commands to: {cmd_vel_topic}')
        self.get_logger().info(f'linear_speed={self.linear_speed:.3f} m/s')
        self.get_logger().info(f'angular_speed={self.angular_speed:.3f} rad/s')
        self.get_logger().info(f'publish_rate={self.publish_rate:.1f} Hz')

    def publish_twist(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.get_logger().info('Stopping robot...')
        for _ in range(self.stop_burst_count):
            self.publish_twist(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def execute_for_duration(self, linear_x: float, angular_z: float, duration_sec: float):
        if duration_sec <= 0.0:
            self.get_logger().warn('Requested motion duration <= 0. Nothing to do.')
            self.stop_robot()
            return

        period = 1.0 / self.publish_rate
        end_time = time.monotonic() + duration_sec

        self.get_logger().info(
            f'Executing motion: linear_x={linear_x:.3f} m/s, '
            f'angular_z={angular_z:.3f} rad/s, '
            f'duration={duration_sec:.3f} s'
        )

        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_twist(linear_x, angular_z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

        self.stop_robot()
        time.sleep(self.settle_time)

    def forward(self, distance_m: float):
        if distance_m == 0.0:
            self.get_logger().warn('Forward distance is 0. Nothing to do.')
            self.stop_robot()
            return

        direction = 1.0 if distance_m > 0.0 else -1.0
        speed = direction * self.linear_speed
        duration = abs(distance_m) / self.linear_speed

        self.get_logger().info(f'Command: forward {distance_m:.3f} m')
        self.execute_for_duration(speed, 0.0, duration)

    def rotate_deg(self, angle_deg: float):
        if angle_deg == 0.0:
            self.get_logger().warn('Rotation angle is 0. Nothing to do.')
            self.stop_robot()
            return

        angle_rad = math.radians(angle_deg)
        direction = 1.0 if angle_rad > 0.0 else -1.0
        angular_speed = direction * self.angular_speed
        duration = abs(angle_rad) / self.angular_speed

        self.get_logger().info(f'Command: rotate {angle_deg:.3f} deg ({angle_rad:.3f} rad)')
        self.execute_for_duration(0.0, angular_speed, duration)


def print_usage():
    print(
        '\nUsage:\n'
        '  ros2 run my_robot_t2 motion_commander.py forward 1.0\n'
        '  ros2 run my_robot_t2 motion_commander.py rotate 90\n'
        '  ros2 run my_robot_t2 motion_commander.py rotate 180\n'
        '  ros2 run my_robot_t2 motion_commander.py rotate -90\n'
        '\nOptional ROS parameters:\n'
        '  --ros-args -p linear_speed:=0.12 -p angular_speed:=0.4\n'
    )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MotionCommander()

        argv = sys.argv
        non_ros_args = []
        skip_next = False

        # Strip ROS arguments roughly enough for simple CLI use
        for i, arg in enumerate(argv[1:], start=1):
            if skip_next:
                skip_next = False
                continue

            if arg == '--ros-args':
                break

            non_ros_args.append(arg)

        if len(non_ros_args) < 2:
            print_usage()
            node.destroy_node()
            rclpy.shutdown()
            return

        command = non_ros_args[0].lower()
        value_str = non_ros_args[1]

        try:
            value = float(value_str)
        except ValueError:
            print(f'Invalid numeric value: {value_str}')
            print_usage()
            node.destroy_node()
            rclpy.shutdown()
            return

        time.sleep(0.5)  # allow publisher to come up

        if command == 'forward':
            node.forward(value)
        elif command == 'rotate':
            node.rotate_deg(value)
        else:
            print(f'Unknown command: {command}')
            print_usage()

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Error: {exc}')
    finally:
        try:
            node.stop_robot()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()