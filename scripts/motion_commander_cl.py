#!/usr/bin/env python3

import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MotionCommander(Node):
    def __init__(self):
        super().__init__('motion_commander')

        # Topics
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')

        # General behaviour
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('odom_wait_timeout', 5.0)
        self.declare_parameter('settle_time', 0.2)
        self.declare_parameter('stop_burst_count', 8)

        # Forward motion tuning
        self.declare_parameter('forward_speed', 0.08)              # m/s nominal
        self.declare_parameter('forward_min_speed', 0.04)          # m/s
        self.declare_parameter('forward_tolerance', 0.015)         # m
        self.declare_parameter('heading_kp', 0.5)                  # rad -> rad/s
        self.declare_parameter('max_heading_correction', 0.08)     # rad/s
        self.declare_parameter('heading_deadband_deg', 2.0)        # deg
        self.declare_parameter('cross_track_kp', 0.8)              # m -> rad/s

        # Rotation tuning
        self.declare_parameter('rotate_speed', 0.25)               # rad/s nominal
        self.declare_parameter('rotate_min_speed', 0.05)           # rad/s
        self.declare_parameter('rotate_tolerance_deg', 2.0)        # deg
        self.declare_parameter('rotate_kp', 0.8)                   # rad -> rad/s
        self.declare_parameter('max_rotate_speed', 0.25)           # rad/s
        self.declare_parameter('rotate_slowdown_angle_deg', 20.0)  # deg

        # Safety timeouts
        self.declare_parameter('forward_timeout_margin', 8.0)      # s
        self.declare_parameter('rotate_timeout_margin', 8.0)       # s

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.odom_wait_timeout = float(self.get_parameter('odom_wait_timeout').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.stop_burst_count = int(self.get_parameter('stop_burst_count').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.forward_min_speed = float(self.get_parameter('forward_min_speed').value)
        self.forward_tolerance = float(self.get_parameter('forward_tolerance').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_heading_correction = float(self.get_parameter('max_heading_correction').value)
        self.heading_deadband = math.radians(float(self.get_parameter('heading_deadband_deg').value))
        self.cross_track_kp = float(self.get_parameter('cross_track_kp').value)

        self.rotate_speed = float(self.get_parameter('rotate_speed').value)
        self.rotate_min_speed = float(self.get_parameter('rotate_min_speed').value)
        self.rotate_tolerance = math.radians(float(self.get_parameter('rotate_tolerance_deg').value))
        self.rotate_kp = float(self.get_parameter('rotate_kp').value)
        self.max_rotate_speed = float(self.get_parameter('max_rotate_speed').value)
        self.rotate_slowdown_angle = math.radians(float(self.get_parameter('rotate_slowdown_angle_deg').value))

        self.forward_timeout_margin = float(self.get_parameter('forward_timeout_margin').value)
        self.rotate_timeout_margin = float(self.get_parameter('rotate_timeout_margin').value)

        if self.control_rate <= 0.0:
            raise ValueError('control_rate must be > 0')
        if self.forward_speed <= 0.0:
            raise ValueError('forward_speed must be > 0')
        if self.forward_min_speed <= 0.0:
            raise ValueError('forward_min_speed must be > 0')
        if self.rotate_speed <= 0.0:
            raise ValueError('rotate_speed must be > 0')
        if self.rotate_min_speed <= 0.0:
            raise ValueError('rotate_min_speed must be > 0')

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)

        self.latest_pose: Optional[Tuple[float, float, float]] = None

        self.get_logger().info(f'cmd_vel_topic: {cmd_vel_topic}')
        self.get_logger().info(f'odom_topic: {odom_topic}')
        self.get_logger().info(f'control_rate: {self.control_rate:.1f} Hz')
        self.get_logger().info(
            f'forward_speed={self.forward_speed:.3f} m/s, rotate_speed={self.rotate_speed:.3f} rad/s'
        )

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self.latest_pose = (float(pos.x), float(pos.y), float(yaw))

    def wait_for_odom(self) -> bool:
        self.get_logger().info('Waiting for odometry...')
        deadline = time.monotonic() + self.odom_wait_timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is not None:
                x, y, yaw = self.latest_pose
                self.get_logger().info(
                    f'Odometry received: x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f} deg'
                )
                return True

        self.get_logger().error('Timed out waiting for odometry.')
        return False

    def get_pose(self) -> Optional[Tuple[float, float, float]]:
        rclpy.spin_once(self, timeout_sec=0.0)
        return self.latest_pose

    def publish_twist(self, linear_x: float = 0.0, angular_z: float = 0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        for _ in range(self.stop_burst_count):
            self.publish_twist(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)
            time.sleep(0.02)

    def forward(self, distance_m: float) -> bool:
        if self.latest_pose is None and not self.wait_for_odom():
            return False

        if abs(distance_m) < 1e-6:
            self.get_logger().warn('Forward distance is zero. Nothing to do.')
            self.stop_robot()
            return True

        start_pose = self.get_pose()
        if start_pose is None:
            self.get_logger().error('No odometry pose available.')
            return False

        start_x, start_y, start_yaw = start_pose
        direction = 1.0 if distance_m > 0.0 else -1.0
        target_distance = abs(distance_m)

        nominal_timeout = target_distance / max(self.forward_min_speed, 1e-3)
        deadline = time.monotonic() + nominal_timeout + self.forward_timeout_margin
        period = 1.0 / self.control_rate

        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)

        self.get_logger().info(
            f'Starting forward move: target={distance_m:.3f} m from '
            f'x={start_x:.3f}, y={start_y:.3f}, yaw={math.degrees(start_yaw):.1f} deg'
        )

        success = False

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.latest_pose
            if pose is None:
                time.sleep(period)
                continue

            x, y, yaw = pose
            dx = x - start_x
            dy = y - start_y

            # Progress along the starting heading
            progress = dx * cos_yaw + dy * sin_yaw

            # Lateral error relative to the starting heading
            cross_track = -dx * sin_yaw + dy * cos_yaw

            signed_progress = progress * direction
            remaining = target_distance - signed_progress

            if remaining <= self.forward_tolerance:
                success = True
                break

            heading_error = wrap_angle(start_yaw - yaw)

            # Deadband to reduce zig-zag behaviour
            if abs(heading_error) < self.heading_deadband:
                heading_error = 0.0

            angular_correction = (
                self.heading_kp * heading_error
                - self.cross_track_kp * cross_track * direction
            )
            angular_correction = clamp(
                angular_correction,
                -self.max_heading_correction,
                self.max_heading_correction
            )

            commanded_speed = clamp(
                remaining * 0.6,
                self.forward_min_speed,
                self.forward_speed
            )
            commanded_speed *= direction

            self.publish_twist(commanded_speed, angular_correction)
            time.sleep(period)

        self.stop_robot()
        time.sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)

        end_pose = self.get_pose()
        if end_pose is None:
            self.get_logger().error('Lost odometry after forward move.')
            return False

        end_x, end_y, end_yaw = end_pose
        dx = end_x - start_x
        dy = end_y - start_y
        progress = dx * cos_yaw + dy * sin_yaw
        cross_track = -dx * sin_yaw + dy * cos_yaw
        heading_change = wrap_angle(end_yaw - start_yaw)

        self.get_logger().info(
            f'Forward result: progress={progress:.3f} m, '
            f'cross_track={cross_track:.3f} m, '
            f'dx={dx:.3f}, dy={dy:.3f}, '
            f'heading_change={math.degrees(heading_change):.2f} deg, '
            f'error={direction * progress - target_distance:+.3f} m'
        )

        if not success:
            self.get_logger().warn('Forward motion ended by timeout before reaching target.')

        return success

    def rotate_deg(self, angle_deg: float) -> bool:
        if self.latest_pose is None and not self.wait_for_odom():
            return False

        if abs(angle_deg) < 1e-6:
            self.get_logger().warn('Rotation angle is zero. Nothing to do.')
            self.stop_robot()
            return True

        start_pose = self.get_pose()
        if start_pose is None:
            self.get_logger().error('No odometry pose available.')
            return False

        _, _, start_yaw = start_pose
        target_angle = math.radians(angle_deg)
        target_abs = abs(target_angle)

        nominal_timeout = target_abs / max(self.rotate_min_speed, 1e-3)
        deadline = time.monotonic() + nominal_timeout + self.rotate_timeout_margin
        period = 1.0 / self.control_rate

        self.get_logger().info(
            f'Starting rotation: target={angle_deg:.1f} deg from yaw={math.degrees(start_yaw):.1f} deg'
        )

        success = False

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.latest_pose
            if pose is None:
                time.sleep(period)
                continue

            _, _, yaw = pose
            delta_yaw = wrap_angle(yaw - start_yaw)
            remaining = wrap_angle(target_angle - delta_yaw)

            if abs(remaining) <= self.rotate_tolerance:
                success = True
                break

            commanded_w = self.rotate_kp * remaining

            # Slow down near target to reduce overshoot
            if abs(remaining) < self.rotate_slowdown_angle:
                commanded_w = clamp(commanded_w, -0.12, 0.12)
                if abs(commanded_w) < self.rotate_min_speed:
                    commanded_w = math.copysign(self.rotate_min_speed, remaining)
            else:
                commanded_w = clamp(commanded_w, -self.max_rotate_speed, self.max_rotate_speed)

            # Also respect rotate_speed as the overall requested cap
            commanded_w = clamp(commanded_w, -self.rotate_speed, self.rotate_speed)

            self.publish_twist(0.0, commanded_w)
            time.sleep(period)

        self.stop_robot()
        time.sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)

        end_pose = self.get_pose()
        if end_pose is None:
            self.get_logger().error('Lost odometry after rotation.')
            return False

        _, _, end_yaw = end_pose
        measured = wrap_angle(end_yaw - start_yaw)
        angle_error = wrap_angle(measured - target_angle)

        self.get_logger().info(
            f'Rotation result: measured={math.degrees(measured):.2f} deg, '
            f'error={math.degrees(angle_error):+.2f} deg'
        )

        if not success:
            self.get_logger().warn('Rotation ended by timeout before reaching target.')

        return success


def print_usage():
    print(
        '\nUsage:\n'
        '  ros2 run my_robot_t2 motion_commander_cl.py forward 1.0\n'
        '  ros2 run my_robot_t2 motion_commander_cl.py rotate 90\n'
        '  ros2 run my_robot_t2 motion_commander_cl.py rotate 180\n'
        '  ros2 run my_robot_t2 motion_commander_cl.py rotate -90\n'
        '\nUseful parameter overrides:\n'
        '  --ros-args -p forward_speed:=0.07 -p heading_kp:=0.5 -p cross_track_kp:=1.0\n'
        '  --ros-args -p rotate_speed:=0.25 -p rotate_kp:=0.8 -p rotate_min_speed:=0.05\n'
    )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MotionCommander()

        argv = sys.argv
        non_ros_args = []

        for arg in argv[1:]:
            if arg == '--ros-args':
                break
            non_ros_args.append(arg)

        if len(non_ros_args) < 2:
            print_usage()
            return

        command = non_ros_args[0].lower()

        try:
            value = float(non_ros_args[1])
        except ValueError:
            print(f'Invalid numeric value: {non_ros_args[1]}')
            print_usage()
            return

        if not node.wait_for_odom():
            return

        time.sleep(0.3)

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
        if node is not None:
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