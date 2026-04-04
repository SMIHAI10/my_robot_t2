#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class AssistedTeleop(Node):
    def __init__(self):
        super().__init__('assisted_teleop')

        # Topics
        self.declare_parameter('input_cmd_vel_topic', '/cmd_vel_teleop')
        self.declare_parameter('output_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')

        # General
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)

        # Heading-hold behaviour
        self.declare_parameter('heading_hold_enabled', True)
        self.declare_parameter('heading_kp', 0.9)
        self.declare_parameter('max_heading_correction', 0.12)
        self.declare_parameter('angular_deadband', 0.08)     # rad/s
        self.declare_parameter('linear_deadband', 0.02)      # m/s
        self.declare_parameter('min_linear_for_hold', 0.03)  # m/s
        self.declare_parameter('heading_capture_delay', 0.10)  # s
        self.declare_parameter('zero_output_when_stale', True)

        input_topic = self.get_parameter('input_cmd_vel_topic').value
        output_topic = self.get_parameter('output_cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)

        self.heading_hold_enabled = bool(self.get_parameter('heading_hold_enabled').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_heading_correction = float(self.get_parameter('max_heading_correction').value)
        self.angular_deadband = float(self.get_parameter('angular_deadband').value)
        self.linear_deadband = float(self.get_parameter('linear_deadband').value)
        self.min_linear_for_hold = float(self.get_parameter('min_linear_for_hold').value)
        self.heading_capture_delay = float(self.get_parameter('heading_capture_delay').value)
        self.zero_output_when_stale = bool(self.get_parameter('zero_output_when_stale').value)

        self.cmd_sub = self.create_subscription(Twist, input_topic, self.cmd_callback, 20)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)
        self.cmd_pub = self.create_publisher(Twist, output_topic, 20)

        self.timer = self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.latest_input_cmd = Twist()
        self.last_cmd_time: Optional[float] = None
        self.latest_yaw: Optional[float] = None

        self.heading_hold_active = False
        self.heading_target: Optional[float] = None
        self.heading_candidate_start_time: Optional[float] = None

        self.last_mode = 'idle'

        self.get_logger().info(f'input_cmd_vel_topic: {input_topic}')
        self.get_logger().info(f'output_cmd_vel_topic: {output_topic}')
        self.get_logger().info(f'odom_topic: {odom_topic}')
        self.get_logger().info(f'control_rate: {self.control_rate:.1f} Hz')
        self.get_logger().info(
            f'heading_hold_enabled={self.heading_hold_enabled}, '
            f'heading_kp={self.heading_kp:.3f}, '
            f'max_heading_correction={self.max_heading_correction:.3f}'
        )

    def cmd_callback(self, msg: Twist):
        self.latest_input_cmd = msg
        self.last_cmd_time = time.monotonic()

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.latest_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def reset_heading_hold(self):
        self.heading_hold_active = False
        self.heading_target = None
        self.heading_candidate_start_time = None

    def publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def control_loop(self):
        now = time.monotonic()

        if self.last_cmd_time is None:
            if self.zero_output_when_stale:
                self.publish_cmd(0.0, 0.0)
            self.reset_heading_hold()
            self._update_mode('idle')
            return

        cmd_age = now - self.last_cmd_time
        if cmd_age > self.cmd_timeout:
            if self.zero_output_when_stale:
                self.publish_cmd(0.0, 0.0)
            self.reset_heading_hold()
            self._update_mode('stale')
            return

        in_lin = float(self.latest_input_cmd.linear.x)
        in_ang = float(self.latest_input_cmd.angular.z)

        # Deadbands
        if abs(in_lin) < self.linear_deadband:
            in_lin = 0.0
        if abs(in_ang) < self.angular_deadband:
            in_ang = 0.0

        # No odom: pass through safely
        if self.latest_yaw is None or not self.heading_hold_enabled:
            self.publish_cmd(in_lin, in_ang)
            self.reset_heading_hold()
            self._update_mode('passthrough_no_odom')
            return

        # Stop command: reset heading hold
        if in_lin == 0.0 and in_ang == 0.0:
            self.publish_cmd(0.0, 0.0)
            self.reset_heading_hold()
            self._update_mode('idle')
            return

        # Intentional turn command: pass through, do not heading-hold
        if abs(in_ang) > 0.0:
            self.publish_cmd(in_lin, in_ang)
            self.reset_heading_hold()
            self._update_mode('manual_turn')
            return

        # Pure linear teleop: hold heading if moving enough
        if abs(in_lin) >= self.min_linear_for_hold:
            if not self.heading_hold_active:
                if self.heading_candidate_start_time is None:
                    self.heading_candidate_start_time = now
                    self.heading_target = self.latest_yaw
                elif (now - self.heading_candidate_start_time) >= self.heading_capture_delay:
                    self.heading_hold_active = True
                    if self.heading_target is None:
                        self.heading_target = self.latest_yaw
            else:
                if self.heading_target is None:
                    self.heading_target = self.latest_yaw

            if self.heading_hold_active and self.heading_target is not None:
                heading_error = wrap_angle(self.heading_target - self.latest_yaw)
                correction = clamp(
                    self.heading_kp * heading_error,
                    -self.max_heading_correction,
                    self.max_heading_correction
                )
                self.publish_cmd(in_lin, correction)
                self._update_mode('heading_hold')
                return
            else:
                # small delay before latching heading target
                self.publish_cmd(in_lin, 0.0)
                self._update_mode('capture_heading')
                return

        # Low-speed linear motion: pass through, don’t force heading hold
        self.publish_cmd(in_lin, 0.0)
        self.reset_heading_hold()
        self._update_mode('low_speed_linear')

    def _update_mode(self, mode: str):
        if mode != self.last_mode:
            self.get_logger().info(f'assisted_teleop mode: {mode}')
            self.last_mode = mode


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AssistedTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.publish_cmd(0.0, 0.0)
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()