#!/usr/bin/env python3

import math
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

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


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class TestResult:
    name: str
    success: bool
    start_pose: Pose2D
    end_pose: Pose2D
    commanded_value: float
    measured_value: float
    error_value: float
    cross_track: float = 0.0
    heading_change_deg: float = 0.0
    duration_sec: float = 0.0


class MotionTestSuite(Node):
    def __init__(self):
        super().__init__('motion_test_suite')

        # Topics
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')

        # General
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('odom_wait_timeout', 5.0)
        self.declare_parameter('settle_time', 0.3)
        self.declare_parameter('pause_between_tests', 2.0)
        self.declare_parameter('stop_burst_count', 8)

        # Forward tuning
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('forward_min_speed', 0.04)
        self.declare_parameter('forward_tolerance', 0.015)
        self.declare_parameter('heading_kp', 0.45)
        self.declare_parameter('max_heading_correction', 0.08)
        self.declare_parameter('heading_deadband_deg', 2.0)
        self.declare_parameter('cross_track_kp', 0.8)
        self.declare_parameter('forward_timeout_margin', 8.0)

        # Rotation tuning
        self.declare_parameter('rotate_speed', 0.25)
        self.declare_parameter('rotate_min_speed', 0.05)
        self.declare_parameter('rotate_tolerance_deg', 2.0)
        self.declare_parameter('rotate_kp', 0.8)
        self.declare_parameter('max_rotate_speed', 0.25)
        self.declare_parameter('rotate_slowdown_angle_deg', 20.0)
        self.declare_parameter('rotate_timeout_margin', 8.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.control_rate = float(self.get_parameter('control_rate').value)
        self.odom_wait_timeout = float(self.get_parameter('odom_wait_timeout').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.pause_between_tests = float(self.get_parameter('pause_between_tests').value)
        self.stop_burst_count = int(self.get_parameter('stop_burst_count').value)

        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.forward_min_speed = float(self.get_parameter('forward_min_speed').value)
        self.forward_tolerance = float(self.get_parameter('forward_tolerance').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.max_heading_correction = float(self.get_parameter('max_heading_correction').value)
        self.heading_deadband = math.radians(float(self.get_parameter('heading_deadband_deg').value))
        self.cross_track_kp = float(self.get_parameter('cross_track_kp').value)
        self.forward_timeout_margin = float(self.get_parameter('forward_timeout_margin').value)

        self.rotate_speed = float(self.get_parameter('rotate_speed').value)
        self.rotate_min_speed = float(self.get_parameter('rotate_min_speed').value)
        self.rotate_tolerance = math.radians(float(self.get_parameter('rotate_tolerance_deg').value))
        self.rotate_kp = float(self.get_parameter('rotate_kp').value)
        self.max_rotate_speed = float(self.get_parameter('max_rotate_speed').value)
        self.rotate_slowdown_angle = math.radians(float(self.get_parameter('rotate_slowdown_angle_deg').value))
        self.rotate_timeout_margin = float(self.get_parameter('rotate_timeout_margin').value)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 20)

        self.latest_pose: Optional[Pose2D] = None

        self.get_logger().info(f'cmd_vel_topic: {cmd_vel_topic}')
        self.get_logger().info(f'odom_topic: {odom_topic}')
        self.get_logger().info(f'control_rate: {self.control_rate:.1f} Hz')

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)
        self.latest_pose = Pose2D(float(pos.x), float(pos.y), float(yaw))

    def wait_for_odom(self) -> bool:
        self.get_logger().info('Waiting for odometry...')
        deadline = time.monotonic() + self.odom_wait_timeout

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_pose is not None:
                p = self.latest_pose
                self.get_logger().info(
                    f'Odometry received: x={p.x:.3f}, y={p.y:.3f}, yaw={math.degrees(p.yaw):.1f} deg'
                )
                return True

        self.get_logger().error('Timed out waiting for odometry.')
        return False

    def get_pose(self) -> Optional[Pose2D]:
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

    def pause_after_test(self):
        self.stop_robot()
        time.sleep(self.pause_between_tests)

    def format_pose(self, pose: Pose2D) -> str:
        return f'x={pose.x:.3f}, y={pose.y:.3f}, yaw={math.degrees(pose.yaw):.2f} deg'

    def run_forward_test(self, distance_m: float) -> Optional[TestResult]:
        if self.latest_pose is None and not self.wait_for_odom():
            return None

        start_pose = self.get_pose()
        if start_pose is None:
            self.get_logger().error('No odometry pose available at forward test start.')
            return None

        start_x, start_y, start_yaw = start_pose.x, start_pose.y, start_pose.yaw
        direction = 1.0 if distance_m >= 0.0 else -1.0
        target_distance = abs(distance_m)

        nominal_timeout = target_distance / max(self.forward_min_speed, 1e-3)
        deadline = time.monotonic() + nominal_timeout + self.forward_timeout_margin
        period = 1.0 / self.control_rate

        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)

        self.get_logger().info(f'Running FORWARD test: {distance_m:.3f} m')
        self.get_logger().info(f'Start pose: {self.format_pose(start_pose)}')

        t0 = time.monotonic()
        success = False

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.latest_pose
            if pose is None:
                time.sleep(period)
                continue

            dx = pose.x - start_x
            dy = pose.y - start_y

            progress = dx * cos_yaw + dy * sin_yaw
            cross_track = -dx * sin_yaw + dy * cos_yaw

            signed_progress = progress * direction
            remaining = target_distance - signed_progress

            if remaining <= self.forward_tolerance:
                success = True
                break

            heading_error = wrap_angle(start_yaw - pose.yaw)
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
            ) * direction

            self.publish_twist(commanded_speed, angular_correction)
            time.sleep(period)

        self.stop_robot()
        time.sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)

        end_pose = self.get_pose()
        if end_pose is None:
            self.get_logger().error('No odometry pose available at forward test end.')
            return None

        dx = end_pose.x - start_x
        dy = end_pose.y - start_y
        progress = dx * cos_yaw + dy * sin_yaw
        cross_track = -dx * sin_yaw + dy * cos_yaw
        heading_change = wrap_angle(end_pose.yaw - start_yaw)
        measured = direction * progress
        error = measured - target_distance
        duration = time.monotonic() - t0

        result = TestResult(
            name=f'forward_{distance_m:.2f}m',
            success=success,
            start_pose=start_pose,
            end_pose=end_pose,
            commanded_value=distance_m,
            measured_value=measured,
            error_value=error,
            cross_track=cross_track,
            heading_change_deg=math.degrees(heading_change),
            duration_sec=duration,
        )

        self.get_logger().info(f'End pose:   {self.format_pose(end_pose)}')
        self.get_logger().info(
            f'Forward result: measured={measured:.3f} m, error={error:+.3f} m, '
            f'cross_track={cross_track:.3f} m, heading_change={math.degrees(heading_change):+.2f} deg, '
            f'duration={duration:.2f} s, success={success}'
        )

        return result

    def run_rotate_test(self, angle_deg: float) -> Optional[TestResult]:
        if self.latest_pose is None and not self.wait_for_odom():
            return None

        start_pose = self.get_pose()
        if start_pose is None:
            self.get_logger().error('No odometry pose available at rotate test start.')
            return None

        start_yaw = start_pose.yaw
        target_angle = math.radians(angle_deg)
        target_abs = abs(target_angle)

        nominal_timeout = target_abs / max(self.rotate_min_speed, 1e-3)
        deadline = time.monotonic() + nominal_timeout + self.rotate_timeout_margin
        period = 1.0 / self.control_rate

        self.get_logger().info(f'Running ROTATE test: {angle_deg:.1f} deg')
        self.get_logger().info(f'Start pose: {self.format_pose(start_pose)}')

        t0 = time.monotonic()
        success = False

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            pose = self.latest_pose
            if pose is None:
                time.sleep(period)
                continue

            delta_yaw = wrap_angle(pose.yaw - start_yaw)
            remaining = wrap_angle(target_angle - delta_yaw)

            if abs(remaining) <= self.rotate_tolerance:
                success = True
                break

            commanded_w = self.rotate_kp * remaining

            if abs(remaining) < self.rotate_slowdown_angle:
                commanded_w = clamp(commanded_w, -0.12, 0.12)
                if abs(commanded_w) < self.rotate_min_speed:
                    commanded_w = math.copysign(self.rotate_min_speed, remaining)
            else:
                commanded_w = clamp(commanded_w, -self.max_rotate_speed, self.max_rotate_speed)

            commanded_w = clamp(commanded_w, -self.rotate_speed, self.rotate_speed)

            self.publish_twist(0.0, commanded_w)
            time.sleep(period)

        self.stop_robot()
        time.sleep(self.settle_time)
        rclpy.spin_once(self, timeout_sec=0.1)

        end_pose = self.get_pose()
        if end_pose is None:
            self.get_logger().error('No odometry pose available at rotate test end.')
            return None

        measured_rad = wrap_angle(end_pose.yaw - start_yaw)
        error_rad = wrap_angle(measured_rad - target_angle)
        duration = time.monotonic() - t0

        result = TestResult(
            name=f'rotate_{angle_deg:.0f}deg',
            success=success,
            start_pose=start_pose,
            end_pose=end_pose,
            commanded_value=angle_deg,
            measured_value=math.degrees(measured_rad),
            error_value=math.degrees(error_rad),
            cross_track=0.0,
            heading_change_deg=math.degrees(measured_rad),
            duration_sec=duration,
        )

        self.get_logger().info(f'End pose:   {self.format_pose(end_pose)}')
        self.get_logger().info(
            f'Rotate result: measured={math.degrees(measured_rad):.2f} deg, '
            f'error={math.degrees(error_rad):+.2f} deg, duration={duration:.2f} s, success={success}'
        )

        return result

    def print_summary(self, results: List[TestResult]):
        print('\n' + '=' * 100)
        print('MOTION TEST SUITE SUMMARY')
        print('=' * 100)
        print(
            f'{"Test":<20} {"Success":<8} {"Commanded":<14} {"Measured":<14} '
            f'{"Error":<12} {"CrossTrack":<12} {"HeadChange":<12} {"Duration":<10}'
        )
        print('-' * 100)

        for r in results:
            if r.name.startswith('forward'):
                commanded = f'{r.commanded_value:.3f} m'
                measured = f'{r.measured_value:.3f} m'
                error = f'{r.error_value:+.3f} m'
                cross_track = f'{r.cross_track:+.3f} m'
                head_change = f'{r.heading_change_deg:+.2f} deg'
            else:
                commanded = f'{r.commanded_value:.1f} deg'
                measured = f'{r.measured_value:.2f} deg'
                error = f'{r.error_value:+.2f} deg'
                cross_track = '-'
                head_change = f'{r.heading_change_deg:+.2f} deg'

            print(
                f'{r.name:<20} {str(r.success):<8} {commanded:<14} {measured:<14} '
                f'{error:<12} {cross_track:<12} {head_change:<12} {r.duration_sec:<10.2f}'
            )

        print('-' * 100)
        print('Detailed poses:')
        for r in results:
            print(f'\n{r.name}')
            print(f'  start: {self.format_pose(r.start_pose)}')
            print(f'  end:   {self.format_pose(r.end_pose)}')
        print('=' * 100 + '\n')

    def run_suite(self) -> bool:
        if not self.wait_for_odom():
            return False

        results: List[TestResult] = []

        tests = [
            ('forward', 1.0),
            ('rotate', 90.0),
            ('rotate', -90.0),
            ('rotate', 180.0),
        ]

        for i, (test_type, value) in enumerate(tests, start=1):
            self.get_logger().info(f'===== Test {i}/{len(tests)} =====')

            if test_type == 'forward':
                result = self.run_forward_test(value)
            else:
                result = self.run_rotate_test(value)

            if result is None:
                self.get_logger().error(f'Test failed to produce a result: {test_type} {value}')
                continue

            results.append(result)

            if i < len(tests):
                self.get_logger().info(f'Pausing {self.pause_between_tests:.1f} s before next test...')
                self.pause_after_test()

        self.print_summary(results)
        return True


def print_usage():
    print(
        '\nUsage:\n'
        '  ros2 run my_robot_t2 motion_test_suite.py\n'
        '\nOptional parameter overrides:\n'
        '  --ros-args -p pause_between_tests:=3.0\n'
        '  --ros-args -p forward_speed:=0.08 -p heading_kp:=0.45 -p cross_track_kp:=0.8\n'
        '  --ros-args -p rotate_speed:=0.25 -p rotate_kp:=0.8 -p rotate_min_speed:=0.05\n'
    )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = MotionTestSuite()

        argv = sys.argv
        non_ros_args = []
        for arg in argv[1:]:
            if arg == '--ros-args':
                break
            non_ros_args.append(arg)

        if len(non_ros_args) > 0 and non_ros_args[0] in ('-h', '--help'):
            print_usage()
            return

        node.run_suite()

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