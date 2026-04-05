#!/usr/bin/env python3

import math
import sys
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def yaw_to_quaternion(yaw_rad: float):
    """Return quaternion (x, y, z, w) for a planar yaw angle."""
    half = yaw_rad * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def build_pose(
    frame_id: str,
    x: float,
    y: float,
    yaw_deg: float,
    stamp,
) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


def build_initial_pose(
    frame_id: str,
    x: float,
    y: float,
    yaw_deg: float,
    stamp,
) -> PoseWithCovarianceStamped:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    msg.pose.pose.position.x = float(x)
    msg.pose.pose.position.y = float(y)
    msg.pose.pose.position.z = 0.0

    qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    # Reasonable planar covariance defaults
    msg.pose.covariance[0] = 0.25      # x
    msg.pose.covariance[7] = 0.25      # y
    msg.pose.covariance[35] = 0.0685   # yaw

    return msg


def print_usage():
    print(
        "\nUsage examples:\n"
        "  ros2 run my_robot_t2 nav2_goal_sender.py goal 1.0 0.0 0.0\n"
        "  ros2 run my_robot_t2 nav2_goal_sender.py goal 2.0 1.0 90.0\n"
        "  ros2 run my_robot_t2 nav2_goal_sender.py goal 0.5 -0.5 180.0\n"
        "\nWith initial pose:\n"
        "  ros2 run my_robot_t2 nav2_goal_sender.py goal 1.0 0.0 0.0 --init 0.0 0.0 0.0\n"
        "\nOptional ROS params:\n"
        "  --ros-args -p global_frame:=map -p robot_base_frame:=base_link\n"
        "  --ros-args -p feedback_interval:=2.0 -p startup_wait_sec:=20.0\n"
    )


class Nav2GoalSender(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='nav2_goal_sender')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('feedback_interval', 2.0)
        self.declare_parameter('startup_wait_sec', 20.0)
        self.declare_parameter('task_timeout_sec', 180.0)
        self.declare_parameter('set_initial_pose_before_goal', False)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw_deg', 0.0)
        self.declare_parameter('clear_costmaps_before_goal', False)

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.feedback_interval = float(self.get_parameter('feedback_interval').value)
        self.startup_wait_sec = float(self.get_parameter('startup_wait_sec').value)
        self.task_timeout_sec = float(self.get_parameter('task_timeout_sec').value)
        self.set_initial_pose_before_goal = bool(
            self.get_parameter('set_initial_pose_before_goal').value
        )
        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw_deg = float(self.get_parameter('initial_yaw_deg').value)
        self.clear_costmaps_before_goal = bool(
            self.get_parameter('clear_costmaps_before_goal').value
        )

    def maybe_set_initial_pose(self):
        if not self.set_initial_pose_before_goal:
            return

        stamp = self.get_clock().now().to_msg()
        init_pose = build_initial_pose(
            self.global_frame,
            self.initial_x,
            self.initial_y,
            self.initial_yaw_deg,
            stamp,
        )

        self.info(
            f'Setting initial pose: '
            f'x={self.initial_x:.3f}, y={self.initial_y:.3f}, yaw={self.initial_yaw_deg:.1f} deg'
        )
        self.setInitialPose(init_pose)
        time.sleep(1.0)

    def send_goal_and_wait(self, x: float, y: float, yaw_deg: float) -> int:
        self.info(
            f'Goal requested: x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f} deg '
            f'in frame "{self.global_frame}"'
        )

        self.maybe_set_initial_pose()

        self.info('Waiting for Nav2 to become active...')
        self.waitUntilNav2Active()

        if self.clear_costmaps_before_goal:
            self.info('Clearing costmaps before goal...')
            self.clearAllCostmaps()

        stamp = self.get_clock().now().to_msg()
        goal_pose = build_pose(self.global_frame, x, y, yaw_deg, stamp)

        self.info('Sending goal...')
        self.goToPose(goal_pose)

        start_time = time.monotonic()
        last_feedback_print = 0.0

        while not self.isTaskComplete():
            now = time.monotonic()

            if (now - start_time) > self.task_timeout_sec:
                self.warn(
                    f'Goal timed out after {self.task_timeout_sec:.1f} s. Cancelling task.'
                )
                self.cancelTask()
                break

            feedback = self.getFeedback()
            if feedback is not None and (now - last_feedback_print) >= self.feedback_interval:
                last_feedback_print = now

                eta_sec: Optional[float] = None
                nav_time_sec: Optional[float] = None
                dist_remaining: Optional[float] = None

                try:
                    eta_sec = (
                        feedback.estimated_time_remaining.sec
                        + feedback.estimated_time_remaining.nanosec * 1e-9
                    )
                except Exception:
                    pass

                try:
                    nav_time_sec = (
                        feedback.navigation_time.sec
                        + feedback.navigation_time.nanosec * 1e-9
                    )
                except Exception:
                    pass

                try:
                    dist_remaining = float(feedback.distance_remaining)
                except Exception:
                    pass

                msg_parts = ['Feedback:']
                if dist_remaining is not None:
                    msg_parts.append(f'distance_remaining={dist_remaining:.3f} m')
                if eta_sec is not None:
                    msg_parts.append(f'eta={eta_sec:.1f} s')
                if nav_time_sec is not None:
                    msg_parts.append(f'navigation_time={nav_time_sec:.1f} s')

                self.info(', '.join(msg_parts))

            time.sleep(0.05)

        result = self.getResult()

        if result == TaskResult.SUCCEEDED:
            self.info('Goal succeeded.')
            return 0
        if result == TaskResult.CANCELED:
            self.warn('Goal was canceled.')
            return 2
        if result == TaskResult.FAILED:
            self.error('Goal failed.')
            return 1

        self.error(f'Goal returned unknown result code: {result}')
        return 3


def main(args=None):
    rclpy.init(args=args)
    navigator = None

    try:
        navigator = Nav2GoalSender()

        argv = sys.argv[1:]
        non_ros_args = []
        for arg in argv:
            if arg == '--ros-args':
                break
            non_ros_args.append(arg)

        if len(non_ros_args) < 4:
            print_usage()
            return

        command = non_ros_args[0].lower()
        if command != 'goal':
            print(f'Unknown command: {command}')
            print_usage()
            return

        try:
            goal_x = float(non_ros_args[1])
            goal_y = float(non_ros_args[2])
            goal_yaw_deg = float(non_ros_args[3])
        except ValueError:
            print('Goal arguments must be numeric: x y yaw_deg')
            print_usage()
            return

        # Optional CLI init pose:
        # ... goal x y yaw --init ix iy iyaw
        if '--init' in non_ros_args:
            init_idx = non_ros_args.index('--init')
            if len(non_ros_args) < init_idx + 4:
                print('Not enough values after --init')
                print_usage()
                return

            try:
                navigator.set_parameters([
                    rclpy.parameter.Parameter(
                        'set_initial_pose_before_goal',
                        rclpy.Parameter.Type.BOOL,
                        True
                    ),
                    rclpy.parameter.Parameter(
                        'initial_x',
                        rclpy.Parameter.Type.DOUBLE,
                        float(non_ros_args[init_idx + 1])
                    ),
                    rclpy.parameter.Parameter(
                        'initial_y',
                        rclpy.Parameter.Type.DOUBLE,
                        float(non_ros_args[init_idx + 2])
                    ),
                    rclpy.parameter.Parameter(
                        'initial_yaw_deg',
                        rclpy.Parameter.Type.DOUBLE,
                        float(non_ros_args[init_idx + 3])
                    ),
                ])
                navigator.set_initial_pose_before_goal = True
                navigator.initial_x = float(non_ros_args[init_idx + 1])
                navigator.initial_y = float(non_ros_args[init_idx + 2])
                navigator.initial_yaw_deg = float(non_ros_args[init_idx + 3])
            except ValueError:
                print('Initial pose values after --init must be numeric')
                print_usage()
                return

        exit_code = navigator.send_goal_and_wait(goal_x, goal_y, goal_yaw_deg)
        raise SystemExit(exit_code)

    except KeyboardInterrupt:
        if navigator is not None:
            try:
                navigator.warn('Keyboard interrupt received, cancelling current task...')
                navigator.cancelTask()
            except Exception:
                pass
    finally:
        if navigator is not None:
            try:
                navigator.destroyNode()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()