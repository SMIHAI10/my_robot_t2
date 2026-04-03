#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from mpu6050 import mpu6050


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 50.0)

        # Simple bias correction from your stationary readings
        self.declare_parameter('gyro_bias_x', -8.0)
        self.declare_parameter('gyro_bias_y', 2.6)
        self.declare_parameter('gyro_bias_z', 1.5)

        self.declare_parameter('accel_bias_x', -0.4)
        self.declare_parameter('accel_bias_y', 0.1)
        self.declare_parameter('accel_bias_z', 12.2)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.i2c_address = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.gyro_bias_x = self.get_parameter('gyro_bias_x').get_parameter_value().double_value
        self.gyro_bias_y = self.get_parameter('gyro_bias_y').get_parameter_value().double_value
        self.gyro_bias_z = self.get_parameter('gyro_bias_z').get_parameter_value().double_value

        self.accel_bias_x = self.get_parameter('accel_bias_x').get_parameter_value().double_value
        self.accel_bias_y = self.get_parameter('accel_bias_y').get_parameter_value().double_value
        self.accel_bias_z = self.get_parameter('accel_bias_z').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        try:
            self.sensor = mpu6050(self.i2c_address)
            self.get_logger().info(f'MPU6050 opened at I2C address 0x{self.i2c_address:02X}')
        except Exception as e:
            self.get_logger().error(f'Failed to open MPU6050: {e}')
            raise

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
        except Exception as e:
            self.get_logger().warning(f'Failed to read MPU6050: {e}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Library returns accel in m/s^2 and gyro in deg/s
        # Apply simple bias correction
        ax = accel['x'] - self.accel_bias_x
        ay = accel['y'] - self.accel_bias_y
        az = accel['z'] - self.accel_bias_z + 9.80665

        gx_deg = gyro['x'] - self.gyro_bias_x
        gy_deg = gyro['y'] - self.gyro_bias_y
        gz_deg = gyro['z'] - self.gyro_bias_z

        # Convert gyro to rad/s for ROS
        msg.angular_velocity.x = math.radians(gx_deg)
        msg.angular_velocity.y = math.radians(gy_deg)
        msg.angular_velocity.z = math.radians(gz_deg)

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # No orientation estimate yet
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # Mark orientation as unavailable
        msg.orientation_covariance[0] = -1.0

        # Rough starting covariances
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance[0] = 0.1
        msg.linear_acceleration_covariance[4] = 0.1
        msg.linear_acceleration_covariance[8] = 0.1

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()