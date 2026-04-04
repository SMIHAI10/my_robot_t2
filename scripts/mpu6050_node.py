#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from mpu6050 import mpu6050


G = 9.80665


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('i2c_address', 0x68)
        self.declare_parameter('publish_rate', 50.0)

        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calibration_samples', 300)
        self.declare_parameter('calibration_delay_sec', 2.0)
        self.declare_parameter('calibration_sample_dt', 0.01)

        # Reject startup calibration if robot is moving too much
        self.declare_parameter('gyro_stationary_threshold_deg_s', 3.0)
        self.declare_parameter('accel_stationary_threshold_m_s2', 1.5)

        # Optional manual fallbacks
        self.declare_parameter('gyro_bias_x', 0.0)
        self.declare_parameter('gyro_bias_y', 0.0)
        self.declare_parameter('gyro_bias_z', 0.0)

        self.declare_parameter('accel_bias_x', 0.0)
        self.declare_parameter('accel_bias_y', 0.0)
        self.declare_parameter('accel_bias_z', 0.0)

        # If True, remove X/Y accel bias found at startup
        self.declare_parameter('calibrate_accel_xy', True)

        # If True, compute Z bias relative to gravity magnitude
        self.declare_parameter('calibrate_accel_z', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.i2c_address = self.get_parameter('i2c_address').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.auto_calibrate = self.get_parameter('auto_calibrate').value
        self.calibration_samples = int(self.get_parameter('calibration_samples').value)
        self.calibration_delay_sec = float(self.get_parameter('calibration_delay_sec').value)
        self.calibration_sample_dt = float(self.get_parameter('calibration_sample_dt').value)

        self.gyro_stationary_threshold_deg_s = float(
            self.get_parameter('gyro_stationary_threshold_deg_s').value
        )
        self.accel_stationary_threshold_m_s2 = float(
            self.get_parameter('accel_stationary_threshold_m_s2').value
        )

        self.calibrate_accel_xy = bool(self.get_parameter('calibrate_accel_xy').value)
        self.calibrate_accel_z = bool(self.get_parameter('calibrate_accel_z').value)

        self.gyro_bias_x = float(self.get_parameter('gyro_bias_x').value)
        self.gyro_bias_y = float(self.get_parameter('gyro_bias_y').value)
        self.gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)

        self.accel_bias_x = float(self.get_parameter('accel_bias_x').value)
        self.accel_bias_y = float(self.get_parameter('accel_bias_y').value)
        self.accel_bias_z = float(self.get_parameter('accel_bias_z').value)

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)

        try:
            self.sensor = mpu6050(self.i2c_address)
            self.get_logger().info(
                f'MPU6050 opened at I2C address 0x{self.i2c_address:02X}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to open MPU6050: {e}')
            raise

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        if self.auto_calibrate:
            self.get_logger().info(
                f'Auto calibration scheduled in {self.calibration_delay_sec:.1f} seconds...'
            )
            self.calibration_timer = self.create_timer(
                self.calibration_delay_sec,
                self.delayed_calibration_callback
            )
        else:
            self.get_logger().warn(
                'Auto calibration disabled. Using configured manual biases.'
            )
    def delayed_calibration_callback(self):
        # one-shot timer
        try:
            self.calibration_timer.cancel()
        except Exception:
            pass

        self.calibrate_imu()

    def read_sensor_once(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()
        return accel, gyro

    def calibrate_imu(self):
        self.get_logger().info('Calibrating IMU. Keep the robot completely still...')
        time.sleep(self.calibration_delay_sec)

        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0
        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0

        valid_samples = 0
        rejected_samples = 0

        for _ in range(self.calibration_samples):
            try:
                accel = self.sensor.get_accel_data()
                gyro = self.sensor.get_gyro_data()
            except Exception as e:
                self.get_logger().warning(f'Calibration read failed: {e}')
                time.sleep(0.01)
                continue

            gx = float(gyro['x'])
            gy = float(gyro['y'])
            gz = float(gyro['z'])

            ax = float(accel['x'])
            ay = float(accel['y'])
            az = float(accel['z'])

            gyro_ok = (
                abs(gx) < self.gyro_stationary_threshold_deg_s
                and abs(gy) < self.gyro_stationary_threshold_deg_s
                and abs(gz) < self.gyro_stationary_threshold_deg_s
            )
            
            if not gyro_ok and rejected_samples < 10:
                self.get_logger().warn(
                    f'Rejecting calibration sample: '
                    f'gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f} deg/s'
                )

            if gyro_ok:
                sum_gx += gx
                sum_gy += gy
                sum_gz += gz

                sum_ax += ax
                sum_ay += ay
                sum_az += az

                valid_samples += 1
            else:
                rejected_samples += 1

            time.sleep(0.01)

        if valid_samples < max(20, int(0.25 * self.calibration_samples)):
            self.get_logger().error(
                f'Calibration failed. Only {valid_samples} valid samples, '
                f'{rejected_samples} rejected. Keeping existing/manual bias values.'
            )
            return

        avg_gx = sum_gx / valid_samples
        avg_gy = sum_gy / valid_samples
        avg_gz = sum_gz / valid_samples

        avg_ax = sum_ax / valid_samples
        avg_ay = sum_ay / valid_samples
        avg_az = sum_az / valid_samples

        self.gyro_bias_x = avg_gx
        self.gyro_bias_y = avg_gy
        self.gyro_bias_z = avg_gz

        self.accel_bias_x = avg_ax
        self.accel_bias_y = avg_ay
        self.accel_bias_z = avg_az - G

        self.get_logger().info(
            f'Calibration complete with {valid_samples} samples, {rejected_samples} rejected.\n'
            f'Gyro bias [deg/s]: x={self.gyro_bias_x:.4f}, y={self.gyro_bias_y:.4f}, z={self.gyro_bias_z:.4f}\n'
            f'Accel bias [m/s^2]: x={self.accel_bias_x:.4f}, y={self.accel_bias_y:.4f}, z={self.accel_bias_z:.4f}'
        )

    def timer_callback(self):
        try:
            accel, gyro = self.read_sensor_once()
        except Exception as e:
            self.get_logger().warning(f'Failed to read MPU6050: {e}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Gyro: sensor gives deg/s, convert to rad/s after bias removal
        gx_deg = float(gyro['x']) - self.gyro_bias_x
        gy_deg = float(gyro['y']) - self.gyro_bias_y
        gz_deg = float(gyro['z']) - self.gyro_bias_z

        # Accel: sensor library already gives m/s^2
        ax = float(accel['x']) - self.accel_bias_x
        ay = float(accel['y']) - self.accel_bias_y
        az = float(accel['z']) - self.accel_bias_z

        msg.angular_velocity.x = math.radians(gx_deg)
        msg.angular_velocity.y = math.radians(gy_deg)
        msg.angular_velocity.z = math.radians(gz_deg)

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # No orientation estimate from MPU6050 raw node
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.orientation_covariance[0] = -1.0

        # Start conservative; tune later if needed
        msg.angular_velocity_covariance[0] = 0.02
        msg.angular_velocity_covariance[4] = 0.02
        msg.angular_velocity_covariance[8] = 0.02

        msg.linear_acceleration_covariance[0] = 0.10
        msg.linear_acceleration_covariance[4] = 0.10
        msg.linear_acceleration_covariance[8] = 0.10

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