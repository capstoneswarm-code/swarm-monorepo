#!/usr/bin/env python3
"""Synthetic wheel odometry + gyro; no hardware access or motor commands.

The wheel yaw rate has a deliberate +0.08 rad/s error so its effect is easy
to observe. Truth is published for comparison, never fed to the EKF.
"""

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


def integrate(pose, speed, yaw_rate, dt):
    """Exact planar integration for constant body speed and yaw rate."""
    x, y, yaw = pose
    next_yaw = yaw + yaw_rate * dt
    if abs(yaw_rate) < 1e-9:
        x += speed * math.cos(yaw) * dt
        y += speed * math.sin(yaw) * dt
    else:
        x += speed / yaw_rate * (math.sin(next_yaw) - math.sin(yaw))
        y -= speed / yaw_rate * (math.cos(next_yaw) - math.cos(yaw))
    return x, y, math.atan2(math.sin(next_yaw), math.cos(next_yaw))


def odometry(stamp, pose, speed, yaw_rate):
    message = Odometry()
    message.header.stamp = stamp
    message.header.frame_id = "odom"
    message.child_frame_id = "base_link"
    message.pose.pose.position.x, message.pose.pose.position.y, yaw = pose
    message.pose.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(yaw / 2.0)
    message.twist.twist.linear.x = speed
    message.twist.twist.linear.y = 0.0
    message.twist.twist.angular.z = yaw_rate
    # Example variances, not calibrated values for your chassis.
    for index in (0, 7, 14, 21, 28, 35):
        message.pose.covariance[index] = 1.0  # Pose is not fused.
        message.twist.covariance[index] = 1.0
    message.twist.covariance[0] = 0.02 ** 2
    # A soft no-sideways-motion constraint for a differential-drive robot.
    message.twist.covariance[7] = 0.02 ** 2
    message.twist.covariance[35] = 0.08 ** 2
    return message


class DemoInputs(Node):
    def __init__(self):
        super().__init__("demo_inputs")
        self.wheel_pub = self.create_publisher(Odometry, "odometry/wheel", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.truth_pub = self.create_publisher(Odometry, "demo/truth", 10)
        self.random = random.Random(7)
        self.truth_pose = (0.0, 0.0, 0.0)
        self.wheel_pose = (0.0, 0.0, 0.0)
        self.previous_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / 30.0, self.tick)
        self.get_logger().info(
            "Synthetic inputs only; no motor commands. "
            "True yaw rate: 0.30 rad/s; biased wheel yaw rate: about 0.38."
        )

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.previous_time).nanoseconds * 1e-9
        self.previous_time = now
        if dt <= 0.0:
            return
        stamp = now.to_msg()
        true_speed, true_yaw_rate = 0.15, 0.30
        wheel_speed = true_speed + self.random.gauss(0.0, 0.02)
        wheel_yaw_rate = true_yaw_rate + 0.08 + self.random.gauss(0.0, 0.04)
        self.truth_pose = integrate(self.truth_pose, true_speed, true_yaw_rate, dt)
        self.wheel_pose = integrate(self.wheel_pose, wheel_speed, wheel_yaw_rate, dt)

        self.wheel_pub.publish(odometry(stamp, self.wheel_pose,
                                       wheel_speed, wheel_yaw_rate))

        imu = Imu()
        imu.header.stamp = stamp
        # Demo data is already expressed at the robot body frame.
        # Real MPU6050 data should normally use imu_link plus a mounting TF.
        imu.header.frame_id = "base_link"
        imu.orientation_covariance[0] = -1.0  # No orientation estimate.
        imu.linear_acceleration_covariance[0] = -1.0  # Not supplied here.
        imu.angular_velocity.z = true_yaw_rate + self.random.gauss(0.0, 0.01)
        for index in (0, 4, 8):
            imu.angular_velocity_covariance[index] = 0.01 ** 2
        self.imu_pub.publish(imu)

        self.truth_pub.publish(odometry(stamp, self.truth_pose,
                                       true_speed, true_yaw_rate))


def main():
    rclpy.init()
    node = DemoInputs()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
