#!/usr/bin/env python3
"""One terminal: wheel keys, hardware measurements, ROS EKF, relative travel."""

import argparse
from contextlib import ExitStack
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import termios
import time
import tty

import yaml

from demo_core import Keys, Travel, body_velocity, integrate, validate_config, wheel_speeds
from hardware import MotorBoard, MPU6050


class Keyboard:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        termios.tcflush(self.fd, termios.TCIFLUSH)
        return self

    def read(self):
        ready, _, _ = select.select([self.fd], [], [], 0.005)
        if not ready:
            return b""
        data = os.read(self.fd, 1024)
        if not data:
            raise KeyboardInterrupt
        return data

    def __exit__(self, *_):
        # Restore immediately; never wait for SSH terminal output to drain.
        termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)


def make_node(config):
    # Delay ROS imports so the non-ROS parts can be tested independently.
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    class Bridge(Node):
        def __init__(self):
            super().__init__("keyboard_bridge", namespace="/hardware_demo")
            self.wheel_pub = self.create_publisher(Odometry, "wheel_odom", 10)
            self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
            self.filtered_sub = self.create_subscription(
                Odometry, "odometry/filtered", self.filtered, 10)
            self.travel = Travel(config["report_speed_deadband_m_s"])
            self.last_filtered = None
            self.last_wheel = None
            self.wheel_pose = (0.0, 0.0, 0.0)
            self.previous_velocity = (0.0, 0.0)
            self.measured_left = self.measured_right = 0.0

        def filtered(self, message):
            if message.header.frame_id != "odom" or message.child_frame_id != "base_link":
                raise RuntimeError("Unexpected frames in filtered odometry")
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if abs(self.get_clock().now().nanoseconds * 1e-9 - stamp) > config["sensor_timeout_s"]:
                return  # Old DDS samples must not arm the motors.
            pose, twist = message.pose.pose, message.twist.twist
            q = pose.orientation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if not math.isfinite(norm) or norm < 1e-9:
                raise RuntimeError("Invalid EKF quaternion")
            x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
            yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            if self.travel.update(stamp, pose.position.x, pose.position.y,
                                  yaw, twist.linear.x, twist.linear.y):
                self.last_filtered = time.monotonic()

        def publish_wheels(self, values, now):
            left, right = wheel_speeds(values, config)
            velocity, yaw_rate = body_velocity(left, right, config["track_width_m"])
            if self.last_wheel is not None:
                dt = now - self.last_wheel
                if dt > config["sensor_timeout_s"]:
                    raise RuntimeError("Wheel telemetry paused; stopping")
                # Trapezoidal speed estimate over the sample interval.
                mean_v = (velocity + self.previous_velocity[0]) / 2
                mean_w = (yaw_rate + self.previous_velocity[1]) / 2
                self.wheel_pose = integrate(self.wheel_pose, mean_v, mean_w, dt)
            self.last_wheel = now
            self.previous_velocity = velocity, yaw_rate
            self.measured_left, self.measured_right = left, right
            message = Odometry()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "odom"
            message.child_frame_id = "base_link"
            px, py, angle = self.wheel_pose
            message.pose.pose.position.x = px
            message.pose.pose.position.y = py
            message.pose.pose.orientation.z = math.sin(angle / 2)
            message.pose.pose.orientation.w = math.cos(angle / 2)
            message.twist.twist.linear.x = velocity
            message.twist.twist.linear.y = 0.0
            message.twist.twist.angular.z = yaw_rate
            for index in (0, 7, 14, 21, 28, 35):
                message.pose.covariance[index] = 1.0  # Not fused.
                message.twist.covariance[index] = 1.0
            message.twist.covariance[0] = config["wheel_speed_std_m_s"] ** 2
            message.twist.covariance[7] = config["lateral_speed_std_m_s"] ** 2
            message.twist.covariance[35] = config["wheel_yaw_rate_std_rad_s"] ** 2
            self.wheel_pub.publish(message)

        def publish_gyro(self, values, std):
            message = Imu()
            message.header.stamp = self.get_clock().now().to_msg()
            # The driver has ALREADY rotated angular velocity into body axes.
            # No accelerations/position-dependent quantities are published.
            message.header.frame_id = "base_link"
            message.orientation_covariance[0] = -1.0
            message.linear_acceleration_covariance[0] = -1.0
            message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = values
            for index in (0, 4, 8):
                message.angular_velocity_covariance[index] = std ** 2
            self.imu_pub.publish(message)

    return Bridge()


def shutdown_ekf(process):
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


def control_loop(node, board, imu, ekf, config):
    import rclpy
    keys = Keys(config["speed_m_s"], config["key_timeout_s"])
    start = time.monotonic()
    next_imu = next_status = start
    last_write = -math.inf
    previous_targets = None
    announced_ready = False
    print("W/S: forward/back | A/D: spin left/right | SPACE: stop | X/Ctrl+C: quit")
    print(f"Targets expire after {config['key_timeout_s']:.1f}s without a motion-key event.")
    board.start_telemetry()
    try:
        with Keyboard() as keyboard:
            while rclpy.ok():
                now = time.monotonic()
                if ekf.poll() is not None:
                    raise RuntimeError("EKF process exited; see ekf.log")
                if node.last_wheel is not None and now - node.last_wheel > config["sensor_timeout_s"]:
                    raise RuntimeError("No fresh wheel telemetry; stopping")
                if node.last_filtered is not None and now - node.last_filtered > config["sensor_timeout_s"]:
                    raise RuntimeError("No fresh filtered odometry; stopping")
                frames = board.speeds()
                if frames:
                    node.publish_wheels(frames[-1], now)
                if now >= next_imu:
                    node.publish_gyro(imu.body_gyro(), imu.gyro_std)
                    next_imu = time.monotonic() + 0.02
                rclpy.spin_once(node, timeout_sec=0.0)
                if node.count_publishers("odometry/filtered") > 1:
                    raise RuntimeError("Multiple EKF output publishers; stop the other instance")
                if node.count_publishers("wheel_odom") > 1 or node.count_publishers("imu/data_raw") > 1:
                    raise RuntimeError("Multiple hardware input publishers; stop the other instance")
                ready = node.last_wheel is not None and node.last_filtered is not None
                if ready and not announced_ready:
                    print("READY: travel and heading zeroed at the first fresh EKF sample.")
                    print("Positive heading = counterclockwise; position axes are relative to startup.")
                    announced_ready = True
                if not ready and now - start > config["startup_timeout_s"]:
                    raise RuntimeError("Startup timed out: need $MSPD frames and EKF output; see ekf.log")
                events = keyboard.read()
                # A blocked I2C/terminal call must not renew an expired key lease.
                now = time.monotonic()
                if events:
                    keys.feed(events, now)
                if keys.quit:
                    break
                if not ready:
                    keys.stop()
                else:
                    # Recheck AFTER I/O: an I2C or terminal stall must not
                    # lead to a nonzero command using old measurements.
                    if (now - node.last_wheel > config["sensor_timeout_s"]
                            or now - node.last_filtered > config["sensor_timeout_s"]):
                        raise RuntimeError("Measurements became stale during I/O; stopping")
                targets = keys.targets(now)
                if targets != previous_targets or now - last_write >= 0.1:
                    board.command(*targets)
                    previous_targets, last_write = targets, now
                if now >= next_status:
                    if ready:
                        t = node.travel
                        print(f"dx={t.forward:+.3f} dy={t.left:+.3f} m | "
                              f"displacement={t.displacement:.3f} m "
                              f"travel={t.path:.3f} m | yaw={math.degrees(t.yaw_change):+.1f} deg | "
                              f"wheel L/R={node.measured_left:+.3f}/{node.measured_right:+.3f} m/s",
                              flush=True)
                    else:
                        print("Waiting for measured wheel speed and fresh EKF output...", flush=True)
                    next_status = now + 0.5
    finally:
        # Stop before shutting down ROS/child processes.
        board.stop()


def interrupted(*_):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("hardware.yaml"))
    parser.add_argument("--domain-id", type=int, default=78,
                        help="ROS domain (default 78, separate from the synthetic demo on 77)")
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error("Choose a domain ID from 0 to 101")
    if not sys.stdin.isatty():
        parser.error("Run in an interactive terminal: ssh -t swarm2")
    with args.config.open() as stream:
        config = yaml.safe_load(stream)
    validate_config(config)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["PYTHONUNBUFFERED"] = "1"
    for name in ("SIGINT", "SIGTERM", "SIGHUP", "SIGQUIT", "SIGTSTP"):
        signal.signal(getattr(signal, name), interrupted)

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from ament_index_python.packages import get_package_prefix
    executable = Path(get_package_prefix("robot_localization")) / "lib/robot_localization/ekf_node"
    if not executable.is_file():
        raise RuntimeError("Install ros-kilted-robot-localization first")
    ekf_config = Path(__file__).with_name("ekf.yaml")
    log_path = Path(__file__).with_name("ekf.log")
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        with ExitStack() as stack:
            board = stack.enter_context(MotorBoard(config))
            imu = stack.enter_context(MPU6050(config))
            print(f"ROS domain {args.domain_id}; left M{config['left_motor']}, right M{config['right_motor']}; "
                  f"nominal target {config['speed_m_s']:.3f} m/s.")
            print(f"Track width {config['track_width_m']:.3f} m. Check hardware.yaml against your chassis.")
            print(f"Keep the robot STILL for {config['gyro_calibration_s']:.1f}s gyro calibration...", flush=True)
            bias = imu.calibrate()
            print("Sensor-axis gyro bias (deg/s): " + ", ".join(f"{math.degrees(v):+.3f}" for v in bias))
            log = stack.enter_context(log_path.open("w"))
            ekf = subprocess.Popen([str(executable), "--ros-args",
                                    "-r", "__node:=ekf_filter_node",
                                    "-r", "__ns:=/hardware_demo",
                                    "--params-file", str(ekf_config)],
                                   stdin=subprocess.DEVNULL, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
            stack.callback(shutdown_ekf, ekf)
            node = make_node(config)
            stack.callback(node.destroy_node)
            # Register after all other cleanup so the first action is motor stop.
            stack.callback(board.stop)
            control_loop(node, board, imu, ekf, config)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
    except Exception as error:
        print(f"Stopped: {error}", file=sys.stderr)
        print("For ROS startup errors, inspect ekf.log beside this script.", file=sys.stderr)
        sys.exit(1)
