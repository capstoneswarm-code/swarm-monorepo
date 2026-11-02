#!/usr/bin/env python3
import time

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

RADIUS = 6  # Visible distance from the sensor, in meters.

rclpy.init()
node = rclpy.create_node("lidar_view")
fig, ax = plt.subplots(figsize=(6, 6))
ax.set(xlim=(-RADIUS, RADIUS), ylim=(-RADIUS, RADIUS),
       aspect="equal", xlabel="x [m]", ylabel="y [m]", title="Live LiDAR: /scan")
ax.grid(True)
ax.plot(0, 0, "r+")
points, = ax.plot([], [], ".", markersize=2)
status = ax.text(0.02, 0.98, "Waiting for /scan", transform=ax.transAxes, va="top")
last_scan = None


def on_scan(msg):
    global last_scan
    r = np.asarray(msg.ranges)
    angles = msg.angle_min + np.arange(r.size) * msg.angle_increment
    valid = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    points.set_data(r[valid] * np.cos(angles[valid]), r[valid] * np.sin(angles[valid]))
    status.set_text(f"{valid.sum()} valid returns | frame: {msg.header.frame_id}")
    last_scan = time.monotonic()


subscription = node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)


def update(_):
    if not rclpy.ok():
        plt.close(fig)
        return points, status
    rclpy.spin_once(node, timeout_sec=0)
    if last_scan is not None and time.monotonic() - last_scan > 2:
        points.set_data([], [])
        status.set_text("No new scan for more than 2 seconds")
    return points, status


animation = FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)
try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
