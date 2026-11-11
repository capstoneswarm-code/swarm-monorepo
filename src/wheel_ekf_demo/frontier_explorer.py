#!/usr/bin/env python3
"""Propose known-free frontier goals; let Nav2 plan and drive them."""

import argparse
import math
import time
from collections import deque

import numpy as np


# SLAM Toolbox's default grid publication interval is 5 seconds.
MAP_STALE_S = 15.0


def adjacent(mask, outside=False):
    padded = np.pad(mask, 1, constant_values=outside)
    return (padded[:-2, 1:-1] | padded[2:, 1:-1]
            | padded[1:-1, :-2] | padded[1:-1, 2:])


def large_frontiers(mask, min_cells):
    remaining = set(map(tuple, np.argwhere(mask)))
    result = np.zeros_like(mask)
    while remaining:
        stack = [remaining.pop()]
        group = []
        while stack:
            row, col = stack.pop()
            group.append((row, col))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    cell = row + dy, col + dx
                    if cell in remaining:
                        remaining.remove(cell)
                        stack.append(cell)
        if len(group) >= min_cells:
            rows, cols = zip(*group)
            result[rows, cols] = True
    return result


def footprint_clear(free, cells):
    """Require the whole circular footprint to be observed free."""
    h, w = free.shape
    padded = np.pad(free, cells, constant_values=False)
    result = free.copy()
    for dy in range(-cells, cells + 1):
        span = math.isqrt(cells * cells - dy * dy)
        row = cells + dy
        for dx in range(-span, span + 1):
            col = cells + dx
            result &= padded[row:row+h, col:col+w]
    return result


def approach_mask(grid, resolution, radius):
    free = grid == 0
    frontier = free & adjacent(grid < 0, outside=True)
    band = large_frontiers(frontier, max(3, math.ceil(0.15 / resolution)))
    if not band.any():
        return band
    # Move back from the unknown boundary through observed free cells.
    for _ in range(math.ceil((radius + 0.20) / resolution)):
        band |= adjacent(band) & free
    return band & footprint_clear(free, math.ceil((radius + 0.02) / resolution))


def world_points(rows, cols, geometry):
    resolution, ox, oy, yaw = geometry
    gx, gy = (cols + 0.5) * resolution, (rows + 0.5) * resolution
    c, s = math.cos(yaw), math.sin(yaw)
    return ox + c*gx - s*gy, oy + s*gx + c*gy


def grid_cell(x, y, geometry):
    resolution, ox, oy, yaw = geometry
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - ox, y - oy
    return math.floor((-s*dx + c*dy) / resolution), math.floor((c*dx + s*dy) / resolution)


def choose_goal(mask, geometry, robot_x, robot_y, excluded, exclusion_radius):
    rows, cols = np.nonzero(mask)
    if not len(rows):
        return None
    xs, ys = world_points(rows, cols, geometry)
    distance = np.hypot(xs - robot_x, ys - robot_y)
    eligible = (distance >= 0.30) & (distance <= 4.0)
    for x, y in excluded:
        eligible &= (xs-x)**2 + (ys-y)**2 >= exclusion_radius**2
    if not eligible.any():
        return None
    best = int(np.argmin(np.where(eligible, distance, np.inf)))
    x, y = float(xs[best]), float(ys[best])
    return x, y, math.atan2(y - robot_y, x - robot_x)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius", required=True, type=float)
    args = parser.parse_args()
    if not math.isfinite(args.radius) or not 0.03 <= args.radius <= 1.2:
        parser.error("Supply the measured footprint radius in meters")

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class Explorer(Node):
        def __init__(self):
            super().__init__("frontier_explorer")
            self.grid = self.geometry = self.mask = None
            self.map_received = -math.inf
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.map_sub = self.create_subscription(
                OccupancyGrid, "/map", self.on_map,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.plan = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
            self.navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            self.lifecycle = self.create_client(GetState, "/bt_navigator/get_state")
            self.probe = self.future = self._goal_handle = self.goal = None
            self.phase = "idle"
            self.phase_since = self.next_goal = 0.0
            self.cancel_since = None
            self.excluded = deque(maxlen=128)
            self.idle_message = None
            self.timer = self.create_timer(0.5, self.step)

        def on_map(self, msg):
            w, h, res = msg.info.width, msg.info.height, msg.info.resolution
            if (msg.header.frame_id != "map" or not w or not h
                    or not 0.01 <= res <= 0.20 or len(msg.data) != w*h):
                return
            origin = msg.info.origin
            q = origin.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            geometry = res, origin.position.x, origin.position.y, yaw
            if not all(math.isfinite(v) for v in geometry):
                return
            grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
            self.map_received = time.monotonic()
            if geometry != self.geometry or not np.array_equal(grid, self.grid):
                self.geometry, self.grid, self.mask = geometry, grid, None

        def note(self, message):
            if message != self.idle_message:
                self.get_logger().info(message)
                self.idle_message = message

        def nav_ready(self):
            if not (self.plan.server_is_ready() and self.navigate.server_is_ready()
                    and self.lifecycle.service_is_ready()):
                return False
            if self.probe is None:
                self.probe = self.lifecycle.call_async(GetState.Request())
                return False
            if not self.probe.done():
                return False
            reply = self.probe.result()
            self.probe = None
            return reply.current_state.id == State.PRIMARY_STATE_ACTIVE

        def pose(self):
            x, y, yaw = self.goal
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            return pose

        def candidates(self):
            if self.mask is None:
                self.mask = approach_mask(self.grid, self.geometry[0], args.radius)
            return self.mask

        def finish(self, now, message, exclude=True):
            if exclude:
                self.excluded.append(self.goal[:2])
            self.get_logger().info(message)
            self.future = self._goal_handle = self.goal = None
            self.phase = "idle"
            self.next_goal = now + 1.0
            self.cancel_since = self.idle_message = None

        def step(self):
            now = time.monotonic()
            fresh_map = self.grid is not None and now - self.map_received <= MAP_STALE_S
            if self.future is not None:
                self.advance(now, fresh_map)
                return
            if now < self.next_goal:
                return
            if not fresh_map:
                self.note("Waiting for a fresh live /map")
                return
            if not self.nav_ready():
                self.note("Waiting for active Nav2")
                return
            try:
                tf = self.tf.lookup_transform("map", "base_link", Time())
            except TransformException:
                self.note("Waiting for map -> base_link")
                return
            pos = tf.transform.translation
            self.goal = choose_goal(self.candidates(), self.geometry, pos.x, pos.y,
                                    self.excluded, max(0.25, args.radius * 1.5))
            if self.goal is None:
                self.note("No eligible frontier approach; parked until the map changes")
                return
            request = ComputePathToPose.Goal()
            request.goal = self.pose()
            request.planner_id = "GridBased"
            request.use_start = False
            self.future = self.plan.send_goal_async(request)
            self.phase, self.phase_since = "plan_handle", now

        def advance(self, now, fresh_map):
            if not self.future.done():
                if self.phase == "nav_result":
                    if self.cancel_since is None and (not fresh_map or now-self.phase_since > 60.0):
                        self.get_logger().warn("Canceling goal: map stale or goal timeout")
                        self._goal_handle.cancel_goal_async()
                        self.cancel_since = now
                    elif self.cancel_since is not None and now-self.cancel_since > 5.0:
                        raise RuntimeError("Navigation did not finish canceling; shutting down")
                elif now-self.phase_since > 10.0:
                    raise RuntimeError(f"Nav2 action timed out in {self.phase}; shutting down")
                return

            reply = self.future.result()
            if self.phase in ("plan_handle", "nav_handle"):
                if not reply.accepted:
                    self.finish(now, f"Nav2 rejected {self.phase}; skipping goal")
                    return
                self._goal_handle = reply
                self.future = reply.get_result_async()
                self.phase = "plan_result" if self.phase == "plan_handle" else "nav_result"
                self.phase_since = now
            elif self.phase == "plan_result":
                if reply.status != GoalStatus.STATUS_SUCCEEDED or not reply.result.path.poses:
                    self.finish(now, "No path to frontier; skipping goal")
                    return
                row, col = grid_cell(*self.goal[:2], self.geometry)
                mask = self.candidates() if fresh_map else None
                if (mask is None or not (0 <= row < mask.shape[0] and 0 <= col < mask.shape[1])
                        or not mask[row, col]):
                    self.finish(now, "Map changed while planning; rechecking", exclude=False)
                    return
                request = NavigateToPose.Goal()
                request.pose = self.pose()
                x, y, _ = self.goal
                self.get_logger().info(f"Exploring toward map ({x:.2f}, {y:.2f}) m")
                self.future = self.navigate.send_goal_async(request)
                self.phase, self.phase_since = "nav_handle", now
            else:
                message = ("Reached frontier approach" if reply.status == GoalStatus.STATUS_SUCCEEDED
                           else f"Navigation ended with status {reply.status}; skipping goal")
                self.finish(now, message)

    rclpy.init(args=[])
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
