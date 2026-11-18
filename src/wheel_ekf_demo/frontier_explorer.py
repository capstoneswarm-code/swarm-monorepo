#!/usr/bin/env python3
"""Propose known-free frontier goals; let Nav2 plan and drive them."""

import argparse
import math
from pathlib import Path
import time
from collections import deque

import numpy as np
from scipy import ndimage as ndi


# SLAM Toolbox's default grid publication interval is 5 seconds.
MAP_STALE_S = 15.0
POSE_STALE_S = 1.0
PREFERRED_GOAL_M = 3.0
RETRY_AFTER_S = 60.0


class StopWindow:
    """Treat brief releases as part of the same block until clear for 1 s."""

    def __init__(self):
        self.stopped = False
        self.blocked_since = self.clear_since = None

    def start_goal(self, now):
        self.blocked_since = now if self.stopped else None
        self.clear_since = None

    def observe(self, now, stopped):
        if stopped:
            if (self.blocked_since is None
                    or (self.clear_since is not None and now - self.clear_since >= 1.0)):
                self.blocked_since = now
            self.clear_since = None
        elif self.stopped:
            self.clear_since = now
        self.stopped = stopped

    def blocked(self, now):
        if self.clear_since is not None and now - self.clear_since >= 1.0:
            self.blocked_since = self.clear_since = None
        return self.blocked_since is not None and now - self.blocked_since >= 3.0


def adjacent(mask, outside=False):
    padded = np.pad(mask, 1, constant_values=outside)
    return (padded[:-2, 1:-1] | padded[2:, 1:-1]
            | padded[1:-1, :-2] | padded[1:-1, 2:])


def large_frontiers(mask, min_cells):
    labels, _ = ndi.label(mask, structure=np.ones((3, 3), dtype=bool))
    counts = np.bincount(labels.ravel())
    keep = counts >= min_cells
    keep[0] = False
    return keep[labels]


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


def choose_goal(mask, geometry, robot_x, robot_y, excluded, exclusion_radius,
                robot_yaw=0.0):
    rows, cols = np.nonzero(mask)
    if not len(rows):
        return None

    xs, ys = world_points(rows, cols, geometry)
    distance = np.hypot(xs - robot_x, ys - robot_y)
    eligible = np.isfinite(distance) & (distance >= 0.50)

    for x, y in excluded:
        eligible &= (xs - x)**2 + (ys - y)**2 >= exclusion_radius**2

    if not eligible.any():
        return None

    bearings = np.arctan2(ys - robot_y, xs - robot_x)
    # Prefer nearby goals, with a penalty for turning or reversing.
    score = distance + 0.65 * (1 - np.cos(bearings - robot_yaw))

    best = int(np.argmin(np.where(eligible, score, np.inf)))
    return float(xs[best]), float(ys[best]), float(bearings[best])


def costmap_reachability(costs, geometry, x, y):
    row, col = grid_cell(x, y, geometry)
    if not (0 <= row < costs.shape[0] and 0 <= col < costs.shape[1]):
        return None, None
    walkable = costs < 253
    # NavFn also clears its single starting cell; leave surrounding cells alone.
    walkable[row, col] = True
    labels, _ = ndi.label(walkable, structure=ndi.generate_binary_structure(2, 1))
    return labels == labels[row, col], int(costs[row, col])


def connected_candidates(mask, geometry, reachable, cost_geometry):
    rows, cols = np.nonzero(mask)
    result = np.zeros_like(mask)
    if not len(rows):
        return result
    xs, ys = world_points(rows, cols, geometry)
    resolution, ox, oy, yaw = cost_geometry
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = xs - ox, ys - oy
    cr = np.floor((-s*dx + c*dy)/resolution).astype(int)
    cc = np.floor((c*dx + s*dy)/resolution).astype(int)
    valid = ((cr >= 0) & (cc >= 0) & (cr < reachable.shape[0]) & (cc < reachable.shape[1]))
    index = np.flatnonzero(valid)
    index = index[reachable[cr[index], cc[index]]]
    result[rows[index], cols[index]] = True
    return result


def arrival_heading(path, fallback):
    """Face along the last part of the planned approach, even after a detour."""
    end = path[-1].pose.position
    for waypoint in reversed(path[:-1]):
        pos = waypoint.pose.position
        dx, dy = end.x - pos.x, end.y - pos.y
        if math.hypot(dx, dy) >= 0.20:
            return math.atan2(dy, dx)
    return fallback


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--radius", required=True, type=float)
    args = parser.parse_args()
    if not math.isfinite(args.radius) or not 0.03 <= args.radius <= 1.2:
        parser.error("Supply the planning footprint radius in meters")

    import rclpy
    from action_msgs.msg import GoalStatus
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState, Costmap
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class Explorer(Node):
        def __init__(self):
            super().__init__("frontier_explorer")
            self.home = None
            self.no_frontier_since = None
            self.home_requested = False
            self.returning_home = False
            self.home_attempts = 0
            self.mission_done = False
            self.grid = self.geometry = self.mask = None
            self.map_received = -math.inf
            self.costs = self.cost_geometry = self.reach_cache = None
            self.cost_received = -math.inf
            self.cost_revision = 0
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.map_sub = self.create_subscription(
                OccupancyGrid, "/map", self.on_map,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
            self.stop_window = StopWindow()
            self.cost_sub = self.create_subscription(
                Costmap, "/global_costmap/costmap_raw", self.on_costmap,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.stop_sub = self.create_subscription(
                CollisionMonitorState, "/collision_monitor_state", self.on_stop_state, 10)
            display_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.goal_pub = self.create_publisher(PoseStamped, "/exploration/goal", display_qos)
            self.status_pub = self.create_publisher(String, "/exploration/status", display_qos)
            self.goal_budget_s = 60.0
            self.last_ready = -math.inf
            self.navigate = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            self.lifecycle = self.create_client(GetState, "/bt_navigator/get_state")
            self.probe = self.future = self._goal_handle = self.goal = None
            self.phase = "idle"
            self.phase_since = self.next_goal = 0.0
            self.cancel_since = None
            self.excluded = deque(maxlen=128)
            self.rejected = deque(maxlen=128)
            self.idle_message = None
            self.timer = self.create_timer(0.25, self.step)

        def on_stop_state(self, msg):
            self.stop_window.observe(time.monotonic(), msg.action_type == CollisionMonitorState.STOP)

        def on_costmap(self, msg):
            info = msg.metadata
            if (msg.header.frame_id != "map" or not info.size_x or not info.size_y
                    or len(msg.data) != info.size_x*info.size_y or info.resolution <= 0):
                return
            q = info.origin.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            geometry = info.resolution, info.origin.position.x, info.origin.position.y, yaw
            if not all(math.isfinite(v) for v in geometry):
                return
            grid = np.asarray(msg.data, dtype=np.uint8).reshape(info.size_y, info.size_x)
            self.cost_received = time.monotonic()
            if geometry != self.cost_geometry or not np.array_equal(grid, self.costs):
                self.costs, self.cost_geometry = grid, geometry
                self.cost_revision += 1
                self.reach_cache = None

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
                self.status_pub.publish(String(data=message))

        def nav_ready(self):
            if not (self.navigate.server_is_ready() and self.lifecycle.service_is_ready()):
                return False
            if time.monotonic() - self.last_ready < 2.0:
                return True
            if self.probe is None:
                self.probe = self.lifecycle.call_async(GetState.Request())
                return False
            if not self.probe.done():
                return False
            reply = self.probe.result()
            self.probe = None
            ready = reply.current_state.id == State.PRIMARY_STATE_ACTIVE
            if ready:
                self.last_ready = time.monotonic()
            return ready

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

        def live_pose(self):
            try:
                tf = self.tf.lookup_transform("map", "base_link", Time())
            except TransformException:
                return None
            stamp = tf.header.stamp.sec * 1_000_000_000 + tf.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
            return tf if abs(age) <= POSE_STALE_S else None
        
        def no_frontier(self, now, x, y):
            if self.no_frontier_since is None:
                self.no_frontier_since = now

            if now - self.no_frontier_since < 10.0:
                self.note("No eligible frontier; waiting before returning home")
                return

            self.home_requested = True
            self.start_home(now, x, y)

        def start_home(self, now, x, y):
            if self.home is None:
                self.note("Cannot return home: starting pose was not recorded")
                return

            if self.home_attempts >= 3:
                self.note(
                    "Return home failed after 3 attempts; parked"
                )
                return

            tree = Path(__file__).with_name("home_nav.xml").resolve()
            if not tree.is_file():
                raise RuntimeError(f"Missing home behavior tree: {tree}")

            self.goal = self.home
            request = NavigateToPose.Goal()
            request.pose = self.pose()
            request.behavior_tree = str(tree)

            self.goal_budget_s = max(
                90.0,
                30.0 + math.hypot(self.home[0] - x, self.home[1] - y) / 0.10,
            )

            self.home_attempts += 1
            self.returning_home = True

            self.get_logger().info(
                f"Returning home: x={self.home[0]:.2f}, "
                f"y={self.home[1]:.2f}, "
                f"yaw={math.degrees(self.home[2]):.1f} deg "
                f"(attempt {self.home_attempts}/3)"
            )

            # Keep the existing viewer's active-goal marker working.
            self.status_pub.publish(String(data="Navigating"))
            self.goal_pub.publish(request.pose)

            self.stop_window.start_goal(now)
            self.future = self.navigate.send_goal_async(request)
            self.phase, self.phase_since = "nav_handle", now

        def finish(self, now, message, *, reached=False, retry=True):
            if self.returning_home:
                self.returning_home = False
                self.future = self._goal_handle = self.goal = None
                self.phase = "idle"
                self.cancel_since = None

                if reached:
                    self.mission_done = True
                    message = "Returned to starting position and orientation; parked"
                else:
                    message = (
                        "Return home unsuccessful; retrying in 10 seconds"
                        if self.home_attempts < 3
                        else "Return home failed after 3 attempts; parked"
                    )

                self.next_goal = now + 10.0
                self.idle_message = None
                self.note(message)
                return
            if reached:
                self.excluded.append((*self.goal[:2], now + 120.0))
            elif retry:
                self.rejected.append((*self.goal[:2], now + RETRY_AFTER_S))
            self.get_logger().info(message)
            self.status_pub.publish(String(data=message))
            self.future = self._goal_handle = self.goal = None
            self.phase = "idle"
            self.next_goal = now + (2.0 if self.cancel_since is not None else 0.1)
            self.cancel_since = self.idle_message = None

        def step(self):
            if self.mission_done:
                return
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
            tf = self.live_pose()
            if tf is None:
                self.note("Waiting for a fresh map -> base_link")
                return
            pos = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            if self.home is None:
                self.home = (pos.x, pos.y, yaw)
                self.get_logger().info(
                    f"Saved starting pose: x={pos.x:.2f}, "
                    f"y={pos.y:.2f}, yaw={math.degrees(yaw):.1f} deg"
                )

            if self.home_requested:
                self.start_home(now, pos.x, pos.y)
                return
            if self.costs is None or now - self.cost_received > 5.0:
                self.note("Waiting for a fresh global costmap")
                return
            key = (self.cost_revision, *grid_cell(pos.x, pos.y, self.cost_geometry))
            if self.reach_cache is None or self.reach_cache[0] != key:
                reachable, start_cost = costmap_reachability(
                    self.costs, self.cost_geometry, pos.x, pos.y)
                self.reach_cache = key, reachable, start_cost
            _, reachable, start_cost = self.reach_cache
            if reachable is None:
                self.note("Robot pose is outside the global costmap")
                return
            mask = connected_candidates(self.candidates(), self.geometry, reachable, self.cost_geometry)
            if not mask.any():
                self.no_frontier(now, pos.x, pos.y)
                return
            while self.rejected and self.rejected[0][2] <= now:
                self.rejected.popleft()
            while self.excluded and self.excluded[0][2] <= now:
                self.excluded.popleft()
            excluded = [(x, y) for x, y, _ in self.excluded]
            excluded += [(x, y) for x, y, _ in self.rejected]
            self.goal = choose_goal(mask, self.geometry, pos.x, pos.y, excluded,
                                    max(0.8, args.radius * 1.5), yaw)
            if self.goal is None:
                self.no_frontier(now, pos.x, pos.y)
                return
            self.no_frontier_since = None
            request = NavigateToPose.Goal()
            request.pose = self.pose()
            tree = Path(__file__).with_name("exploration_nav.xml")
            if not tree.is_file():
                raise RuntimeError("Missing exploration_nav.xml: run the update installer first")
            request.behavior_tree = str(tree)
            self.goal_budget_s = max(60.0, 20.0 + math.hypot(
                self.goal[0]-pos.x, self.goal[1]-pos.y)/0.10)
            self.get_logger().info(f"Exploring toward map ({self.goal[0]:.2f}, {self.goal[1]:.2f}) m")
            self.status_pub.publish(String(data="Navigating"))
            self.goal_pub.publish(request.pose)
            self.stop_window.start_goal(now)
            self.future = self.navigate.send_goal_async(request)
            self.phase, self.phase_since = "nav_handle", now

        def advance(self, now, fresh_map):
            if not self.future.done():
                if self.phase == "nav_result":
                    if self.cancel_since is not None:
                        if now-self.cancel_since > 5.0:
                            raise RuntimeError("Navigation did not finish canceling; shutting down")
                    else:
                        reason = None
                        if not fresh_map or self.live_pose() is None:
                            reason = "map or map pose stale"
                        elif self.stop_window.blocked(now):
                            reason = "repeated Collision Monitor stops"
                        elif now-self.phase_since > self.goal_budget_s:
                            reason = "goal timeout"
                        if reason:
                            self.get_logger().warn(f"Canceling goal: {reason}")
                            self._goal_handle.cancel_goal_async()
                            self.cancel_since = now
                elif now-self.phase_since > 10.0:
                    raise RuntimeError(f"Nav2 action timed out in {self.phase}; shutting down")
                return

            reply = self.future.result()
            if self.phase == "nav_handle":
                if not reply.accepted:
                    self.finish(now, f"Nav2 rejected {self.phase}; skipping goal")
                    return
                self._goal_handle = reply
                self.future = reply.get_result_async()
                self.phase = "nav_result"
                self.phase_since = now
            else:
                reached = reply.status == GoalStatus.STATUS_SUCCEEDED
                message = ("Reached frontier approach" if reached
                           else f"Navigation ended with status {reply.status}; skipping goal for 60 s")
                self.finish(now, message, reached=reached)

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
