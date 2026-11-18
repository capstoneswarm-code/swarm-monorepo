#!/usr/bin/env python3
"""Exchange only raw map snapshots and intent; local ROS domains stay isolated."""
import argparse
import json
import math
import os
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import ProxyHandler, Request, build_opener

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException

from core import Grid, MAX_BODY, agreement, inverse, merge, near_transform, register, transform


def json_bytes(obj):
    return json.dumps(obj, allow_nan=False, separators=(',', ':')).encode()


class PeerNode(Node):
    def __init__(self, robot, peer, url, port, run_id):
        super().__init__('cooperative_peer')
        self.robot, self.peer, self.url, self.run_id = robot, peer, url.rstrip('/'), run_id
        self.owner = robot < peer
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.raw = self.packet = self.peer_grid = self.peer_packet = None
        self.raw_at = self.peer_at = self.intent_at = -math.inf
        self.peer_state = self.intent = None
        self.alignment = self.output = self.candidate = None
        self.candidate_epochs = None
        self.candidate_count = 0
        self.checked_key = None
        self.pose = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/swarm/merged_map', qos)
        self.peer_pub = self.create_publisher(String, '/swarm/peer', qos)
        self.status_pub = self.create_publisher(String, '/swarm/status', qos)
        self.raw_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.intent_sub = self.create_subscription(String, '/swarm/reservation', self.on_intent, 10)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.timer = self.create_timer(1.0, self.publish)
        self.last_status = None
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                with node.lock:
                    if self.path == '/state':
                        now = time.monotonic()
                        fresh_intent = node.intent if now-node.intent_at < 4.0 else None
                        obj = dict(protocol=1, robot=node.robot, run_id=node.run_id,
                            revision=node.packet['revision'] if node.packet else None,
                            map_age=now-node.raw_at if node.packet else None,
                            intent=fresh_intent, pose=node.pose,
                            alignment=node.alignment if node.owner else None)
                    elif self.path == '/map' and node.packet:
                        obj = dict(run_id=node.run_id, grid=node.packet)
                    else:
                        self.send_error(404)
                        return
                body = json_bytes(obj)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, *_):
                pass

        # No unauthenticated campus-facing listener. Reach via SSH forwarding.
        self.http = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.http.daemon_threads = True
        self.threads = [
            threading.Thread(target=self.http.serve_forever, daemon=True),
            threading.Thread(target=self.exchange_loop, daemon=True),
            threading.Thread(target=self.registration_loop, daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def on_map(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().error('Expected local raw map frame "map"')
            return
        try:
            q = msg.info.origin.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            a = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width).copy()
            grid = Grid(a, msg.info.resolution, msg.info.origin.position.x,
                        msg.info.origin.position.y, yaw)
            packet = grid.packet()
            with self.lock:
                self.raw, self.packet, self.raw_at = grid, packet, time.monotonic()
        except (ValueError, TypeError) as error:
            self.get_logger().error(f'Invalid raw map: {error}')

    def on_intent(self, msg):
        try:
            intent = json.loads(msg.data)
            goal = intent.get('goal')
            if intent.get('run_id') != self.run_id:
                return
            if goal is not None and (len(goal) != 3 or not all(math.isfinite(v) for v in goal)):
                return
            with self.lock:
                self.intent, self.intent_at = intent, time.monotonic()
        except (ValueError, TypeError):
            pass

    def fetch(self, endpoint):
        # Ignore proxy environment variables for the loopback SSH tunnel.
        request = Request(self.url+endpoint, headers={'Accept': 'application/json'})
        with build_opener(ProxyHandler({})).open(request, timeout=2.0) as response:
            body = response.read(MAX_BODY+1)
        if len(body) > MAX_BODY:
            raise ValueError('Peer response too large')
        return json.loads(body)

    def exchange_loop(self):
        last_error = None
        while not self.stop.is_set():
            try:
                state = self.fetch('/state')
                if state.get('protocol') != 1 or state.get('robot') != self.peer:
                    raise ValueError('Wrong peer or protocol on SSH tunnel')
                session = state.get('run_id')
                if not isinstance(session, str) or not 1 <= len(session) <= 64:
                    raise ValueError('Invalid peer run identifier')
                age = state.get('map_age')
                if age is not None and (not isinstance(age, (float, int))
                        or not math.isfinite(age) or age < 0):
                    raise ValueError('Invalid peer map age')
                if state.get('intent') is not None and not isinstance(state['intent'], dict):
                    raise ValueError('Invalid peer intent')
                with self.lock:
                    old = self.peer_state
                    need_map = state.get('revision') and (self.peer_packet is None
                        or old is None or old['run_id'] != session
                        or self.peer_packet['revision'] != state['revision'])
                packet = grid = None
                if need_map:
                    response = self.fetch('/map')
                    if response['run_id'] != session:
                        raise ValueError('Peer restarted during map transfer')
                    packet = response['grid']
                    grid = Grid.unpack(packet)
                    if packet['revision'] != state['revision']:
                        raise ValueError('Map changed during transfer; retry next cycle')
                with self.lock:
                    if old is None or old['run_id'] != session:
                        self.alignment = self.output = self.candidate = None
                        self.candidate_count = 0
                        self.checked_key = None
                        self.peer_grid = self.peer_packet = None
                    if grid is not None:
                        self.peer_grid, self.peer_packet = grid, packet
                    self.peer_state, self.peer_at = state, time.monotonic()
                last_error = None
            except Exception as error:
                message = f'Peer link unavailable: {error}'
                if message != last_error:
                    self.get_logger().warn(message)
                    last_error = message
            self.stop.wait(1.0)

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            if (self.raw is None or self.peer_grid is None or self.peer_state is None
                    or now-self.raw_at > 15.0 or now-self.peer_at > 5.0
                    or self.peer_state.get('map_age') is None
                    or self.peer_state['map_age']+now-self.peer_at > 15.0):
                return None
            epochs = {self.robot: self.run_id, self.peer: self.peer_state['run_id']}
            key = (self.run_id, self.peer_state['run_id'],
                   self.packet['revision'], self.peer_packet['revision'])
            return self.raw, self.peer_grid, epochs, key, self.peer_state

    def registration_loop(self):
        while not self.stop.is_set():
            try:
                snap = self.snapshot()
                if snap is not None:
                    local, peer, epochs, key, state = snap
                    if self.owner:
                        if key != self.checked_key:
                            result = register(local, peer)
                            with self.lock:
                                previous = self.alignment
                            if result is not None:
                                pose, quality = result
                                if previous and not near_transform(pose, previous['pose']):
                                    with self.lock:
                                        self.alignment = self.output = None
                                if previous and previous['epochs'] == epochs and near_transform(pose, previous['pose']):
                                    self.candidate_count = 2
                                elif (self.candidate and self.candidate_epochs == epochs
                                        and near_transform(pose, self.candidate)):
                                    self.candidate_count += 1
                                else:
                                    self.candidate_count = 1
                                self.candidate = pose
                                self.candidate_epochs = epochs
                                if self.candidate_count >= 2:
                                    with self.lock:
                                        self.alignment = dict(epochs=epochs, pose=pose,
                                            quality=quality, revisions=key)
                            elif previous and previous['epochs'] == epochs and agreement(local, peer, previous['pose']):
                                pass
                            else:
                                self.candidate = None
                                self.candidate_count = 0
                                with self.lock:
                                    self.alignment = self.output = None
                            self.checked_key = key
                    else:
                        incoming = state.get('alignment')
                        valid = incoming and incoming.get('epochs') == epochs
                        if valid:
                            pose = incoming.get('pose')
                            valid = (isinstance(pose, (list, tuple)) and len(pose) == 3
                                and all(math.isfinite(v) for v in pose)
                                and agreement(local, peer, inverse(pose)) is not None)
                        with self.lock:
                            self.alignment = incoming if valid else None
                            if not valid:
                                self.output = None
                    with self.lock:
                        aligned = self.alignment
                    if aligned and aligned['epochs'] == epochs:
                        pose = aligned['pose'] if self.owner else inverse(aligned['pose'])
                        # Recheck current grid evidence before any navigation use.
                        if agreement(local, peer, pose) is not None:
                            merged = merge(local, peer, pose)
                            with self.lock:
                                self.output = (key, epochs, merged, pose, time.monotonic())
                        else:
                            with self.lock:
                                self.output = self.alignment = None
            except Exception as error:
                self.get_logger().warn(f'Alignment refused: {error}')
                with self.lock:
                    self.output = self.alignment = None
            self.stop.wait(5.0)

    def publish(self):
        now = time.monotonic()
        try:
            tf = self.tf.lookup_transform('map', 'base_link', Time())
            stamp = tf.header.stamp.sec*10**9+tf.header.stamp.nanosec
            if abs(self.get_clock().now().nanoseconds-stamp) > 10**9:
                raise ValueError('Stale pose')
            p, q = tf.transform.translation, tf.transform.rotation
            pose = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
            if not all(math.isfinite(v) for v in pose):
                raise ValueError('Non-finite pose')
        except (TransformException, ValueError):
            pose = None
        snap = self.snapshot()
        with self.lock:
            self.pose = pose
            output = self.output
            fresh = self.raw is not None and now-self.raw_at <= 15.0
            aligned = bool(snap and output and output[1] == snap[2]
                           and now-output[4] < 12.0)
            grid = output[2] if aligned else self.raw
            state = self.peer_state
            info = dict(robot=self.peer, ready=False, goal=None, kind=None, separation_m=3.0)
            if state and now-self.peer_at < 5.0:
                intent = state.get('intent')
                if intent and intent.get('run_id') == state['run_id']:
                    info['exploring'] = intent.get('kind') == 'explore' and intent.get('goal') is not None
            if aligned:
                intent = state.get('intent')
                info.update(ready=True, run_id=state['run_id'], alignment=output[3])
                if intent and intent.get('run_id') == state['run_id']:
                    goal = intent.get('goal')
                    if goal is not None and len(goal) == 3 and all(math.isfinite(v) for v in goal):
                        x, y = transform(np.asarray([goal[:2]]), output[3])[0]
                        info.update(goal=[float(x), float(y), goal[2]+output[3][2]],
                                    kind=intent.get('kind'))
            mode = ('Aligned: merged map and goal coordination enabled' if aligned
                    else 'Independent: waiting for peer or distinctive overlapping maps')
        self.peer_pub.publish(String(data=json_bytes(info).decode()))
        self.status_pub.publish(String(data=mode))
        if mode != self.last_status:
            self.get_logger().info(mode)
            self.last_status = mode
        if fresh and grid is not None:
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.info.resolution = float(grid.resolution)
            msg.info.height, msg.info.width = grid.data.shape
            msg.info.origin.position.x, msg.info.origin.position.y = grid.x, grid.y
            msg.info.origin.orientation.z = math.sin(grid.yaw/2)
            msg.info.origin.orientation.w = math.cos(grid.yaw/2)
            msg.data = grid.data.ravel().tolist()
            self.map_pub.publish(msg)

    def close(self):
        self.stop.set()
        self.http.shutdown()
        self.http.server_close()
        for thread in self.threads:
            thread.join(timeout=2.5)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot', required=True, choices=['swarm1', 'swarm2'])
    p.add_argument('--peer-url', default='http://127.0.0.1:8769')
    p.add_argument('--port', type=int, default=8768)
    args = p.parse_args()
    peer = 'swarm2' if args.robot == 'swarm1' else 'swarm1'
    run_id = os.environ['SWARM_RUN_ID']
    rclpy.init(args=[])
    node = PeerNode(args.robot, peer, args.peer_url, args.port, run_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
