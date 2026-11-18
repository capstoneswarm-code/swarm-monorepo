"""Small adapter for the existing explorer; no wheel/IMU/control changes."""
import json
import math
import os
import time

from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

from core import separation_mask, must_yield


class Cooperation:
    def __init__(self, node):
        self.node = node
        self.robot = os.environ['SWARM_ROBOT_ID']
        self.run_id = os.environ['SWARM_RUN_ID']
        self.peer = None
        self.peer_at = -math.inf
        self.goal = None
        self.kind = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, '/swarm/reservation', 10)
        self.subscription = node.create_subscription(String, '/swarm/peer', self.on_peer, qos)
        self.timer = node.create_timer(0.5, self.publish)

    def on_peer(self, msg):
        try:
            peer = json.loads(msg.data)
            goal = peer.get('goal')
            if peer.get('robot') not in ('swarm1', 'swarm2') or peer['robot'] == self.robot:
                return
            if goal is not None and (len(goal) != 3 or not all(math.isfinite(v) for v in goal)):
                return
            self.peer, self.peer_at = peer, time.monotonic()
        except (ValueError, TypeError):
            pass

    def active_peer(self):
        if self.peer and self.peer.get('ready') and time.monotonic()-self.peer_at < 3.0:
            return self.peer
        return None

    def peer_exploring(self):
        # Peer activity can postpone mission completion even before geometric
        # alignment; it must never be used as a spatial reservation until aligned.
        return bool(self.peer and time.monotonic()-self.peer_at < 3.0
                    and self.peer.get('exploring'))

    def publish(self):
        data = dict(robot=self.robot, run_id=self.run_id, goal=self.goal, kind=self.kind)
        self.publisher.publish(String(data=json.dumps(data, allow_nan=False)))

    def send_goal(self, client, request, explorer, **kwargs):
        p, q = request.pose.pose.position, request.pose.pose.orientation
        self.goal = [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]
        self.kind = 'home' if getattr(explorer, 'returning_home', False) else 'explore'
        self.publish()
        return client.send_goal_async(request, **kwargs)

    def release(self):
        self.goal = self.kind = None
        self.publish()

    def filter(self, mask, geometry):
        peer = self.active_peer()
        if not peer or peer.get('kind') != 'explore' or peer.get('goal') is None:
            return mask, False
        result = separation_mask(mask, geometry, peer['goal'], 3.0)
        return result, bool(mask.any() and not result.any())

    def before_step(self, explorer):
        peer = self.active_peer()
        if (self.kind == 'explore' and peer and peer.get('kind') == 'explore'
                and must_yield(self.robot, peer['robot'], self.goal, peer.get('goal'), 3.0)
                and explorer.phase == 'nav_result' and explorer._goal_handle is not None
                and explorer.cancel_since is None):
            self.node.get_logger().info('Goal conflict within 3 m: yielding to swarm1 and choosing again')
            explorer._goal_handle.cancel_goal_async()
            explorer.cancel_since = time.monotonic()
        # Existing action advancement handles the cancel acknowledgement. Never
        # issue a replacement before the old action has actually terminated.

