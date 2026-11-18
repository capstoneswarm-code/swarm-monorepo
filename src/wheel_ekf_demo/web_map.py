#!/usr/bin/env python3
"""Loopback web map with map-frame robot pose, heading, goal, and status.

Run from ~/Documents/swarm-mono with Kilted sourced. Forward TCP 8765 over SSH.
Publishes /robot_pose (geometry_msgs/PoseStamped) at 5 Hz while TF is fresh.
Map PNGs update at most once a second and transfer only when the grid changes.
"""

import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import struct
from threading import Lock, Thread
import time
import zlib
from uuid import uuid4

import numpy as np


PAGE = r'''<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Swarm live map</title>
<style>
body{margin:0;background:#222;color:#eee;font:15px system-ui}
header{padding:12px 18px} #readout{font-variant-numeric:tabular-nums}
canvas{display:block;max-width:96vw;max-height:84vh;width:auto;height:auto;
margin:auto;background:#7f7f7f;image-rendering:pixelated}
small{color:#bbb} .blue{color:#4dabff}.orange{color:#ffb347}
</style>
<header><b>Live map</b> · <span class="blue">● robot and heading</span> ·
<span class="orange">× active goal</span><br>
<span id="readout">Waiting for map and map → base_link…</span><br>
<small id="status"></small></header>
<canvas id="map" width="700" height="450"></canvas>
<script>
const canvas=document.getElementById('map'), ctx=canvas.getContext('2d');
const readout=document.getElementById('readout'),status=document.getElementById('status');
let image=null, meta=null, busy=false, trail=[];
let runId=null, viewerId=null;
function pixel(x,y){
 const dx=x-meta.ox,dy=y-meta.oy,c=Math.cos(meta.yaw),s=Math.sin(meta.yaw);
 return [(c*dx+s*dy)/meta.resolution,meta.height-(-s*dx+c*dy)/meta.resolution];
}
function draw(state){
 if(!image||!meta)return;
 ctx.clearRect(0,0,canvas.width,canvas.height);ctx.drawImage(image,0,0);
 const unit=Math.max(1,meta.width/850);
 if(trail.length>1){
  ctx.beginPath();trail.forEach((p,i)=>{const a=pixel(p[0],p[1]);
   i?ctx.lineTo(...a):ctx.moveTo(...a);});
  ctx.strokeStyle='#197bc5';ctx.lineWidth=unit;ctx.stroke();
 }
 const g=state.goal;
 if(g&&state.status==='Navigating'){
  const [x,y]=pixel(g.x,g.y),r=6*unit;
  ctx.strokeStyle='#ff9500';ctx.lineWidth=2*unit;ctx.beginPath();
  ctx.moveTo(x-r,y-r);ctx.lineTo(x+r,y+r);ctx.moveTo(x-r,y+r);ctx.lineTo(x+r,y-r);ctx.stroke();
 }
 if(state.pose){
  const p=state.pose,[x,y]=pixel(p.x,p.y),r=Math.max(state.radius/meta.resolution,5*unit);
  const yaw=p.yaw-meta.yaw;
  ctx.fillStyle='#168de2';ctx.strokeStyle='#053d68';ctx.lineWidth=1.5*unit;
  ctx.beginPath();ctx.arc(x,y,r,0,2*Math.PI);ctx.fill();ctx.stroke();
  ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+2*r*Math.cos(yaw),y-2*r*Math.sin(yaw));
  ctx.strokeStyle='#003559';ctx.lineWidth=2*unit;ctx.stroke();
 }
}
async function update(){
 if(busy)return;busy=true;
 try{
  const state=await (await fetch('/state',{cache:'no-store'})).json();
  if(state.run_id!==runId || state.viewer_id!==viewerId){
   trail=[];
   if(state.viewer_id!==viewerId){meta=null;image=null;}
   runId=state.run_id;viewerId=state.viewer_id;
  }
  if(state.version && (!meta||state.version>meta.version)){
   const map=await (await fetch('/map',{cache:'no-store'})).json();
   const next=new Image();next.src='data:image/png;base64,'+map.png;
   await next.decode();meta=map;image=next;
   canvas.width=meta.width;canvas.height=meta.height;
  }
  if(state.pose){
   const p=state.pose,last=trail.at(-1);
   if(!last||Math.hypot(last[0]-p.x,last[1]-p.y)>.08){
    trail.push([p.x,p.y]);if(trail.length>500)trail.shift();
   }
   readout.textContent=`map: x=${p.x.toFixed(2)} m, y=${p.y.toFixed(2)} m, heading=${(p.yaw*180/Math.PI).toFixed(1)}°`;
  }else readout.textContent='Robot pose unavailable or stale; waiting for fresh map → base_link';
  status.textContent=state.status||'Viewer active; waiting for exploration status';
  draw(state);
 }catch(e){readout.textContent='Viewer connection interrupted';}
 finally{busy=false;}
}
setInterval(update,200);update();
</script>'''


def png_grid(grid):
    """A small grayscale PNG encoder; no plotting library required."""
    pixels = np.where(grid < 0, 127, 255 - np.clip(grid.astype(np.int16), 0, 100)*255//100).astype(np.uint8)
    h, w = pixels.shape
    rows = np.zeros((h, w+1), dtype=np.uint8)
    rows[:, 1:] = pixels[::-1]
    def chunk(kind, data):
        return struct.pack('!I', len(data)) + kind + data + struct.pack('!I', zlib.crc32(kind+data)&0xffffffff)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('!2I5B', w, h, 8, 0, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(rows.tobytes(), 2)) + chunk(b'IEND', b''))


class Shared:
    def __init__(self, radius):
        self.lock = Lock()
        self.radius, self.version = radius, 0
        self.pose = self.goal = self.map_json = None
        self.status = ''
        self.run_id = ''
        self.viewer_id = str(uuid4())

    def state(self):
        with self.lock:
            return json.dumps(dict(version=self.version, radius=self.radius,
                pose=self.pose, goal=self.goal, status=self.status,
                run_id=self.run_id, viewer_id=self.viewer_id), allow_nan=False).encode()


def handler_for(shared):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_GET(self):
            path = self.path.split('?', 1)[0]
            code, kind = 200, 'application/json'
            if path == '/':
                body, kind = PAGE.encode(), 'text/html; charset=utf-8'
            elif path == '/state':
                body = shared.state()
            elif path == '/map':
                with shared.lock:
                    body = shared.map_json
                if body is None:
                    code, body = 503, b'{"error":"Waiting for map"}'
            else:
                code, body = 404, b'{"error":"Not found"}'
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--radius', type=float, default=0.14, help='Actual chassis radius, m; display only')
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535 or not math.isfinite(args.radius) or args.radius <= 0:
        parser.error('Use a valid port and positive radius')
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import String
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from rclpy.executors import ExternalShutdownException
    from tf2_ros import Buffer, TransformException, TransformListener

    shared = Shared(args.radius)

    class Viewer(Node):
        def __init__(self):
            super().__init__('swarm_map_viewer')
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
            self.pending = None
            self.previous_grid = self.previous_geometry = None
            self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.on_map,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            display_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.goal_sub = self.create_subscription(PoseStamped, '/exploration/goal', self.on_goal, display_qos)
            self.status_sub = self.create_subscription(String, '/exploration/status', self.on_status, display_qos)
            self.pose_after_ns = 0
            self.run_sub = self.create_subscription(String, '/navigation/run_id', self.on_run, display_qos)
            self.pose_timer = self.create_timer(0.2, self.publish_pose)
            self.map_timer = self.create_timer(1.0, self.render_map)

        def on_map(self, msg):
            if (msg.header.frame_id == 'map' and msg.info.width and msg.info.height
                    and len(msg.data) == msg.info.width*msg.info.height):
                self.pending = msg

        def on_goal(self, msg):
            if msg.header.frame_id == 'map':
                with shared.lock:
                    shared.goal = dict(x=msg.pose.position.x, y=msg.pose.position.y)

        def on_run(self, msg):
            with shared.lock:
                if shared.run_id == msg.data:
                    return
                shared.run_id = msg.data
                shared.pose = shared.goal = None
                shared.status = 'New run: waiting for fresh pose'
            self.pose_after_ns = self.get_clock().now().nanoseconds

        def on_status(self, msg):
            with shared.lock:
                shared.status = msg.data

        def publish_pose(self):
            try:
                tf = self.tf.lookup_transform('map', 'base_link', Time())
                stamp = tf.header.stamp.sec*10**9 + tf.header.stamp.nanosec
                if stamp < self.pose_after_ns:
                    raise ValueError('Waiting for pose from the new run')
                if abs((self.get_clock().now().nanoseconds-stamp)*1e-9) > 1.0:
                    raise ValueError('Stale map pose')
                q, p = tf.transform.rotation, tf.transform.translation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                if not all(math.isfinite(v) for v in (p.x,p.y,yaw)):
                    raise ValueError('Invalid map pose')
                msg = PoseStamped()
                msg.header = tf.header
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = p.x, p.y, p.z
                msg.pose.orientation = q
                self.pose_pub.publish(msg)
                with shared.lock:
                    shared.pose = dict(x=p.x, y=p.y, yaw=yaw)
            except (TransformException, ValueError):
                with shared.lock:
                    shared.pose = None

        def render_map(self):
            if self.pending is None:
                return
            msg, self.pending = self.pending, None
            info = msg.info
            p, q = info.origin.position, info.origin.orientation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            geometry = info.resolution, p.x, p.y, yaw
            if info.resolution <= 0 or not all(math.isfinite(v) for v in geometry):
                return
            grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
            if geometry == self.previous_geometry and np.array_equal(grid, self.previous_grid):
                return
            png = base64.b64encode(png_grid(grid)).decode()
            with shared.lock:
                version = shared.version+1
                shared.map_json = json.dumps(dict(version=version, width=info.width,
                    height=info.height, resolution=info.resolution, ox=p.x, oy=p.y,
                    yaw=yaw, png=png), allow_nan=False).encode()
                shared.version = version
            self.previous_grid, self.previous_geometry = grid, geometry

    rclpy.init(args=[])
    node = Viewer()
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler_for(shared))
    server.daemon_threads = True
    worker = Thread(target=server.serve_forever, daemon=True)
    worker.start()
    print(f'Map viewer: http://127.0.0.1:{args.port}; publishing /robot_pose at 5 Hz', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
