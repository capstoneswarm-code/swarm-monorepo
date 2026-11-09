#!/usr/bin/env python3
"""Manual or Nav2 drive, hardware measurements, ROS EKF, and relative travel."""

import argparse
from contextlib import ExitStack, nullcontext
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import secrets
import select
import signal
import subprocess
import sys
import termios
from threading import Lock, Thread
import time
import tty

import yaml

from demo_core import Keys, Travel, body_velocity, integrate, validate_config, wheel_speeds
from hardware import MotorBoard, MPU6050

NAV_CMD_TOPIC = "/hardware_demo/nav_cmd_vel"


class NavVelocity:
    """Turn fresh body-frame TwistStamped commands into bounded wheel targets."""

    def __init__(self, track_width):
        self.track_width = track_width
        self.wheels = (0.0, 0.0)
        self.expires = -math.inf
        self.timeout = 0.30

    def update(self, now, age, linear, angular):
        if (not all(math.isfinite(x) for x in (age, linear, angular))
                or not -0.05 <= age < self.timeout):
            self.wheels = (0.0, 0.0)
            self.expires = -math.inf
            return
        half = self.track_width / 2.0
        left, right = linear - angular * half, linear + angular * half
        # A common scale keeps the path curvature when applying hardware caps.
        ratios = [1.0, abs(linear) / 0.15, abs(angular) / 0.8,
                  max(abs(left), abs(right)) / 0.20]
        scale = max(ratios)
        if not math.isfinite(scale):
            self.wheels = (0.0, 0.0)
            self.expires = -math.inf
            return
        self.wheels = (left / scale, right / scale)
        # Account for time already spent in transport as well as receipt time.
        self.expires = now + self.timeout - max(age, 0.0)

    def targets(self, now):
        return self.wheels if now < self.expires else (0.0, 0.0)


DRIVE_PAGE = r"""<!doctype html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot drive</title>
<style>
body {font: 16px system-ui; max-width: 34rem; margin: 3rem auto; padding: 1rem;
      background: #222; color: #eee;}
kbd {padding: .15rem .4rem; background: #444; border-radius: .2rem;}
button {padding: .6rem 1.2rem; font: inherit;}
</style>
<h1>Robot drive</h1>
<p>Hold <kbd>W</kbd>/<kbd>S</kbd> to move or <kbd>A</kbd>/<kbd>D</kbd> to spin.
Release to slow down. Press <kbd>Space</kbd> to stop.</p>
<button id="stop">Stop</button>
<p id="state" role="status">Click this page, then hold a movement key.</p>
<script>
const token = "__CONTROL_TOKEN__";
const keys = {KeyW: "w", KeyS: "s", KeyA: "a", KeyD: "d"};
const labels = {w: "Forward", s: "Backward", a: "Spin left", d: "Spin right",
                idle: "Slowing down", stop: "Stopped"};
const state = document.getElementById("state");
let held = [], sequence = 0, press = 0, pending = 0;

async function send(command, urgent = false) {
  if (pending && !urgent) return;
  const seq = ++sequence;
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 220);
  pending++;
  try {
    const reply = await fetch("/drive", {
      method: "POST", cache: "no-store", signal: controller.signal,
      headers: {"Content-Type": "application/json", "X-Control-Token": token},
      body: JSON.stringify({seq, press, command})
    });
    if (seq !== sequence) return;
    if (reply.ok) {
      state.textContent = labels[command];
    } else {
      held = [];
      state.textContent = reply.status === 503
        ? "Wait for READY in the robot terminal, then press again."
        : "Control expired. Release the keys and press again, or reload this page.";
    }
  } catch {
    if (seq === sequence) {
      held = [];
      state.textContent = "Connection interrupted. Release the keys and press again.";
    }
  } finally {
    clearTimeout(timer);
    pending--;
  }
}

function stop() {
  held = [];
  send("stop", true);
}

window.addEventListener("keydown", event => {
  if (event.ctrlKey || event.metaKey || event.altKey) return;
  if (event.code === "Space") {
    event.preventDefault();
    if (!event.repeat) stop();
    return;
  }
  const key = keys[event.code];
  if (!key) return;
  event.preventDefault();
  if (event.repeat || held.includes(key) || document.hidden || !document.hasFocus()) return;
  held.push(key);
  press++;
  send(key);
});
window.addEventListener("keyup", event => {
  const key = keys[event.code];
  if (!held.includes(key)) return;
  event.preventDefault();
  held = held.filter(item => item !== key);
  send(held.at(-1) || "idle", true);
});
window.addEventListener("blur", stop);
window.addEventListener("pagehide", stop);
document.addEventListener("visibilitychange", () => { if (document.hidden) stop(); });
document.getElementById("stop").addEventListener("click", stop);
setInterval(() => {
  if (held.length && !document.hidden && document.hasFocus()) send(held.at(-1));
}, 50);
</script>
"""


class BrowserDrive:
    """One active browser session; keep movement valid for 300 ms at a time."""

    def __init__(self):
        self.lock = Lock()
        self.ready = False
        self.token = None
        self.seq = self.press = self.blocked_press = -1
        self.command = "stop"
        self.expires = -math.inf

    def new_session(self):
        with self.lock:
            self.token = secrets.token_urlsafe(24)
            self.seq = self.press = self.blocked_press = -1
            self.command = "stop"
            self.expires = -math.inf
            return self.token

    def enable(self):
        with self.lock:
            self.ready = True

    def expire(self, now):
        # Called with the lock held. A late heartbeat for an expired key
        # cannot re-start motion; the browser must report a fresh key press.
        if self.command in ("w", "a", "s", "d") and now >= self.expires:
            self.blocked_press = max(self.blocked_press, self.press)
            self.command = "stop"

    def current(self, now):
        with self.lock:
            self.expire(now)
            return self.command

    def submit(self, token, seq, press, command, now):
        with self.lock:
            if token is None or token != self.token or seq <= self.seq:
                return 409
            self.expire(now)
            self.seq = seq
            if command in ("stop", "idle"):
                self.blocked_press = max(self.blocked_press, self.press, press)
                # A late release must not replace an expired hard stop with a ramp.
                self.command = "stop" if self.command == "stop" else command
                return 204
            if not self.ready:
                self.blocked_press = max(self.blocked_press, press)
                return 503
            if press <= self.blocked_press or press < self.press:
                return 409
            self.command, self.press = command, press
            self.expires = now + 0.30
            return 204


def drive_handler(drive):
    class Handler(BaseHTTPRequestHandler):
        def local_host(self):
            return self.headers.get("Host", "").split(":", 1)[0] in (
                "127.0.0.1", "localhost",
            )

        def reply(self, status, body=b""):
            self.send_response(status)
            if body:
                self.send_header("Content-Type", "text/html; charset=utf-8")
            if status != 204:
                self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            try:
                self.end_headers()
                if body:
                    self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass

        def do_GET(self):
            if not self.local_host():
                self.reply(403)
            elif self.path == "/":
                page = DRIVE_PAGE.replace("__CONTROL_TOKEN__", drive.new_session())
                self.reply(200, page.encode("utf-8"))
            else:
                self.reply(404)

        def do_POST(self):
            if self.path != "/drive":
                self.reply(404)
                return
            if (not self.local_host()
                    or self.headers.get("Origin") != "http://" + self.headers["Host"]):
                self.reply(403)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 128:
                    raise ValueError
                self.connection.settimeout(0.25)
                packet = json.loads(self.rfile.read(size))
                seq, press, command = packet["seq"], packet["press"], packet["command"]
                if (type(seq) is not int or seq < 0 or type(press) is not int or press < 0
                        or command not in ("w", "s", "a", "d", "idle", "stop")):
                    raise ValueError
            except (ValueError, KeyError, TypeError, OSError):
                self.reply(400)
                return
            self.reply(drive.submit(self.headers.get("X-Control-Token"), seq, press,
                                    command, time.monotonic()))

        def log_message(self, *_):
            pass

    return Handler


def ramp_wheel(current, target, dt, accel, decel):
    # A reversal first decelerates to zero; acceleration resumes next tick.
    if current * target < 0.0:
        target = 0.0
    rate = decel if abs(target) < abs(current) else accel
    step = rate * dt
    delta = target - current
    return target if abs(delta) <= step else current + math.copysign(step, delta)


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


def make_node(config, nav2=False):
    # Delay ROS imports so the non-ROS parts can be tested independently.
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    if nav2:
        from geometry_msgs.msg import TwistStamped
        from rclpy.qos import QoSProfile, ReliabilityPolicy

    class Bridge(Node):
        def __init__(self):
            super().__init__("nav_bridge" if nav2 else "keyboard_bridge",
                             namespace="/hardware_demo")
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
            if nav2:
                self.nav_velocity = NavVelocity(config["track_width_m"])
                self.nav_sub = self.create_subscription(
                    TwistStamped, NAV_CMD_TOPIC, self.receive_nav_velocity,
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                )

        def receive_nav_velocity(self, msg):
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
            self.nav_velocity.update(time.monotonic(), age,
                                     msg.twist.linear.x, msg.twist.angular.z)

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


def control_loop(node, board, imu, ekf, config, browser=None, nav2=False):
    import rclpy
    keys = None if nav2 else Keys(config["speed_m_s"], config["key_timeout_s"])
    if browser is not None:
        speed = config["speed_m_s"]
        turn = float(config.get("turn_speed_m_s", 0.12))
        accel = float(config.get("drive_accel_m_s2", 0.6))
        decel = float(config.get("drive_decel_m_s2", 1.5))
        if not all(math.isfinite(value) and value > 0 for value in (turn, accel, decel)):
            raise ValueError("Turn speed, acceleration, and deceleration must be positive and finite")
        turn = min(turn, speed)
        motions = {"w": (speed, speed), "s": (-speed, -speed),
                   "a": (-turn, turn), "d": (turn, -turn), "idle": (0.0, 0.0)}
    start = time.monotonic()
    next_imu = next_status = start
    last_write = -math.inf
    previous_targets = None
    announced_ready = False
    if nav2:
        print(f"Nav2 mode: waiting for TwistStamped on {NAV_CMD_TOPIC}; Ctrl+C stops motors.")
    elif browser is None:
        print("W/S: forward/back | A/D: spin left/right | SPACE: stop | X/Ctrl+C: quit")
        print(f"Targets expire after {config['key_timeout_s']:.1f}s without a motion-key event.")
    else:
        print(f"Browser drive: turn speed {turn:.3f} m/s; Ctrl+C here stops the demo.")
    board.start_telemetry()
    try:
        with (Keyboard() if browser is None and not nav2 else nullcontext()) as keyboard:
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
                if nav2 and node.count_publishers(NAV_CMD_TOPIC) > 1:
                    raise RuntimeError("Multiple Nav2 velocity publishers; stopping")
                ready = node.last_wheel is not None and node.last_filtered is not None
                if ready and not announced_ready:
                    if browser is not None:
                        browser.enable()
                    print("READY: travel and heading zeroed at the first fresh EKF sample.")
                    print("Positive heading = counterclockwise; position axes are relative to startup.")
                    announced_ready = True
                if not ready and now - start > config["startup_timeout_s"]:
                    raise RuntimeError("Startup timed out: need $MSPD frames and EKF output; see ekf.log")
                if keyboard is not None:
                    events = keyboard.read()
                else:
                    time.sleep(0.005)
                    events = b""
                # A blocked I2C/terminal call must not renew an expired key lease.
                now = time.monotonic()
                if events and keys is not None:
                    keys.feed(events, now)
                if keys is not None and keys.quit:
                    break
                if not ready:
                    if keys is not None:
                        keys.stop()
                else:
                    # Recheck AFTER I/O: an I2C or terminal stall must not
                    # lead to a nonzero command using old measurements.
                    if (now - node.last_wheel > config["sensor_timeout_s"]
                            or now - node.last_filtered > config["sensor_timeout_s"]):
                        raise RuntimeError("Measurements became stale during I/O; stopping")
                if nav2:
                    targets = node.nav_velocity.targets(now) if ready else (0.0, 0.0)
                    # Nav2 smooths motion upstream. Pass zero straight through
                    # so Collision Monitor and the command watchdog can stop us.
                    if targets != previous_targets or now - last_write >= 0.1:
                        board.command(*targets)
                        previous_targets, last_write = targets, now
                elif browser is None:
                    targets = keys.targets(now)
                    if targets != previous_targets or now - last_write >= 0.1:
                        board.command(*targets)
                        previous_targets, last_write = targets, now
                else:
                    motion = browser.current(now) if ready else "stop"
                    hard_stop = motion == "stop"
                    interval = 0.05  # Motor updates at 20 Hz; hard stops bypass this.
                    if ((hard_stop and previous_targets != (0.0, 0.0))
                            or now - last_write >= interval):
                        if hard_stop:
                            targets = (0.0, 0.0)
                        else:
                            dt = min(interval, now - last_write)
                            targets = tuple(
                                ramp_wheel(current, target, dt, accel, decel)
                                for current, target in zip(
                                    previous_targets or (0.0, 0.0), motions[motion]
                                )
                            )
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
    parser.add_argument("--control-port", type=int, default=8766,
                        help="Loopback port for browser control (default 8766)")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--terminal", action="store_true",
                      help="Use the original terminal keyboard controls")
    mode.add_argument("--nav2", action="store_true",
                      help="Use only the final Nav2 velocity commands")
    args = parser.parse_args()
    if not 0 <= args.domain_id <= 101:
        parser.error("Choose a domain ID from 0 to 101")
    if not 1024 <= args.control_port <= 65535:
        parser.error("Choose a control port from 1024 to 65535")
    if args.terminal and not sys.stdin.isatty():
        parser.error("Run in an interactive terminal: ssh -t swarm2")
    with args.config.open() as stream:
        config = yaml.safe_load(stream)
    # Autonomous motion has its own limits; ignore manual speed/lease settings.
    validated = dict(config, speed_m_s=0.15, key_timeout_s=0.8) if args.nav2 else config
    validate_config(validated)
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
                  f"nominal target {validated['speed_m_s']:.3f} m/s.")
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
            node = make_node(config, args.nav2)
            stack.callback(node.destroy_node)
            browser = None
            if not args.terminal and not args.nav2:
                browser = BrowserDrive()
                server = ThreadingHTTPServer(
                    ("127.0.0.1", args.control_port), drive_handler(browser)
                )
                Thread(target=server.serve_forever, daemon=True).start()
                stack.callback(server.server_close)
                stack.callback(server.shutdown)
                print(f"Browser controls: http://127.0.0.1:{args.control_port}", flush=True)
            # Register after all other cleanup so the first action is motor stop.
            stack.callback(board.stop)
            control_loop(node, board, imu, ekf, config, browser, args.nav2)
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
