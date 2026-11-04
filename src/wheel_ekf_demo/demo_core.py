"""Hardware-independent protocol, kinematics, controls, and reporting."""

import math


def validate_config(config):
    for name in ("left_motor", "right_motor"):
        if type(config[name]) is not int or config[name] not in (1, 2, 3, 4):
            raise ValueError(f"{name} must be an integer from 1 to 4")
    if config["left_motor"] == config["right_motor"]:
        raise ValueError("Left and right motors must use different channels")
    for name in ("left_command_sign", "right_command_sign",
                 "left_encoder_sign", "right_encoder_sign"):
        if config[name] not in (-1, 1):
            raise ValueError(f"{name} must be +1 or -1")
    for name in ("track_width_m", "speed_m_s", "key_timeout_s",
                 "sensor_timeout_s", "startup_timeout_s", "gyro_calibration_s",
                 "left_speed_scale", "right_speed_scale", "wheel_speed_std_m_s",
                 "lateral_speed_std_m_s", "wheel_yaw_rate_std_rad_s", "gyro_std_rad_s"):
        value = config[name]
        if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be a positive finite number")
    if not 0.1 <= config["key_timeout_s"] <= 2.0:
        raise ValueError("key_timeout_s must be between 0.1 and 2.0")
    if not 0.05 <= config["sensor_timeout_s"] <= 1.0:
        raise ValueError("sensor_timeout_s must be between 0.05 and 1.0")
    angles = config["imu_mount_rpy_deg"]
    if len(angles) != 3 or not all(math.isfinite(x) for x in angles):
        raise ValueError("imu_mount_rpy_deg must contain three finite angles")
    if config["imu_address"] not in (0x68, 0x69):
        raise ValueError("MPU6050 address must be 0x68 or 0x69")
    if config["report_speed_deadband_m_s"] < 0:
        raise ValueError("Reporting speed deadband cannot be negative")


class BoardParser:
    """Parse fragmented/coalesced serial frames, without assuming read lines."""
    def __init__(self):
        self.buffer = b""

    def feed(self, data):
        self.buffer += data
        frames = []
        while b"#" in self.buffer:
            raw, self.buffer = self.buffer.split(b"#", 1)
            raw = raw[raw.rfind(b"$"):] if b"$" in raw else b""
            if not raw.startswith(b"$MSPD:"):
                continue
            try:
                values = tuple(float(x) for x in raw[6:].split(b","))
                if len(values) == 4 and all(math.isfinite(x) for x in values):
                    frames.append(values)
            except ValueError:
                continue
        # Bound memory even if the cable returns arbitrary unterminated text.
        self.buffer = self.buffer[-4096:]
        return frames


def wheel_speeds(values, config):
    left = (values[config["left_motor"] - 1] / 1000.0
            * config["left_encoder_sign"] * config["left_speed_scale"])
    right = (values[config["right_motor"] - 1] / 1000.0
             * config["right_encoder_sign"] * config["right_speed_scale"])
    if max(abs(left), abs(right)) > 2.0:
        raise ValueError("Implausible measured wheel speed (>2 m/s); check telemetry")
    return left, right


def body_velocity(left, right, track_width):
    return (left + right) / 2.0, (right - left) / track_width


def speed_command(left, right, config):
    values = [0, 0, 0, 0]
    values[config["left_motor"] - 1] = round(left * 1000 * config["left_command_sign"])
    values[config["right_motor"] - 1] = round(right * 1000 * config["right_command_sign"])
    return ("$spd:" + ",".join(map(str, values)) + "#").encode("ascii")


class Keys:
    def __init__(self, speed, timeout):
        self.speed, self.timeout = speed, timeout
        self.left = self.right = 0.0
        self.last_motion = -math.inf
        self.quit = False
        self.escape = False

    def stop(self):
        self.left = self.right = 0.0
        self.last_motion = -math.inf

    def feed(self, data, now):
        # Expire previous commands BEFORE a new key can renew the lease.
        self.targets(now)
        for byte in data:
            if self.escape:
                # Consume an ANSI CSI/SS3 key sequence, including arrow keys.
                if 0x40 <= byte <= 0x7e and byte not in (ord("["), ord("O")):
                    self.escape = False
                continue
            if byte == 27:
                self.stop()
                self.escape = True
                continue
            char = chr(byte).lower()
            if char in ("x", "\x03", "\x04"):
                self.stop()
                self.quit = True
                return
            if char == "w":
                self.left, self.right = self.speed, -self.speed
            elif char == "s":
                self.left, self.right = -self.speed, self.speed
            elif char == "a":
                self.left, self.right = -self.speed, -self.speed
            elif char == "d":
                self.left, self.right = self.speed, self.speed
            else:
                self.stop()
                continue
            self.last_motion = now

    def targets(self, now):
        if now - self.last_motion > self.timeout:
            self.stop()
        return self.left, self.right


def integrate(pose, velocity, yaw_rate, dt):
    x, y, yaw = pose
    angle = yaw + yaw_rate * dt
    if abs(yaw_rate) < 1e-9:
        x += velocity * math.cos(yaw) * dt
        y += velocity * math.sin(yaw) * dt
    else:
        x += velocity / yaw_rate * (math.sin(angle) - math.sin(yaw))
        y -= velocity / yaw_rate * (math.cos(angle) - math.cos(yaw))
    return x, y, math.atan2(math.sin(angle), math.cos(angle))


def rotate_sensor(vector, rpy_deg):
    """Rotate a sensor-frame vector into the body with Rz * Ry * Rx."""
    roll, pitch, yaw = map(math.radians, rpy_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, y, z = vector
    y, z = cr * y - sr * z, sr * y + cr * z
    x, z = cp * x + sp * z, -sp * x + cp * z
    return cy * x - sy * y, sy * x + cy * y, z


class Travel:
    """Relative EKF pose, unwrapped heading, and velocity-integrated travel."""
    def __init__(self, deadband):
        self.deadband = deadband
        self.origin = self.previous = None
        self.yaw_change = self.path = self.signed_distance = 0.0
        self.forward = self.left = self.displacement = 0.0

    def update(self, stamp, x, y, yaw, vx, vy):
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw, vx, vy)):
            raise ValueError("Non-finite EKF output")
        speed = math.hypot(vx, vy)
        speed = speed if speed >= self.deadband else 0.0
        signed = vx if abs(vx) >= self.deadband else 0.0
        sample = (stamp, yaw, speed, signed)
        if self.origin is None:
            self.origin = (x, y, yaw)
            self.previous = sample
            return True
        dt = stamp - self.previous[0]
        if dt <= 0:
            return False
        if dt > 1.0:
            raise ValueError("EKF output time jumped or paused for >1 second")
        angle = yaw - self.previous[1]
        self.yaw_change += math.atan2(math.sin(angle), math.cos(angle))
        self.path += 0.5 * (speed + self.previous[2]) * dt
        self.signed_distance += 0.5 * (signed + self.previous[3]) * dt
        dx, dy = x - self.origin[0], y - self.origin[1]
        cy, sy = math.cos(self.origin[2]), math.sin(self.origin[2])
        self.forward, self.left = cy * dx + sy * dy, -sy * dx + cy * dy
        self.displacement = math.hypot(dx, dy)
        self.previous = sample
        return True
