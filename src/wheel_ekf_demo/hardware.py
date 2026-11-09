"""Small drivers for the Yahboom serial motor board and GPIO MPU6050."""

import math
import statistics
import struct
import sys
import time

from demo_core import BoardParser, rotate_sensor, speed_command


class MotorBoard:
    def __init__(self, config, serial_factory=None):
        if serial_factory is None:
            from serial import Serial
            serial_factory = Serial
        self.config = config
        self.parser = BoardParser()
        self.port = serial_factory(config["serial_port"], config["baudrate"],
                                   timeout=0, write_timeout=0.1, exclusive=True)
        try:
            self.stop()
            self.write(b"$upload:0,0,0#")
        except BaseException:
            self.close()
            raise

    def write(self, data):
        if self.port.write(data) != len(data):
            raise OSError("Incomplete motor serial write")

    def stop(self):
        # Release outputs rather than keeping a zero-speed PID active.
        self.write(b"$pwm:0,0,0,0#")

    def start_telemetry(self):
        self.port.reset_input_buffer()
        self.parser = BoardParser()
        self.write(b"$upload:0,0,1#")  # Encoder-derived wheel speed, mm/s.

    def command(self, left, right):
        if left == 0.0 and right == 0.0:
            self.stop()
        else:
            self.write(speed_command(left, right, self.config))

    def speeds(self):
        size = min(self.port.in_waiting, 16384)
        return self.parser.feed(self.port.read(size)) if size else []

    def close(self):
        if not self.port.is_open:
            return
        try:
            self.stop()
        except Exception as error:
            print(f"Could not send motor stop: {error}", file=sys.stderr)
        try:
            self.write(b"$upload:0,0,0#")
        except Exception:
            pass
        finally:
            self.port.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


class MPU6050:
    def __init__(self, config, bus_factory=None):
        if bus_factory is None:
            from smbus import SMBus
            bus_factory = SMBus
        self.config = config
        self.address = config["imu_address"]
        self.bus = bus_factory(config["i2c_bus"])
        self.bias = (0.0, 0.0, 0.0)
        self.gyro_std = config["gyro_std_rad_s"]
        try:
            identity = self.bus.read_byte_data(self.address, 0x75)
            if identity != 0x68:
                raise RuntimeError(f"Expected MPU6050 WHO_AM_I=0x68; got 0x{identity:02x}")
            for register, value in ((0x6B, 0x01), (0x6C, 0x00),
                                    (0x1A, 0x03), (0x19, 0x09),
                                    (0x1B, 0x00), (0x1C, 0x00)):
                self.bus.write_byte_data(self.address, register, value)
            time.sleep(0.15)
        except BaseException:
            self.bus.close()
            raise

    def sample(self):
        data = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
        if len(data) != 14:
            raise OSError("Incomplete MPU6050 sample")
        ax, ay, az, _, gx, gy, gz = struct.unpack(">7h", bytes(data))
        accel_g = tuple(value / 16384.0 for value in (ax, ay, az))
        gyro = tuple(math.radians(value / 131.0) for value in (gx, gy, gz))
        return accel_g, gyro

    def calibrate(self):
        samples, gravity = [], []
        until = time.monotonic() + self.config["gyro_calibration_s"]
        while time.monotonic() < until:
            accel, gyro = self.sample()
            samples.append(gyro)
            gravity.append(math.sqrt(sum(value * value for value in accel)))
            time.sleep(0.01)
        if len(samples) < 10:
            raise RuntimeError("Not enough gyro calibration samples")
        columns = list(zip(*samples))
        bias = tuple(statistics.mean(values) for values in columns)
        noise = max(statistics.stdev(values) for values in columns)
        if (max(abs(value) for value in bias) > 0.1 or noise > 0.035
                or not 0.85 < statistics.mean(gravity) < 1.15
                or statistics.stdev(gravity) > 0.05):
            raise RuntimeError("IMU calibration unstable: keep the robot still and restart")
        self.bias = bias
        self.gyro_std = max(self.gyro_std, noise)
        return bias

    def body_gyro(self):
        _, gyro = self.sample()
        corrected = tuple(value - bias for value, bias in zip(gyro, self.bias))
        return rotate_sensor(corrected, self.config["imu_mount_rpy_deg"])

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.bus.close()
