"""Run on any Python machine: python3 -m unittest -v test_demo.py."""

import copy
from contextlib import redirect_stdout
import io
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from demo_core import (BoardParser, Keys, Travel, body_velocity, integrate,
                       rotate_sensor, speed_command, validate_config, wheel_speeds)
from hardware import MotorBoard, MPU6050
import robot_demo


def config():
    return yaml.safe_load(Path(__file__).with_name("hardware.yaml").read_text())


class CoreTests(unittest.TestCase):
    def test_serial_fragmentation_noise_and_multiple_frames(self):
        parser = BoardParser()
        self.assertEqual(parser.feed(b"banner\r\n$MS"), [])
        self.assertEqual(parser.feed(b"PD:80,-80,0,0#\r\n$MAll:1,2,3,4#"
                                     b"$MSPD:0,1,2,3#"),
                         [(80., -80., 0., 0.), (0., 1., 2., 3.)])

    def test_corrupt_frames_are_not_fake_zero_measurements(self):
        parser = BoardParser()
        self.assertEqual(parser.feed(b"$MSPD:nan,0,0,0#$MSPD:1,2,3#"
                                     b"$MSPD:abc,0,0,0#"), [])
        parser.feed(b"x" * 20000)
        self.assertLessEqual(len(parser.buffer), 4096)
        self.assertEqual(parser.feed(b"$MSPD:1,2,3,4#"), [(1., 2., 3., 4.)])

    def test_channels_units_and_independent_signs(self):
        cfg = config()
        cfg.update(left_motor=1, right_motor=3, right_command_sign=-1,
                   right_encoder_sign=-1)
        self.assertEqual(speed_command(.08, .08, cfg), b"$spd:80,0,-80,0#")
        self.assertEqual(wheel_speeds((80., 0., -80., 0.), cfg), (.08, .08))
        with self.assertRaises(ValueError):
            speed_command(.5, 0., cfg)

    def test_differential_drive_direction(self):
        self.assertEqual(body_velocity(.08, .08, .16), (.08, 0.))
        self.assertEqual(body_velocity(-.08, .08, .16), (0., 1.))
        self.assertEqual(body_velocity(.08, 0., .16), (.04, -.5))

    def test_kinematics_closed_circle_and_reverse(self):
        result = integrate((0., 0., 0.), .2, .5, 4 * math.pi)
        for value in result:
            self.assertAlmostEqual(value, 0., places=9)
        self.assertEqual(integrate((0., 0., 0.), -.2, 0., 2.), (-.4, 0., 0.))

    def test_wasd_directions_uppercase_and_two_wheels(self):
        keys = Keys(.08, .8)
        expected = {b"W": (.08, .08), b"A": (-.08, .08),
                    b"S": (-.08, -.08), b"D": (.08, -.08)}
        for key, targets in expected.items():
            keys.feed(key, 0.)
            self.assertEqual(keys.targets(.1), targets)
            keys.feed(key.lower(), .2)
            self.assertEqual(keys.targets(.3), targets)
        keys.feed(b" ", .4)
        self.assertEqual(keys.targets(.4), (0., 0.))

    def test_timeout_stops_and_new_key_replaces_both_targets(self):
        keys = Keys(.08, .8)
        keys.feed(b"w", 0.)
        self.assertEqual(keys.targets(.81), (0., 0.))
        keys.feed(b"d", .82)
        self.assertEqual(keys.targets(.83), (.08, -.08))
        keys = Keys(.08, .8)
        keys.feed(b"w", 0.)
        keys.feed(b"a", 1.)  # No intervening targets() call.
        self.assertEqual(keys.targets(1.), (-.08, .08))

    def test_arrow_escape_cannot_become_reverse_command(self):
        keys = Keys(.08, .8)
        keys.feed(b"w", 0.)
        keys.feed(b"\x1b[A", .1)
        self.assertEqual(keys.targets(.1), (0., 0.))
        keys.feed(b"w", .2)
        self.assertEqual(keys.targets(.2), (.08, .08))
        keys.feed(b"x", .3)
        self.assertTrue(keys.quit)
        self.assertEqual(keys.targets(.3), (0., 0.))

    def test_gyro_mount_rotation(self):
        vector = rotate_sensor((0., 0., 1.), (180., 0., 0.))
        self.assertAlmostEqual(vector[2], -1.)
        vector = rotate_sensor((1., 0., 0.), (0., 0., 90.))
        self.assertAlmostEqual(vector[0], 0.)
        self.assertAlmostEqual(vector[1], 1.)

    def test_heading_unwrap_over_full_revolution(self):
        travel = Travel(.005)
        for i in range(9):
            yaw = math.atan2(math.sin(i * math.pi / 2), math.cos(i * math.pi / 2))
            travel.update(i * .1, 0., 0., yaw, 0., 0.)
        self.assertAlmostEqual(math.degrees(travel.yaw_change), 720.)
        self.assertEqual(travel.path, 0.)

    def test_relative_pose_and_distance_definitions(self):
        travel = Travel(.005)
        travel.update(1., 10., 20., math.pi/2, 1., 0.)
        travel.update(2., 10., 21., math.pi/2, 1., 0.)
        self.assertAlmostEqual(travel.forward, 1.)
        self.assertAlmostEqual(travel.left, 0.)
        travel.update(3., 10., 20., math.pi/2, -1., 0.)
        self.assertAlmostEqual(travel.displacement, 0.)
        self.assertAlmostEqual(travel.path, 2.)

    def test_reporting_deadband_and_stale_timestamp(self):
        travel = Travel(.005)
        travel.update(1., 0., 0., 0., .001, 0.)
        travel.update(1.1, 0., 0., 0., .001, 0.)
        self.assertEqual(travel.path, 0.)
        self.assertFalse(travel.update(1.0, 0., 0., 0., 1., 0.))
        with self.assertRaises(ValueError):
            travel.update(3., 0., 0., 0., 1., 0.)

    def test_invalid_configs_and_masks(self):
        cfg = config()
        validate_config(cfg)
        for changes in ({"right_motor": cfg["left_motor"]},
                        {"left_motor": 0}, {"speed_m_s": 1.},
                        {"track_width_m": float("nan")},
                        {"left_encoder_sign": 0}):
            bad = copy.deepcopy(cfg)
            bad.update(changes)
            with self.assertRaises(ValueError):
                validate_config(bad)
        params = yaml.safe_load(Path(__file__).with_name("ekf.yaml").read_text())[
            "/hardware_demo/ekf_filter_node"]["ros__parameters"]
        for key, expected in (("odom0_config", [6, 7, 11]), ("imu0_config", [11])):
            mask = params[key]
            self.assertEqual(len(mask), 15)
            self.assertEqual([i for i, v in enumerate(mask) if v], expected)
        self.assertFalse(params["use_control"])


class DriverTests(unittest.TestCase):
    def test_motor_port_ownership_commands_and_cleanup(self):
        port = Mock(is_open=True, in_waiting=0)
        port.write.side_effect = lambda data: len(data)
        factory = Mock(return_value=port)
        with MotorBoard(config(), factory) as board:
            self.assertTrue(factory.call_args.kwargs["exclusive"])
            board.start_telemetry()
            board.command(.08, 0.)
            board.command(0., 0.)
        writes = [args[0][0] for args in port.write.call_args_list]
        self.assertIn(b"$upload:0,0,1#", writes)
        self.assertIn(b"$spd:80,0,0,0#", writes)
        self.assertEqual(writes[-2:], [b"$pwm:0,0,0,0#", b"$upload:0,0,0#"])
        self.assertFalse(any(b"mphase" in data or b"flash_reset" in data for data in writes))
        port.close.assert_called_once()

    def test_motor_cleanup_even_if_body_raises(self):
        port = Mock(is_open=True)
        port.write.side_effect = lambda data: len(data)
        with self.assertRaises(RuntimeError):
            with MotorBoard(config(), Mock(return_value=port)):
                raise RuntimeError("sensor fault")
        port.write.assert_any_call(b"$pwm:0,0,0,0#")
        port.close.assert_called_once()

    def test_imu_units_bias_and_mount(self):
        cfg = config()
        cfg["gyro_calibration_s"] = .12
        bus = Mock()
        bus.read_byte_data.return_value = 0x68
        bus.read_i2c_block_data.return_value = list(struct.pack(">7h", 0, 0, 16384, 0, 131, -262, 0))
        clock = SimpleNamespace(now=0.)
        def sleep(duration):
            clock.now += duration
        with patch("hardware.time.monotonic", lambda: clock.now), patch("hardware.time.sleep", sleep):
            with MPU6050(cfg, Mock(return_value=bus)) as imu:
                accel, gyro = imu.sample()
                self.assertEqual(accel, (0., 0., 1.))
                self.assertAlmostEqual(gyro[0], math.pi / 180)
                self.assertAlmostEqual(gyro[1], -2 * math.pi / 180)
                imu.calibrate()
                for value in imu.body_gyro():
                    self.assertAlmostEqual(value, 0.)
        bus.close.assert_called_once()

    def test_wrong_imu_identity_fails_closed(self):
        bus = Mock()
        bus.read_byte_data.return_value = 0x70
        with self.assertRaises(RuntimeError):
            MPU6050(config(), Mock(return_value=bus))
        bus.close.assert_called_once()


class FakeKeyboard:
    events = b"w"
    def __enter__(self):
        return self
    def read(self):
        return self.events
    def __exit__(self, *_):
        pass


class StopBehaviorTests(unittest.TestCase):
    def run_loop(self, node=None, imu=None, process=None, clock=None):
        board = Mock()
        board.speeds.return_value = []
        node = node or SimpleNamespace(last_wheel=None, last_filtered=None,
                                       count_publishers=lambda _: 1,
                                       publish_gyro=Mock())
        imu = imu or SimpleNamespace(body_gyro=lambda: (0., 0., 0.), gyro_std=.02)
        process = process or SimpleNamespace(poll=lambda: None)
        fake_ros = SimpleNamespace(ok=Mock(side_effect=[True, False]), spin_once=Mock())
        with patch.dict(sys.modules, {"rclpy": fake_ros}), \
                patch("robot_demo.Keyboard", FakeKeyboard), \
                patch("robot_demo.time.monotonic", clock or (lambda: 10.)), \
                redirect_stdout(io.StringIO()):
            try:
                robot_demo.control_loop(node, board, imu, process, config())
            finally:
                self.assertTrue(board.stop.called)
                self.assertFalse(any(call.args != (0., 0.) for call in board.command.call_args_list))
        return board

    def test_keys_cannot_drive_before_inputs_ready(self):
        self.run_loop()

    def test_ekf_exit_stops_motors(self):
        with self.assertRaisesRegex(RuntimeError, "EKF process exited"):
            self.run_loop(process=SimpleNamespace(poll=lambda: 1))

    def test_stale_encoders_stop_motors(self):
        node = SimpleNamespace(last_wheel=8., last_filtered=10.)
        with self.assertRaisesRegex(RuntimeError, "wheel telemetry"):
            self.run_loop(node=node)

    def test_imu_read_error_stops_motors(self):
        imu = SimpleNamespace(body_gyro=Mock(side_effect=OSError("I2C fault")), gyro_std=.02)
        with self.assertRaisesRegex(OSError, "I2C fault"):
            self.run_loop(imu=imu)

    def test_stall_during_io_cannot_issue_motor_command(self):
        node = SimpleNamespace(last_wheel=10., last_filtered=10.,
                               publish_gyro=Mock(), count_publishers=lambda _: 1)
        clock = iter([10., 10., 11., 11.])
        with self.assertRaisesRegex(RuntimeError, "stale during I/O"):
            self.run_loop(node=node, clock=lambda: next(clock))


if __name__ == "__main__":
    unittest.main()
