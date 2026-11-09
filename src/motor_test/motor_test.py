#!/usr/bin/env python3
"""Two-second one-wheel turn. Usage: python3 motor_test.py PORT [MOTOR_1_TO_4]."""
import signal
import sys
import time

import serial

if len(sys.argv) not in (2, 3):
    raise SystemExit(__doc__)
motor = int(sys.argv[2]) if len(sys.argv) == 3 else 1
if motor not in (1, 2, 3, 4):
    raise SystemExit("Motor number must be 1, 2, 3, or 4")


def interrupted(*_):
    raise KeyboardInterrupt


signal.signal(signal.SIGTERM, interrupted)
signal.signal(signal.SIGHUP, interrupted)

with serial.Serial(sys.argv[1], 115200, timeout=0.2, write_timeout=1,
                   exclusive=True) as board:
    def pwm(values):
        board.write(("$pwm:" + ",".join(map(str, values)) + "#").encode("ascii"))
        board.flush()

    try:
        pwm([0, 0, 0, 0])
        time.sleep(0.5)
        values = [0, 0, 0, 0]
        values[motor - 1] = 2000
        print(f"Turning with M{motor} for 2 seconds; other outputs commanded zero.")
        pwm(values)
        time.sleep(2.0)
    except KeyboardInterrupt:
        pass
    finally:
        pwm([0, 0, 0, 0])
        print("Stop command sent.")
