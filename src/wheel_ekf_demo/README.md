# Keyboard wheel control + real sensor EKF demo

For the two-wheel Yahboom chassis, its USB serial four-channel motor board,
and an MPU6050 wired directly to a Raspberry Pi running Ubuntu 24.04 / ROS 2
Kilted. Everything runs on the Pi through an ordinary interactive SSH terminal.
No GUI, Docker, colcon build, LiDAR, or simulator is required.

One Python command starts the hardware bridge, keyboard reader, EKF process,
and distance/heading reporter. Motor commands are controls, not observations:
the EKF fuses MEASURED encoder-derived wheel speed and the gyro. It does not
treat commanded speed or PWM as a separate position measurement.

## Files

- robot_demo.py: entry point, ROS publishers/subscriber, EKF lifecycle, keyboard.
- hardware.py: serial board and MPU6050 I2C access.
- demo_core.py: serial parsing, controls, motion math, relative travel.
- hardware.yaml: physical wiring, geometry, speed, sensor mount, uncertainties.
- ekf.yaml: robot_localization configuration.
- test_demo.py: hardware-independent and mocked-fault tests.

## 1. Copy and install

On the Mac, after downloading the ZIP:

```bash
scp ~/Downloads/wheel_ekf_demo.zip swarm2:~/
```

On the Pi:

```bash
python3 -m zipfile -e ~/wheel_ekf_demo.zip ~/Documents/swarm-mono/
sudo apt update
sudo apt install -y --no-install-recommends \
  ros-kilted-robot-localization python3-serial python3-smbus python3-yaml
cd ~/Documents/swarm-mono/wheel_ekf_demo
```

Stop other motor scripts first: this program must be the only serial-port
owner. It opens that device exclusively. It uses ROS_DOMAIN_ID=78 by default,
separate from the previous synthetic demo on 77. Stop the synthetic demo too
so you do not mistake its output for hardware measurements.

## 2. Check the physical configuration

Edit hardware.yaml:

```bash
nano hardware.yaml
```

The following are EXAMPLES rather than measurements of your robot:

```yaml
left_motor: 1
right_motor: 2
track_width_m: 0.16
```

Set the two actual motor connector numbers, 1 through 4, and measure the
distance between the left/right wheel centerlines. The fact that the board
has four outputs does not imply you have four wheels; the other two outputs
are commanded zero. Check the USB by-id path too; the default is the CH340
device from your earlier terminal output.

The initial target is 80 mm/s per wheel. This is a speed command to the board's
encoder PID, not a raw PWM command. Equal requested speeds rely on correctly
configured encoder motors and working feedback; they are not a guarantee of
equal physical motion with incorrect board calibration or a stalled wheel.

Verify the board's stored gearbox ratio, encoder settings, and wheel diameter
against your actual motors. Your earlier read_flash output showed ratio 30
and wheel diameter 67 mm; those are stored settings, not proof of the physical
parts. Incorrect settings affect BOTH speed control and speed telemetry.
This demo does not overwrite persistent motor-board parameters. Distances
are approximate until that calibration and chassis geometry are correct.

Optional left_speed_scale/right_speed_scale multiply measured speed to
compensate a measured reporting error; they do not change target speed
commands. Correct board settings are preferred so both control and odometry
have the right scale.

### MPU6050 connection and orientation

This driver expects /dev/i2c-1 and an MPU6050 at 0x68 (or configure 0x69).
It cannot read an IMU connected only to the MSPM0 through the motor USB port.
The IMU must be accessible directly on the Pi's configured GPIO I2C bus.
Use Pi-safe 3.3 V I2C signaling and the appropriate supply for your breakout.

| Signal | Raspberry Pi physical pin |
|---|---|
| SDA | 3 / GPIO2 |
| SCL | 5 / GPIO3 |
| GND | 6 |
| 3.3 V, for a compatible breakout | 1 |

imu_mount_rpy_deg describes the rotation from the sensor's labeled axes into
the robot's x-forward/y-left/z-up axes. Identity is appropriate ONLY when
those axes already agree. An upside-down sensor may need a 180-degree roll
or pitch, depending on its mounting. Verify positive counterclockwise gyro
turning after setting this rotation. The script rotates the gyro vector into
base_link itself, so no static IMU transform is needed for this gyro-only demo.

If permission is denied, inspect the device owners:

```bash
ls -l /dev/serial/by-id/ /dev/i2c-1
id -nG
```

Normally serial uses dialout and I2C uses i2c. If those groups own the devices:

```bash
sudo usermod -aG dialout,i2c "$USER"
```

Disconnect and reconnect SSH to acquire the new groups. If /dev/i2c-1 does
not exist, finish enabling the Pi's GPIO I2C interface first; this program
does not modify boot configuration or ask to run the whole ROS stack as root.

## 3. Start

Keep the robot still during the three-second gyro calibration. For the first
channel/direction check, raise the wheels off the ground and keep the motor
power switch accessible.

```bash
source /opt/ros/kilted/setup.bash
cd ~/Documents/swarm-mono/wheel_ekf_demo
python3 robot_demo.py
```

The app sets its own domain ID, starts the EKF, and waits for real wheel-speed
telemetry and a fresh filtered estimate. No motion key is honored before
READY. The display is zeroed at the first valid EKF sample while the motors
are still stopped. Startup calibration is excluded; keep the robot stationary
until READY to make this equivalent to the start of the measurement session.

Use --domain-id if 78 is already used by another robot. This changes the
environment for this process and its EKF child, not your parent SSH shell.

```bash
python3 robot_demo.py --domain-id 79
```

## 4. Keys

| Key | Effect |
|---|---|
| W | Forward: both wheels forward |
| S | Backward: both wheels backward |
| A | Spin left: left wheel backward, right wheel forward |
| D | Spin right: left wheel forward, right wheel backward |
| Space | Stop both wheels immediately |
| X or Ctrl+C | Stop and exit |

Uppercase and lowercase both work; Enter is not required. Other ordinary
keys also clear the wheel targets; arrow escape sequences cannot be mistaken
for A or other motion keys.

An SSH terminal supplies key presses and autorepeat, not key-release events.
Each W/A/S/D event replaces BOTH wheel targets with the selected motion.
All targets clear after 0.8 s without a motion-key event. Holding a key
normally supplies repeats; releasing it stops the command after that timeout.
This is terminal control behavior, not independent key-up tracking. Space is
the immediate stop. The configured timeout cannot exceed 2 seconds.

First confirm the actual left/right channels and physical forward direction.
If W drives either wheel backward, change that wheel's command_sign. Independently
check that displayed wheel L/R velocity is positive when rolling forward;
change the corresponding encoder sign if needed. A command-sign reversal
often requires the same encoder-sign reversal for body-frame measurements.
Test W with raised wheels initially; set the chassis on the floor for actual
travel measurements. Spinning raised wheels necessarily produces encoder
odometry even though the chassis has not translated.

## 5. Read the output

Illustrative format, not a recorded hardware result:

```text
dx=+0.215 dy=+0.004 m | displacement=0.215 m travel=0.219 m | yaw=+3.4 deg | wheel L/R=+0.078/+0.081 m/s
```

- dx/dy: filtered position relative to the initial pose, expressed as initial
  forward/left directions, in meters.
- displacement: straight-line distance from the initial position.
- travel: accumulated speed magnitude from the filtered velocity, in meters;
  reversing adds travel. It uses a small configurable speed deadband to reduce
  stationary noise accumulation. It does not redefine the EKF position.
- yaw: signed accumulated heading change in degrees; positive is
  counterclockwise. It unwraps, so a full turn can display +360 instead of 0.
- wheel L/R: actual board-reported, sign/scale-corrected wheel speeds in m/s.

With a forward-and-back motion you can have nonzero travel and near-zero
displacement. During a turn in place the center's travel should stay small
while heading changes. These are odometry estimates: wheel slip, incorrect
geometry, speed scaling, and residual gyro bias cause error. No absolute
position or absolute yaw reference is included.

## 6. ROS topics and TF

| Interface | Publisher |
|---|---|
| /hardware_demo/wheel_odom | Hardware bridge, nav_msgs/msg/Odometry |
| /hardware_demo/imu/data_raw | Hardware bridge, sensor_msgs/msg/Imu |
| /hardware_demo/odometry/filtered | robot_localization EKF |
| odom -> base_link TF | EKF only |

To inspect in a SECOND SSH terminal on the Pi, use the same domain:

```bash
source /opt/ros/kilted/setup.bash
export ROS_DOMAIN_ID=78
ros2 topic echo /hardware_demo/odometry/filtered --field twist.twist
```

Or inspect TF:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

The configuration fuses encoder-derived forward speed, turning rate, and a
soft zero-lateral-velocity rolling constraint; the gyro adds turning rate.
The accelerometer is used only to check startup calibration stability, not
double-integrated into position. No fabricated absolute orientation or
duplicate encoder-derived pose measurement is fused. Increasing
lateral_speed_std_m_s weakens the no-sideways-motion constraint if the robot
slides sideways; disable that field's EKF mask if this assumption is unsuitable.

## Stop behavior and limitations

The app refreshes motor commands at 10 Hz and publishes IMU data at about
50 Hz. Wheel messages are published when actual serial speed frames arrive;
missing data is never replaced with a fake zero-speed measurement. Timestamps
are Pi receipt times because these serial frames have no hardware timestamps.

The software sends a zero-output PWM command on normal exit, Ctrl+C, handled
termination/hangup, an IMU error, stale wheel/filtered data, or an EKF exit.
The keyboard lease expires automatically without motion-key events. Cleanup
also terminates the EKF child and restores normal terminal input.

These are software protections, not a verified independent board watchdog.
SIGKILL, a frozen OS, or a disconnected serial link can prevent a stop command
from reaching the motor board. Do not rely on this script as an emergency stop.
Keep a physical means to remove motor power available during testing.

## Troubleshooting

- Permission denied: device groups and reconnect SSH as described above.
- Missing MPU6050 / WHO_AM_I mismatch: confirm direct Pi I2C wiring, bus,
  address, and actual chip identity; no fallback IMU readings are fabricated.
- Startup timeout: look for fresh $MSPD frames and inspect ekf.log beside the
  script. Wrong ROS setup/parameters or another process owning the serial
  device can prevent startup.
- Wheels do not move: check battery power, channel selection, encoder wiring,
  and correct speed-control configuration. This demo uses encoder PID rather
  than the earlier raw PWM motor test.
- Fresh but zero wheel speed while moving: encoder feedback is not verified;
  fix the board/motor configuration before trusting the odometry. This demo
  cannot reliably distinguish a stalled motor from missing encoder pulses.
- Wrong direction or yaw: check command signs, encoder signs, sensor mounting
  rotation, and wheel spacing before tuning EKF covariances.
- EKF failure: the full child output is in ekf.log, overwritten at each run.
- Terminal left in a bad state after a hard kill: run stty sane in that SSH
  terminal. Normal exits restore it automatically.

## Verification

```bash
python3 -m unittest -v test_demo.py
```

22 tests passed in the authoring environment. They exercise serial framing,
units, wheel mapping, direction signs, keyboard timeouts, ANSI-key handling,
motion integration, full-turn heading unwrapping, relative coordinates, IMU
bias/mount conversion, and mocked stop behavior during sensor/EKF failures.
Python syntax and YAML measurement masks were checked. ROS 2 and the actual
Pi hardware are not available in that environment: physical speed, sensor
access, Kilted execution, and stopping behavior need on-robot verification.

## Primary references

- Chassis USB wiring / encoder speed units:
  https://www.yahboom.net/study/wheeled_chassis/14350
- Four-channel protocol ($spd, $pwm, $upload and $MSPD):
  https://www.yahboom.net/study/MD520/14104
- robot_localization configuration:
  https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/params/ekf.yaml
- Sensor fusion selection and duplicate measurements:
  https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst
