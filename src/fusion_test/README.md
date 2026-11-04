# Minimal wheel odometry + gyro EKF example

Target: Ubuntu 24.04 ARM64 and ROS 2 Kilted on a Raspberry Pi.
No Docker, colcon build, GUI, Gazebo, or hardware required for the demo.
The only additional ROS package is robot_localization.

The Python node supplies synthetic wheel and gyro data for a moving robot.
It does not access the motor board, publish cmd_vel, or drive physical wheels.
It is a data-pipeline example, not an MPU6050 driver or encoder driver.

## Install

```bash
sudo apt update
sudo apt install -y --no-install-recommends ros-kilted-robot-localization
```

Extract this folder anywhere on the Pi, for example ~/robot_localization_example.
All terminal examples below use that location. No workspace build is needed.

## Run

Use domain 77 in ALL demo terminals to separate the synthetic data from the
normal robot graph. This assumes no real robot nodes are using domain 77.
This is a demonstration on the Pi itself, so campus multicast discovery is
not needed between the Pi and the Mac: run the terminals through SSH.

Terminal 1, EKF:

```bash
source /opt/ros/kilted/setup.bash
export ROS_DOMAIN_ID=77
ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node \
  --params-file ~/robot_localization_example/ekf.yaml
```

Terminal 2, synthetic measurements:

```bash
source /opt/ros/kilted/setup.bash
export ROS_DOMAIN_ID=77
python3 ~/robot_localization_example/demo_inputs.py
```

Terminal 3, inspect results:

```bash
source /opt/ros/kilted/setup.bash
export ROS_DOMAIN_ID=77
ros2 topic echo /odometry/filtered --field twist.twist
```

Expect forward speed near 0.15 m/s and yaw rate near 0.30 rad/s, with noise
and startup transients. The wheel-only yaw measurement is deliberately
biased to about 0.38 rad/s. Gyro covariance is smaller, so the filtered
yaw rate should generally stay much closer to the gyro's 0.30 rad/s.
This does not mean an EKF can automatically estimate or remove arbitrary
real sensor bias; it illustrates weighting measurements with different
uncertainties. The synthetic wheel bias also is not an independent Gaussian
noise sample, so this example is not a statistical consistency benchmark.

Stop the echo with Ctrl+C and compare individual inputs:

```bash
ros2 topic echo /odometry/wheel --field twist.twist
```

```bash
ros2 topic echo /imu/data_raw --field angular_velocity
```

Check TF, and optionally inspect pose:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

```bash
ros2 topic echo /odometry/filtered --field pose.pose
```

/demo/truth is a reference trajectory for comparison; the EKF does not
subscribe to it. Velocity-only fusion has no absolute position reference,
so do not expect the filtered pose to match truth exactly or stay drift-free.

## Topics and transform ownership

| Interface | Type | Publisher | Consumer |
|---|---|---|---|
| /odometry/wheel | nav_msgs/msg/Odometry | demo_inputs or real encoder bridge | EKF |
| /imu/data_raw | sensor_msgs/msg/Imu | demo_inputs or real IMU publisher | EKF |
| /odometry/filtered | nav_msgs/msg/Odometry | EKF | tools, control, navigation |
| odom -> base_link | TF | EKF only | SLAM Toolbox, tools |
| /demo/truth | nav_msgs/msg/Odometry | demo_inputs | observation only |

The 15-element config vectors are ordered:
x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az.
The EKF fuses wheel vx and yaw rate plus the differential-drive vy=0
constraint; it fuses gyro yaw rate only. It does not fuse encoder-derived
pose a second time, MPU6050 acceleration, or a made-up absolute heading.
The vy=0 constraint assumes rolling without meaningful lateral slip. If
your platform slides sideways, increase its variance or disable vy fusion.

## Replace synthetic inputs with the real sensors

1. Stop BOTH demo processes and any demo observation commands. Open fresh
   terminals for real operation; source /opt/ros/kilted/setup.bash and your
   robot workspace. Do not export the demo's ROS_DOMAIN_ID=77 in them.
   Use the same chosen ROS_DOMAIN_ID for all real robot processes.
2. Have your motor/encoder bridge publish /odometry/wheel. header.frame_id
   is odom, child_frame_id is base_link. Its twist must come from measured
   encoder changes and calibrated wheel geometry, not PWM or cmd_vel.
   Forward travel is +x; counterclockwise turning is +angular.z.
   Publish vy=0 and a realistic covariance only if the no-sideways-motion
   constraint is justified. Replace all example covariances with estimates
   from actual stationary and motion tests. Stop publishing odom->base_link
   TF from this bridge when the EKF is publishing it.
3. Have your MPU6050 publisher publish /imu/data_raw. Use frame_id imu_link,
   gyro in rad/s, and correctly measured mounting axes. A raw accelerometer
   reading in g is not m/s^2. This EKF configuration ignores acceleration
   and orientation; mark unavailable orientation with
   orientation_covariance[0]=-1, and provide gyro covariance. Calibrate the
   stationary gyro bias; MPU6050 has no absolute compass heading.
4. Publish the measured fixed base_link->imu_link mounting transform with
   tf2_ros static_transform_publisher or your URDF. All translation values
   are meters and all angles radians. Do not copy an identity transform
   unless the physical sensor axes and mounting origin actually match.
5. Start the same EKF command from Terminal 1 in the real robot domain.
   The YAML uses relative input topic names, so remap them if your existing
   driver topics differ. The root node name must be ekf_filter_node to
   match this YAML. For multiple robots, namespace the node, topics, and
   parameter selector consistently; do not simply share this root-level
   configuration between robots on the same graph.
6. With wheels still, filtered velocity should be near zero. Move straight
   and check forward velocity and yaw signs. Rotate counterclockwise and
   check that encoder and gyro angular.z agree in sign. Verify that
   odom->base_link keeps updating before adding SLAM.

## Connect SLAM Toolbox after real odometry works

Run the RPLIDAR driver, publish the measured base_link->laser mounting TF,
and configure ordinary single-robot SLAM Toolbox with:

```yaml
odom_frame: odom
base_frame: base_link
map_frame: map
scan_topic: /scan
use_sim_time: false
```

These keys belong inside your existing SLAM node's ros__parameters block;
this fragment is not a complete SLAM launch configuration.

SLAM reads /scan and the TF chain, not /odometry/filtered directly.
SLAM owns map->odom; the EKF owns odom->base_link. Static TF connects the
body to the sensors. The decentralized multi-robot variant may introduce
global_odom and requires its own shared-reference setup; establish this
single-robot pipeline first. Do not feed the SLAM-corrected pose back into
this same local EKF as if it were independent of its existing measurements.

## Validation and limits

The delivered Python has been syntax-checked and its planar integration
and YAML measurement selections have been checked without ROS. ROS 2 is
not installed in the authoring environment, so end-to-end Kilted execution
and hardware behavior have not been tested here. Run the observation
commands above on the Pi to verify the complete pipeline.

Official references:

- https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/params/ekf.yaml
- https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst
- https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst
