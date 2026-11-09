# Single robot: explore and map with Nav2

Start with the robot in an unmapped space. This setup launches the existing
LiDAR and SLAM Toolbox configuration alongside the motor/IMU bridge, EKF and
Nav2. SLAM Toolbox builds `/map` and supplies `map → odom` live. The EKF
supplies `odom → base_link`; your LiDAR mount supplies `base_link → laser`.
There is no saved map, AMCL, or starting pose to supply.

The explorer looks for boundaries between known free and unknown space.
It chooses a point on the known side with room for the robot's footprint.
Nav2 checks for a path through observed free space before accepting that
point as a navigation goal. It replans and controls the robot as the map
updates. The robot waits when there is no eligible goal. This is a demo
explorer for reachable frontiers, so parking does not prove full coverage.

## Copy and install

Save `swarm2_nav2_explore_update.zip` to **`~/Downloads` on the Mac**.
Transfer the ZIP to **`~/Downloads` on the Pi**. Run the project commands
from **`~/Documents/swarm-mono`** on each machine.

On the Mac:

```bash
mkdir -p ~/Downloads ~/Documents/swarm-mono
cd ~/Documents/swarm-mono
ssh swarm2 'cd ~/Documents/swarm-mono && mkdir -p ~/Downloads'
scp ~/Downloads/swarm2_nav2_explore_update.zip swarm2:~/Downloads/
ssh swarm2
```

On the Pi:

```bash
cd ~/Documents/swarm-mono
sudo apt update
sudo apt install -y --no-install-recommends ros-kilted-navigation2 python3-numpy
python3 -m zipfile -e ~/Downloads/swarm2_nav2_explore_update.zip src/
```

APT handles its own system package cache. The update archive stays in
Downloads. Extracting it updates the Nav2 files and `robot_demo.py`.
It keeps your `hardware.yaml`, `hardware.py`, `demo_core.py`, `ekf.yaml`,
`mapping.launch.py`, and `web_map.py`. If you edited `robot_demo.py` locally,
save or commit your edits before extracting. Keep the mapping launch
configured to start a new map and retain your measured LiDAR mount.

The launcher sources Kilted and the existing workspace overlay. If it
cannot find `sllidar_ros2`, build the already checked-out driver from the
workspace root, then rerun the launcher:

```bash
cd ~/Documents/swarm-mono
source /opt/ros/kilted/setup.bash
colcon build --base-paths src --symlink-install --packages-select sllidar_ros2
```

## Run

Stop the manual demo and any separate mapping launch. Measure the distance
from the axle midpoint to the farthest edge of the chassis, including the
caster or overhang. Use that distance in meters as `robot_radius`.
**0.14 below is only an example.** Keep the robot stationary during gyro
calibration.

```bash
cd ~/Documents/swarm-mono
bash src/wheel_ekf_demo/start_nav2.sh robot_radius:=0.14
```

This starts automatic exploration once live mapping, transforms and Nav2
are ready. For a stationary first check, append `explore:=false` and do
not send a navigation goal. Verify the live map and Nav2 activation in a
second Pi terminal:

```bash
cd ~/Documents/swarm-mono
source /opt/ros/kilted/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=78
ros2 lifecycle get /bt_navigator
timeout 12s ros2 topic echo /map --once --field info --qos-reliability best_effort
ros2 topic info /hardware_demo/nav_cmd_vel --verbose
```

Expect `active`, a nonempty grid, and a `TwistStamped` publisher from
Collision Monitor with a subscriber from the hardware bridge. For the
first moving test, restart without `explore:=false` in a clear area and
keep the launch terminal available. **Ctrl+C in that terminal** stops
the stack. The bridge caps autonomous body speed at 0.15 m/s and
0.8 rad/s, and stops when its command stream expires. The demo still
needs verification on your physical robot.

For the existing map viewer, open a second Mac terminal:

```bash
cd ~/Documents/swarm-mono
ssh -x -t -L 127.0.0.1:8765:127.0.0.1:8765 swarm2 \
  'cd ~/Documents/swarm-mono && source /opt/ros/kilted/setup.bash && ROS_DOMAIN_ID=78 python3 src/wheel_ekf_demo/web_map.py'
```

Open http://127.0.0.1:8765 on the Mac. If you want a persistent map,
save it while SLAM Toolbox is still running. A saved map is an output
of this setup.

To return to manual driving, stop Nav2 first, then run from the Pi
workspace root:

```bash
source /opt/ros/kilted/setup.bash
python3 src/wheel_ekf_demo/robot_demo.py
```
