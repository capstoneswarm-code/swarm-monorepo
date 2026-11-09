# swarm-monorepo

SWARM: Scalable Warehouse Autonomous Robotic Mapping.

## Offline anomaly detection

The Python package under `src/swarm_anomaly` compares already aligned occupancy
maps and reports candidate added/removed obstacles. Known coordinate transforms
can align different map grids before comparison. It includes synthetic room
examples, unknown-area handling, a small-region filter, JSON results, and visual
overlays. It runs without ROS or robot hardware.

See [the anomaly-detection guide](docs/ANOMALY_DETECTION.md) for installation,
example commands, tests, supported map formats, and current limitations.

The [warehouse alert demo](docs/WAREHOUSE_ALERTS.md) adds explicit aisles and
intersections, blockage/bridge/detour analysis, ranked explanations, and an
interactive map. Run `swarm-anomaly warehouse-demo --output artifacts/warehouse-demo/ranked`
after installation, then open the generated `index.html`. It uses synthetic maps;
live integration and email/SMS notifications remain future work.

## Simulation

The first development milestone is one simulated robot building and saving a map.
The Docker setup includes ROS 2 Jazzy, Gazebo Harmonic, SLAM Toolbox, RViz, and
Nav2's TurtleBot3 example, with a browser desktop for Linux and macOS hosts.

From this directory, run:

```bash
docker compose up -d --build sim
```

Open <http://localhost:6080/vnc.html?autoconnect=true&resize=scale>. Drive from
another host terminal with `docker compose exec sim swarm-teleop`, then save a map
with `docker compose exec sim swarm-save-map`.

See [How to run the SWARM simulation](docs/SIMULATION.md) for the exact first-time
build commands, everyday startup, driving, map saving, shutdown, troubleshooting,
recording, and attribution. macOS remains unverified
until this setup is tested on a Mac.

## Physical robot mapping

The Raspberry Pi hardware demo targets ROS 2 Kilted. Follow the
[hardware mapping instructions](src/wheel_ekf_demo/README.md) to start manual
motor control, wheel/gyro sensor fusion, the LiDAR driver, SLAM Toolbox, and the
browser map viewer.

- `src/wheel_ekf_demo`: the physical robot's control and mapping pipeline, hardware
  configuration, and unit tests.
- `src/fusion_test`: a synthetic wheel/gyro example for testing EKF sensor fusion.
- `src/motor_test`: a short test for an individual motor on the physical board.

The Jazzy simulation and Kilted hardware demo run in separate environments. The
simulation uses ROS domain 42; the hardware instructions use domain 78.

## Physical LiDAR tools

- `src/sllidar_ros2`: the upstream Slamtec ROS 2 driver submodule.
- `src/rplidar_test/lidar_view.py`: a live plot of incoming `/scan` measurements.

The physical sensor tools and simulation are separate entry points.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md) for Python annotations and formatting,
shell checks, validation commands, and commit size and message conventions.
