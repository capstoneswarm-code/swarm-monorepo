# swarm-monorepo

SWARM: Scalable Warehouse Autonomous Robotic Mapping.

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

## Physical LiDAR tools

- `src/sllidar_ros2`: the upstream Slamtec ROS 2 driver submodule.
- `src/rplidar_test/lidar_view.py`: a live plot of incoming `/scan` measurements.

The physical sensor tools and simulation are separate entry points.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md) for Python annotations and formatting,
shell checks, validation commands, and commit size and message conventions.
