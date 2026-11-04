# How to run the SWARM simulation

This setup runs an Ubuntu 24.04 / ROS 2 Jazzy container with Gazebo Harmonic,
SLAM Toolbox, Nav2's supplied TurtleBot3 example, and RViz. Gazebo generates the
sensor data; no physical robot or LiDAR is needed.

The container displays its desktop through noVNC in a browser. Docker Compose
publishes only a localhost web port. The simulator and ROS processes run together
inside the container, so ROS does not need to be installed on the host.

The normal workflow is: start the environment, open the browser desktop, drive
with the keyboard, save the map, and stop the environment when finished.

## Where to run the commands

Unless a section explicitly says otherwise, run commands in a terminal on your
computer, inside the `swarm-monorepo` directory. Docker runs the simulation software
inside the container for you.

If your terminal is currently in the parent capstone folder (`18-500`), enter the repo:

```bash
cd swarm-monorepo
```

If you are already in `swarm-monorepo`, skip that command. The directory should
contain `compose.yaml`; running `pwd` will show your current location.

## Requirements

- Linux with Docker Engine and Docker Compose, or macOS with Docker Desktop.
- Allow approximately 6 GB of memory for the container and several GB of disk space.
- Build for the machine's native architecture. The same Dockerfile targets amd64
  and arm64; there is no forced architecture or host-specific display socket.

Software rendering is the default for both platforms. Rendering performance depends
on the computer. macOS support must be verified on a Mac; a successful Linux test
does not establish that the arm64 image or macOS runtime works.

The service runs as UID/GID 1000 to keep generated files owned by the usual Linux
desktop user. On Linux with a different user ID, set `SWARM_UID` and `SWARM_GID`
to the output of `id -u` and `id -g` before starting Compose. Docker Desktop handles
the host file-sharing layer on macOS; verify that saved files are writable there.

## First-time build and start

These are the commands used to build and start the verified Linux setup:

```bash
docker compose --progress plain build sim
docker compose up -d --no-build sim
docker compose ps
```

- `build sim` creates the environment from the Dockerfile, downloading and installing
  its packages. `--progress plain` prints build progress as text. The first build
  can take several minutes; later builds reuse cached work.
- `up -d --no-build sim` starts the environment using that built image. `-d` runs it
  in the background, so it continues running after you close this terminal.
- `ps` shows the service status. Look for the `sim` service to be running, followed
  by `healthy` once its browser desktop is available.

Once the desktop starts, open this URL in your browser:

<http://localhost:6080/vnc.html?autoconnect=true&resize=scale>

Gazebo and RViz start automatically. Wait for the robot to spawn, scans to arrive,
and the mapping node to activate. Container health reports desktop availability,
not successful robot bringup.

RViz uses a compact mapping view derived from Nav2's configuration, centered on
the initial mapping area. It opens on the right side of the desktop; Gazebo opens
on the left. Resize or move the windows as needed. The default desktop is 1600 by
900 pixels and scales to the browser. Override `SWARM_SCREEN` to change its size.

## Start on later sessions

This single command builds anything that changed and starts the environment:

```bash
docker compose up -d --build sim
```

Then open the same browser URL. If the environment is already running, open the
URL directly. To check its status or see startup messages:

```bash
docker compose ps
docker compose logs --tail 100 sim
```

## Drive the virtual robot

In a host terminal inside `swarm-monorepo`, run:

```bash
docker compose exec sim swarm-teleop
```

Keep that terminal focused. Use `i` to move forward, `j` / `l` to turn, and `k`
to stop. Start slowly and explore around the obstacles, then revisit the starting
area. Watch the map grow in RViz. The helper detects whether `/cmd_vel` uses
`Twist` or `TwistStamped` before publishing, allowing two seconds for a fresh
discovery query rather than relying on the CLI daemon's cached graph.

Press `k` to stop the robot, then Ctrl+C to exit keyboard control. Exiting keyboard
control leaves Gazebo and RViz running.

Do not submit navigation goals while using manual control. For a different command
topic, pass `-e SWARM_CMD_VEL_TOPIC=/your/topic` to `docker compose exec`.

## Save the map

While the simulation is running, open another host terminal inside
`swarm-monorepo` and run:

```bash
docker compose exec sim swarm-save-map
```

The map image and YAML metadata appear in the host checkout as:

```text
swarm-monorepo/
  maps/
    first_map.pgm
    first_map.yaml
```

The PGM is the map image. The YAML records information such as its resolution
and origin, which other ROS tools need to interpret the image.

The helper allows up to ten seconds for map delivery, including discovery time.
If it still times out, check that SLAM is active and publishing `/map`.

To choose another name:

```bash
docker compose exec sim swarm-save-map /workspace/maps/trial_02
```

The earlier verification run used
`docker compose exec sim swarm-save-map /workspace/maps/smoke_test`. Use a different
prefix for each experiment to preserve previous maps. `/workspace` inside the
container is the mounted `swarm-monorepo` directory on your computer.

An image/YAML map is sufficient for inspection and later evaluation. Resuming the
SLAM session also requires serializing SLAM Toolbox's pose graph and scan data.
Generated maps and recordings are ignored by Git.

## Inspect or record a run

Open a sourced ROS shell:

```bash
docker compose exec sim bash
```

Then inspect the data:

```bash
ros2 node list
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

Use the robot's actual frame names if they differ. Mapping needs the transforms
between map, odometry, robot body, and laser at the scan timestamps. The existing
physical LiDAR viewer in `src/rplidar_test` is a separate tool.

To record a run into the mounted workspace:

```bash
mkdir -p /workspace/bags
ros2 bag record -o /workspace/bags/trial_01 \
  --topics /scan /odom /tf /tf_static /clock /map
```

Start recording before driving. Verify the resulting bag includes static
transforms before relying on it for replay. Replay requires its own clean ROS
session and consistent simulation-clock settings.

## Desktop only and manual launch

To start the desktop without automatically launching the simulator:

```bash
SWARM_AUTOSTART_SIM=0 docker compose up --build
```

Then launch it from a container terminal:

```bash
swarm-sim
```

For a simulation without Gazebo or RViz windows, run this in a container shell
instead of starting another instance:

```bash
SWARM_HEADLESS=True SWARM_RVIZ=False swarm-sim
```

The virtual display remains available for sensor rendering. Hiding the GUI does
not eliminate the renderer needed by the virtual LiDAR.

Additional Nav2 launch arguments can be passed to `swarm-sim`, for example
`world:=/workspace/worlds/room.sdf` after creating a compatible world.

## Stop the simulation

Run this on the host, inside `swarm-monorepo`:

```bash
docker compose down
```

This stops the simulation and browser desktop and removes the service container.
Saved maps, recordings, and source files persist in the host checkout. The built
image remains available for the next start.

If you used the foreground form `docker compose up --build`, press Ctrl+C in that
terminal first, then run `docker compose down`.

## Troubleshooting

To inspect an existing service:

```bash
docker compose ps
docker compose logs --tail 100 sim
docker compose exec sim glxinfo -B
```

If `docker compose exec` says the service is not running, start it with
`docker compose up -d --build sim`. If the desktop opens but there are no scans or
map updates, inspect the ROS topics using the commands above and check the logs.

If port 6080 is occupied, start with `SWARM_DESKTOP_PORT=6081 docker compose up`
and use that port in the browser. To view the exact installed package versions:

```bash
docker compose exec sim cat /opt/swarm/build-packages.txt
```

## What the helper commands start

| Command | What it does |
| --- | --- |
| `swarm-desktop` | Starts the virtual desktop and browser connection, then launches the simulation by default. |
| `swarm-sim` | Launches the supplied TurtleBot3 world with SLAM enabled, simulation time, Gazebo, and RViz. |
| `swarm-teleop` | Sends keyboard velocity commands to the virtual robot. |
| `swarm-save-map` | Saves the current occupancy map as an image and YAML metadata. |

These helpers are installed in the container. Run them from the host with
`docker compose exec sim ...`, or directly from a terminal inside the browser
desktop. The container automatically sources the ROS environment for the helpers.
`swarm-desktop` normally runs automatically rather than being started again by hand.

The environment definition is in [compose.yaml](../compose.yaml) and
[the Dockerfile](../docker/sim/Dockerfile). The helper implementations are in
[docker/sim](../docker/sim).

## What comes after this baseline

1. Create a room with measured wall and obstacle geometry.
2. Derive an independent ground-truth reference and evaluate the saved map.
3. Tune SLAM and repeat recorded experiments.
4. Integrate Joey's physical scan, odometry, and motor interfaces.
5. Extend to two robots and evaluate accuracy, time reduction, and recovery.

## Verified baseline: November 3, 2026

Tested on this Arch Linux amd64 host using the Ubuntu container and Mesa llvmpipe
software rendering, including a fresh container start:

- The desktop HTTP endpoint and browser WebSocket-to-VNC connection responded.
- Gazebo and RViz rendered; the RViz map was inspected visually.
- Virtual LiDAR supplied 360 valid returns in the sampled scan.
- Scans, odometry, maps, and advancing simulation-clock messages were received.
- The transform from the map to the laser frame resolved.
- A short simulated forward command moved the robot approximately 0.23 m.
- The keyboard-control helper started and exited cleanly.
- Map saving wrote a PGM and YAML to `maps/pre_push_20261103` on the host.
- A short bag recorded all six documented topics, including `/tf_static`.

The image was rebuilt with the current helper scripts before these checks. Ruff
lint and formatting, ShellCheck, shfmt, and Compose configuration checks passed.

Local evidence is under `artifacts/`, `maps/`, and `bags/`; these generated files
are not tracked in Git. The example map resolution is 5 cm per cell. These checks
verify the pipeline, not SWARM's map-accuracy, parallel-speedup, or Pi CPU targets.

Initial TF and GLSL diagnostics appeared during startup; subsequent transforms
resolved and the map rendered. If a map stays blank, inspect the current logs and
topics instead of relying on the desktop health status.

The ROS base image lists amd64 and arm64 variants, but the complete arm64 build
and macOS runtime have not been tested.

## References and provenance

- [Gazebo / ROS version pairing](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Nav2 Jazzy simulation quickstart](https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/)
- [Nav2 mapping and map saving](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_slam/navigation2_with_slam/)
- [SLAM Toolbox interfaces](https://github.com/SteveMacenski/slam_toolbox#api)
- [Keyboard teleoperation](https://github.com/ros2/teleop_twist_keyboard)
- [noVNC](https://novnc.com/info.html)

This setup reuses these upstream packages and their provided robot/world example.
The container configuration, helper scripts, and instructions were prepared with
OpenAI Codex assistance. Gazebo and SLAM Toolbox supply the simulation and mapping
implementations; this setup does not implement a new SLAM algorithm.
