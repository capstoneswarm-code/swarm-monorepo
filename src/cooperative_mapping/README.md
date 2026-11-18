# Two-robot cooperative mapping — Kilted / Ubuntu 24.04

This is an **experimental, separately launched cooperative extension** of the
current working single-robot stack. Install this directory as
`~/Documents/swarm-mono/src/cooperative_mapping/` on BOTH Pis.

`src/wheel_ekf_demo/` remains the single-robot version. This extension reads its
current files and builds an adapted COPY under
`src/cooperative_mapping/runtime/<robot>/`. It never patches those original
files. It preserves the current hardware calibration, LiDAR mount, motor bridge,
EKF, speed limits, collision monitor, frontier ranking, and return-home code
already present in your original explorer.

## What it does

- Each robot continues running its own SLAM Toolbox, EKF and Nav2.
- Keep swarm1 on ROS domain **79**, swarm2 on **78**. No shared `/tf`, motor
  command topic, or ROS namespace conversion is required.
- Every second, exchange small state/goal messages over an SSH tunnel. Download
  compressed RAW occupancy-grid snapshots only when their content changes
  (normally following SLAM's roughly five-second map publication interval).
- Support arbitrary initial positions/headings by attempting **automatic 2D
  map registration** with OpenCV SIFT, mutual descriptor matching, RANSAC,
  rigid SE(2) fitting, and bidirectional occupied-cell checks.
- Require consistent results from at least two different map snapshot pairs
  before the first alignment is accepted. Swarm1 computes the pair alignment;
  swarm2 uses its inverse and validates it against its own map. Both compose
  their own merged maps locally.
- Publish `/swarm/merged_map` in EACH robot's own `map` coordinates. Adapt the
  copied explorer and Nav2 global static layer to use that merged map. Raw local
  SLAM remains on `/map`; only raw maps are transmitted, preventing merge loops.
- Preserve the existing frontier selection, with one additional rule: reject
  candidate goal positions within **3 m** of the peer's fresh exploration goal.
- If both select nearby goals before receiving each other's reservation,
  **swarm1 keeps its goal; swarm2 cancels, waits for Nav2's action result, and
  selects again**. Do not issue a replacement while cancellation is pending.
- Expire peer goal reservations when contact is lost. Continue mapping and
  exploring independently; reconnect and recompute the merged map from the
  latest raw snapshots. The automatic SSH tunnel retries disconnected links.
- Clear session-specific alignment on a peer restart. Do not reuse an old run's
  transform for a newly initialized SLAM map.

This exchanges and composes **occupancy-grid SLAM results**, not pose graphs.
It does not add inter-robot loop closures to either SLAM Toolbox optimizer.

## Arbitrary starts require observable overlap

Unknown starting pose is supported; **no overlap is not**. Robots must eventually
observe common, sufficiently distinctive geometry. Until matching passes its
checks, the published cooperative map contains ONLY local observations and
cross-robot geometric goal exclusions are disabled. No initial transform is
invented, and matching does not use the requested 3 m goal separation as a hint.

Repetitive hallways and similar rooms can still produce a false alignment.
Geometric gates and repeated agreement reduce that risk but cannot prove a
unique physical match. Inspect the merged map during the first supervised
test. Local maps can also deform differently; a single rigid transform cannot
correct arbitrary nonrigid SLAM drift. Rejected alignment means independent
operation, not a claim that the environment has been fully explored.

For the first cooperative test, use different, unmeasured starting positions
and headings that see some of the SAME asymmetric room/corridor junction. This
tests arbitrary-pose alignment without first requiring the robots to discover
an overlap from completely separate regions of the building.

The 3 m rule is an **estimated goal-separation rule**, not collision avoidance
or a guarantee that routes cannot intersect. The existing local costmaps and
Collision Monitor remain active on both robots.

## 1. Install the new directory

Download `cooperative_mapping.zip` to `~/Downloads` on your Mac. From the Mac:

```bash
cd ~/Documents/swarm-mono
scp ~/Downloads/cooperative_mapping.zip swarm2:~/Downloads/
scp ~/Downloads/cooperative_mapping.zip swarm1@172.26.94.49:~/Downloads/
```

Use swarm1's currently working address if it changes. On EACH Pi:

```bash
cd ~/Documents/swarm-mono
python3 -m zipfile -e ~/Downloads/cooperative_mapping.zip src/
sudo apt update
sudo apt install -y --no-install-recommends \
  python3-opencv python3-numpy python3-scipy python3-yaml openssh-client
```

There is no new ROS package to build and no Docker, simulator, RViz, web
framework, or additional ROS message package. Your existing ROS/driver workspace
must already be built and working. APT maintains its usual system package cache;
the downloaded ZIP stays in Downloads.

The source directory must have the current `nav2.launch.py`, `mapping.launch.py`,
`robot_demo.py`, `hardware.py`, `demo_core.py`, `ekf.yaml`, `frontier_explorer.py`,
`manual_nav.xml`, `exploration_nav.xml`, and `nav2_params_<this_robot>.yaml`.
Include `web_map.py` for visualization and `home_nav.xml` if return-home uses it.
Use the current versions committed to your project. The runtime adapter refuses
an unrecognized explorer structure instead of silently replacing your logic.

Stop the original single-robot navigation/manual/mapping processes before
starting cooperative mode. Each robot still has one hardware bridge and one
LiDAR driver; the cooperative launcher starts those itself.

## 2. Allow the Pis to establish SSH tunnels to each other

This is Pi-to-Pi SSH, separate from Mac-to-Pi SSH. Each Pi keeps its own private
key; transfer only public keys using ssh-copy-id. These commands assume the
currently observed addresses; substitute working addresses when necessary.

On **swarm1**:

```bash
cd ~/Documents/swarm-mono
if [ ! -f "$HOME/.ssh/swarm_peer_ed25519" ]; then
  ssh-keygen -t ed25519 -N '' -f "$HOME/.ssh/swarm_peer_ed25519"
fi
ssh-copy-id -i "$HOME/.ssh/swarm_peer_ed25519.pub" swarm2@172.26.61.141
ssh -i "$HOME/.ssh/swarm_peer_ed25519" -o IdentitiesOnly=yes \
  -o BatchMode=yes swarm2@172.26.61.141 hostname
```

The last command should print `swarm2` without a password prompt.

On **swarm2**:

```bash
cd ~/Documents/swarm-mono
if [ ! -f "$HOME/.ssh/swarm_peer_ed25519" ]; then
  ssh-keygen -t ed25519 -N '' -f "$HOME/.ssh/swarm_peer_ed25519"
fi
ssh-copy-id -i "$HOME/.ssh/swarm_peer_ed25519.pub" swarm1@172.26.94.49
ssh -i "$HOME/.ssh/swarm_peer_ed25519" -o IdentitiesOnly=yes \
  -o BatchMode=yes swarm1@172.26.94.49 hostname
```

The last command should print `swarm1` without a password prompt. Normal first
connection host-key verification is retained. Once campus DNS works for both
names, `swarm1@swarm1` and `swarm2@swarm2` can replace the IP addresses. A fixed
IP in a command will need updating if DHCP changes it.

The peer HTTP server listens on **127.0.0.1:8768**, never on a campus-facing
interface. Each maintained SSH tunnel forwards local **8769** to the other
Pi's **8768**. Browser visualization continues on **8765** on each Pi.

## 3. Prepare and check each adapted runtime without launching hardware

On swarm1, use its ACTUAL calibrated hardware YAML:

```bash
cd ~/Documents/swarm-mono
bash src/cooperative_mapping/start.sh --radius 0.14 \
  --hardware-config "$PWD/src/wheel_ekf_demo/hardware.yaml" --prepare-only
```

On swarm2:

```bash
cd ~/Documents/swarm-mono
bash src/cooperative_mapping/start.sh --radius 0.14 \
  --hardware-config "$PWD/src/manual_demo/hardware.yaml" --prepare-only
```

These paths follow the latest commands you supplied. Change them if your
calibrated configurations are elsewhere. Likewise use your measured radius.

Run the local checks on either Pi:

```bash
python3 -m unittest discover -s src/cooperative_mapping -v
```

## 4. First run: mapping and Nav2 active, explorer disabled

On swarm1:

```bash
cd ~/Documents/swarm-mono
bash src/cooperative_mapping/start.sh --radius 0.14 \
  --hardware-config "$PWD/src/wheel_ekf_demo/hardware.yaml" \
  --peer-ssh swarm2@172.26.61.141 --no-explore
```

On swarm2:

```bash
cd ~/Documents/swarm-mono
bash src/cooperative_mapping/start.sh --radius 0.14 \
  --hardware-config "$PWD/src/manual_demo/hardware.yaml" \
  --peer-ssh swarm1@172.26.94.49 --no-explore
```

Keep both stationary during gyro calibration. The peer link can start before
the other robot; connection-refused messages should clear when both are up.

In another terminal on EACH Pi, inspect the local domain:

```bash
cd ~/Documents/swarm-mono
source /opt/ros/kilted/setup.bash
source install/setup.bash
if [ "$(hostname -s)" = swarm1 ]; then
  export ROS_DOMAIN_ID=79
else
  export ROS_DOMAIN_ID=78
fi
curl -fsS http://127.0.0.1:8769/state | python3 -m json.tool
ros2 topic echo /swarm/status --once
timeout 10s ros2 topic echo /swarm/merged_map nav_msgs/msg/OccupancyGrid \
  --once --field info --qos-reliability best_effort --qos-durability volatile
ros2 lifecycle get /bt_navigator
```

Expect the OTHER robot in the HTTP response, a nonempty grid, and active Nav2.
Initially `Independent: waiting for peer or distinctive overlapping maps` is
normal. A stationary first snapshot alone is intentionally insufficient for
automatic alignment; subsequent changed maps provide the consistency check.

## 5. Enable cooperative exploration

Stop both runs using Ctrl+C, then repeat the commands above **without
`--no-explore`**. Both robots map and explore independently at first. Once
registration succeeds, logs and `/swarm/status` report:

```text
Aligned: merged map and goal coordination enabled
```

At that point each explorer can use peer-observed space in its local-coordinate
merged map and avoids the peer's goal by 3 m. A simultaneous conflict produces:

```text
Goal conflict within 3 m: yielding to swarm1 and choosing again
```

Reservations concern exploration goals only; return-home goals do not force
the other robot to abandon exploration. If your original explorer already has
the `no_frontier()` return-home helper, its completion countdown waits while a
connected peer reports an active exploration goal, including before alignment.
If it has no return-home implementation, this extension does not invent one.

If all remaining candidate cells are reserved, the robot waits and rechecks;
it does not declare the map complete merely because another robot reserved
those cells. A failed/expired peer reservation releases them automatically.

## 6. View the cooperative maps

On your Mac, open a terminal for swarm1:

```bash
cd ~/Documents/swarm-mono
ssh -x -t -L 127.0.0.1:8767:127.0.0.1:8765 swarm1@172.26.94.49 \
  'cd ~/Documents/swarm-mono && bash src/cooperative_mapping/view.sh --radius 0.14'
```

Open http://127.0.0.1:8767. In another Mac terminal, for swarm2:

```bash
cd ~/Documents/swarm-mono
ssh -x -t -L 127.0.0.1:8765:127.0.0.1:8765 swarm2 \
  'cd ~/Documents/swarm-mono && bash src/cooperative_mapping/view.sh --radius 0.14'
```

Open http://127.0.0.1:8765. Stop any old viewer occupying that Pi's port first.
These are the adapted runtime copies of your existing web viewer, subscribed
to `/swarm/merged_map`; they display the local robot's marker and goal. The two
views use different map coordinate frames, so the room may have a different
rotation and origin in each browser even when their maps agree geometrically.
This bundle does not add a second robot marker or alter your existing trail
history behavior. `/swarm/peer` exposes the registered peer goal and transform.

## Validation and practical limits

Local tests cover rigid-frame math, arbitrary rotated/translated registration
on synthetic asymmetric occupancy maps, refusal of unrelated/empty maps,
rotated map origins, map-packet corruption/length checks, conservative occupancy
composition, recomposition without merged-map feedback, 3 m filtering,
deterministic tie-breaking, stale and unaligned reservations, cancellation
sequencing, and generation of the runtime copy without changing source files.

No physical Pi, ROS2 Kilted runtime, real Wi-Fi interruption, Nav2 motion, or
real-building registration was available here. Those remain on-robot checks.
Automatic matching is a prototype, not an established global localization
guarantee. In particular, do not infer that a visually plausible match is
correct just because both robots accepted the same estimated transform.

Registration runs no more than once per five seconds on changed map pairs;
OpenCV uses one thread. Network work and registration run outside ROS sensor
callbacks. Nevertheless benchmark the real Pi CPU load and controller timing;
these tests do not establish that the current 1 m/s profile is suitable.

If either link drops, fresh spatial reservations stop being applied and the
cooperative map falls back to the local map. Raw snapshots remain cached in
memory; after reconnecting, the latest snapshots replace them and alignment is
revalidated. There is no persistent map database across process restarts.
Existing sensor freshness and action cancellation checks are retained.

Return to the single-robot version by stopping this runner and using your
original `src/wheel_ekf_demo/start_nav2.sh` command. No rollback patch is needed.

## Files

- `start.sh`, `run.py`: start/stop the separate runtime and maintained tunnel.
- `prepare.py`: adapt the current explorer/viewer and Nav2 YAML in a copy.
- `peer_node.py`: raw-map/state exchange and local merged-map publication.
- `core.py`: registration, frame conversions, composition, reservation math.
- `cooperative_hooks.py`: goal reservation, filtering and conflict cancellation.
- `tunnel.sh`: authenticated loopback forwarding with reconnect attempts.
- `view.sh`: run the copied browser viewer on the correct domain.
- `test_cooperation.py`: offline checks; no ROS runtime needed.

Technical references: OpenCV's SIFT tutorial
https://docs.opencv.org/4.x/da/df5/tutorial_py_sift_intro.html and
estimateAffinePartial2D API https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html.
Existing grid-merging packages also require map overlap when initial poses are
unknown; see https://github.com/robo-friends/m-explore-ros2/tree/main/map_merge.
