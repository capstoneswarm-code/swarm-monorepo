# Offline map comparison

This package starts the anomaly-detection workflow without ROS, Gazebo, or a
physical robot. Synthetic fixtures test the comparison logic; they do not establish
SLAM accuracy or physical obstacle-detection performance.

## Development setup

From the repository root, using Python 3.12 or newer:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e . -r requirements-dev.txt
python -m unittest discover -s tests/anomaly -v
ruff check .
ruff format --check .
```

Only `src/swarm_anomaly` is installed by this Python package. The physical robot
scripts and simulation helpers retain their existing entry points.

## Map conventions

`swarm_anomaly.maps.load_map` reads ROS YAML metadata and opaque, 8-bit grayscale
PGM/PNG images in `trinary` mode. Relative image paths resolve against the YAML
directory. Thresholds and `negate` come from the YAML: probability at or above
`occupied_thresh` is occupied; at or below `free_thresh` is free. Intermediate
values remain unknown. Unsupported image types and raw/scale modes produce an
error instead of a potentially misleading comparison.

Internally, cells are -1 (unknown), 0 (free), or 100 (occupied). Row zero is at the
bottom. Resolution is metres per cell; origin is the lower-left cell corner's
`[x, y, yaw]` pose. Image rows are flipped on import/export. These conventions follow
[ROS map metadata](https://github.com/ros2/common_interfaces/blob/jazzy/nav_msgs/msg/MapMetaData.msg)
and the supported trinary subset of
[Nav2 map loading](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_map_server/src/map_io.cpp).

`swarm_anomaly.synthetic.make_scenarios` creates a 6 x 4 m room at 5 cm/cell with
unchanged, added, removed, moved, partially observed, fully unobserved, and isolated
speckle cases. Boxes are 0.5 x 0.5 m. These are deliberately simple grids with
known changes, not generated LiDAR observations.

## Comparison contract

`swarm_anomaly.detection.compare_maps(reference, observed, min_cells=4)` compares
already aligned maps. Dimensions, resolution, and origin must match (numeric
tolerance 1e-9; yaw is compared modulo a full turn). The caller must also confirm
the maps refer to the same physical coordinate frame: identical metadata alone
cannot establish this. Automatic registration and resampling are future work.

An added obstacle is reference-free / observed-occupied. A removed obstacle is
reference-occupied / observed-free. Unknown cells in either map are excluded.
The result reports the jointly known cell count and fraction, so incomplete
coverage is visible even when no changes are found. Zero comparable cells means
insufficient evidence, not an unchanged room.

Each change groups four-connected cells of the same kind. Diagonal contact does
not join regions. Components below `min_cells` are discarded; the default is four
cells, or 0.01 square metres at 5 cm/cell. Filtered cell counts are reported.
Filtering may remove genuine small obstacles, so this threshold needs physical
validation. There is no probabilistic confidence score or wall-distance tolerance.

Results include type, cell count, area, centroid, and bounding polygon in metres
in the input map's coordinate frame. Grid bounds are `[x_min, y_min, x_max, y_max]`
with exclusive upper bounds; the world polygon accounts for the origin's yaw.
A moved box creates an added region and a removed region. Associating those
regions with one physical object is not implemented.

Detected regions are candidate differences. Misalignment, SLAM distortion, and
sensor noise can create them without a physical change. Synthetic tests verify
the comparison rules, not real-world detection accuracy or SLAM quality.
