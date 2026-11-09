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

## Run without robot data

After activating the environment above, generate the examples and compare a room
with an added box:

```bash
swarm-anomaly fixtures --output artifacts/anomaly/maps
swarm-anomaly compare \
  --reference artifacts/anomaly/maps/reference.yaml \
  --observed artifacts/anomaly/maps/added.yaml \
  --output artifacts/anomaly/added
```

Open `artifacts/anomaly/added/overlay.png` to see reference, observed, and changed
regions side by side. Red marks additions; blue marks removals; gray marks unknown
or excluded cells in the changes panel. `changes.json` contains the region IDs,
types, sizes, coordinates, coverage, and filter settings for later UI integration.
These are **candidate differences**, not verified physical anomalies.

The CLI refuses to overwrite existing map pairs or comparison outputs. Pick a
new output directory when rerunning a case. Generated files under `artifacts/`
remain ignored by Git. `python -m swarm_anomaly` is equivalent to `swarm-anomaly`.
Successful processing exits 0, including cases with no comparable cells; inspect
the JSON `status` and `coverage` fields. Invalid inputs or existing outputs exit 2.

Change `--observed` and `--output` to exercise the other scenarios:

| Observed map | Expected result with the default filter |
| --- | --- |
| `unchanged.yaml` | No regions; 100% comparable coverage |
| `added.yaml` | One added 0.25 m² region |
| `removed.yaml` | One removed 0.25 m² region |
| `moved.yaml` | One added and one removed 0.25 m² region |
| `partial.yaml` | No regions; 50% comparable coverage |
| `unobserved.yaml` | No regions; `status: no_comparable_cells`, 0% coverage |
| `speckle.yaml` | No retained regions; one filtered cell |

Use `--min-cells 1` to retain the isolated speckle. Each run's `changes.json` records
this choice. Geometry is in the input map frame, with metres for positions/lengths,
square metres for areas, and radians for origin yaw. The JSON schema is version 1;
it is a prototype file interface, not an established ROS message or alert API.

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

## Next milestones

1. Add and validate alignment/resampling using synthetic maps with known transforms.
   Independently saved SLAM maps will often have different origins and dimensions;
   this version intentionally rejects those pairs.
2. Obtain exported physical maps with their YAML metadata and an agreed reference
   layout. Measure mounting transforms and check calibration with the hardware team.
3. Validate against labelled additions/removals and unchanged repeated runs. Measure
   false alerts, missed regions, and location error. Tune filtering using these data.
4. Add a ROS `/map` adapter and agree on alert outputs with the interface owner.
5. Evaluate SLAM accuracy separately against independently measured ground truth.
   Synthetic comparison tests and a self-comparison of a Gazebo map do not establish
   IoU, wall error, or anomaly-detection accuracy on the physical robot.

Development was exercised on Linux with Python 3.12. The detector has no ROS, Docker,
or OS-specific runtime code; macOS operation still needs a test on a Mac.
