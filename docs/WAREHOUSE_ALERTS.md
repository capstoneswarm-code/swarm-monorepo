# Warehouse connectivity and ranked alerts

This prototype follows the design report's section H (Abnormality Severity and
Connectivity Analysis). It uses an explicitly defined warehouse graph and synthetic
occupancy maps. It does not extract topology from arbitrary SLAM maps.

## Run the demo

From the repository root, using Python 3.12 or newer:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
swarm-anomaly warehouse-demo --output artifacts/warehouse-demo/ranked
```

Open `artifacts/warehouse-demo/ranked/index.html` in your browser. No ROS, Gazebo,
web server, actual warehouse map, or network connection is required to use the
generated viewer. Dependencies need to be installed first. Generated files are
ignored by Git; use a new output directory for each run to preserve previous results.

The default case produces five alerts: storage access, dead-end access, partial
aisle obstruction, an open-area addition, and an obstacle removal. The small
storage blockage ranks above the larger open-area obstacle because it cuts off
access to a 16 m² region. Storage and the dead end together account for 20 m² of
newly inaccessible declared regions.

Try the alternative-route, simultaneous-blockage, and unknown-area cases:

```bash
swarm-anomaly warehouse-demo --scenario detour --output artifacts/warehouse-demo/detour
swarm-anomaly warehouse-demo --scenario joint_blockages --output artifacts/warehouse-demo/joint
swarm-anomaly warehouse-demo --scenario unknown --output artifacts/warehouse-demo/unknown
```

| `--scenario` | Expected result at the default 0.6 m required width |
| --- | --- |
| `unchanged` | No alerts |
| `open_area` | One low-priority addition; no modeled aisle impact |
| `partial` | One medium alert; width reduced from 1.2 m to 0.9 m |
| `bottleneck` | One critical blockage; storage's 16 m² becomes inaccessible |
| `dead_end` | One critical blockage; a 4 m² dead end becomes inaccessible |
| `detour` | One high alert; a 12 m alternative replaces a 4 m aisle, adding 8 m |
| `joint_blockages` | Two critical alerts; together they isolate 24 m² |
| `ranked` | Five alerts, ordered critical / critical / medium / low / info |
| `unknown` | No detected-change alerts; incomplete observation explicitly reported |

`--required-width METRES` changes the footprint-plus-clearance assumption. For
example, `--required-width 1.0` makes the `partial` case blocked rather than merely
restricted. It must be positive, finite, and permit passage through the reference.

## Viewer and exports

Select an alert card or a numbered map marker to link the explanation to its
location. Markers also support keyboard focus and Enter/Space. If an alternate
route exists, selecting that alert displays its path. The aisle-graph checkbox
toggles edges, junction labels, and alternate paths. Dashed red rectangles mark
inaccessible regions. In an aisle alert, the numbered marker identifies the aisle
midpoint; colored occupancy cells identify the detected obstacle itself.

Marker and change colors follow alert severity: critical red, high orange, review
purple, medium gold, low blue, and info green-grey. Edge colors indicate aisle
state, so every blocked edge is red even if its alert is high rather than critical.
This differs from the basic `compare` overlay, which colors additions and removals
without severity. Unknown observations remain visible and never imply a removal.

Each demo directory contains:

- `warehouse.json`: the declared region and aisle geometry.
- `reference.yaml` / `reference.pgm`: expected occupancy map in ROS map format.
- `observed.yaml` / `observed.pgm`: the selected scenario in the same frame.
- `alerts.json`: ranked explanations, measurements, graph effects, raw changes,
  map geometry, coverage, and explicit snapshot status.
- `index.html`: a standalone interactive viewer with the map embedded inside it.
- `map.png`: a static overview with numbered alerts for sharing alongside the report.

Keep the JSON and PNG beside the HTML for its export links. This is an offline
operator-view prototype; it is not integrated with RViz, Zhuoyi's live map viewer,
or a ROS `/map` subscription. The CLI currently generates the built-in scenarios.
Custom layouts can use the Python model and analysis API; loading arbitrary
warehouse JSON and saved maps into this graph pipeline is not yet a CLI workflow.

## Layout and scope

`warehouse_synthetic.make_warehouse_scenarios()` defines a warehouse with seven
regions and seven straight aisles. A central loop supplies alternate routes;
separate aisles lead to a 16 m² storage region and a 4 m² dead end. Loading is the
entry point used for reachability. All aisles are 1.2 m wide; required travel width
is 0.6 m, representing a configurable footprint plus clearance, not a measured
parameter of the physical robot.

Regions have non-overlapping rectangular areas, and aisles connect their declared
centres. Geometry must align to the axis-aligned map's cell boundaries. The model
assumes each region remains internally connected and aisle junctions are usable;
obstacles inside a region require further local analysis before navigation use.
Reported inaccessible area will count declared region rectangles, not every square
metre of aisle floor or semantic business value.

Fixtures include unchanged, open-area obstacle, partial narrowing, storage
bottleneck, blocked dead end, available detour, simultaneous loop blockages,
mixed ranking, and incomplete-observation scenarios. They are made from known
rectangles and graph geometry independently of the connectivity analysis.

## Connectivity analysis

`analyze_connectivity(layout, reference, observed)` requires aligned maps. Each
reference aisle must be fully observed and traversable. The observation's minimum
contiguous free width is measured across aisle cross-sections. A square footprint
of the required width, rounded up to whole cells, must also have a continuous
four-connected route between the aisle ends. A favourable width measurement alone
does not establish a route. This conservative raster approximation is not a Nav2
planner or a model of the physical robot's turning dynamics.

Blocked aisles are removed together. A restricted aisle stays in the graph with
cost `length * baseline_width / remaining_width`; this is a prototype travel-cost
penalty expressed in metre-equivalents, not a measured delay or detour distance.
The separate alternate-route distance uses physical aisle lengths. Bridge status
is computed for the original and updated graphs. For each blocked aisle, analysis
also evaluates restoring that aisle while leaving every other blockage in place.
Its restored-region area means access regained by clearing **all** obstacles on
that aisle under the model, not necessarily by moving one individual object.

Unknown aisles remain optimistically available at baseline cost, are explicitly
listed, and make route/impact estimates incomplete. They are never called clear or
blocked from missing data. Alternate routes that use unknown aisles are flagged as
unconfirmed. Only newly unreachable regions relative to the reference are counted.

Clearance uses all occupied cells, even if the detector's minimum-region filter
excluded a tiny change. This prevents the grouping threshold from silently opening
a physically narrow passage. Temporal confirmation remains future work.

Graph analysis uses NetworkX's [bridge detection](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.bridges.bridges.html)
and [weighted shortest paths](https://networkx.org/documentation/stable/reference/algorithms/generated/networkx.algorithms.shortest_paths.generic.shortest_path.html).

## Ranking policy

`rank_alerts` groups impacts by aisle, retaining the underlying added-region IDs.
One physical obstacle can affect multiple aisles. Other spatial changes remain
separate alerts; a moved object is still an addition plus a removal, not object
recognition. Current prototype levels are:

| Level | Meaning |
| --- | --- |
| Critical | Clearing the blocked aisle would restore entry access to a region |
| High | Aisle blocked, but clearing it alone adds no entry-reachable region |
| Review | A detected change overlaps an aisle with incomplete observations |
| Medium | Aisle narrowed, with a route remaining for the model footprint |
| Low | Added spatial change without an attributed aisle capacity impact |
| Info | Removed obstacle |

Within each level, order by restored region area, bridge status after other
blockages, detour distance, lost width, changed area, then stable lexical ID.
These are documented prototype rules, not thresholds validated with operators.
Each alert includes reasons and measurements. Per-alert restored areas can overlap:
the two-blockage scenario has two alternative repairs, each restoring the same
24 m². Use the global unreachable area, never the sum of per-alert areas.

Every JSON report is marked `unconfirmed_snapshot`. The aisle IDs are stable in
the declared layout; pixel-region IDs are not stable across time. No temporal
tracking, persistent confirmation, or notifications are implied by a high rank.

## Verification

```bash
python -m pip install -r requirements-dev.txt
python -m unittest discover -s tests/anomaly -v
ruff check .
ruff format --check .
```

The tests cover graph bridges, alternate routes, joint disconnections, footprint
clearance, unknown observations, ranking, saved-map reproduction, CLI errors, and
exported reports. Synthetic geometry provides known expected answers; these tests
do not measure detection accuracy on a physical robot. Browser selection, keyboard
controls, graph visibility, detour display, and a narrow viewport were also checked
in Chromium on Linux. A native macOS run remains unverified.

## Delivery stages and next work

1. **Implemented:** explicit warehouse geometry and repeatable fixtures.
2. **Implemented:** aisle clearance, blockage, bridge, and detour analysis.
3. **Implemented:** deterministic severity ranking with written reasons.
4. **Implemented:** an offline operator map displaying alerts and connectivity effects.
5. **Next:** email or SMS notifications consuming confirmed alert events. This requires
   persistent event IDs, confirmation across observations, deduplication/cooldown,
   escalation/resolution handling, delivery retries, a provider/sender, and chosen
   recipients. Prototype snapshot results must not automatically trigger messages.

There is no email/SMS transport or live notification sending in this demo.
Start step 5 with a local event stream and a dry-run delivery log: confirm a
candidate across observations, assign an event ID, suppress repeat messages, and
emit resolved events. Then add a selected email/SMS provider and recipient
configuration. Keep credentials in environment variables or ignored local files.
Messages should include severity, affected aisle, reason, time, and map location.
Live map ingestion, alignment reliability, and validation with labelled physical
changes remain separate integration work.
