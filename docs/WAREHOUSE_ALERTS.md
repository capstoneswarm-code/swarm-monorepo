# Warehouse connectivity and ranked alerts

This prototype follows the design report's section H (Abnormality Severity and
Connectivity Analysis). It uses an explicitly defined warehouse graph and synthetic
occupancy maps. It does not extract topology from arbitrary SLAM maps.

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

## Planned delivery stages

1. Explicit warehouse geometry and repeatable fixtures.
2. Aisle clearance, blockage, bridge, and detour analysis.
3. Deterministic severity ranking with written reasons.
4. An operator map displaying ranked alerts and connectivity effects.
5. Email or SMS notifications consuming confirmed alert events. This requires
   persistent event IDs, confirmation across observations, deduplication/cooldown,
   escalation/resolution handling, delivery retries, a provider/sender, and chosen
   recipients. Prototype snapshot results must not automatically trigger messages.

There is no email/SMS transport or live notification sending in this demo.
