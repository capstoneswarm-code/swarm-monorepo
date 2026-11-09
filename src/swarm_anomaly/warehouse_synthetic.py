"""A hand-defined warehouse with a loop, critical access aisle, and small dead end."""

import numpy as np

from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap
from swarm_anomaly.warehouse import Aisle, Bounds, Region, Warehouse, grid_slices


def make_warehouse_scenarios() -> tuple[Warehouse, OccupancyMap, dict[str, OccupancyMap]]:
    layout = Warehouse(
        name="Synthetic warehouse",
        entry_region="loading",
        required_width_m=0.6,
        regions=(
            Region("loading", "Loading", (2, 4), (1, 3, 3, 5)),
            Region("west", "West junction", (6, 4), (5, 3, 7, 5)),
            Region("east", "East junction", (10, 4), (9, 3, 11, 5)),
            Region("north_west", "North-west junction", (6, 8), (5, 7, 7, 9)),
            Region("north_east", "North-east junction", (10, 8), (9, 7, 11, 9)),
            Region("storage", "Storage", (14, 4), (13, 2, 17, 6)),
            Region("dead_end", "Small dead end", (6, 12), (5, 11, 7, 13)),
        ),
        aisles=(
            Aisle("entry", "loading", "west", 1.2),
            Aisle("lower", "west", "east", 1.2),
            Aisle("left", "west", "north_west", 1.2),
            Aisle("upper", "north_west", "north_east", 1.2),
            Aisle("right", "east", "north_east", 1.2),
            Aisle("storage_access", "east", "storage", 1.2),
            Aisle("dead_end_access", "north_west", "dead_end", 1.2),
        ),
    )
    blank = OccupancyMap(np.full((140, 180), OCCUPIED, dtype=np.int8), 0.1)
    cells = blank.cells.copy()
    for bounds in (
        *[region.bounds_m for region in layout.regions],
        *[layout.aisle_bounds(edge) for edge in layout.aisles],
    ):
        cells[grid_slices(bounds, blank)] = FREE
    baseline_box = (15.2, 4.6, 15.8, 5.2)
    cells[grid_slices(baseline_box, blank)] = OCCUPIED
    reference = OccupancyMap(cells, blank.resolution)
    obstacles: dict[str, tuple[Bounds, ...]] = {
        "unchanged": (),
        "open_area": ((15.2, 2.5, 16.2, 3.5),),
        "partial": ((7.4, 3.4, 8.0, 3.7),),
        "bottleneck": ((11.5, 3.4, 12.1, 4.6),),
        "dead_end": ((5.4, 9.6, 6.6, 10.2),),
        "detour": ((7.4, 3.4, 8.0, 4.6),),
        "joint_blockages": ((7.4, 3.4, 8.0, 4.6), (7.4, 7.4, 8.0, 8.6)),
        "ranked": (
            (15.2, 2.5, 16.2, 3.5),
            (7.4, 3.4, 8.0, 3.7),
            (11.5, 3.4, 12.1, 4.6),
            (5.4, 9.6, 6.6, 10.2),
        ),
        "unknown": (),
    }
    maps: dict[str, OccupancyMap] = {}
    for name, rectangles in obstacles.items():
        observed = reference.cells.copy()
        for bounds in rectangles:
            observed[grid_slices(bounds, reference)] = OCCUPIED
        if name == "ranked":
            observed[grid_slices(baseline_box, reference)] = FREE
        elif name == "unknown":
            observed[grid_slices((7.2, 3.4, 8.4, 4.6), reference)] = UNKNOWN
        maps[name] = OccupancyMap(observed, reference.resolution)
    return layout, reference, maps
