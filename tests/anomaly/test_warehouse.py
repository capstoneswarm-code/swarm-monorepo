"""Check the explicit geometry and independently constructed scenario changes."""

import unittest
from dataclasses import replace

import numpy as np

from swarm_anomaly.detection import compare_maps
from swarm_anomaly.maps import FREE, UNKNOWN
from swarm_anomaly.warehouse import Aisle, Region, grid_slices
from swarm_anomaly.warehouse_synthetic import make_warehouse_scenarios


class WarehouseTests(unittest.TestCase):
    def test_reference_aisles_are_free_and_regions_have_distinct_areas(self) -> None:
        layout, reference, _ = make_warehouse_scenarios()
        for aisle in layout.aisles:
            self.assertTrue(
                np.all(reference.cells[grid_slices(layout.aisle_bounds(aisle), reference)] == FREE)
            )
        self.assertEqual(layout.region("storage").area_m2, 16)
        self.assertEqual(layout.region("dead_end").area_m2, 4)

    def test_scenarios_have_known_changes(self) -> None:
        _, reference, maps = make_warehouse_scenarios()
        counts = {
            "unchanged": 0,
            "open_area": 1,
            "partial": 1,
            "bottleneck": 1,
            "dead_end": 1,
            "detour": 1,
            "joint_blockages": 2,
            "ranked": 5,
            "unknown": 0,
        }
        for name, count in counts.items():
            with self.subTest(name=name):
                self.assertEqual(len(compare_maps(reference, maps[name]).changes), count)
        self.assertTrue(np.any(maps["unknown"].cells == UNKNOWN))

    def test_invalid_graph_geometry_is_rejected(self) -> None:
        layout, _, _ = make_warehouse_scenarios()
        for aisles in (
            (Aisle("bad", "loading", "missing", 1.2),),
            (Aisle("diagonal", "loading", "north_east", 1.2),),
            (layout.aisles[0], Aisle("duplicate", "west", "loading", 1.2)),
        ):
            with self.subTest(aisles=aisles), self.assertRaises(ValueError):
                replace(layout, aisles=aisles)
        with self.assertRaises(ValueError):
            replace(layout, regions=(*layout.regions, layout.regions[0]))
        with self.assertRaises(ValueError):
            replace(layout, entry_region="missing")
        with self.assertRaises(ValueError):
            replace(layout, required_width_m=0)

    def test_geometry_must_align_to_cells_and_fit_map(self) -> None:
        _, reference, _ = make_warehouse_scenarios()
        for bounds in ((0.05, 0, 1, 1), (-1, 0, 1, 1), (0, 0, 19, 1)):
            with self.subTest(bounds=bounds), self.assertRaises(ValueError):
                grid_slices(bounds, reference)

    def test_region_centre_and_aisle_width_are_validated(self) -> None:
        with self.assertRaises(ValueError):
            Region("bad", "Bad", (2, 2), (0, 0, 1, 1))
        with self.assertRaises(ValueError):
            Aisle("bad", "a", "b", float("nan"))
