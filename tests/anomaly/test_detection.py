"""Known changes and adverse inputs, independent of ROS and robot hardware."""

import math
import unittest

import numpy as np

from swarm_anomaly.detection import compare_maps
from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap
from swarm_anomaly.synthetic import make_scenarios


class DetectionTests(unittest.TestCase):
    def test_synthetic_scenarios_have_expected_regions(self) -> None:
        maps = make_scenarios()
        expectations = {
            "unchanged": [],
            "added": ["added"],
            "removed": ["removed"],
            "moved": ["added", "removed"],
            "partial": [],
            "unobserved": [],
            "speckle": [],
        }
        for name, expected in expectations.items():
            with self.subTest(scenario=name):
                result = compare_maps(maps["reference"], maps[name])
                self.assertEqual([change.kind for change in result.changes], expected)
                for change in result.changes:
                    self.assertEqual(change.cell_count, 100)
                    self.assertAlmostEqual(change.area_m2, 0.25)

    def test_box_coordinates_use_metres_and_cell_edges(self) -> None:
        maps = make_scenarios()
        result = compare_maps(maps["reference"], maps["added"])
        change = result.changes[0]
        np.testing.assert_allclose(change.centroid_m, (3.75, 2.25))
        self.assertEqual(change.bounds_cells, (70, 40, 80, 50))
        np.testing.assert_allclose(
            change.bounds_polygon_m, ((3.5, 2), (4, 2), (4, 2.5), (3.5, 2.5))
        )
        self.assertEqual(np.count_nonzero(result.labels == change.id), 100)

    def test_unknown_cells_in_either_map_do_not_create_changes(self) -> None:
        reference = OccupancyMap(np.array([[UNKNOWN, OCCUPIED, FREE, FREE]], dtype=np.int8), 1)
        observed = OccupancyMap(
            np.array([[OCCUPIED, UNKNOWN, UNKNOWN, OCCUPIED]], dtype=np.int8), 1
        )
        result = compare_maps(reference, observed, min_cells=1)
        self.assertEqual(result.comparable_cells, 1)
        self.assertEqual(result.comparable_fraction, 0.25)
        self.assertEqual(len(result.changes), 1)
        self.assertEqual(result.changes[0].bounds_cells, (3, 0, 4, 1))

    def test_partial_and_absent_coverage_are_reported(self) -> None:
        maps = make_scenarios()
        partial = compare_maps(maps["reference"], maps["partial"])
        absent = compare_maps(maps["reference"], maps["unobserved"])
        self.assertEqual(partial.comparable_fraction, 0.5)
        self.assertEqual(absent.comparable_cells, 0)
        self.assertEqual(absent.changes, ())

    def test_small_region_filter_is_configurable_and_counted(self) -> None:
        maps = make_scenarios()
        filtered = compare_maps(maps["reference"], maps["speckle"])
        retained = compare_maps(maps["reference"], maps["speckle"], min_cells=1)
        self.assertEqual(filtered.filtered_change_cells, 1)
        self.assertEqual(retained.filtered_change_cells, 0)
        self.assertEqual(retained.changes[0].cell_count, 1)

    def test_diagonal_cells_are_separate_and_boundary_cells_are_included(self) -> None:
        reference = OccupancyMap(np.zeros((3, 3), dtype=np.int8), 1)
        observed = OccupancyMap(np.eye(3, dtype=np.int8) * OCCUPIED, 1)
        result = compare_maps(reference, observed, min_cells=1)
        self.assertEqual([change.cell_count for change in result.changes], [1, 1, 1])
        self.assertEqual([change.id for change in result.changes], [1, 2, 3])

    def test_adjacent_added_and_removed_cells_are_separate_regions(self) -> None:
        reference = OccupancyMap(np.array([[FREE, OCCUPIED]], dtype=np.int8), 1)
        observed = OccupancyMap(np.array([[OCCUPIED, FREE]], dtype=np.int8), 1)
        result = compare_maps(reference, observed, min_cells=1)
        self.assertEqual([change.kind for change in result.changes], ["added", "removed"])

    def test_regions_respect_nonzero_origin_and_rotation(self) -> None:
        origin = (10.0, 20.0, math.pi / 2)
        reference = OccupancyMap(np.zeros((1, 1), dtype=np.int8), 0.5, origin)
        observed = OccupancyMap(np.full((1, 1), OCCUPIED, dtype=np.int8), 0.5, origin)
        change = compare_maps(reference, observed, min_cells=1).changes[0]
        np.testing.assert_allclose(change.centroid_m, (9.75, 20.25))
        np.testing.assert_allclose(
            change.bounds_polygon_m, ((10, 20), (10, 20.5), (9.5, 20.5), (9.5, 20))
        )

    def test_incompatible_geometry_is_rejected(self) -> None:
        reference = OccupancyMap(np.zeros((2, 2), dtype=np.int8), 0.05)
        variants = (
            OccupancyMap(np.zeros((3, 2), dtype=np.int8), 0.05),
            OccupancyMap(reference.cells, 0.1),
            OccupancyMap(reference.cells, 0.05, (0.05, 0, 0)),
            OccupancyMap(reference.cells, 0.05, (0, 0, 0.1)),
        )
        for observed in variants:
            with self.subTest(observed=observed), self.assertRaisesRegex(ValueError, "aligned"):
                compare_maps(reference, observed)

    def test_full_turn_is_the_same_orientation(self) -> None:
        reference = OccupancyMap(np.zeros((1, 1), dtype=np.int8), 0.05)
        observed = OccupancyMap(reference.cells, 0.05, (0, 0, math.tau))
        self.assertEqual(compare_maps(reference, observed).changes, ())

    def test_invalid_minimum_size_is_rejected(self) -> None:
        grid = make_scenarios()["reference"]
        for value in (0, -1, True, 0.5):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, "positive integer"):
                compare_maps(grid, grid, min_cells=value)
