"""Known geometric constructions independent of the resampling implementation."""

import math
import unittest

import numpy as np

from swarm_anomaly.alignment import RigidTransform, resample_to_reference
from swarm_anomaly.detection import compare_maps
from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap


class AlignmentTests(unittest.TestCase):
    def setUp(self) -> None:
        self.cells = np.array([[FREE, OCCUPIED, UNKNOWN], [OCCUPIED, FREE, FREE]], dtype=np.int8)

    def test_identity_preserves_cells_with_rotated_nonzero_origin(self) -> None:
        reference = OccupancyMap(self.cells, 0.05, (12.3, -4.1, 0.37))
        aligned = resample_to_reference(reference, reference, transform=RigidTransform())
        np.testing.assert_array_equal(aligned.cells, reference.cells)
        self.assertEqual(aligned.resolution, reference.resolution)
        self.assertEqual(aligned.origin, reference.origin)

    def test_different_extent_and_origin_in_common_frame(self) -> None:
        reference = OccupancyMap(self.cells, 0.5)
        padded = np.full((6, 9), UNKNOWN, dtype=np.int8)
        padded[2:4, 3:6] = self.cells
        observed = OccupancyMap(padded, 0.5, (-1.5, -1.0, 0))
        aligned = resample_to_reference(reference, observed, transform=RigidTransform())
        np.testing.assert_array_equal(aligned.cells, reference.cells)

    def test_grid_origin_yaw_is_applied_in_common_frame(self) -> None:
        reference = OccupancyMap(self.cells, 0.5)
        observed = OccupancyMap(np.rot90(self.cells), 0.5, (1.5, 0, math.pi / 2))
        aligned = resample_to_reference(reference, observed, transform=RigidTransform())
        np.testing.assert_array_equal(aligned.cells, reference.cells)

    def test_translation_direction_is_observed_to_reference(self) -> None:
        reference = OccupancyMap(self.cells, 0.5)
        observed = OccupancyMap(self.cells, 0.5, (-2, 1, 0))
        aligned = resample_to_reference(reference, observed, transform=RigidTransform(2, -1))
        np.testing.assert_array_equal(aligned.cells, reference.cells)
        inverse = resample_to_reference(reference, observed, transform=RigidTransform(-2, 1))
        self.assertTrue(np.all(inverse.cells == UNKNOWN))

    def test_known_quarter_turn_between_frames(self) -> None:
        reference = OccupancyMap(self.cells, 0.5)
        observed = OccupancyMap(np.rot90(self.cells), 0.5)
        aligned = resample_to_reference(
            reference, observed, transform=RigidTransform(1.5, 0, math.pi / 2)
        )
        np.testing.assert_array_equal(aligned.cells, reference.cells)

    def test_arbitrary_rotation_samples_expected_physical_cell(self) -> None:
        # Observed cell centre (2.5, 3.5), rotated +30 degrees then translated (1, -2).
        target_x, target_y = 1.4150635094610964, 2.2810889132455356
        source = np.full((6, 6), UNKNOWN, dtype=np.int8)
        source[3, 2] = OCCUPIED
        observed = OccupancyMap(source, 1)
        reference = OccupancyMap(
            np.zeros((1, 1), dtype=np.int8), 1, (target_x - 0.5, target_y - 0.5, 0)
        )
        aligned = resample_to_reference(
            reference, observed, transform=RigidTransform(1, -2, math.pi / 6)
        )
        self.assertEqual(aligned.cells[0, 0], OCCUPIED)

    def test_coarse_cells_expand_without_blending_unknown_values(self) -> None:
        observed = OccupancyMap(self.cells, 1)
        reference = OccupancyMap(np.zeros((4, 6), dtype=np.int8), 0.5)
        aligned = resample_to_reference(reference, observed, transform=RigidTransform())
        expected = np.repeat(np.repeat(self.cells, 2, axis=0), 2, axis=1)
        np.testing.assert_array_equal(aligned.cells, expected)

    def test_fine_cells_sample_onto_a_coarse_reference(self) -> None:
        observed = OccupancyMap(np.repeat(np.repeat(self.cells, 2, axis=0), 2, axis=1), 0.5)
        reference = OccupancyMap(self.cells, 1)
        aligned = resample_to_reference(reference, observed, transform=RigidTransform())
        np.testing.assert_array_equal(aligned.cells, self.cells)

    def test_partial_overlap_retains_unknown_border_and_detects_visible_change(self) -> None:
        reference = OccupancyMap(np.zeros((4, 4), dtype=np.int8), 1)
        observed = OccupancyMap(
            np.array([[OCCUPIED, FREE], [FREE, FREE]], dtype=np.int8), 1, (1, 1, 0)
        )
        aligned = resample_to_reference(reference, observed, transform=RigidTransform())
        self.assertTrue(np.all(aligned.cells[0, :] == UNKNOWN))
        self.assertTrue(np.all(aligned.cells[:, -1] == UNKNOWN))
        result = compare_maps(reference, aligned, min_cells=1)
        self.assertEqual(result.comparable_fraction, 0.25)
        self.assertEqual(len(result.changes), 1)
        self.assertEqual(result.changes[0].centroid_m, (1.5, 1.5))

    def test_no_overlap_produces_no_comparable_cells(self) -> None:
        reference = OccupancyMap(self.cells, 0.5)
        aligned = resample_to_reference(reference, reference, transform=RigidTransform(1000, 1000))
        result = compare_maps(reference, aligned)
        self.assertEqual(result.comparable_cells, 0)
        self.assertEqual(result.changes, ())

    def test_fractional_translation_respects_half_open_boundaries(self) -> None:
        reference = OccupancyMap(np.zeros((1, 3), dtype=np.int8), 1)
        observed = OccupancyMap(np.array([[OCCUPIED, FREE]], dtype=np.int8), 1)
        aligned = resample_to_reference(reference, observed, transform=RigidTransform(0.5, 0))
        np.testing.assert_array_equal(aligned.cells, [[OCCUPIED, FREE, UNKNOWN]])

    def test_tall_map_is_fully_sampled(self) -> None:
        cells = np.tile(self.cells, (140, 1))
        grid = OccupancyMap(cells, 0.05, (1, 2, 0.7))
        aligned = resample_to_reference(grid, grid, transform=RigidTransform())
        np.testing.assert_array_equal(aligned.cells, cells)

    def test_nonfinite_transform_is_rejected(self) -> None:
        for values in ((float("nan"), 0, 0), (0, float("inf"), 0), (0, 0, float("inf"))):
            with self.subTest(values=values), self.assertRaisesRegex(ValueError, "finite"):
                RigidTransform(*values)
