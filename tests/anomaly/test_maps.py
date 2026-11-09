"""Check file conventions against explicit pixels and physical coordinates."""

import math
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap, load_map, save_map
from swarm_anomaly.synthetic import make_scenarios


class MapTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.path = self.root / "map.yaml"

    def write_input(self, **overrides: object) -> None:
        Image.fromarray(np.array([[0, 254, 205], [254, 0, 205]], dtype=np.uint8)).save(
            self.root / "map.pgm"
        )
        metadata = {
            "image": "map.pgm",
            "resolution": 0.1,
            "origin": [1.0, 2.0, 0.0],
            "negate": 0,
            "free_thresh": 0.196,
            "occupied_thresh": 0.65,
        }
        metadata.update(overrides)
        self.path.write_text(yaml.safe_dump(metadata), encoding="utf-8")

    def test_load_flips_image_rows_and_preserves_metadata(self) -> None:
        self.write_input()
        grid = load_map(self.path)
        np.testing.assert_array_equal(
            grid.cells, [[FREE, OCCUPIED, UNKNOWN], [OCCUPIED, FREE, UNKNOWN]]
        )
        self.assertEqual(grid.origin, (1.0, 2.0, 0.0))
        self.assertAlmostEqual(grid.resolution, 0.1)
        self.assertEqual(grid.cells.dtype, np.int8)

    def test_negation_and_threshold_boundaries(self) -> None:
        self.write_input(negate=True, free_thresh=0.2, occupied_thresh=0.6)
        Image.fromarray(np.array([[51, 102, 153]], dtype=np.uint8)).save(self.root / "map.pgm")
        np.testing.assert_array_equal(load_map(self.path).cells, [[FREE, UNKNOWN, OCCUPIED]])

    def test_image_path_is_relative_to_yaml_directory(self) -> None:
        self.write_input(image="nested/map.png")
        (self.root / "nested").mkdir()
        Image.fromarray(np.array([[0]], dtype=np.uint8)).save(self.root / "nested/map.png")
        self.assertEqual(load_map(self.path).cells[0, 0], OCCUPIED)

    def test_bad_metadata_is_rejected(self) -> None:
        for overrides in (
            {"mode": "scale"},
            {"mode": "raw"},
            {"resolution": 0},
            {"resolution": float("nan")},
            {"resolution": True},
            {"origin": [1, 2]},
            {"origin": [0, 0, float("inf")]},
            {"free_thresh": 0.8},
            {"occupied_thresh": 2},
            {"negate": 2},
            {"image": ""},
        ):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.write_input(**overrides)
                load_map(self.path)

    def test_non_mapping_and_invalid_yaml_are_rejected(self) -> None:
        for text in ("[]", "origin: ["):
            with self.subTest(text=text), self.assertRaises(ValueError):
                self.path.write_text(text, encoding="utf-8")
                load_map(self.path)

    def test_colour_and_transparent_images_are_rejected(self) -> None:
        for mode in ("RGB", "RGBA", "LA"):
            with self.subTest(mode=mode), self.assertRaisesRegex(ValueError, "grayscale"):
                self.write_input(image="map.png")
                Image.new(mode, (2, 2)).save(self.root / "map.png")
                load_map(self.path)

    def test_saved_map_round_trip_and_overwrite_protection(self) -> None:
        original = OccupancyMap(
            np.array([[FREE, UNKNOWN, OCCUPIED]], dtype=np.int8), 0.05, (1, 2, 0.3)
        )
        save_map(original, self.path)
        loaded = load_map(self.path)
        np.testing.assert_array_equal(loaded.cells, original.cells)
        self.assertEqual(loaded.origin, original.origin)
        with self.assertRaises(FileExistsError):
            save_map(original, self.path)

    def test_grid_coordinates_include_rotation_and_cell_centre(self) -> None:
        grid = OccupancyMap(np.zeros((2, 2), dtype=np.int8), 0.5, (10, 20, math.pi / 2))
        np.testing.assert_allclose(grid.grid_to_world(0.5, 1.5), (9.25, 20.25))

    def test_invalid_cells_cannot_be_silently_cast(self) -> None:
        for cells in (np.array([[256]]), np.array([[0.5]]), np.array([]), np.array([0])):
            with self.subTest(cells=cells), self.assertRaises(ValueError):
                OccupancyMap(cells, 0.05)

    def test_map_owns_read_only_cell_data(self) -> None:
        original = np.zeros((1, 1), dtype=np.int8)
        grid = OccupancyMap(original, 0.05)
        original[0, 0] = OCCUPIED
        self.assertEqual(grid.cells[0, 0], FREE)
        with self.assertRaises(ValueError):
            grid.cells[0, 0] = OCCUPIED

    def test_synthetic_box_dimensions_are_known(self) -> None:
        scenarios = make_scenarios()
        reference = scenarios["reference"]
        self.assertEqual(reference.cells.shape, (80, 120))
        self.assertEqual(np.count_nonzero(scenarios["added"].cells != reference.cells), 100)
        self.assertEqual(np.count_nonzero(scenarios["moved"].cells != reference.cells), 200)
        self.assertEqual(np.count_nonzero(scenarios["partial"].cells == UNKNOWN), 4800)
