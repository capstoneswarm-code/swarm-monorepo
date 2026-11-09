"""Exercise the user workflow through its actual module entry point."""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from typing import Any

import yaml
from PIL import Image


class CommandTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.maps = self.root / "maps"
        result = self.run_command("fixtures", "--output", str(self.maps))
        self.assertEqual(result.returncode, 0, result.stderr)

    def run_command(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, "-m", "swarm_anomaly", *args],
            capture_output=True,
            text=True,
            check=False,
            timeout=15,
        )

    def compare(self, scenario: str, *options: str) -> subprocess.CompletedProcess[str]:
        return self.run_command(
            "compare",
            "--reference",
            str(self.maps / "reference.yaml"),
            "--observed",
            str(self.maps / f"{scenario}.yaml"),
            "--output",
            str(self.root / scenario),
            *options,
        )

    def read_report(self, scenario: str) -> dict[str, Any]:
        return json.loads((self.root / scenario / "changes.json").read_text(encoding="utf-8"))

    def test_saved_maps_produce_regions_and_a_readable_overlay(self) -> None:
        result = self.compare("moved")
        self.assertEqual(result.returncode, 0, result.stderr)
        report = self.read_report("moved")
        self.assertEqual(report["status"], "compared")
        self.assertEqual([change["kind"] for change in report["changes"]], ["added", "removed"])
        self.assertEqual(report["changes"][0]["centroid_m"], [3.75, 2.25])
        with Image.open(self.root / "moved/overlay.png") as image:
            self.assertEqual(image.format, "PNG")
            self.assertEqual(image.mode, "RGB")
            self.assertGreater(image.width, 1000)
            image.verify()

    def test_partial_and_missing_observations_keep_coverage_in_report(self) -> None:
        for scenario, fraction in (("partial", 0.5), ("unobserved", 0.0)):
            with self.subTest(scenario=scenario):
                result = self.compare(scenario)
                self.assertEqual(result.returncode, 0, result.stderr)
                report = self.read_report(scenario)
                self.assertEqual(report["coverage"]["comparable_fraction"], fraction)
                self.assertEqual(report["changes"], [])
        self.assertEqual(self.read_report("unobserved")["status"], "no_comparable_cells")
        self.assertIn("Insufficient evidence", result.stdout)

    def test_mismatched_origin_fails_before_creating_output(self) -> None:
        path = self.maps / "added.yaml"
        metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
        metadata["origin"][0] = 0.05
        path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
        result = self.compare("added")
        self.assertEqual(result.returncode, 2)
        self.assertIn("already be aligned", result.stderr)
        self.assertNotIn("Traceback", result.stderr)
        self.assertFalse((self.root / "added").exists())

    def test_invalid_filter_reports_an_actionable_error(self) -> None:
        result = self.compare("added", "--min-cells", "0")
        self.assertEqual(result.returncode, 2)
        self.assertIn("positive integer", result.stderr)

    def test_comparison_outputs_are_not_overwritten(self) -> None:
        self.assertEqual(self.compare("added").returncode, 0)
        original = (self.root / "added/changes.json").read_bytes()
        result = self.compare("added")
        self.assertEqual(result.returncode, 2)
        self.assertIn("already exists", result.stderr)
        self.assertEqual((self.root / "added/changes.json").read_bytes(), original)

    def test_fixtures_are_not_overwritten(self) -> None:
        original = (self.maps / "reference.pgm").read_bytes()
        result = self.run_command("fixtures", "--output", str(self.maps))
        self.assertEqual(result.returncode, 2)
        self.assertEqual((self.maps / "reference.pgm").read_bytes(), original)

    def test_missing_input_is_reported_without_traceback(self) -> None:
        result = self.compare("does_not_exist")
        self.assertEqual(result.returncode, 2)
        self.assertIn("does_not_exist.yaml", result.stderr)
        self.assertNotIn("Traceback", result.stderr)
