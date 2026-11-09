"""Exercise exported warehouse reports, inputs, and standalone viewer output."""

import json
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from typing import Any

from PIL import Image

from swarm_anomaly.alerts import rank_alerts
from swarm_anomaly.connectivity import analyze_connectivity
from swarm_anomaly.maps import load_map
from swarm_anomaly.warehouse_synthetic import make_warehouse_scenarios
from swarm_anomaly.warehouse_view import render_warehouse_html


class WarehouseDemoTests(unittest.TestCase):
    def setUp(self) -> None:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.output = Path(directory.name) / "demo"

    def run_demo(self, *options: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                "-m",
                "swarm_anomaly",
                "warehouse-demo",
                "--output",
                str(self.output),
                *options,
            ],
            capture_output=True,
            text=True,
            check=False,
            timeout=15,
        )

    def read_report(self) -> dict[str, Any]:
        return json.loads((self.output / "alerts.json").read_text(encoding="utf-8"))

    def test_default_demo_exports_reproducible_maps_rankings_and_views(self) -> None:
        result = self.run_demo()
        self.assertEqual(result.returncode, 0, result.stderr)
        report = self.read_report()
        self.assertEqual(report["scenario"], "ranked")
        self.assertEqual(report["confirmation"], "unconfirmed_snapshot")
        self.assertEqual(report["unreachable_region_area_m2"], 20)
        self.assertEqual(
            [alert["severity"] for alert in report["alerts"]],
            ["critical", "critical", "medium", "low", "info"],
        )
        layout, _, _ = make_warehouse_scenarios()
        analysis = analyze_connectivity(
            layout,
            load_map(self.output / "reference.yaml"),
            load_map(self.output / "observed.yaml"),
        )
        self.assertEqual(
            [alert.id for alert in rank_alerts(layout, analysis)],
            [alert["id"] for alert in report["alerts"]],
        )
        exported_layout = json.loads((self.output / "warehouse.json").read_text(encoding="utf-8"))
        self.assertEqual(exported_layout["entry_region"], "loading")
        with Image.open(self.output / "map.png") as image:
            self.assertEqual(image.format, "PNG")
            self.assertGreater(image.width, 700)
            image.verify()
        page = (self.output / "index.html").read_text(encoding="utf-8")
        self.assertIn("data:image/png;base64,", page)
        self.assertIn("16.0 m²", page)
        self.assertIn('href="alerts.json"', page)
        for alert in report["alerts"]:
            self.assertIn(f'data-alert="{alert["id"]}"', page)
        self.assertIn("notifications are not enabled", result.stdout)

    def test_detour_report_and_view_include_the_alternate_route(self) -> None:
        result = self.run_demo("--scenario", "detour")
        self.assertEqual(result.returncode, 0, result.stderr)
        report = self.read_report()
        self.assertEqual(report["alerts"][0]["detour_extra_m"], 8)
        self.assertEqual(report["unreachable_region_area_m2"], 0)
        page = (self.output / "index.html").read_text(encoding="utf-8")
        self.assertIn('<polyline class="alternate" data-alert="aisle-lower"', page)
        self.assertIn("+8.0 m", page)

    def test_unknown_case_is_visible_as_incomplete_not_a_blockage(self) -> None:
        result = self.run_demo("--scenario", "unknown")
        self.assertEqual(result.returncode, 0, result.stderr)
        report = self.read_report()
        self.assertEqual(report["status"], "incomplete_observation")
        self.assertEqual(report["unknown_aisles"], ["lower"])
        self.assertEqual(report["alerts"], [])
        page = (self.output / "index.html").read_text(encoding="utf-8")
        self.assertIn("route estimates are incomplete", page)
        self.assertIn("No changes to rank in observed areas", page)

    def test_invalid_inputs_fail_before_writing_outputs(self) -> None:
        for arguments in (
            ("--scenario", "missing"),
            ("--required-width", "0"),
            ("--required-width", "nan"),
            ("--required-width", "2"),
        ):
            with self.subTest(arguments=arguments):
                result = self.run_demo(*arguments)
                self.assertEqual(result.returncode, 2)
                self.assertNotIn("Traceback", result.stderr)
                self.assertFalse(self.output.exists())

    def test_existing_report_is_preserved_without_partial_new_outputs(self) -> None:
        self.output.mkdir()
        report = self.output / "alerts.json"
        report.write_text("keep this report", encoding="utf-8")
        result = self.run_demo()
        self.assertEqual(result.returncode, 2)
        self.assertIn("Output already exists", result.stderr)
        self.assertEqual(report.read_text(encoding="utf-8"), "keep this report")
        self.assertEqual(list(self.output.iterdir()), [report])

    def test_layout_labels_and_scenario_are_escaped_in_html(self) -> None:
        layout, reference, scenarios = make_warehouse_scenarios()
        label = '<script>alert("label")</script>'
        layout = replace(
            layout,
            name=label,
            regions=tuple(replace(region, label=label) for region in layout.regions),
        )
        analysis = analyze_connectivity(layout, reference, scenarios["bottleneck"])
        page = render_warehouse_html(
            layout, scenarios["bottleneck"], analysis, rank_alerts(layout, analysis), scenario=label
        )
        self.assertNotIn(label, page)
        self.assertIn("&lt;script&gt;", page)
