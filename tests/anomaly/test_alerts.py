"""Rank operational impact with reasons and explicit snapshot uncertainty."""

import json
import unittest

from swarm_anomaly.alerts import alert_report, rank_alerts
from swarm_anomaly.connectivity import analyze_connectivity
from swarm_anomaly.maps import OCCUPIED, OccupancyMap
from swarm_anomaly.warehouse import grid_slices
from swarm_anomaly.warehouse_synthetic import make_warehouse_scenarios


class AlertTests(unittest.TestCase):
    def test_storage_is_ranked_above_dead_end_and_larger_open_area_obstacle(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        analysis = analyze_connectivity(layout, reference, maps["ranked"])
        alerts = rank_alerts(layout, analysis)
        self.assertEqual(
            [alert.severity for alert in alerts], ["critical", "critical", "medium", "low", "info"]
        )
        self.assertEqual(alerts[0].aisle_id, "storage_access")
        self.assertEqual(alerts[1].aisle_id, "dead_end_access")
        self.assertEqual([alert.rank for alert in alerts], [1, 2, 3, 4, 5])
        self.assertLess(alerts[0].added_or_removed_area_m2, alerts[3].added_or_removed_area_m2)
        self.assertIn("16.0 m²", " ".join(alerts[0].reasons))

    def test_detour_alert_explains_extra_distance(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        alerts = rank_alerts(layout, analyze_connectivity(layout, reference, maps["detour"]))
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "high")
        self.assertEqual(alerts[0].detour_extra_m, 8)
        self.assertIn("+8.0 m", " ".join(alerts[0].reasons))

    def test_joint_blockage_alerts_explain_updated_bridge_status(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        analysis = analyze_connectivity(layout, reference, maps["joint_blockages"])
        alerts = rank_alerts(layout, analysis)
        self.assertEqual(len(alerts), 2)
        self.assertTrue(all(alert.severity == "critical" for alert in alerts))
        self.assertTrue(all("becomes a bridge" in " ".join(alert.reasons) for alert in alerts))
        report = alert_report(layout, analysis, alerts)
        self.assertEqual(report["unreachable_region_area_m2"], 24)
        self.assertEqual(report["confirmation"], "unconfirmed_snapshot")
        json.dumps(report, allow_nan=False)

    def test_unknown_aisle_without_detected_change_does_not_make_an_alert(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        analysis = analyze_connectivity(layout, reference, maps["unknown"])
        alerts = rank_alerts(layout, analysis)
        self.assertEqual(alerts, ())
        self.assertEqual(alert_report(layout, analysis, alerts)["status"], "incomplete_observation")

    def test_known_change_in_unknown_aisle_requests_review(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        cells = maps["unknown"].cells.copy()
        cells[grid_slices((8.6, 3.4, 9.0, 4.6), reference)] = OCCUPIED
        analysis = analyze_connectivity(
            layout, reference, OccupancyMap(cells, reference.resolution)
        )
        alerts = rank_alerts(layout, analysis)
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0].severity, "review")
        self.assertEqual(alerts[0].category, "needs_observation")

    def test_unchanged_map_has_no_alerts(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        self.assertEqual(
            rank_alerts(layout, analyze_connectivity(layout, reference, maps["unchanged"])), ()
        )
