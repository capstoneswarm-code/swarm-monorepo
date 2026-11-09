"""Bridge/detour expectations follow the declared graph, not measured outputs."""

import unittest

import numpy as np

from swarm_anomaly.connectivity import _measure_passage, analyze_connectivity
from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap
from swarm_anomaly.warehouse import grid_slices
from swarm_anomaly.warehouse_synthetic import make_warehouse_scenarios


class ConnectivityTests(unittest.TestCase):
    def test_unchanged_layout_has_only_the_three_expected_bridges(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        result = analyze_connectivity(layout, reference, maps["unchanged"])
        self.assertEqual(
            set(result.baseline_bridges), {"entry", "storage_access", "dead_end_access"}
        )
        self.assertEqual(result.current_bridges, result.baseline_bridges)
        self.assertEqual(result.unreachable_regions, ())
        self.assertTrue(all(impact.state.status == "clear" for impact in result.aisles))

    def test_partial_obstruction_reduces_width_and_increases_cost(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        result = analyze_connectivity(layout, reference, maps["partial"])
        lower = next(item for item in result.aisles if item.state.aisle_id == "lower")
        self.assertEqual(lower.state.status, "restricted")
        self.assertAlmostEqual(lower.state.remaining_width_m, 0.9)
        self.assertAlmostEqual(lower.state.travel_cost_m, 4 * 1.2 / 0.9)
        self.assertEqual(result.unreachable_regions, ())

    def test_blocked_loop_edge_has_a_twelve_metre_alternate_route(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        result = analyze_connectivity(layout, reference, maps["detour"])
        lower = next(item for item in result.aisles if item.state.aisle_id == "lower")
        self.assertEqual(lower.state.status, "blocked")
        self.assertFalse(lower.baseline_bridge)
        self.assertFalse(lower.bridge_after_other_blockages)
        self.assertEqual(lower.alternate_distance_m, 12)
        self.assertEqual(lower.detour_extra_m, 8)
        self.assertTrue(lower.alternate_route_observed)
        self.assertEqual(lower.restored_regions, ())
        self.assertIn("upper", result.current_bridges)

    def test_bottleneck_isolates_storage_area(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        result = analyze_connectivity(layout, reference, maps["bottleneck"])
        storage = next(item for item in result.aisles if item.state.aisle_id == "storage_access")
        self.assertEqual(result.unreachable_regions, ("storage",))
        self.assertEqual(result.unreachable_region_area_m2, 16)
        self.assertTrue(storage.baseline_bridge)
        self.assertIsNone(storage.alternate_distance_m)
        self.assertEqual(storage.restored_region_area_m2, 16)

    def test_two_nonbridge_blockages_jointly_disconnect_regions(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        result = analyze_connectivity(layout, reference, maps["joint_blockages"])
        self.assertEqual(set(result.unreachable_regions), {"east", "north_east", "storage"})
        self.assertEqual(result.unreachable_region_area_m2, 24)
        for impact in result.aisles:
            if impact.state.aisle_id in ("lower", "upper"):
                self.assertFalse(impact.baseline_bridge)
                self.assertTrue(impact.bridge_after_other_blockages)
                self.assertEqual(impact.restored_region_area_m2, 24)
                self.assertIsNone(impact.alternate_distance_m)

    def test_unknown_aisle_is_not_called_blocked(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        result = analyze_connectivity(layout, reference, maps["unknown"])
        lower = next(item for item in result.aisles if item.state.aisle_id == "lower")
        self.assertEqual(lower.state.status, "unknown")
        self.assertIsNone(lower.state.remaining_width_m)
        self.assertEqual(result.unknown_aisles, ("lower",))
        self.assertEqual(result.unreachable_regions, ())

    def test_alternate_route_with_unknown_section_is_not_confirmed(self) -> None:
        layout, reference, maps = make_warehouse_scenarios()
        cells = maps["detour"].cells.copy()
        cells[grid_slices((7.2, 7.4, 8.4, 8.6), reference)] = UNKNOWN
        result = analyze_connectivity(layout, reference, OccupancyMap(cells, reference.resolution))
        lower = next(item for item in result.aisles if item.state.aisle_id == "lower")
        self.assertEqual(lower.alternate_distance_m, 12)
        self.assertFalse(lower.alternate_route_observed)

    def test_cross_section_width_does_not_guarantee_a_continuous_path(self) -> None:
        cells = np.array(
            [[FREE, FREE, OCCUPIED, OCCUPIED], [OCCUPIED] * 4, [OCCUPIED, OCCUPIED, FREE, FREE]],
            dtype=np.int8,
        )
        width, traversable = _measure_passage(cells, 0.1, 0.1, horizontal=True)
        self.assertAlmostEqual(width, 0.1)
        self.assertFalse(traversable)

    def test_required_footprint_cannot_squeeze_through_a_narrow_gap(self) -> None:
        cells = np.full((12, 40), FREE, dtype=np.int8)
        cells[:7, 20:25] = OCCUPIED
        width, traversable = _measure_passage(cells, 0.1, 0.6, horizontal=True)
        self.assertAlmostEqual(width, 0.5)
        self.assertFalse(traversable)

    def test_invalid_reference_aisle_is_rejected(self) -> None:
        layout, _, maps = make_warehouse_scenarios()
        with self.assertRaisesRegex(ValueError, "Reference aisle"):
            analyze_connectivity(layout, maps["bottleneck"], maps["bottleneck"])
