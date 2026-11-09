"""Clearance and connectivity effects for explicitly modelled warehouse aisles."""

from collections import deque
from contextlib import suppress
from dataclasses import dataclass
from math import ceil
from typing import Literal

import networkx as nx
import numpy as np
from numpy.typing import NDArray

from swarm_anomaly.detection import Comparison, compare_maps
from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap
from swarm_anomaly.warehouse import Aisle, Warehouse, grid_slices


@dataclass(frozen=True)
class AisleState:
    aisle_id: str
    status: Literal["clear", "restricted", "blocked", "unknown"]
    baseline_width_m: float
    remaining_width_m: float | None
    travel_cost_m: float | None
    added_area_m2: float
    change_ids: tuple[int, ...]


@dataclass(frozen=True)
class AisleImpact:
    state: AisleState
    baseline_bridge: bool
    bridge_after_other_blockages: bool
    alternate_route: tuple[str, ...]
    alternate_distance_m: float | None
    detour_extra_m: float | None
    alternate_route_observed: bool | None
    restored_regions: tuple[str, ...]
    restored_region_area_m2: float


@dataclass(frozen=True, eq=False)
class ConnectivityAnalysis:
    comparison: Comparison
    aisles: tuple[AisleImpact, ...]
    baseline_bridges: tuple[str, ...]
    current_bridges: tuple[str, ...]
    unreachable_regions: tuple[str, ...]
    unreachable_region_area_m2: float
    unknown_aisles: tuple[str, ...]


def _measure_passage(
    cells: NDArray[np.int8], resolution: float, required_width: float, *, horizontal: bool
) -> tuple[float | None, bool | None]:
    if np.any(cells == UNKNOWN):
        return None, None
    free = cells == FREE
    if not horizontal:
        free = free.T
    # Minimum across-aisle free run over all longitudinal slices.
    run = np.zeros(free.shape[1], dtype=np.int32)
    longest = np.zeros_like(run)
    for row in free:
        run = np.where(row, run + 1, 0)
        longest = np.maximum(longest, run)
    width = int(longest.min()) * resolution
    footprint = max(1, ceil(required_width / resolution - 1e-9))
    if footprint > min(free.shape):
        return width, False
    # A square footprint must have a continuous 4-connected route end to end.
    windows = np.lib.stride_tricks.sliding_window_view(free, (footprint, footprint))
    available = windows.all(axis=(-1, -2))
    pending = deque((int(row), 0) for row in np.flatnonzero(available[:, 0]))
    available[:, 0] = False
    height, length = available.shape
    while pending:
        row, column = pending.popleft()
        if column == length - 1:
            return width, True
        for y, x in ((row - 1, column), (row + 1, column), (row, column - 1), (row, column + 1)):
            if 0 <= y < height and 0 <= x < length and available[y, x]:
                available[y, x] = False
                pending.append((y, x))
    return width, False


def _aisle_state(
    layout: Warehouse,
    aisle: Aisle,
    reference: OccupancyMap,
    observed: OccupancyMap,
    comparison: Comparison,
) -> AisleState:
    section = grid_slices(layout.aisle_bounds(aisle), reference)
    baseline_width, baseline_route = _measure_passage(
        reference.cells[section],
        reference.resolution,
        layout.required_width_m,
        horizontal=layout.horizontal(aisle),
    )
    if baseline_width is None or not baseline_route:
        raise ValueError(f"Reference aisle '{aisle.id}' must be fully observed and traversable")
    width, route = _measure_passage(
        observed.cells[section],
        observed.resolution,
        layout.required_width_m,
        horizontal=layout.horizontal(aisle),
    )
    status: Literal["clear", "restricted", "blocked", "unknown"]
    if width is None:
        status = "unknown"
    elif not route:
        status = "blocked"
    elif width < baseline_width - 1e-9:
        status = "restricted"
    else:
        status = "clear"
    identifiers = set(np.unique(comparison.labels[section]))
    changes = tuple(
        change.id
        for change in comparison.changes
        if change.kind == "added" and change.id in identifiers
    )
    added = (reference.cells[section] == FREE) & (observed.cells[section] == OCCUPIED)
    return AisleState(
        aisle_id=aisle.id,
        status=status,
        baseline_width_m=baseline_width,
        remaining_width_m=width,
        travel_cost_m=(
            layout.aisle_length(aisle) * baseline_width / width if route and width else None
        ),
        added_area_m2=int(np.count_nonzero(added)) * reference.resolution**2,
        change_ids=changes,
    )


def _bridges(graph: nx.Graph) -> tuple[str, ...]:
    return tuple(sorted(str(graph.edges[a, b]["id"]) for a, b in nx.bridges(graph)))


def analyze_connectivity(
    layout: Warehouse, reference: OccupancyMap, observed: OccupancyMap, *, min_cells: int = 4
) -> ConnectivityAnalysis:
    """Apply all blocked aisles together; unknown aisles remain optimistically open.

    Graph effects are snapshot estimates under the explicit layout's assumptions.
    They do not establish real robot traversability or persistent obstruction.
    """
    comparison = compare_maps(reference, observed, min_cells=min_cells)
    for region in layout.regions:
        grid_slices(region.bounds_m, reference)
    states = {
        aisle.id: _aisle_state(layout, aisle, reference, observed, comparison)
        for aisle in layout.aisles
    }
    baseline = nx.Graph()
    baseline.add_nodes_from(region.id for region in layout.regions)
    for aisle in layout.aisles:
        length = layout.aisle_length(aisle)
        baseline.add_edge(aisle.start, aisle.end, id=aisle.id, length_m=length, cost_m=length)
    current = baseline.copy()
    for aisle in layout.aisles:
        state = states[aisle.id]
        if state.status == "blocked":
            current.remove_edge(aisle.start, aisle.end)
        elif state.travel_cost_m is not None:
            current.edges[aisle.start, aisle.end]["cost_m"] = state.travel_cost_m
    before = set(nx.node_connected_component(baseline, layout.entry_region))
    after = set(nx.node_connected_component(current, layout.entry_region))
    baseline_bridges, current_bridges = _bridges(baseline), _bridges(current)
    impacts: list[AisleImpact] = []
    for aisle in layout.aisles:
        state = states[aisle.id]
        restored = current.copy()
        restored.add_edge(aisle.start, aisle.end, **baseline.edges[aisle.start, aisle.end])
        recovered = tuple(
            sorted(set(nx.node_connected_component(restored, layout.entry_region)) - after)
        )
        alternate = current.copy()
        if alternate.has_edge(aisle.start, aisle.end):
            alternate.remove_edge(aisle.start, aisle.end)
        route: tuple[str, ...] = ()
        distance: float | None = None
        confirmed: bool | None = None
        with suppress(nx.NetworkXNoPath):
            route = tuple(nx.shortest_path(alternate, aisle.start, aisle.end, weight="length_m"))
        if route:
            edges = [alternate.edges[a, b] for a, b in zip(route[:-1], route[1:], strict=True)]
            distance = sum(float(edge["length_m"]) for edge in edges)
            confirmed = all(states[edge["id"]].status != "unknown" for edge in edges)
        impacts.append(
            AisleImpact(
                state=state,
                baseline_bridge=aisle.id in baseline_bridges,
                bridge_after_other_blockages=aisle.id in _bridges(restored),
                alternate_route=route,
                alternate_distance_m=distance,
                detour_extra_m=None
                if distance is None
                else max(0.0, distance - layout.aisle_length(aisle)),
                alternate_route_observed=confirmed,
                restored_regions=recovered,
                restored_region_area_m2=sum(layout.region(name).area_m2 for name in recovered),
            )
        )
    unreachable = tuple(sorted(before - after))
    return ConnectivityAnalysis(
        comparison=comparison,
        aisles=tuple(impacts),
        baseline_bridges=baseline_bridges,
        current_bridges=current_bridges,
        unreachable_regions=unreachable,
        unreachable_region_area_m2=sum(layout.region(name).area_m2 for name in unreachable),
        unknown_aisles=tuple(
            sorted(name for name, state in states.items() if state.status == "unknown")
        ),
    )
