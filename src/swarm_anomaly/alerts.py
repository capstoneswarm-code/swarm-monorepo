"""Explainable snapshot ranking; notification delivery is a separate future stage."""

from dataclasses import asdict, dataclass, replace
from typing import Literal

from swarm_anomaly.connectivity import AisleImpact, ConnectivityAnalysis
from swarm_anomaly.warehouse import Point, Warehouse

type Severity = Literal["critical", "high", "review", "medium", "low", "info"]
PRIORITY: dict[Severity, int] = {
    "critical": 5,
    "high": 4,
    "review": 3,
    "medium": 2,
    "low": 1,
    "info": 0,
}


@dataclass(frozen=True)
class RankedAlert:
    id: str
    rank: int
    severity: Severity
    category: str
    title: str
    reasons: tuple[str, ...]
    aisle_id: str | None
    change_ids: tuple[int, ...]
    location_m: Point
    added_or_removed_area_m2: float
    width_lost_m: float
    restored_region_area_m2: float
    detour_extra_m: float | None
    bridge_after_other_blockages: bool


def _aisle_alert(layout: Warehouse, impact: AisleImpact) -> RankedAlert | None:
    state = impact.state
    if state.status == "clear" or (state.status == "unknown" and not state.added_area_m2):
        return None
    aisle = next(edge for edge in layout.aisles if edge.id == state.aisle_id)
    start, end = layout.region(aisle.start), layout.region(aisle.end)
    reasons: list[str] = []
    severity: Severity
    category: str
    if state.status == "unknown":
        severity, category = "review", "needs_observation"
        reasons.append(
            "Observed changes overlap an incompletely observed aisle; blockage is unclassified."
        )
    elif state.status == "restricted":
        severity, category = "medium", "partial_obstruction"
        reasons.append(
            f"Free width fell from {state.baseline_width_m:.2f} m "
            f"to {state.remaining_width_m:.2f} m; "
            f"a route for the {layout.required_width_m:.2f} m model footprint remains."
        )
        reasons.append(
            f"Model travel cost is {state.travel_cost_m:.2f} metre-equivalents, "
            "not a measured delay."
        )
    else:
        severity = "critical" if impact.restored_regions else "high"
        category = "path_blockage"
        reasons.append(
            f"No continuous route remains for the {layout.required_width_m:.2f} m model footprint."
        )
        if impact.restored_regions:
            labels = ", ".join(layout.region(name).label for name in impact.restored_regions)
            reasons.append(
                "Clearing this aisle would restore access from "
                f"{layout.region(layout.entry_region).label} "
                f"to {labels} ({impact.restored_region_area_m2:.1f} m² of declared regions)."
            )
        if impact.alternate_distance_m is not None:
            qualifier = "Observed" if impact.alternate_route_observed else "Unconfirmed"
            reasons.append(
                f"{qualifier} alternate route: {impact.alternate_distance_m:.1f} m "
                f"(+{impact.detour_extra_m:.1f} m versus this aisle)."
            )
        else:
            reasons.append("No alternate route connects the aisle endpoints in the updated graph.")
    if impact.baseline_bridge:
        reasons.append("This aisle is a bridge in the reference graph.")
    elif impact.bridge_after_other_blockages:
        reasons.append("This aisle becomes a bridge when the other blockages are applied.")
    if impact.alternate_route_observed is False:
        reasons.append(
            "The alternate route includes unknown aisles; connectivity estimates are optimistic."
        )
    width_lost = (
        max(0.0, state.baseline_width_m - state.remaining_width_m)
        if state.remaining_width_m is not None
        else 0.0
    )
    return RankedAlert(
        id=f"aisle-{aisle.id}",
        rank=0,
        severity=severity,
        category=category,
        title=f"{start.label} → {end.label}",
        reasons=tuple(reasons),
        aisle_id=aisle.id,
        change_ids=state.change_ids,
        location_m=(
            (start.center_m[0] + end.center_m[0]) / 2,
            (start.center_m[1] + end.center_m[1]) / 2,
        ),
        added_or_removed_area_m2=state.added_area_m2,
        width_lost_m=width_lost,
        restored_region_area_m2=impact.restored_region_area_m2,
        detour_extra_m=impact.detour_extra_m,
        bridge_after_other_blockages=impact.bridge_after_other_blockages,
    )


def rank_alerts(layout: Warehouse, analysis: ConnectivityAnalysis) -> tuple[RankedAlert, ...]:
    """Rank aisle impacts first, then retain unassociated spatial changes for review.

    Aisle alerts aggregate obstacles on that aisle. Clearing an aisle can restore
    regions also listed by another alert, so per-alert recovery areas are not additive.
    """
    alerts = [
        alert for impact in analysis.aisles if (alert := _aisle_alert(layout, impact)) is not None
    ]
    associated = {identifier for alert in alerts for identifier in alert.change_ids}
    for change in analysis.comparison.changes:
        if change.id in associated:
            continue
        removed = change.kind == "removed"
        transition = "occupied to free" if removed else "free to occupied"
        alerts.append(
            RankedAlert(
                id=f"change-{change.id}",
                rank=0,
                severity="info" if removed else "low",
                category="removed_obstacle" if removed else "area_change",
                title="Removed obstacle" if removed else "Added spatial change",
                reasons=(
                    f"{change.area_m2:.2f} m² changed from {transition}.",
                    "No aisle blockage is attributed to this region; "
                    "region-internal routing is not assessed.",
                ),
                aisle_id=None,
                change_ids=(change.id,),
                location_m=change.centroid_m,
                added_or_removed_area_m2=change.area_m2,
                width_lost_m=0,
                restored_region_area_m2=0,
                detour_extra_m=None,
                bridge_after_other_blockages=False,
            )
        )
    alerts.sort(
        key=lambda alert: (
            -PRIORITY[alert.severity],
            -alert.restored_region_area_m2,
            -int(alert.bridge_after_other_blockages),
            -(alert.detour_extra_m or 0),
            -alert.width_lost_m,
            -alert.added_or_removed_area_m2,
            alert.id,
        )
    )
    return tuple(replace(alert, rank=index) for index, alert in enumerate(alerts, 1))


def alert_report(
    layout: Warehouse, analysis: ConnectivityAnalysis, alerts: tuple[RankedAlert, ...]
) -> dict[str, object]:
    comparison = analysis.comparison
    status = "assessed"
    if not comparison.comparable_cells:
        status = "no_comparable_cells"
    elif analysis.unknown_aisles or comparison.comparable_fraction < 1:
        status = "incomplete_observation"
    return {
        "schema_version": 1,
        "kind": "warehouse_snapshot",
        "status": status,
        "confirmation": "unconfirmed_snapshot",
        "ranking_policy": "warehouse-demo-v1",
        "layout": asdict(layout),
        "coverage": {
            "comparable_fraction": comparison.comparable_fraction,
            "comparable_cells": comparison.comparable_cells,
            "total_cells": comparison.total_cells,
        },
        "baseline_bridges": analysis.baseline_bridges,
        "current_bridges": analysis.current_bridges,
        "unknown_aisles": analysis.unknown_aisles,
        "unreachable_regions": analysis.unreachable_regions,
        "unreachable_region_area_m2": analysis.unreachable_region_area_m2,
        "aisles": [asdict(impact) for impact in analysis.aisles],
        "changes": [asdict(change) for change in comparison.changes],
        "alerts": [asdict(alert) for alert in alerts],
    }
