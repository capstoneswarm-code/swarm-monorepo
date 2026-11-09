"""Export reproducible inputs, ranked JSON, and a local operator view."""

import json
from dataclasses import asdict, replace
from pathlib import Path

from swarm_anomaly.alerts import alert_report, rank_alerts
from swarm_anomaly.connectivity import analyze_connectivity
from swarm_anomaly.maps import save_map
from swarm_anomaly.warehouse_synthetic import make_warehouse_scenarios
from swarm_anomaly.warehouse_view import render_warehouse_html, render_warehouse_png


def write_warehouse_demo(
    output: Path, *, scenario: str = "ranked", required_width: float = 0.6
) -> None:
    layout, reference, scenarios = make_warehouse_scenarios()
    if scenario not in scenarios:
        raise ValueError(
            f"Unknown warehouse scenario '{scenario}'; choose from {', '.join(scenarios)}"
        )
    layout = replace(layout, required_width_m=required_width)
    observed = scenarios[scenario]
    analysis = analyze_connectivity(layout, reference, observed)
    alerts = rank_alerts(layout, analysis)
    report = alert_report(layout, analysis, alerts)
    report["scenario"] = scenario
    report["map_geometry"] = {
        "resolution_m": reference.resolution,
        "origin": reference.origin,
        "width": reference.cells.shape[1],
        "height": reference.cells.shape[0],
    }
    targets = (
        "warehouse.json",
        "reference.yaml",
        "reference.pgm",
        "observed.yaml",
        "observed.pgm",
        "alerts.json",
        "map.png",
        "index.html",
    )
    for name in targets:
        if (output / name).exists():
            raise FileExistsError(
                f"Output already exists: {output / name}. Choose a new directory."
            )
    output.mkdir(parents=True, exist_ok=True)
    save_map(reference, output / "reference.yaml")
    save_map(observed, output / "observed.yaml")
    (output / "warehouse.json").write_text(
        json.dumps(asdict(layout), indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    (output / "alerts.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    render_warehouse_png(layout, observed, analysis, alerts).save(output / "map.png")
    (output / "index.html").write_text(
        render_warehouse_html(layout, observed, analysis, alerts, scenario=scenario),
        encoding="utf-8",
    )
    print(
        f"{len(alerts)} ranked alerts; "
        f"{analysis.unreachable_region_area_m2:.1f} m² of regions inaccessible."
    )
    print(
        f"Open {output / 'index.html'} in a browser. Snapshot only; notifications are not enabled."
    )
