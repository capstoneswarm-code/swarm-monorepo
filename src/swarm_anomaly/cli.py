"""Generate synthetic examples and compare saved occupancy maps."""

import argparse
import json
from collections.abc import Iterable, Sequence
from dataclasses import asdict
from pathlib import Path

from swarm_anomaly.detection import compare_maps
from swarm_anomaly.maps import load_map, save_map
from swarm_anomaly.synthetic import make_scenarios
from swarm_anomaly.visualization import render_comparison


def _prepare_output(directory: Path, names: Iterable[str]) -> None:
    for name in names:
        target = directory / name
        if target.exists():
            raise FileExistsError(
                f"Output already exists: {target}. Choose a new output directory."
            )
    directory.mkdir(parents=True, exist_ok=True)


def _write_fixtures(output: Path) -> None:
    scenarios = make_scenarios()
    _prepare_output(
        output, (f"{name}{suffix}" for name in scenarios for suffix in (".yaml", ".pgm"))
    )
    for name, grid in scenarios.items():
        save_map(grid, output / f"{name}.yaml")
    print(f"Saved {len(scenarios)} synthetic map pairs to {output}")


def _write_comparison(
    reference_path: Path, observed_path: Path, output: Path, min_cells: int
) -> None:
    reference, observed = load_map(reference_path), load_map(observed_path)
    result = compare_maps(reference, observed, min_cells=min_cells)
    report = {
        "schema_version": 1,
        "reference_map": str(reference_path.resolve()),
        "observed_map": str(observed_path.resolve()),
        "status": "compared" if result.comparable_cells else "no_comparable_cells",
        "geometry": {
            "width": reference.cells.shape[1],
            "height": reference.cells.shape[0],
            "resolution_m": reference.resolution,
            "origin": reference.origin,
        },
        "parameters": {"min_cells": min_cells, "connectivity": 4},
        "coverage": {
            "comparable_cells": result.comparable_cells,
            "total_cells": result.total_cells,
            "comparable_fraction": result.comparable_fraction,
        },
        "filtered_change_cells": result.filtered_change_cells,
        "changes": [asdict(change) for change in result.changes],
    }
    _prepare_output(output, ("changes.json", "overlay.png"))
    render_comparison(reference, observed, result).save(output / "overlay.png")
    (output / "changes.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    if result.comparable_cells:
        print(
            f"Found {len(result.changes)} candidate regions; "
            f"comparable area {result.comparable_fraction:.1%}"
        )
    else:
        print("Insufficient evidence: no cells are known in both maps")
    print(f"Saved {output / 'changes.json'} and {output / 'overlay.png'}")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare aligned occupancy maps without ROS")
    commands = parser.add_subparsers(dest="command", required=True)
    fixtures = commands.add_parser("fixtures", help="Generate a room and known change scenarios")
    fixtures.add_argument(
        "--output", type=Path, required=True, help="Directory for synthetic map pairs"
    )
    compare = commands.add_parser("compare", help="Compare already aligned trinary maps")
    compare.add_argument("--reference", type=Path, required=True, help="Expected map YAML")
    compare.add_argument("--observed", type=Path, required=True, help="Observed map YAML")
    compare.add_argument(
        "--output", type=Path, required=True, help="Directory for JSON and PNG results"
    )
    compare.add_argument(
        "--min-cells", type=int, default=4, help="Minimum connected region size (default: 4)"
    )
    args = parser.parse_args(argv)
    try:
        if args.command == "fixtures":
            _write_fixtures(args.output)
        else:
            _write_comparison(args.reference, args.observed, args.output, args.min_cells)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 0
