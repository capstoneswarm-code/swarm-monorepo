#!/usr/bin/env python3
"""Tune the existing swarm workspace; run with the robot launch stopped.

Usage from ~/Documents/swarm-mono:
    python3 ~/Downloads/tune_nav2_performance.py --speed 0.5
    python3 ~/Downloads/tune_nav2_performance.py --speed 1.0

Updates only robot_demo.py, nav2_params.yaml, and selected explorer functions.
Never changes hardware calibration, mounting transforms, or map files.
Backups are written to a new nav_tuning_backups directory for each invocation.
The speed is a command ceiling, not a measured stopping-performance guarantee.
"""

import argparse
import ast
from datetime import datetime
import math
from pathlib import Path
import re
import shutil
import statistics
import tempfile
import time
import types

import numpy as np
from scipy import ndimage as ndi
import yaml


def large_frontiers(mask, min_cells):
    labels, _ = ndi.label(mask, structure=np.ones((3, 3), dtype=bool))
    counts = np.bincount(labels.ravel())
    keep = counts >= min_cells
    keep[0] = False
    return keep[labels]


def replace_top_function(code, name, replacement):
    nodes = [n for n in ast.parse(code).body
             if isinstance(n, ast.FunctionDef) and n.name == name]
    if len(nodes) != 1:
        raise ValueError(f"Expected one top-level {name} function; files unchanged")
    node = nodes[0]
    lines = code.splitlines(keepends=True)
    lines[node.lineno-1:node.end_lineno] = [replacement.rstrip() + "\n"]
    return "".join(lines)


def change_once(code, pattern, replacement):
    changed, count = re.subn(pattern, lambda _: replacement, code)
    if count != 1:
        raise ValueError(f"Expected one match for {pattern!r}, got {count}; files unchanged")
    return changed


def make_updates(root, speed):
    bridge = root / "robot_demo.py"
    explorer = root / "frontier_explorer.py"
    params = root / "nav2_params.yaml"
    source = bridge.read_text()
    tree = ast.parse(source)
    nodes = [n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "NavVelocity"]
    if len(nodes) != 1:
        raise ValueError("Expected NavVelocity bridge class; files unchanged")
    node = nodes[0]
    lines = source.splitlines(keepends=True)
    block = "".join(lines[node.lineno-1:node.end_lineno])
    number = r"[0-9]+(?:\.[0-9]+)?"
    block = change_once(block, rf"abs\(linear\) / {number}", f"abs(linear) / {speed:.3f}")
    block = change_once(block, rf"max\(abs\(left\), abs\(right\)\) / {number}",
                        f"max(abs(left), abs(right)) / {speed + 0.15:.3f}")
    lines[node.lineno-1:node.end_lineno] = [block]
    source = change_once("".join(lines), rf"dict\(config, speed_m_s={number},",
                         f"dict(config, speed_m_s={speed:.3f},")

    config = yaml.safe_load(params.read_text())
    planner = config["planner_server"]["ros__parameters"]["GridBased"]
    planner["use_astar"] = True
    follow = config["controller_server"]["ros__parameters"]["FollowPath"]
    follow.update(
        desired_linear_vel=speed, use_velocity_scaled_lookahead_dist=True,
        min_lookahead_dist=0.30, max_lookahead_dist=1.20,
        lookahead_time=1.1, approach_velocity_scaling_dist=1.20,
        max_allowed_time_to_collision_up_to_carrot=2.0)
    # Retain measured/tuned acceleration and braking limits, reverse/yaw caps,
    # resolution, costmap update rates, and controller frequency.
    config["velocity_smoother"]["ros__parameters"]["max_velocity"][0] = speed
    monitor = config["collision_monitor"]["ros__parameters"]
    if "StopZone" not in monitor["polygons"] or monitor["StopZone"]["action_type"] != "stop":
        raise ValueError("Expected enabled StopZone in the command chain; files unchanged")
    if not monitor.get("enabled", True) or not monitor["StopZone"].get("enabled", True):
        raise ValueError("Collision Monitor or StopZone is disabled; files unchanged")
    if "FootprintApproach" not in monitor["polygons"]:
        monitor["polygons"].append("FootprintApproach")
    monitor["FootprintApproach"] = {
        "type": "polygon", "action_type": "approach",
        "footprint_topic": "/local_costmap/published_footprint",
        "time_before_collision": 2.0, "simulation_time_step": 0.05,
        "min_points": 3, "visualize": False, "enabled": True}

    code = explorer.read_text()
    tool_source = Path(__file__).read_text()
    tool_tree = ast.parse(tool_source)
    # Keep the existing vectorized NumPy clearance/dilation: they benchmark
    # faster here than replacing the whole pipeline with SciPy transforms.
    for name in ("large_frontiers",):
        fn = next(n for n in tool_tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
        replacement = ast.get_source_segment(tool_source, fn)
        code = replace_top_function(code, name, replacement)
    if "from scipy import ndimage as ndi" not in code:
        code = change_once(code, r"(?m)^import numpy as np$",
                            "import numpy as np\nfrom scipy import ndimage as ndi")
    code = code.replace("eligible = (distance >= 0.30) & (distance <= 4.0)",
                        "eligible = np.isfinite(distance) & (distance >= 0.30)")
    code = change_once(code, rf"(?m)^PREFERRED_GOAL_M = {number}$",
                        "PREFERRED_GOAL_M = 3.0")
    # Far goals need a path-length-based time allowance, not a 60 s deadline.
    if "self.goal_budget_s =" not in code:
        code = change_once(code, r"(?m)^            self\.cancel_since = None$",
                           "            self.cancel_since = None\n            self.goal_budget_s = 60.0")
        code = change_once(code, r"now-self\.phase_since > 60\.0", "now-self.phase_since > self.goal_budget_s")
        anchor = "                request = NavigateToPose.Goal()"
        if code.count(anchor) != 1:
            raise ValueError("Expected NavigateToPose action; files unchanged")
        code = code.replace(anchor,
            "                points = reply.result.path.poses\n"
            "                length = sum(math.hypot(b.pose.position.x - a.pose.position.x,\n"
            "                                        b.pose.position.y - a.pose.position.y)\n"
            "                             for a, b in zip(points, points[1:]))\n"
            "                self.goal_budget_s = max(60.0, 20.0 + length / 0.15)\n"
            + anchor, 1)
    if "eligible = np.isfinite(distance) & (distance >= 0.30)" not in code:
        raise ValueError("Unexpected frontier distance filter; files unchanged")
    if 'if __name__ == "__main__":' not in code:
        raise ValueError("Explorer is missing its entry point; files unchanged")
    updates = {bridge: source, explorer: code,
               params: yaml.safe_dump(config, sort_keys=False)}
    for path, content in updates.items():
        if path.suffix == ".py":
            compile(content, str(path), "exec")
    return updates


def benchmark_frontiers(before, after):
    """Measure just the changed frontier grouping on this machine."""
    code = {}
    for name, source in (("before", before), ("after", after)):
        namespace = types.ModuleType(name)
        exec(compile(source, f"<{name}>", "exec"), namespace.__dict__)
        code[name] = namespace
    mask = np.zeros((500, 500), dtype=bool)
    mask[20:480:5, 20:480] = True
    result = {}
    for name, namespace in code.items():
        namespace.large_frontiers(mask, 3)
        samples = []
        for _ in range(3):
            started = time.perf_counter()
            namespace.large_frontiers(mask, 3)
            samples.append(time.perf_counter() - started)
        result[name] = statistics.median(samples)
    print("Frontier grouping benchmark (synthetic 500x500 grid, this machine): "
          f"{result['before']*1000:.1f} -> {result['after']*1000:.1f} ms")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed", type=float, required=True, help="Autonomous command ceiling, 0.15–1.0 m/s")
    parser.add_argument("--check-only", action="store_true", help="Validate without modifying the workspace")
    parser.add_argument("--benchmark", action="store_true", help="Time frontier grouping before applying")
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.15 <= args.speed <= 1.0:
        parser.error("Use a speed from 0.15 to 1.0 m/s")
    workspace = Path.home() / "Documents/swarm-mono"
    if Path.cwd().resolve() != workspace.resolve():
        parser.error("Run this command from ~/Documents/swarm-mono")
    try:
        updates = make_updates(workspace / "src/wheel_ekf_demo", args.speed)
    except (OSError, ValueError, KeyError, SyntaxError, StopIteration) as error:
        parser.exit(1, f"Not applied: {error}\n")
    if args.benchmark:
        explorer = workspace / "src/wheel_ekf_demo/frontier_explorer.py"
        benchmark_frontiers(explorer.read_text(), updates[explorer])
    if args.check_only:
        print("Validated all three files; no changes written.")
        return
    parent = workspace / "nav_tuning_backups"
    parent.mkdir(exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("%Y%m%d-%H%M%S-"), dir=parent))
    for path in updates:
        shutil.copy2(path, backup / path.name)
    try:
        for path, content in updates.items():
            path.write_text(content)
    except OSError:
        for path in updates:
            shutil.copy2(backup / path.name, path)
        raise
    print(f"Applied {args.speed:.2f} m/s command ceiling, A* search, and compiled frontier operations.")
    print(f"Backup: {backup}")
    print("No maximum frontier distance. Calibration and sensor transforms preserved.")
    print("Test straight-line tracking and braking in clear space before exploration.")


if __name__ == "__main__":
    main()
