"""Trinary maps with ROS coordinates: row zero is the bottom of the image."""

from dataclasses import dataclass
from math import cos, isfinite, sin
from pathlib import Path

import numpy as np
import yaml
from numpy.typing import NDArray
from PIL import Image

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass(frozen=True, eq=False)
class OccupancyMap:
    """A grid of -1/0/100 cells; origin is the lower-left corner in metres/radians."""

    cells: NDArray[np.int8]
    resolution: float
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        cells = np.asarray(self.cells)
        if cells.ndim != 2 or not cells.size:
            raise ValueError("Map cells must be a nonempty two-dimensional grid")
        if not np.isin(cells, (UNKNOWN, FREE, OCCUPIED)).all():
            raise ValueError("Map cells must be -1 (unknown), 0 (free), or 100 (occupied)")
        if not isfinite(self.resolution) or self.resolution <= 0:
            raise ValueError("Map resolution must be finite and greater than zero")
        if len(self.origin) != 3 or not all(isfinite(value) for value in self.origin):
            raise ValueError("Map origin must contain three finite numbers: x, y, yaw")
        # Own the data so editing an input array cannot silently change a reference map.
        cells = np.array(cells, dtype=np.int8, copy=True)
        cells.setflags(write=False)
        object.__setattr__(self, "cells", cells)
        object.__setattr__(self, "origin", tuple(float(value) for value in self.origin))

    def grid_to_world(self, column: float, row: float) -> tuple[float, float]:
        """Transform grid coordinates; add 0.5 to indices to address cell centres."""
        x, y, yaw = self.origin
        dx, dy = column * self.resolution, row * self.resolution
        return x + cos(yaw) * dx - sin(yaw) * dy, y + sin(yaw) * dx + cos(yaw) * dy


def _number(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not isfinite(value):
        raise ValueError(f"Map YAML field '{field}' must be a finite number")
    return float(value)


def load_map(path: Path) -> OccupancyMap:
    """Load ROS map YAML with an 8-bit grayscale PGM/PNG in trinary mode.

    Colour, transparency, and raw/scale modes are rejected rather than guessed.
    Thresholds are inclusive: occupied >= occupied_thresh, free <= free_thresh.
    """
    try:
        metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as error:
        raise ValueError(f"Invalid map YAML: {path}") from error
    if not isinstance(metadata, dict):
        raise ValueError("Map YAML must contain a mapping")
    if metadata.get("mode", "trinary") != "trinary":
        raise ValueError("Only trinary map mode is supported")
    image_name = metadata.get("image")
    if not isinstance(image_name, str) or not image_name.strip():
        raise ValueError("Map YAML field 'image' must be a nonempty path")
    resolution = _number(metadata.get("resolution"), "resolution")
    origin = metadata.get("origin")
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError("Map YAML field 'origin' must be [x, y, yaw]")
    x, y, yaw = (_number(value, "origin") for value in origin)
    free = _number(metadata.get("free_thresh"), "free_thresh")
    occupied = _number(metadata.get("occupied_thresh"), "occupied_thresh")
    if not 0 <= free < occupied <= 1:
        raise ValueError("Map thresholds must satisfy 0 <= free_thresh < occupied_thresh <= 1")
    negate = metadata.get("negate")
    if not isinstance(negate, int | bool) or negate not in (0, 1):
        raise ValueError("Map YAML field 'negate' must be 0 or 1")
    with Image.open(path.parent / image_name) as image:
        if image.format not in ("PPM", "PNG") or image.mode != "L" or "transparency" in image.info:
            raise ValueError("Map image must be an opaque 8-bit grayscale PGM or PNG")
        gray = np.asarray(image, dtype=np.float64)
    probability = gray / 255.0 if negate else (255.0 - gray) / 255.0
    cells = np.full(gray.shape, UNKNOWN, dtype=np.int8)
    cells[probability >= occupied] = OCCUPIED
    cells[probability <= free] = FREE
    return OccupancyMap(np.flipud(cells), resolution, (x, y, yaw))


def save_map(grid: OccupancyMap, path: Path) -> None:
    """Save a YAML/PGM pair with fixed trinary thresholds, without overwriting files."""
    if path.suffix not in (".yaml", ".yml"):
        raise ValueError("Map metadata output must end in .yaml or .yml")
    image_path = path.with_suffix(".pgm")
    for target in (path, image_path):
        if target.exists():
            raise FileExistsError(f"Output already exists: {target}")
    pixels = np.full(grid.cells.shape, 205, dtype=np.uint8)
    pixels[grid.cells == FREE] = 254
    pixels[grid.cells == OCCUPIED] = 0
    metadata = {
        "image": image_path.name,
        "mode": "trinary",
        "resolution": float(grid.resolution),
        "origin": list(grid.origin),
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.flipud(pixels)).save(image_path)
    path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
