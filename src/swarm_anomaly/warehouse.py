"""Explicit, axis-aligned warehouse geometry for connectivity experiments."""

import re
from dataclasses import dataclass
from math import isclose, isfinite, remainder, tau

from swarm_anomaly.maps import OccupancyMap

type Point = tuple[float, float]
type Bounds = tuple[float, float, float, float]


def _identifier(value: str) -> None:
    if not re.fullmatch(r"[a-z][a-z0-9_-]*", value):
        raise ValueError(
            "Identifiers must start with a lowercase letter and use letters/digits/_/-"
        )


def _overlap(a: Bounds, b: Bounds) -> bool:
    return max(a[0], b[0]) < min(a[2], b[2]) and max(a[1], b[1]) < min(a[3], b[3])


@dataclass(frozen=True)
class Region:
    id: str
    label: str
    center_m: Point
    bounds_m: Bounds

    def __post_init__(self) -> None:
        _identifier(self.id)
        if not self.label.strip() or not all(isfinite(v) for v in (*self.center_m, *self.bounds_m)):
            raise ValueError("Regions need a label and finite geometry")
        x0, y0, x1, y1 = self.bounds_m
        x, y = self.center_m
        if not (x0 < x < x1 and y0 < y < y1):
            raise ValueError("Region centre must be strictly inside its bounds")

    @property
    def area_m2(self) -> float:
        x0, y0, x1, y1 = self.bounds_m
        return (x1 - x0) * (y1 - y0)


@dataclass(frozen=True)
class Aisle:
    id: str
    start: str
    end: str
    width_m: float

    def __post_init__(self) -> None:
        _identifier(self.id)
        if not isfinite(self.width_m) or self.width_m <= 0:
            raise ValueError("Aisle width must be finite and positive")
        if self.start == self.end:
            raise ValueError("An aisle must connect two distinct regions")


@dataclass(frozen=True)
class Warehouse:
    name: str
    regions: tuple[Region, ...]
    aisles: tuple[Aisle, ...]
    entry_region: str
    required_width_m: float

    def __post_init__(self) -> None:
        regions = {region.id: region for region in self.regions}
        if (
            not self.regions
            or len(regions) != len(self.regions)
            or self.entry_region not in regions
        ):
            raise ValueError("Warehouse needs unique regions and a valid entry region")
        if not isfinite(self.required_width_m) or self.required_width_m <= 0:
            raise ValueError("Required travel width must be finite and positive")
        if len({edge.id for edge in self.aisles}) != len(self.aisles):
            raise ValueError("Aisle identifiers must be unique")
        pairs: set[frozenset[str]] = set()
        for aisle in self.aisles:
            if aisle.start not in regions or aisle.end not in regions:
                raise ValueError("Aisle endpoints must name existing regions")
            pair = frozenset((aisle.start, aisle.end))
            if pair in pairs:
                raise ValueError("Parallel graph edges are not supported; declare another junction")
            pairs.add(pair)
            a, b = regions[aisle.start].center_m, regions[aisle.end].center_m
            if (a[0] == b[0]) == (a[1] == b[1]):
                raise ValueError("Aisles must be nonzero horizontal or vertical segments")
        for index, region in enumerate(self.regions):
            if any(
                _overlap(region.bounds_m, other.bounds_m) for other in self.regions[index + 1 :]
            ):
                raise ValueError("Region areas must not overlap")
        for index, aisle in enumerate(self.aisles):
            for other in self.aisles[index + 1 :]:
                if not {aisle.start, aisle.end} & {other.start, other.end} and _overlap(
                    self.aisle_bounds(aisle), self.aisle_bounds(other)
                ):
                    raise ValueError("Intersecting aisles require an explicit shared junction")

    def region(self, identifier: str) -> Region:
        for region in self.regions:
            if region.id == identifier:
                return region
        raise KeyError(identifier)

    def horizontal(self, aisle: Aisle) -> bool:
        return self.region(aisle.start).center_m[1] == self.region(aisle.end).center_m[1]

    def aisle_bounds(self, aisle: Aisle) -> Bounds:
        a, b = self.region(aisle.start).center_m, self.region(aisle.end).center_m
        half = aisle.width_m / 2
        if self.horizontal(aisle):
            return min(a[0], b[0]), a[1] - half, max(a[0], b[0]), a[1] + half
        return a[0] - half, min(a[1], b[1]), a[0] + half, max(a[1], b[1])

    def aisle_length(self, aisle: Aisle) -> float:
        a, b = self.region(aisle.start).center_m, self.region(aisle.end).center_m
        return abs(a[0] - b[0]) + abs(a[1] - b[1])


def grid_slices(bounds: Bounds, grid: OccupancyMap) -> tuple[slice, slice]:
    """Require exact cell-edge geometry so aisle widths are not silently rounded."""
    if abs(remainder(grid.origin[2], tau)) > 1e-9:
        raise ValueError("Warehouse analysis requires an axis-aligned reference grid")
    x0, y0, x1, y1 = bounds
    coordinates = (
        (x0 - grid.origin[0]) / grid.resolution,
        (y0 - grid.origin[1]) / grid.resolution,
        (x1 - grid.origin[0]) / grid.resolution,
        (y1 - grid.origin[1]) / grid.resolution,
    )
    indices = tuple(round(value) for value in coordinates)
    if not all(
        isclose(value, index, rel_tol=0, abs_tol=1e-7)
        for value, index in zip(coordinates, indices, strict=True)
    ):
        raise ValueError("Warehouse geometry must align with map cell edges")
    left, bottom, right, top = indices
    height, width = grid.cells.shape
    if not (0 <= left < right <= width and 0 <= bottom < top <= height):
        raise ValueError("Warehouse geometry must lie inside the map")
    return slice(bottom, top), slice(left, right)
