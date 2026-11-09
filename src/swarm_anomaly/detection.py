"""Detect occupancy changes between maps already expressed on the same grid."""

from collections import deque
from collections.abc import Iterator
from dataclasses import dataclass
from math import isclose, remainder, tau
from typing import Literal

import numpy as np
from numpy.typing import NDArray

from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap


@dataclass(frozen=True)
class Change:
    """One four-connected region, with a centroid and bounding polygon in metres."""

    id: int
    kind: Literal["added", "removed"]
    cell_count: int
    area_m2: float
    centroid_m: tuple[float, float]
    bounds_cells: tuple[int, int, int, int]
    bounds_polygon_m: tuple[tuple[float, float], ...]


@dataclass(frozen=True, eq=False)
class Comparison:
    """Labels contain Change.id, or zero for cells without a retained change."""

    changes: tuple[Change, ...]
    labels: NDArray[np.int32]
    comparable_cells: int
    total_cells: int
    filtered_change_cells: int
    min_cells: int

    @property
    def comparable_fraction(self) -> float:
        return self.comparable_cells / self.total_cells


def _check_geometry(reference: OccupancyMap, observed: OccupancyMap) -> None:
    tolerance = 1e-9
    same_resolution = isclose(
        reference.resolution, observed.resolution, rel_tol=0, abs_tol=tolerance
    )
    same_position = all(
        isclose(a, b, rel_tol=0, abs_tol=tolerance)
        for a, b in zip(reference.origin[:2], observed.origin[:2], strict=True)
    )
    same_yaw = abs(remainder(reference.origin[2] - observed.origin[2], tau)) <= tolerance
    if not (
        reference.cells.shape == observed.cells.shape
        and same_resolution
        and same_position
        and same_yaw
    ):
        raise ValueError(
            "Maps must already be aligned: dimensions, resolution, and origin must match. "
            "Register and resample the maps before comparison."
        )


def _components(mask: NDArray[np.bool_]) -> Iterator[list[tuple[int, int]]]:
    """Yield four-connected cells as (row, column) pairs in deterministic order."""
    remaining = mask.copy()
    height, width = mask.shape
    for row, column in zip(*np.nonzero(mask), strict=True):
        if not remaining[row, column]:
            continue
        remaining[row, column] = False
        pending = deque([(int(row), int(column))])
        component: list[tuple[int, int]] = []
        while pending:
            y, x = pending.popleft()
            component.append((y, x))
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < height and 0 <= nx < width and remaining[ny, nx]:
                    remaining[ny, nx] = False
                    pending.append((ny, nx))
        yield component


def compare_maps(
    reference: OccupancyMap, observed: OccupancyMap, *, min_cells: int = 4
) -> Comparison:
    """Find added/removed regions only where both maps classify a cell as known.

    Matching metadata is necessary but cannot prove physical alignment. Callers
    must verify the maps share a coordinate frame. No registration is performed.
    """
    if isinstance(min_cells, bool) or not isinstance(min_cells, int) or min_cells < 1:
        raise ValueError("min_cells must be a positive integer")
    _check_geometry(reference, observed)
    comparable = (reference.cells != UNKNOWN) & (observed.cells != UNKNOWN)
    masks: dict[Literal["added", "removed"], NDArray[np.bool_]] = {
        "added": (reference.cells == FREE) & (observed.cells == OCCUPIED),
        "removed": (reference.cells == OCCUPIED) & (observed.cells == FREE),
    }
    labels = np.zeros(reference.cells.shape, dtype=np.int32)
    changes: list[Change] = []
    filtered_cells = 0
    for kind, mask in masks.items():
        for component in _components(mask):
            if len(component) < min_cells:
                filtered_cells += len(component)
                continue
            rows, columns = np.asarray(component).T
            x0, x1 = int(columns.min()), int(columns.max()) + 1
            y0, y1 = int(rows.min()), int(rows.max()) + 1
            identifier = len(changes) + 1
            labels[rows, columns] = identifier
            changes.append(
                Change(
                    id=identifier,
                    kind=kind,
                    cell_count=len(component),
                    area_m2=len(component) * reference.resolution**2,
                    centroid_m=reference.grid_to_world(
                        float(columns.mean()) + 0.5, float(rows.mean()) + 0.5
                    ),
                    bounds_cells=(x0, y0, x1, y1),
                    bounds_polygon_m=tuple(
                        reference.grid_to_world(x, y)
                        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))
                    ),
                )
            )
    labels.setflags(write=False)
    return Comparison(
        changes=tuple(changes),
        labels=labels,
        comparable_cells=int(np.count_nonzero(comparable)),
        total_cells=reference.cells.size,
        filtered_change_cells=filtered_cells,
        min_cells=min_cells,
    )
