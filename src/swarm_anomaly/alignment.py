"""Resample an observation onto a reference grid using a known rigid transform."""

from dataclasses import dataclass
from math import cos, isfinite, sin

import numpy as np

from swarm_anomaly.maps import UNKNOWN, OccupancyMap


@dataclass(frozen=True)
class RigidTransform:
    """Observed-frame to reference-frame transform: p_ref = R(yaw) p_obs + (x, y)."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError("Transform translation and yaw must be finite")


def resample_to_reference(
    reference: OccupancyMap, observed: OccupancyMap, *, transform: RigidTransform
) -> OccupancyMap:
    """Sample observed cells at reference-cell centres, retaining trinary values.

    Both maps' origins and resolutions are respected. Supply an identity transform
    only when they share a physical coordinate frame. This function applies a
    known transform; it does not estimate registration from the map contents.
    Out-of-bounds samples stay unknown. Cell boundaries are lower-inclusive and
    upper-exclusive; coordinates within 1e-9 cells of a boundary are snapped to it
    to stabilize floating-point roundoff. No occupancy probabilities are averaged.
    """
    # Locate the reference origin in the observed map frame, then in its grid.
    dx = reference.origin[0] - transform.x_m
    dy = reference.origin[1] - transform.y_m
    c, s = cos(transform.yaw_rad), sin(transform.yaw_rad)
    ox = c * dx + s * dy - observed.origin[0]
    oy = -s * dx + c * dy - observed.origin[1]
    c, s = cos(observed.origin[2]), sin(observed.origin[2])
    offset_x = (c * ox + s * oy) / observed.resolution
    offset_y = (-s * ox + c * oy) / observed.resolution
    scale = reference.resolution / observed.resolution
    angle = reference.origin[2] - transform.yaw_rad - observed.origin[2]
    if not all(isfinite(value) for value in (offset_x, offset_y, scale, angle)):
        raise ValueError("Map geometry and transform exceed the supported numeric range")
    c, s = cos(angle), sin(angle)
    height, width = reference.cells.shape
    source_height, source_width = observed.cells.shape
    cells = np.full((height, width), UNKNOWN, dtype=np.int8)
    columns = (np.arange(width, dtype=np.float64)[None, :] + 0.5) * scale
    # Limit temporary coordinate-array memory for large maps.
    for start in range(0, height, 256):
        stop = min(start + 256, height)
        rows = (np.arange(start, stop, dtype=np.float64)[:, None] + 0.5) * scale
        source_x = c * columns - s * rows + offset_x
        source_y = s * columns + c * rows + offset_y
        for coordinates in (source_x, source_y):
            nearest = np.rint(coordinates)
            np.copyto(coordinates, nearest, where=np.abs(coordinates - nearest) <= 1e-9)
        valid = (
            (source_x >= 0)
            & (source_x < source_width)
            & (source_y >= 0)
            & (source_y < source_height)
        )
        # Mask before casting to indices: negative or distant samples must not wrap.
        indices_x = np.floor(source_x[valid]).astype(np.intp)
        indices_y = np.floor(source_y[valid]).astype(np.intp)
        block = cells[start:stop]
        block[valid] = observed.cells[indices_y, indices_x]
    return OccupancyMap(cells, reference.resolution, reference.origin)
