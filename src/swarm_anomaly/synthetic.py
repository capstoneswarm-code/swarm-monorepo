"""Deterministic comparison fixtures, not a sensor or SLAM simulation."""

import numpy as np

from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap


def make_scenarios() -> dict[str, OccupancyMap]:
    """Create a 6 x 4 m room and independently specified changes at 5 cm/cell.

    The reference contains a 0.5 x 0.5 m box at grid rows 20:30, columns 30:40.
    Added/moved boxes occupy rows 40:50, columns 70:80. Each box has 100 cells.
    """
    reference = np.full((80, 120), FREE, dtype=np.int8)
    reference[[0, -1], :] = OCCUPIED
    reference[:, [0, -1]] = OCCUPIED
    reference[20:30, 30:40] = OCCUPIED
    scenarios = {"reference": reference, "unchanged": reference.copy()}
    added = reference.copy()
    added[40:50, 70:80] = OCCUPIED
    scenarios["added"] = added
    removed = reference.copy()
    removed[20:30, 30:40] = FREE
    scenarios["removed"] = removed
    moved = removed.copy()
    moved[40:50, 70:80] = OCCUPIED
    scenarios["moved"] = moved
    partial = reference.copy()
    partial[:, :60] = UNKNOWN
    scenarios["partial"] = partial
    scenarios["unobserved"] = np.full_like(reference, UNKNOWN)
    noise = reference.copy()
    noise[60, 90] = OCCUPIED
    scenarios["speckle"] = noise
    return {name: OccupancyMap(cells, 0.05) for name, cells in scenarios.items()}
