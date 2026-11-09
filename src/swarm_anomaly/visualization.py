"""Render a comparison for review without a GUI or plotting backend."""

import numpy as np
from numpy.typing import NDArray
from PIL import Image, ImageDraw, ImageFont

from swarm_anomaly.detection import Comparison
from swarm_anomaly.maps import FREE, OCCUPIED, UNKNOWN, OccupancyMap

ADDED_COLOR = (212, 71, 49)
REMOVED_COLOR = (36, 114, 190)
UNKNOWN_COLOR = (155, 164, 174)


def _pixels(grid: OccupancyMap) -> NDArray[np.uint8]:
    pixels = np.full((*grid.cells.shape, 3), UNKNOWN_COLOR, dtype=np.uint8)
    pixels[grid.cells == FREE] = (246, 248, 250)
    pixels[grid.cells == OCCUPIED] = (38, 48, 59)
    return pixels


def render_comparison(
    reference: OccupancyMap,
    observed: OccupancyMap,
    result: Comparison,
    *,
    observed_label: str = "Observed",
) -> Image.Image:
    """Show reference, observation, and retained changes with a shared pixel scale."""
    overlay = _pixels(observed)
    overlay[(reference.cells == UNKNOWN) | (observed.cells == UNKNOWN)] = UNKNOWN_COLOR
    for change in result.changes:
        overlay[result.labels == change.id] = (
            ADDED_COLOR if change.kind == "added" else REMOVED_COLOR
        )
    height, width = reference.cells.shape
    scale = min(4.0, 640 / width, 480 / height)
    image_size = (max(1, round(width * scale)), max(1, round(height * scale)))
    panel_width = max(320, image_size[0])
    margin, gap, map_top = 24, 20, 100
    canvas = Image.new(
        "RGB", (2 * margin + 3 * panel_width + 2 * gap, map_top + image_size[1] + 122), "white"
    )
    draw = ImageDraw.Draw(canvas)
    title_font = ImageFont.load_default(size=24)
    label_font = ImageFont.load_default(size=18)
    draw.text((margin, 18), "SWARM map comparison", fill=(26, 43, 60), font=title_font)
    panels = (
        ("Reference", _pixels(reference)),
        (observed_label, _pixels(observed)),
        ("Changes", overlay),
    )
    for index, (label, pixels) in enumerate(panels):
        left = margin + index * (panel_width + gap)
        draw.text((left, 65), label, fill=(26, 43, 60), font=label_font)
        image = Image.fromarray(np.flipud(pixels)).resize(image_size, Image.Resampling.NEAREST)
        canvas.paste(image, (left + (panel_width - image_size[0]) // 2, map_top))
    legend_y = map_top + image_size[1] + 20
    for index, (label, color) in enumerate(
        (
            ("Added obstacle", ADDED_COLOR),
            ("Removed obstacle", REMOVED_COLOR),
            ("Unknown / excluded", UNKNOWN_COLOR),
        )
    ):
        left = margin + index * (panel_width + gap)
        draw.rectangle((left, legend_y, left + 14, legend_y + 14), fill=color)
        draw.text((left + 24, legend_y - 2), label, fill=(26, 43, 60), font=label_font)
    status = (
        f"{len(result.changes)} candidate regions | "
        f"Comparable area: {result.comparable_fraction:.1%}"
        if result.comparable_cells
        else "Insufficient evidence: no cells are known in both maps"
    )
    draw.text((margin, legend_y + 32), status, fill=(26, 43, 60), font=label_font)
    draw.text(
        (margin, legend_y + 62),
        f"Minimum region: {result.min_cells} cells | "
        f"Filtered change cells: {result.filtered_change_cells}",
        fill=(70, 85, 100),
        font=label_font,
    )
    return canvas
