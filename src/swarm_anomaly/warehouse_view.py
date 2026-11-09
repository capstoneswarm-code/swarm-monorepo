"""Standalone HTML/SVG and PNG views of snapshot warehouse alerts."""

import base64
from html import escape
from importlib.resources import files
from io import BytesIO
from string import Template

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from swarm_anomaly.alerts import RankedAlert, Severity
from swarm_anomaly.connectivity import ConnectivityAnalysis
from swarm_anomaly.maps import FREE, OCCUPIED, OccupancyMap
from swarm_anomaly.warehouse import Point, Warehouse

COLORS: dict[Severity, str] = {
    "critical": "#bd2441",
    "high": "#cf5d14",
    "review": "#7954a8",
    "medium": "#9c6900",
    "low": "#256caa",
    "info": "#416c60",
}
AISLE_COLORS = {
    "clear": "#8596a7",
    "restricted": "#bd8410",
    "blocked": "#bd2441",
    "unknown": "#7954a8",
}
SCALE = 4


def _point(grid: OccupancyMap, point: Point) -> Point:
    x = (point[0] - grid.origin[0]) / grid.resolution * SCALE
    y = (grid.cells.shape[0] - (point[1] - grid.origin[1]) / grid.resolution) * SCALE
    return x, y


def _base_image(
    observed: OccupancyMap, analysis: ConnectivityAnalysis, alerts: tuple[RankedAlert, ...]
) -> Image.Image:
    pixels = np.full((*observed.cells.shape, 3), (150, 158, 169), dtype=np.uint8)
    pixels[observed.cells == FREE] = (247, 245, 239)
    pixels[observed.cells == OCCUPIED] = (44, 57, 70)
    # Paint lower ranks first, retaining the strongest impact where alerts overlap.
    for alert in reversed(alerts):
        color = COLORS[alert.severity]
        rgb = tuple(int(color[index : index + 2], 16) for index in (1, 3, 5))
        pixels[np.isin(analysis.comparison.labels, alert.change_ids)] = rgb
    return Image.fromarray(np.flipud(pixels)).resize(
        (observed.cells.shape[1] * SCALE, observed.cells.shape[0] * SCALE), Image.Resampling.NEAREST
    )


def render_warehouse_png(
    layout: Warehouse,
    observed: OccupancyMap,
    analysis: ConnectivityAnalysis,
    alerts: tuple[RankedAlert, ...],
) -> Image.Image:
    image = _base_image(observed, analysis, alerts)
    draw = ImageDraw.Draw(image)
    states = {impact.state.aisle_id: impact.state for impact in analysis.aisles}
    for aisle in layout.aisles:
        endpoints = [
            _point(observed, layout.region(name).center_m) for name in (aisle.start, aisle.end)
        ]
        draw.line(endpoints, fill=AISLE_COLORS[states[aisle.id].status], width=4)
    font = ImageFont.load_default(size=12)
    for region in layout.regions:
        x, y = _point(observed, region.center_m)
        fill = "#bd2441" if region.id in analysis.unreachable_regions else "#ffffff"
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill=fill, outline="#263b50", width=2)
        draw.text(
            (x + 8, y - 18),
            region.label,
            fill="#263b50",
            font=font,
            stroke_width=1,
            stroke_fill="#f7f5ef",
        )
    for alert in alerts:
        x, y = _point(observed, alert.location_m)
        draw.ellipse(
            (x - 13, y - 13, x + 13, y + 13), fill=COLORS[alert.severity], outline="white", width=2
        )
        draw.text(
            (x, y), str(alert.rank), fill="white", font=ImageFont.load_default(size=16), anchor="mm"
        )
    canvas = Image.new("RGB", (image.width + 48, image.height + 144), "#ffffff")
    canvas.paste(image, (24, 76))
    caption = ImageDraw.Draw(canvas)
    caption.text(
        (24, 14), "SWARM warehouse alerts", fill="#17314a", font=ImageFont.load_default(size=24)
    )
    caption.text(
        (24, 46),
        f"{len(alerts)} ranked alerts · "
        f"{analysis.unreachable_region_area_m2:.1f} m² inaccessible regions",
        fill="#425970",
        font=ImageFont.load_default(size=16),
    )
    caption.text(
        (24, image.height + 94),
        "Synthetic snapshot · numbers match the interactive alert list",
        fill="#425970",
        font=ImageFont.load_default(size=16),
    )
    if analysis.unknown_aisles:
        caption.text(
            (24, image.height + 118),
            "Incomplete observations: connectivity estimates are optimistic",
            fill=COLORS["review"],
            font=ImageFont.load_default(size=14),
        )
    return canvas


def _svg_map(
    layout: Warehouse,
    observed: OccupancyMap,
    analysis: ConnectivityAnalysis,
    alerts: tuple[RankedAlert, ...],
) -> str:
    image = _base_image(observed, analysis, alerts)
    buffer = BytesIO()
    image.save(buffer, format="PNG")
    encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
    parts = [
        f'<svg viewBox="0 0 {image.width} {image.height}" '
        'role="group" aria-label="Warehouse map and ranked alerts">',
        f'<image width="{image.width}" height="{image.height}" '
        f'href="data:image/png;base64,{encoded}"/>',
    ]
    for region in layout.regions:
        if region.id in analysis.unreachable_regions:
            x0, y0, x1, y1 = region.bounds_m
            left, top = _point(observed, (x0, y1))
            right, bottom = _point(observed, (x1, y0))
            parts.append(
                f'<rect x="{left}" y="{top}" width="{right - left}" height="{bottom - top}" '
                'fill="#bd2441" fill-opacity=".18" stroke="#bd2441" stroke-dasharray="5 3"/>'
            )
    parts.append('<g id="aisle-graph">')
    states = {impact.state.aisle_id: impact.state for impact in analysis.aisles}
    for aisle in layout.aisles:
        x1, y1 = _point(observed, layout.region(aisle.start).center_m)
        x2, y2 = _point(observed, layout.region(aisle.end).center_m)
        state = states[aisle.id]
        parts.append(
            f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
            f'stroke="{AISLE_COLORS[state.status]}" stroke-width="4">'
            f"<title>{escape(aisle.id)}: {state.status}</title></line>"
        )
    for region in layout.regions:
        x, y = _point(observed, region.center_m)
        fill = "#bd2441" if region.id in analysis.unreachable_regions else "white"
        parts.append(
            f'<circle cx="{x}" cy="{y}" r="6" fill="{fill}" stroke="#263b50" stroke-width="2"/>'
            f'<text x="{x + 8}" y="{y - 10}" class="region-label">{escape(region.label)}</text>'
        )
    for alert in alerts:
        impact = next(
            (item for item in analysis.aisles if item.state.aisle_id == alert.aisle_id), None
        )
        if impact is not None and impact.alternate_route:
            points = " ".join(
                f"{x},{y}"
                for name in impact.alternate_route
                for x, y in [_point(observed, layout.region(name).center_m)]
            )
            parts.append(
                f'<polyline class="alternate" data-alert="{alert.id}" points="{points}" '
                'fill="none" stroke="#149474" stroke-width="5" stroke-dasharray="9 5"/>'
            )
    parts.append("</g>")
    for alert in alerts:
        x, y = _point(observed, alert.location_m)
        parts.append(
            f'<g class="alert-marker" data-alert="{alert.id}" tabindex="0" role="button" '
            f'aria-label="Select alert {alert.rank}: {escape(alert.title, quote=True)}">'
            f'<circle cx="{x}" cy="{y}" r="13" fill="{COLORS[alert.severity]}" '
            'stroke="white" stroke-width="2"/>'
            f'<text x="{x}" y="{y}" dy=".35em" text-anchor="middle" '
            f'fill="white" font-size="16">{alert.rank}</text></g>'
        )
    parts.append("</svg>")
    return "".join(parts)


def render_warehouse_html(
    layout: Warehouse,
    observed: OccupancyMap,
    analysis: ConnectivityAnalysis,
    alerts: tuple[RankedAlert, ...],
    *,
    scenario: str,
) -> str:
    cards: list[str] = []
    for alert in alerts:
        reasons = "".join(
            f'<span class="reason">{escape(reason)}</span>' for reason in alert.reasons
        )
        cards.append(
            f'<button class="alert-card" data-alert="{alert.id}" aria-pressed="false" '
            f'style="--severity:{COLORS[alert.severity]}">'
            f'<span class="badge">#{alert.rank} · {alert.severity.upper()}</span>'
            f"<strong>{escape(alert.title)}</strong>"
            f'<span class="category">{escape(alert.category.replace("_", " "))}</span>'
            f'{reasons}<span class="measurement">Changed area: '
            f"{alert.added_or_removed_area_m2:.2f} m²</span></button>"
        )
    if not cards:
        text = (
            "No changes to rank in observed areas."
            if analysis.comparison.comparable_fraction < 1
            else "No candidate alerts in this snapshot."
        )
        cards.append(f'<p class="empty">{text}</p>')
    inaccessible = (
        ", ".join(layout.region(name).label for name in analysis.unreachable_regions)
        or "None in this model"
    )
    if analysis.unknown_aisles:
        warning = (
            "Some aisles are unobserved. They remain optimistically available; "
            "route estimates are incomplete."
        )
    elif analysis.comparison.comparable_fraction < 1:
        warning = "Incomplete map coverage: unknown areas cannot establish additions or removals."
    else:
        warning = ""
    template = Template(
        files("swarm_anomaly").joinpath("templates/warehouse.html").read_text(encoding="utf-8")
    )
    return template.substitute(
        title=escape(layout.name),
        scenario=escape(scenario.replace("_", " ")),
        alert_count=len(alerts),
        inaccessible_area=f"{analysis.unreachable_region_area_m2:.1f}",
        coverage=f"{analysis.comparison.comparable_fraction:.1%}",
        notice=f'<div class="notice">{escape(warning)}</div>' if warning else "",
        inaccessible=escape(inaccessible),
        entry_label=escape(layout.region(layout.entry_region).label),
        clearance=f"{layout.required_width_m:.2f}",
        map_svg=_svg_map(layout, observed, analysis, alerts),
        cards="".join(cards),
    )
