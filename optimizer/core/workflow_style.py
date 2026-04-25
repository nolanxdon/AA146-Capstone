from __future__ import annotations

from typing import Iterable


MIT_RED_SWEEP: tuple[str, ...] = (
    "#fee5d9",
    "#fcbba1",
    "#fc9272",
    "#fb6a4a",
    "#ef3b2c",
    "#cb181d",
    "#99000d",
)

FLAP_STATE_COLORS: dict[float, str] = {
    0.0: "#7f1d1d",
    20.0: "#b91c1c",
    40.0: "#ef4444",
}

SCENARIO_COLORS: dict[str, str] = {
    "clean_blowing": "#991b1b",
    "slotted_flap_blowing": "#ef4444",
}

AIRFOIL_RED_STYLES: dict[str, dict[str, str]] = {
    "s1210": {"color": "#7f1d1d", "marker": "o"},
    "e423": {"color": "#a61c1c", "marker": "s"},
    "dae51": {"color": "#c62828", "marker": "^"},
    "naca0012": {"color": "#e45757", "marker": "D"},
    "naca2412": {"color": "#f28c8c", "marker": "v"},
}


def sweep_red_shades(count: int) -> list[str]:
    if count <= 0:
        return []
    if count <= len(MIT_RED_SWEEP):
        return list(MIT_RED_SWEEP[:count])
    colors = list(MIT_RED_SWEEP)
    while len(colors) < count:
        colors.append(MIT_RED_SWEEP[-1])
    return colors


def flap_state_color(deflection_deg: float) -> str:
    rounded = float(round(deflection_deg))
    return FLAP_STATE_COLORS.get(rounded, "#b91c1c")


def airfoil_red_styles(names: Iterable[str]) -> dict[str, dict[str, str]]:
    styles: dict[str, dict[str, str]] = {}
    fallback_colors = sweep_red_shades(max(len(tuple(names)), 1))
    for idx, name in enumerate(names):
        key = name.lower()
        styles[key] = AIRFOIL_RED_STYLES.get(
            key,
            {"color": fallback_colors[min(idx, len(fallback_colors) - 1)], "marker": "o"},
        )
    return styles
