from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from optimizer.core.airfoil_polar_comparison import AirfoilPolarComparisonConfig, run_airfoil_polar_comparison
from optimizer.core.control_surface_sizing import run_rectangular_control_surface_sizing
from optimizer.core.motor_height_trade import run_motor_height_trade
from optimizer.core.wing_speed_sweep import default_speed_sweep_grid, run_wing_speed_sweep


@dataclass(frozen=True)
class WingWorkflowOutput:
    output_root: Path
    summary_csv: Path
    summary_md: Path
    airfoil_frontend_root: Path
    dae51_root: Path


def _load_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        if not rows:
            return
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _best_motor_drop_fraction(summary_csv: Path) -> float:
    rows = _load_rows(summary_csv)
    flap_rows = [row for row in rows if row.get("scenario") == "slotted_flap_blowing"]
    if not flap_rows:
        return 0.12
    best = min(flap_rows, key=lambda row: float(row["equivalent_vstall_mps"]))
    return float(best["prop_drop_fraction_of_chord"])


def run_wing_workflow(
    *,
    rank: int = 6,
    blade_count_metadata: int = 3,
    frontend_airfoils: tuple[str, ...] = ("s1210", "e423", "dae51", "naca0012", "naca2412"),
    detailed_airfoil: str = "dae51",
    flap_deflections_deg: tuple[float, ...] = (0.0, 20.0, 40.0),
    speeds_mps: tuple[float, ...] = default_speed_sweep_grid(),
    output_root: Path = Path("outputs/wing_workflow"),
) -> WingWorkflowOutput:
    frontend_root = output_root / "airfoil_frontend"
    dae51_root = output_root / detailed_airfoil.lower()
    control_root = dae51_root / "control_surface_sizing"
    motor_root = dae51_root / "motor_height_trade"
    speed_root = dae51_root / "speed_sweep"

    print("1/4 Airfoil front-end analysis...")
    frontend_outputs = run_airfoil_polar_comparison(
        AirfoilPolarComparisonConfig(
            airfoils=tuple(airfoil.lower() for airfoil in frontend_airfoils),
            output_root=frontend_root,
            legacy_output_root=frontend_root / "legacy_style_polars",
        )
    )

    print("2/4 DAE51 control-surface sizing...")
    control_output = run_rectangular_control_surface_sizing(
        rank=rank,
        blade_count_metadata=blade_count_metadata,
        airfoil_name=detailed_airfoil,
        output_root=control_root,
    )

    print("3/4 DAE51 motor-height trade...")
    motor_output = run_motor_height_trade(
        rank=rank,
        blade_count_metadata=blade_count_metadata,
        airfoil_name=detailed_airfoil,
        fixed_flap_span_fraction=control_output.flap.flap_span_fraction,
        fixed_flap_chord_fraction=control_output.flap.flap_chord_fraction,
        fixed_aileron_span_fraction=control_output.aileron.aileron_span_fraction,
        fixed_aileron_chord_fraction=control_output.aileron.aileron_chord_fraction,
        output_root=motor_root,
    )
    best_drop_fraction = _best_motor_drop_fraction(motor_output.summary_csv)

    print(f"4/4 DAE51 frozen-geometry speed sweep with motor drop {best_drop_fraction:.3f}c...")
    speed_output = run_wing_speed_sweep(
        rank=rank,
        blade_count_metadata=blade_count_metadata,
        airfoil_name=detailed_airfoil,
        speeds_mps=speeds_mps,
        flap_deflections_deg=flap_deflections_deg,
        prop_drop_fraction_of_chord=best_drop_fraction,
        fixed_flap_span_fraction=control_output.flap.flap_span_fraction,
        fixed_flap_chord_fraction=control_output.flap.flap_chord_fraction,
        fixed_aileron_span_fraction=control_output.aileron.aileron_span_fraction,
        fixed_aileron_chord_fraction=control_output.aileron.aileron_chord_fraction,
        output_root=speed_root,
    )

    summary_csv = dae51_root / "wing_workflow_summary.csv"
    summary_md = dae51_root / "wing_workflow_summary.md"
    summary_rows = [
        {
            "rank": control_output.concept.rank,
            "airfoil": detailed_airfoil.upper(),
            "n_props": control_output.concept.n_props,
            "prop_diameter_in": control_output.concept.prop_diameter_in,
            "prop_pitch_in": control_output.concept.prop_pitch_in,
            "prop_family": control_output.concept.prop_family,
            "blade_count_metadata": control_output.concept.blade_count_metadata,
            "control_flap_span_fraction": control_output.flap.flap_span_fraction,
            "control_flap_chord_fraction": control_output.flap.flap_chord_fraction,
            "control_aileron_span_fraction": control_output.aileron.aileron_span_fraction,
            "control_aileron_chord_fraction": control_output.aileron.aileron_chord_fraction,
            "optimized_motor_drop_fraction_of_chord": best_drop_fraction,
            "airfoil_frontend_root": str(frontend_root),
            "control_summary_csv": str(control_output.summary_csv),
            "motor_summary_csv": str(motor_output.summary_csv),
            "speed_summary_csv": str(speed_output.summary_csv),
        }
    ]
    _write_csv(summary_csv, summary_rows)

    summary_lines = [
        f"# Unified Wing Workflow | {detailed_airfoil.upper()}",
        "",
        "## Workflow",
        "",
        "1. Multi-airfoil front-end section analysis",
        "2. DAE51 rectangular-wing control-surface sizing at the low-speed design point",
        "3. DAE51 motor-height trade on the frozen control geometry",
        "4. Frozen-geometry DAE51 speed sweep from 5 to 15 m/s with RPM re-solved at each speed",
        "",
        "## Key outputs",
        "",
        f"- Airfoil front-end summary: [{frontend_outputs['summary_md'].name}]({Path(frontend_outputs['summary_md']).relative_to(output_root)})",
        f"- DAE51 control summary: [{control_output.summary_md.name}]({control_output.summary_md.relative_to(output_root)})",
        f"- DAE51 motor-height summary: [{motor_output.summary_md.name}]({motor_output.summary_md.relative_to(output_root)})",
        f"- DAE51 speed-sweep summary: [{speed_output.summary_md.name}]({speed_output.summary_md.relative_to(output_root)})",
        "",
        "## Frozen design passed downstream",
        "",
        f"- Flap geometry: `span={control_output.flap.flap_span_fraction:.2f}` semispan, `c_f/c={control_output.flap.flap_chord_fraction:.2f}`",
        f"- Aileron geometry: `span={control_output.aileron.aileron_span_fraction:.2f}` semispan, `c_a/c={control_output.aileron.aileron_chord_fraction:.2f}`",
        f"- Optimized motor drop used for the speed sweep: `{best_drop_fraction:.3f} c`",
        f"- Speed sweep flap states: `{', '.join(f'{value:.0f} deg' for value in flap_deflections_deg)}`",
        f"- Speed sweep speeds: `{min(speeds_mps):.1f}` to `{max(speeds_mps):.1f} m/s` in `0.5 m/s` increments",
        "",
    ]
    summary_md.parent.mkdir(parents=True, exist_ok=True)
    summary_md.write_text("\n".join(summary_lines), encoding="utf-8")

    return WingWorkflowOutput(
        output_root=output_root,
        summary_csv=summary_csv,
        summary_md=summary_md,
        airfoil_frontend_root=frontend_root,
        dae51_root=dae51_root,
    )
