from __future__ import annotations

import csv
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from optimizer.core.control_surface_sizing import (
    RectangularWingControlConfig,
    _load_selected_concept,
    _pick_flap_and_aileron,
    _selected_flap_section_polars,
    _total_high_lift_curve_data,
    evaluate_aileron_candidate,
    evaluate_flap_candidate,
    ensure_stage3_runtime,
)


@dataclass(frozen=True)
class MotorHeightTradeOutput:
    output_dir: Path
    summary_csv: Path
    summary_md: Path
    overlay_curve_csv: Path
    metric_plot: Path
    overlay_plot: Path
    geometry_plot: Path


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        if not rows:
            return
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _interp_x_for_y(x: np.ndarray, y: np.ndarray, y_target: float) -> float | None:
    for i in range(1, len(x)):
        y0 = y[i - 1]
        y1 = y[i]
        if (y0 - y_target) * (y1 - y_target) <= 0.0 and y1 != y0:
            t = (y_target - y0) / (y1 - y0)
            return float(x[i - 1] + t * (x[i] - x[i - 1]))
    return None


def _interp_y_for_x(x: np.ndarray, y: np.ndarray, x_target: float) -> float | None:
    if x_target < x.min() or x_target > x.max():
        return None
    return float(np.interp(x_target, x, y))


def _render_metric_plot(
    rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    drop_mm = [1000.0 * float(row["prop_drop_m"]) for row in rows]
    vstall = [float(row["equivalent_vstall_mps"]) for row in rows]
    clmax = [float(row["equivalent_reference_clmax"]) for row in rows]
    alpha_req = [
        (float(row["alpha_required_for_4ms_cl_deg"]) if row["alpha_required_for_4ms_cl_deg"] not in {"", None} else np.nan)
        for row in rows
    ]
    roll_rate = [float(row["roll_rate_at_nominal_degps"]) for row in rows]

    fig, axs = plt.subplots(2, 2, figsize=(11.8, 8.2))
    axs = axs.ravel()

    axs[0].plot(drop_mm, vstall, marker="o", color="#e76f51")
    axs[0].set_title("Equivalent Stall Speed vs Prop Drop")
    axs[0].set_xlabel("Motor vertical drop [mm]")
    axs[0].set_ylabel("Equivalent stall speed [m/s]")
    axs[0].grid(True, alpha=0.25)

    axs[1].plot(drop_mm, clmax, marker="o", color="#7c3aed")
    axs[1].set_title("Equivalent Reference CLmax vs Prop Drop")
    axs[1].set_xlabel("Motor vertical drop [mm]")
    axs[1].set_ylabel("Equivalent reference CLmax")
    axs[1].grid(True, alpha=0.25)

    axs[2].plot(drop_mm, alpha_req, marker="o", color="#1d3557")
    axs[2].set_title("Alpha Needed to Meet 4 m/s Lift Requirement")
    axs[2].set_xlabel("Motor vertical drop [mm]")
    axs[2].set_ylabel("Required alpha [deg]")
    axs[2].grid(True, alpha=0.25)

    axs[3].plot(drop_mm, roll_rate, marker="o", color="#2a9d8f")
    axs[3].set_title("Estimated Roll Rate vs Prop Drop")
    axs[3].set_xlabel("Motor vertical drop [mm]")
    axs[3].set_ylabel("Roll rate at 14 deg aileron [deg/s]")
    axs[3].grid(True, alpha=0.25)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_overlay_plot(
    overlay_rows: list[dict[str, Any]],
    cl_required: float,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    grouped: dict[float, list[dict[str, Any]]] = {}
    for row in overlay_rows:
        grouped.setdefault(float(row["prop_drop_m"]), []).append(row)

    fig, ax = plt.subplots(figsize=(8.8, 5.4))
    colors = plt.cm.viridis(np.linspace(0.08, 0.92, len(grouped)))
    for color, (drop_m, rows) in zip(colors, sorted(grouped.items())):
        rows_sorted = sorted(rows, key=lambda row: float(row["alpha_deg"]))
        alpha = [float(row["alpha_deg"]) for row in rows_sorted]
        cl_all = [float(row["cl_all_high_lift"]) for row in rows_sorted]
        ax.plot(alpha, cl_all, color=color, linewidth=2.0, label=f"{1000.0 * drop_m:.0f} mm drop")

    ax.axhline(cl_required, color="#111827", linestyle="--", linewidth=1.0, label="Required at 4 m/s")
    ax.set_xlabel("Alpha [deg]")
    ax.set_ylabel("Equivalent wing-system CL")
    ax.set_title("All-High-Lift Whole-Wing CL Curves vs Motor Drop")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left", ncol=2)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_geometry_plot(
    rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    drop_mm = [1000.0 * float(row["prop_drop_m"]) for row in rows]
    drop_over_c = [float(row["prop_drop_fraction_of_chord"]) for row in rows]
    clean_immersion = [float(row["cambridge_clean_immersion_at_clmax"]) for row in rows]
    flap_immersion = [float(row["cambridge_flap_immersion_at_clmax"]) for row in rows]
    flap_margin_mm = [1000.0 * float(row["cambridge_flap_margin_m_at_clmax"]) for row in rows]

    fig, axs = plt.subplots(1, 3, figsize=(12.0, 4.3))
    axs[0].plot(drop_mm, drop_over_c, marker="o", color="#f4a261")
    axs[0].set_title("Motor Height in Chord Coordinates")
    axs[0].set_xlabel("Motor vertical drop [mm]")
    axs[0].set_ylabel(r"$y_p/c$")
    axs[0].grid(True, alpha=0.25)

    axs[1].plot(drop_mm, clean_immersion, marker="o", color="#1d3557", label="Clean strip")
    axs[1].plot(drop_mm, flap_immersion, marker="s", color="#7c3aed", label="Flap strip")
    axs[1].set_title("Cambridge Jet Immersion at CLmax")
    axs[1].set_xlabel("Motor vertical drop [mm]")
    axs[1].set_ylabel("Immersion factor")
    axs[1].grid(True, alpha=0.25)
    axs[1].legend()

    axs[2].plot(drop_mm, flap_margin_mm, marker="o", color="#e76f51")
    axs[2].axhline(0.0, color="#111827", linestyle="--", linewidth=1.0)
    axs[2].set_title("Flap-Strip Jet Margin at CLmax")
    axs[2].set_xlabel("Motor vertical drop [mm]")
    axs[2].set_ylabel("Margin [mm]")
    axs[2].grid(True, alpha=0.25)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _write_summary_markdown(
    rows: list[dict[str, Any]],
    output: MotorHeightTradeOutput,
    *,
    rank: int,
    n_props: int,
    diameter_in: float,
    pitch_in: float,
) -> None:
    best_vstall = min(rows, key=lambda row: float(row["equivalent_vstall_mps"]))
    best_roll = max(rows, key=lambda row: float(row["roll_rate_at_nominal_degps"]))
    peak_cl = max(float(row["equivalent_reference_clmax"]) for row in rows)
    first_plateau = next(
        (
            row
            for row in rows
            if float(row["equivalent_reference_clmax"]) >= 0.9995 * peak_cl
        ),
        best_vstall,
    )
    lines = [
        f"# Motor Height Trade Study: Rank {rank}",
        "",
        f"- Prop concept: `{n_props} x {diameter_in:.1f} x {pitch_in:.1f} in`",
        "",
        "## Key takeaways",
        "",
        f"- Lowest equivalent stall speed in the sweep occurred at `{1000.0 * float(best_vstall['prop_drop_m']):.0f} mm` drop: `{float(best_vstall['equivalent_vstall_mps']):.3f} m/s`.",
        f"- The lift benefit effectively saturated by about `{1000.0 * float(first_plateau['prop_drop_m']):.0f} mm` drop (`{float(first_plateau['prop_drop_fraction_of_chord']):.2f} c`), after which further lowering changed `CLmax` only negligibly.",
        f"- Highest 14 deg roll-rate estimate occurred at `{1000.0 * float(best_roll['prop_drop_m']):.0f} mm` drop: `{float(best_roll['roll_rate_at_nominal_degps']):.1f} deg/s`.",
        "",
        "## Artifacts",
        "",
        f"- Metric plot: ![]({output.metric_plot.name})",
        "",
        f"- Whole-wing CL overlay: ![]({output.overlay_plot.name})",
        "",
        f"- Geometry/immersion plot: ![]({output.geometry_plot.name})",
        "",
        "## Notes",
        "",
        "- This trade keeps the selected propulsion architecture and the selected rectangular slotted-flap/aileron geometry fixed while sweeping only motor vertical drop.",
        "- The vertical-drop effect now enters through a Cambridge-style jet-immersion criterion rather than through a purely empirical bonus term.",
        "- The blown benefit remains strong only while the wing stays submerged in the uniform 2D jet; once the jet rides above the wing, the effective blown contribution collapses rapidly.",
        "- These results remain concept-level sensitivity trends, not a CFD-calibrated vertical-placement optimum.",
    ]
    output.summary_md.parent.mkdir(parents=True, exist_ok=True)
    output.summary_md.write_text("\n".join(lines), encoding="utf-8")


def run_motor_height_trade(
    *,
    rank: int = 6,
    blade_count_metadata: int = 3,
    prop_drop_fraction_of_chord_values: tuple[float, ...] = (0.00, 0.04, 0.08, 0.12, 0.16, 0.20, 0.24),
    output_root: Path = Path("outputs/motor_height_trade"),
) -> MotorHeightTradeOutput:
    base_config = RectangularWingControlConfig(blade_count_metadata=blade_count_metadata)
    concept = _load_selected_concept(rank=rank, blade_count_metadata=blade_count_metadata)
    baseline_flap, baseline_aileron, _, _ = _pick_flap_and_aileron(base_config, concept)

    output_dir = output_root / (
        f"rank{concept.rank:02d}_n{concept.n_props}_d{concept.prop_diameter_in:.1f}"
        f"_p{concept.prop_pitch_in:.1f}_{concept.prop_family}_b{concept.blade_count_metadata}"
    ).replace(".", "p")
    output_dir.mkdir(parents=True, exist_ok=True)

    output = MotorHeightTradeOutput(
        output_dir=output_dir,
        summary_csv=output_dir / "motor_height_trade_summary.csv",
        summary_md=output_dir / "motor_height_trade_summary.md",
        overlay_curve_csv=output_dir / "motor_height_trade_curves.csv",
        metric_plot=output_dir / "motor_height_trade_metrics.png",
        overlay_plot=output_dir / "motor_height_trade_cl_overlay.png",
        geometry_plot=output_dir / "motor_height_trade_geometry.png",
    )

    summary_rows: list[dict[str, Any]] = []
    overlay_rows: list[dict[str, Any]] = []
    cl_required = base_config.mission.gross_weight_n / max(
        0.5 * base_config.mission.air_density_kgpm3 * base_config.mission.low_speed_mps**2 * base_config.mission.wing_area_m2,
        1e-9,
    )

    for drop_fraction in prop_drop_fraction_of_chord_values:
        config = replace(
            base_config,
            prop_drop_fraction_of_chord=drop_fraction,
            prop_drop_fraction_of_diameter=None,
        )
        flap = evaluate_flap_candidate(
            config,
            concept,
            baseline_flap.flap_span_fraction,
            baseline_flap.flap_chord_fraction,
            baseline_flap.flap_deflection_deg,
            polar_cache={},
        )
        aileron = evaluate_aileron_candidate(
            config,
            flap,
            baseline_aileron.aileron_span_fraction,
            baseline_aileron.aileron_chord_fraction,
        )
        if aileron is None:
            raise ValueError("Baseline aileron geometry became infeasible during the motor-height trade sweep.")
        polars = _selected_flap_section_polars(config, concept, flap)
        total_curve_rows = _total_high_lift_curve_data(config, concept, flap, polars)
        drop_m = flap.prop_drop_m
        for row in total_curve_rows:
            overlay_rows.append(
                {
                    "prop_drop_fraction_of_chord": drop_fraction,
                    "prop_drop_fraction_of_diameter": drop_m / max(concept.prop_diameter_m, 1e-9),
                    "prop_drop_m": drop_m,
                    **row,
                }
            )

        alpha = np.asarray([row["alpha_deg"] for row in total_curve_rows], dtype=float)
        cl_all = np.asarray([row["cl_all_high_lift"] for row in total_curve_rows], dtype=float)
        cd_all = np.asarray([row["cd_all_high_lift"] for row in total_curve_rows], dtype=float)
        peak_index = int(np.argmax(cl_all))
        alpha_req = _interp_x_for_y(alpha, cl_all, cl_required)
        if alpha_req is None and cl_all[0] >= cl_required:
            alpha_req = float(alpha[0])
        cd_req = _interp_y_for_x(alpha, cd_all, alpha_req) if alpha_req is not None else None

        summary_rows.append(
            {
                "rank": concept.rank,
                "n_props": concept.n_props,
                "prop_diameter_in": concept.prop_diameter_in,
                "prop_pitch_in": concept.prop_pitch_in,
                "prop_pitch_ratio": concept.prop_pitch_ratio,
                "prop_family": concept.prop_family,
                "blade_count_metadata": concept.blade_count_metadata,
                "prop_drop_fraction_of_chord": drop_fraction,
                "prop_drop_fraction_of_diameter": drop_m / max(concept.prop_diameter_m, 1e-9),
                "prop_drop_m": drop_m,
                "recommended_flap_span_fraction": flap.flap_span_fraction,
                "recommended_flap_chord_fraction": flap.flap_chord_fraction,
                "recommended_flap_deflection_deg": flap.flap_deflection_deg,
                "cambridge_clean_immersion_at_clmax": flap.cambridge_clean_immersion_at_clmax,
                "cambridge_flap_immersion_at_clmax": flap.cambridge_flap_immersion_at_clmax,
                "cambridge_flap_margin_m_at_clmax": flap.cambridge_flap_margin_m_at_clmax,
                "recommended_aileron_span_fraction": aileron.aileron_span_fraction,
                "recommended_aileron_chord_fraction": aileron.aileron_chord_fraction,
                "equivalent_reference_clmax": flap.equivalent_reference_clmax,
                "equivalent_vstall_mps": flap.equivalent_vstall_mps,
                "low_speed_lift_margin": flap.low_speed_lift_margin,
                "peak_total_cl": float(cl_all[peak_index]),
                "alpha_at_peak_total_cl_deg": float(alpha[peak_index]),
                "alpha_required_for_4ms_cl_deg": "" if alpha_req is None else alpha_req,
                "cd_at_required_alpha": "" if cd_req is None else cd_req,
                "roll_rate_at_nominal_degps": aileron.roll_rate_at_nominal_degps,
                "roll_rate_at_max_degps": aileron.roll_rate_at_max_degps,
            }
        )

    _write_csv(output.summary_csv, summary_rows)
    _write_csv(output.overlay_curve_csv, overlay_rows)
    _render_metric_plot(summary_rows, output.metric_plot)
    _render_overlay_plot(overlay_rows, cl_required, output.overlay_plot)
    _render_geometry_plot(summary_rows, output.geometry_plot)
    _write_summary_markdown(
        summary_rows,
        output,
        rank=concept.rank,
        n_props=concept.n_props,
        diameter_in=concept.prop_diameter_in,
        pitch_in=concept.prop_pitch_in,
    )
    return output
