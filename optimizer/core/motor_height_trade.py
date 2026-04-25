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


SCENARIO_DEFS: tuple[dict[str, str], ...] = (
    {
        "name": "clean_blowing",
        "label": "Clean blown wing",
        "cl_key": "cl_blow_only",
        "cd_key": "cd_blow_only",
        "cm_key": "cm_blow_only",
        "immersion_key": "cambridge_clean_immersion_factor",
        "margin_key": "cambridge_clean_margin_m",
        "severity_key": "overdrop_clean_severity_scalar",
        "peak_shift_key": "overdrop_clean_peak_shift_deg",
        "color": "#1d4ed8",
    },
    {
        "name": "slotted_flap_blowing",
        "label": "Slotted flap + blowing",
        "cl_key": "cl_all_high_lift",
        "cd_key": "cd_all_high_lift",
        "cm_key": "cm_all_high_lift",
        "immersion_key": "cambridge_flap_immersion_factor",
        "margin_key": "cambridge_flap_margin_m",
        "severity_key": "overdrop_flap_severity_scalar",
        "peak_shift_key": "overdrop_flap_peak_shift_deg",
        "color": "#d97706",
    },
)


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


def _scenario_curve_metrics(
    alpha: np.ndarray,
    cl: np.ndarray,
    *,
    cl_required: float,
) -> dict[str, float | None]:
    peak_index = int(np.argmax(cl))
    peak_cl = float(cl[peak_index])
    alpha_peak = float(alpha[peak_index])
    post_peak_alpha = alpha[peak_index:]
    post_peak_cl = cl[peak_index:]
    alpha_poststall_90pct = _interp_x_for_y(post_peak_alpha, post_peak_cl, 0.9 * peak_cl)
    cl_plus_5 = _interp_y_for_x(alpha, cl, alpha_peak + 5.0)
    if cl_plus_5 is None:
        cl_plus_5 = float(cl[-1])
    alpha_req = _interp_x_for_y(alpha, cl, cl_required)
    if alpha_req is None and cl[0] >= cl_required:
        alpha_req = float(alpha[0])
    return {
        "peak_cl": peak_cl,
        "alpha_at_peak_deg": alpha_peak,
        "alpha_poststall_90pct_deg": alpha_poststall_90pct,
        "poststall_drop_5deg": peak_cl - cl_plus_5,
        "alpha_required_deg": alpha_req,
    }


def _rows_for_scenario(rows: list[dict[str, Any]], scenario: str) -> list[dict[str, Any]]:
    scenario_rows = [row for row in rows if row["scenario"] == scenario]
    scenario_rows.sort(key=lambda row: float(row["prop_drop_m"]))
    return scenario_rows


def _overlay_rows_for_scenario(rows: list[dict[str, Any]], scenario: str) -> list[dict[str, Any]]:
    scenario_rows = [row for row in rows if row["scenario"] == scenario]
    scenario_rows.sort(key=lambda row: (float(row["prop_drop_m"]), float(row["alpha_deg"])))
    return scenario_rows


def _render_metric_plot(
    rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    fig, axs = plt.subplots(2, 2, figsize=(11.8, 8.4))
    axs = axs.ravel()

    for scenario_def in SCENARIO_DEFS:
        scenario_rows = _rows_for_scenario(rows, scenario_def["name"])
        drop_mm = [1000.0 * float(row["prop_drop_m"]) for row in scenario_rows]
        clmax = [float(row["equivalent_reference_clmax"]) for row in scenario_rows]
        vstall = [float(row["equivalent_vstall_mps"]) for row in scenario_rows]
        alpha_peak = [float(row["alpha_at_peak_total_cl_deg"]) for row in scenario_rows]
        poststall_drop = [float(row["poststall_drop_5deg"]) for row in scenario_rows]
        color = scenario_def["color"]
        label = scenario_def["label"]

        axs[0].plot(drop_mm, clmax, marker="o", linewidth=2.0, color=color, label=label)
        axs[1].plot(drop_mm, vstall, marker="o", linewidth=2.0, color=color, label=label)
        axs[2].plot(drop_mm, alpha_peak, marker="o", linewidth=2.0, color=color, label=label)
        axs[3].plot(drop_mm, poststall_drop, marker="o", linewidth=2.0, color=color, label=label)

    axs[0].set_title("CLmax vs Motor Drop")
    axs[0].set_ylabel("Equivalent reference CLmax")
    axs[1].set_title("Equivalent Stall Speed vs Motor Drop")
    axs[1].set_ylabel("Equivalent stall speed [m/s]")
    axs[2].set_title("Alpha at CLmax vs Motor Drop")
    axs[2].set_ylabel("Alpha at CLmax [deg]")
    axs[3].set_title("Post-Stall Drop over +5 deg")
    axs[3].set_ylabel(r"$C_{L,\max} - C_L(\alpha_{\max}+5^\circ)$")

    for ax in axs:
        ax.set_xlabel("Motor vertical drop [mm]")
        ax.grid(True, alpha=0.25)
        ax.legend()

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

    fig, axs = plt.subplots(1, 2, figsize=(13.2, 5.3), sharey=True)
    color_map = plt.cm.viridis(np.linspace(0.08, 0.92, 7))

    for ax, scenario_def in zip(axs, SCENARIO_DEFS):
        grouped: dict[float, list[dict[str, Any]]] = {}
        for row in _overlay_rows_for_scenario(overlay_rows, scenario_def["name"]):
            grouped.setdefault(float(row["prop_drop_m"]), []).append(row)

        for color, (drop_m, rows_for_drop) in zip(color_map, sorted(grouped.items())):
            alpha = [float(row["alpha_deg"]) for row in rows_for_drop]
            cl_total = [float(row["cl_total"]) for row in rows_for_drop]
            ax.plot(alpha, cl_total, color=color, linewidth=2.0, label=f"{1000.0 * drop_m:.0f} mm drop")

        ax.axhline(cl_required, color="#111827", linestyle="--", linewidth=1.0, label="Required at 4 m/s")
        ax.set_xlabel("Alpha [deg]")
        ax.set_title(scenario_def["label"])
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper left", ncol=2)

    axs[0].set_ylabel("Equivalent wing-system CL")
    fig.suptitle("Whole-Wing CL Curves vs Motor Drop")
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

    clean_rows = _rows_for_scenario(rows, "clean_blowing")
    flap_rows = _rows_for_scenario(rows, "slotted_flap_blowing")
    drop_mm = [1000.0 * float(row["prop_drop_m"]) for row in clean_rows]
    drop_over_c = [float(row["prop_drop_fraction_of_chord"]) for row in clean_rows]

    fig, axs = plt.subplots(1, 3, figsize=(13.3, 4.4))

    axs[0].plot(drop_mm, drop_over_c, marker="o", color="#f4a261")
    axs[0].set_title("Motor Height in Chord Coordinates")
    axs[0].set_xlabel("Motor vertical drop [mm]")
    axs[0].set_ylabel(r"$\Delta z_p / c$")
    axs[0].grid(True, alpha=0.25)

    axs[1].plot(
        drop_mm,
        [float(row["cambridge_immersion_at_clmax"]) for row in clean_rows],
        marker="o",
        linewidth=2.0,
        color=SCENARIO_DEFS[0]["color"],
        label=SCENARIO_DEFS[0]["label"],
    )
    axs[1].plot(
        drop_mm,
        [float(row["cambridge_immersion_at_clmax"]) for row in flap_rows],
        marker="s",
        linewidth=2.0,
        color=SCENARIO_DEFS[1]["color"],
        label=SCENARIO_DEFS[1]["label"],
    )
    axs[1].set_title("Jet Immersion at CLmax")
    axs[1].set_xlabel("Motor vertical drop [mm]")
    axs[1].set_ylabel("Immersion factor")
    axs[1].grid(True, alpha=0.25)
    axs[1].legend()

    axs[2].plot(
        drop_mm,
        [float(row["overdrop_severity_at_clmax"]) for row in clean_rows],
        marker="o",
        linewidth=2.0,
        color=SCENARIO_DEFS[0]["color"],
        label="Severity (clean)",
    )
    axs[2].plot(
        drop_mm,
        [float(row["overdrop_severity_at_clmax"]) for row in flap_rows],
        marker="s",
        linewidth=2.0,
        color=SCENARIO_DEFS[1]["color"],
        label="Severity (flap)",
    )
    ax2b = axs[2].twinx()
    ax2b.plot(
        drop_mm,
        [float(row["overdrop_peak_shift_deg_at_clmax"]) for row in clean_rows],
        linestyle="--",
        linewidth=1.8,
        color=SCENARIO_DEFS[0]["color"],
        alpha=0.8,
        label="Peak shift (clean)",
    )
    ax2b.plot(
        drop_mm,
        [float(row["overdrop_peak_shift_deg_at_clmax"]) for row in flap_rows],
        linestyle="--",
        linewidth=1.8,
        color=SCENARIO_DEFS[1]["color"],
        alpha=0.8,
        label="Peak shift (flap)",
    )
    axs[2].set_title("Over-Drop Penalty at CLmax")
    axs[2].set_xlabel("Motor vertical drop [mm]")
    axs[2].set_ylabel("Severity scalar")
    ax2b.set_ylabel("Peak shift [deg]")
    axs[2].grid(True, alpha=0.25)
    lines_a, labels_a = axs[2].get_legend_handles_labels()
    lines_b, labels_b = ax2b.get_legend_handles_labels()
    axs[2].legend(lines_a + lines_b, labels_a + labels_b, loc="upper left", fontsize=8)

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
    lines = [
        f"# Motor Height Trade Study: Rank {rank}",
        "",
        f"- Prop concept: `{n_props} x {diameter_in:.1f} x {pitch_in:.1f} in`",
        "- Scenarios compared:",
        "  - `clean_blowing`: clean blown wing with no flap deployed",
        "  - `slotted_flap_blowing`: slotted flap plus blowing",
        "",
        "## Key takeaways",
        "",
    ]

    for scenario_def in SCENARIO_DEFS:
        scenario_rows = _rows_for_scenario(rows, scenario_def["name"])
        best_vstall = min(scenario_rows, key=lambda row: float(row["equivalent_vstall_mps"]))
        peak_cl = max(float(row["equivalent_reference_clmax"]) for row in scenario_rows)
        plateau = next(
            (
                row
                for row in scenario_rows
                if float(row["equivalent_reference_clmax"]) >= 0.9995 * peak_cl
            ),
            best_vstall,
        )
        harshest = max(scenario_rows, key=lambda row: float(row["poststall_drop_5deg"]))
        lines.extend(
            [
                f"### {scenario_def['label']}",
                "",
                f"- Lowest equivalent stall speed in the sweep occurred at `{1000.0 * float(best_vstall['prop_drop_m']):.0f} mm` drop: `{float(best_vstall['equivalent_vstall_mps']):.3f} m/s`.",
                f"- `CLmax` effectively saturated by about `{1000.0 * float(plateau['prop_drop_m']):.0f} mm` drop (`{float(plateau['prop_drop_fraction_of_chord']):.2f} c`).",
                f"- The strongest modeled post-stall harshness occurred at `{1000.0 * float(harshest['prop_drop_m']):.0f} mm` drop with `poststall_drop_5deg = {float(harshest['poststall_drop_5deg']):.3f}`.",
                "",
            ]
        )

    lines.extend(
        [
            "## Artifacts",
            "",
            f"- Metric plot: ![]({output.metric_plot.name})",
            "",
            f"- Whole-wing CL overlay: ![]({output.overlay_plot.name})",
            "",
            f"- Geometry/penalty plot: ![]({output.geometry_plot.name})",
            "",
            "## Notes",
            "",
            "- The selected propulsion architecture, flap geometry, and aileron geometry are held fixed while sweeping only motor vertical drop.",
            "- The trade now combines two mechanisms: Cambridge-style jet immersion and a bounded over-drop penalty that only becomes significant near stall.",
            "- The clean-wing and slotted-flap branches are evaluated separately so the motor-height sensitivity of each high-lift system can be compared directly.",
            "- The over-drop penalty is still a concept-level surrogate; it is intended to reproduce the qualitative MIT-style trend that excessive drop can sharpen stall, not to claim an experimentally calibrated absolute optimum.",
        ]
    )
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
            concept,
            flap,
            baseline_aileron.aileron_span_fraction,
            baseline_aileron.aileron_chord_fraction,
        )
        if aileron is None:
            raise ValueError("Baseline aileron geometry became infeasible during the motor-height trade sweep.")

        polars = _selected_flap_section_polars(config, concept, flap)
        total_curve_rows = _total_high_lift_curve_data(config, concept, flap, polars)
        drop_m = flap.prop_drop_m

        for scenario_def in SCENARIO_DEFS:
            alpha = np.asarray([row["alpha_deg"] for row in total_curve_rows], dtype=float)
            cl_total = np.asarray([row[scenario_def["cl_key"]] for row in total_curve_rows], dtype=float)
            cd_total = np.asarray([row[scenario_def["cd_key"]] for row in total_curve_rows], dtype=float)
            cm_total = np.asarray([row[scenario_def["cm_key"]] for row in total_curve_rows], dtype=float)

            for curve_row, cl_value, cd_value, cm_value in zip(total_curve_rows, cl_total, cd_total, cm_total):
                overlay_rows.append(
                    {
                        "scenario": scenario_def["name"],
                        "scenario_label": scenario_def["label"],
                        "prop_drop_fraction_of_chord": drop_fraction,
                        "prop_drop_fraction_of_diameter": drop_m / max(concept.prop_diameter_m, 1e-9),
                        "prop_drop_m": drop_m,
                        "alpha_deg": curve_row["alpha_deg"],
                        "cl_total": float(cl_value),
                        "cd_total": float(cd_value),
                        "cm_total": float(cm_value),
                    }
                )

            metrics = _scenario_curve_metrics(alpha, cl_total, cl_required=cl_required)
            peak_index = int(np.argmax(cl_total))
            alpha_req = metrics["alpha_required_deg"]
            cd_req = _interp_y_for_x(alpha, cd_total, alpha_req) if alpha_req is not None else None
            summary_rows.append(
                {
                    "rank": concept.rank,
                    "scenario": scenario_def["name"],
                    "scenario_label": scenario_def["label"],
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
                    "recommended_aileron_span_fraction": aileron.aileron_span_fraction,
                    "recommended_aileron_chord_fraction": aileron.aileron_chord_fraction,
                    "equivalent_reference_clmax": float(metrics["peak_cl"]),
                    "equivalent_vstall_mps": (
                        2.0 * config.mission.gross_weight_n
                        / max(config.mission.air_density_kgpm3 * config.mission.wing_area_m2 * max(float(metrics["peak_cl"]), 1e-9), 1e-9)
                    ) ** 0.5,
                    "low_speed_lift_margin": float(metrics["peak_cl"]) / max(cl_required, 1e-9) - 1.0,
                    "alpha_at_peak_total_cl_deg": float(metrics["alpha_at_peak_deg"]),
                    "alpha_poststall_90pct_deg": (
                        "" if metrics["alpha_poststall_90pct_deg"] is None else float(metrics["alpha_poststall_90pct_deg"])
                    ),
                    "poststall_drop_5deg": float(metrics["poststall_drop_5deg"]),
                    "alpha_required_for_4ms_cl_deg": "" if alpha_req is None else alpha_req,
                    "cd_at_required_alpha": "" if cd_req is None else cd_req,
                    "cm_at_peak_total_cl": float(cm_total[peak_index]),
                    "cambridge_immersion_at_clmax": float(total_curve_rows[peak_index][scenario_def["immersion_key"]]),
                    "cambridge_margin_m_at_clmax": float(total_curve_rows[peak_index][scenario_def["margin_key"]]),
                    "overdrop_severity_at_clmax": float(total_curve_rows[peak_index][scenario_def["severity_key"]]),
                    "overdrop_peak_shift_deg_at_clmax": float(total_curve_rows[peak_index][scenario_def["peak_shift_key"]]),
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
