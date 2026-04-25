from __future__ import annotations

import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

from optimizer.core.control_surface_sizing import (
    AileronSizingResult,
    FlapSizingResult,
    RectangularWingControlConfig,
    SelectedPropConcept,
    _curve_peak_and_stall_metrics,
    _display_airfoil_name,
    _flapped_section_polars,
    _interp_x_for_y,
    _interp_y_for_x,
    _load_selected_concept,
    _overdrop_adjusted_blown_polars,
    _pick_flap_and_aileron,
    _prop_drop_m,
    _slugify_airfoil,
    _slugify_concept,
    _total_high_lift_curve_data,
    evaluate_aileron_candidate,
    evaluate_flap_candidate,
    ensure_stage3_runtime,
)
from optimizer.core.data_models import Stage1Candidate
from optimizer.core.physics import bisection_solve, prop_operating_point, slipstream_velocity_after_prop
from optimizer.core.workflow_style import flap_state_color


@dataclass(frozen=True)
class WingSpeedSweepOutput:
    output_dir: Path
    summary_csv: Path
    summary_md: Path
    curve_csv: Path
    performance_plot: Path
    operating_plot: Path
    selected_curve_plot: Path


def default_speed_sweep_grid(
    speed_min_mps: float = 5.0,
    speed_max_mps: float = 15.0,
    step_mps: float = 0.5,
) -> tuple[float, ...]:
    count = int(round((speed_max_mps - speed_min_mps) / step_mps)) + 1
    return tuple(round(speed_min_mps + idx * step_mps, 6) for idx in range(count))


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        if not rows:
            return
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _rounded_reynolds(reynolds: float) -> float:
    return max(1.0, 1000.0 * round(reynolds / 1000.0))


def _build_eval_concept(
    concept: SelectedPropConcept,
    *,
    rpm: float,
    veff_mps: float,
    power_electric_w: float,
) -> SelectedPropConcept:
    return replace(
        concept,
        low_speed_rpm=float(rpm),
        low_speed_veff_mps=float(veff_mps),
        low_speed_required_veff_mps=float(veff_mps),
        low_speed_power_w=float(power_electric_w),
    )


def _build_eval_flap(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    *,
    flap_deflection_deg: float,
    veff_mps: float,
) -> FlapSizingResult:
    prop_drop_m = _prop_drop_m(config, concept)
    return replace(
        flap,
        flap_deflection_deg=float(flap_deflection_deg),
        actual_veff_mps=float(veff_mps),
        prop_drop_m=prop_drop_m,
        prop_drop_fraction_of_chord=prop_drop_m / max(config.mission.chord_m, 1e-9),
        prop_drop_fraction_of_diameter=prop_drop_m / max(concept.prop_diameter_m, 1e-9),
    )


def _selected_state_polars(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    *,
    polar_cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]],
) -> dict[str, np.ndarray]:
    mission = config.mission
    chord = mission.chord_m
    re_inf = mission.air_density_kgpm3 * mission.low_speed_mps * chord / mission.dynamic_viscosity_pas
    re_blown = mission.air_density_kgpm3 * concept.low_speed_veff_mps * chord / mission.dynamic_viscosity_pas
    re_inf = _rounded_reynolds(re_inf)
    re_blown = _rounded_reynolds(re_blown)
    induced_velocity_mps = max(0.0, 0.5 * (concept.low_speed_veff_mps - mission.low_speed_mps))

    unblown = _flapped_section_polars(
        config,
        re_inf,
        flap.flap_chord_fraction,
        flap.flap_deflection_deg,
        cache=polar_cache,
    )
    blown = _flapped_section_polars(
        config,
        re_blown,
        flap.flap_chord_fraction,
        flap.flap_deflection_deg,
        cache=polar_cache,
    )
    overdrop = _overdrop_adjusted_blown_polars(
        config,
        concept,
        alpha_deg=np.asarray(unblown["alpha_deg"], dtype=float),
        flap_chord_fraction=flap.flap_chord_fraction,
        flap_deflection_deg=flap.flap_deflection_deg,
        clean_blown_cl=np.asarray(blown["clean_cl"], dtype=float),
        clean_blown_cd=np.asarray(blown["clean_cd"], dtype=float),
        clean_blown_cm=np.asarray(blown["clean_cm"], dtype=float),
        flap_blown_cl=np.asarray(blown["flapped_cl"], dtype=float),
        flap_blown_cd=np.asarray(blown["flapped_cd"], dtype=float),
        flap_blown_cm=np.asarray(blown["flapped_cm"], dtype=float),
    )
    return {
        "airfoil_name": np.asarray([config.airfoil_name], dtype=object),
        "alpha_deg": unblown["alpha_deg"],
        "clean_cl_unblown": unblown["clean_cl"],
        "flapped_cl_unblown": unblown["flapped_cl"],
        "clean_cd_unblown": unblown["clean_cd"],
        "flapped_cd_unblown": unblown["flapped_cd"],
        "clean_cm_unblown": unblown["clean_cm"],
        "flapped_cm_unblown": unblown["flapped_cm"],
        "clean_cl_blown": overdrop["clean"]["adjusted_cl"],
        "flapped_cl_blown": overdrop["flap"]["adjusted_cl"],
        "clean_cd_blown": overdrop["clean"]["adjusted_cd"],
        "flapped_cd_blown": overdrop["flap"]["adjusted_cd"],
        "clean_cm_blown": overdrop["clean"]["adjusted_cm"],
        "flapped_cm_blown": overdrop["flap"]["adjusted_cm"],
        "freestream_velocity_mps": np.asarray([mission.low_speed_mps], dtype=float),
        "blown_effective_velocity_mps": np.asarray([concept.low_speed_veff_mps], dtype=float),
        "required_effective_velocity_mps": np.asarray([concept.low_speed_veff_mps], dtype=float),
        "induced_velocity_mps": np.asarray([induced_velocity_mps], dtype=float),
        "re_inf": np.asarray([re_inf], dtype=float),
        "re_blown": np.asarray([re_blown], dtype=float),
    }


def _initial_rpm_guess(concept: SelectedPropConcept, speed_mps: float, flap_deflection_deg: float) -> float:
    rpm_guess = float(
        np.interp(
            speed_mps,
            [4.0, 10.0, 15.0],
            [
                concept.low_speed_rpm,
                concept.cruise_rpm,
                max(3000.0, 0.90 * concept.cruise_rpm),
            ],
        )
    )
    if flap_deflection_deg > 0.0:
        rpm_guess = max(rpm_guess, 0.92 * concept.low_speed_rpm)
    return rpm_guess


def _evaluate_at_rpm(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    candidate: Stage1Candidate,
    *,
    rpm: float,
    flap_deflection_deg: float,
    polar_cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]],
) -> dict[str, Any]:
    mission = config.mission
    op = prop_operating_point(mission, candidate, rpm, mission.low_speed_mps)
    veff_mps = slipstream_velocity_after_prop(mission, candidate, op["thrust_total_n"], mission.low_speed_mps)
    eval_concept = _build_eval_concept(
        concept,
        rpm=rpm,
        veff_mps=veff_mps,
        power_electric_w=op["power_electric_total_w"],
    )
    eval_flap = _build_eval_flap(
        config,
        eval_concept,
        flap,
        flap_deflection_deg=flap_deflection_deg,
        veff_mps=veff_mps,
    )
    polars = _selected_state_polars(config, eval_concept, eval_flap, polar_cache=polar_cache)
    curve_rows = _total_high_lift_curve_data(config, eval_concept, eval_flap, polars)

    alpha = np.asarray([row["alpha_deg"] for row in curve_rows], dtype=float)
    cl = np.asarray([row["cl_all_high_lift"] for row in curve_rows], dtype=float)
    cd = np.asarray([row["cd_all_high_lift"] for row in curve_rows], dtype=float)
    cm = np.asarray([row["cm_all_high_lift"] for row in curve_rows], dtype=float)
    q_inf = 0.5 * mission.air_density_kgpm3 * mission.low_speed_mps**2
    cl_required = mission.gross_weight_n / max(q_inf * mission.wing_area_m2, 1e-9)
    alpha_required = _interp_x_for_y(alpha, cl, cl_required)
    if alpha_required is None and cl[0] >= cl_required:
        alpha_required = float(alpha[0])
    cd_required = _interp_y_for_x(alpha, cd, alpha_required) if alpha_required is not None else None
    cm_required = _interp_y_for_x(alpha, cm, alpha_required) if alpha_required is not None else None
    drag_required = (
        q_inf * mission.wing_area_m2 * cd_required
        if cd_required is not None
        else None
    )
    metrics = _curve_peak_and_stall_metrics(alpha, cl)
    induced_velocity_mps = max(0.0, 0.5 * (veff_mps - mission.low_speed_mps))
    return {
        "op": op,
        "curve_rows": curve_rows,
        "alpha": alpha,
        "cl": cl,
        "cd": cd,
        "cm": cm,
        "cl_required": cl_required,
        "alpha_required_deg": alpha_required,
        "cd_required": cd_required,
        "cm_required": cm_required,
        "drag_required_n": drag_required,
        "metrics": metrics,
        "veff_mps": veff_mps,
        "induced_velocity_mps": induced_velocity_mps,
        "re_inf": float(polars["re_inf"][0]),
        "re_blown": float(polars["re_blown"][0]),
        "eval_concept": eval_concept,
        "eval_flap": eval_flap,
    }


def _solve_speed_state(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    *,
    speed_mps: float,
    flap_deflection_deg: float,
) -> dict[str, Any]:
    mission = replace(
        config.mission,
        low_speed_mps=float(speed_mps),
        cruise_speed_mps=float(speed_mps),
        low_speed_rpm_bounds=(3000, 14000),
        cruise_rpm_bounds=(3000, 14000),
    )
    config_eval = replace(config, mission=mission)
    candidate = Stage1Candidate(
        n_props=concept.n_props,
        prop_diameter_in=concept.prop_diameter_in,
        prop_pitch_ratio=concept.prop_pitch_ratio,
        prop_family=concept.prop_family,
    )
    polar_cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]] = {}
    rpm_lower = float(min(mission.low_speed_rpm_bounds[0], mission.cruise_rpm_bounds[0]))
    rpm_upper = float(max(mission.low_speed_rpm_bounds[1], mission.cruise_rpm_bounds[1]))
    rpm_guess = float(np.clip(_initial_rpm_guess(concept, speed_mps, flap_deflection_deg), rpm_lower, rpm_upper))
    state_cache: dict[float, dict[str, Any]] = {}

    def evaluate_rpm(rpm: float) -> dict[str, Any]:
        key = round(float(rpm), 3)
        cached = state_cache.get(key)
        if cached is not None:
            return cached
        result = _evaluate_at_rpm(
            config_eval,
            concept,
            flap,
            candidate,
            rpm=float(rpm),
            flap_deflection_deg=flap_deflection_deg,
            polar_cache=polar_cache,
        )
        state_cache[key] = result
        return result

    def residual(rpm: float) -> float:
        current = evaluate_rpm(rpm)
        peak_cl = float(current["metrics"]["peak_cl"])
        cl_required = float(current["cl_required"])
        if current["alpha_required_deg"] is None or current["drag_required_n"] is None:
            return peak_cl - cl_required
        return min(
            peak_cl - cl_required,
            float(current["op"]["thrust_total_n"] - float(current["drag_required_n"])),
        )

    solved_rpm = bisection_solve(
        lower=rpm_lower,
        upper=rpm_upper,
        residual_fn=residual,
        tolerance=mission.rpm_solver_tolerance_rpm,
    )
    if solved_rpm is None:
        solved_rpm = rpm_upper
    final_eval = evaluate_rpm(solved_rpm)
    feasible = (
        final_eval["alpha_required_deg"] is not None
        and final_eval["drag_required_n"] is not None
        and float(final_eval["metrics"]["peak_cl"]) >= float(final_eval["cl_required"]) - 1e-9
        and final_eval["op"]["thrust_total_n"] >= float(final_eval["drag_required_n"]) - 1e-9
    )
    return {
        "config": config_eval,
        "speed_mps": speed_mps,
        "flap_deflection_deg": flap_deflection_deg,
        "rpm": solved_rpm,
        "feasible": feasible,
        **final_eval,
    }


def _summary_row(
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    aileron: AileronSizingResult,
    state: dict[str, Any],
) -> dict[str, Any]:
    mission = state["config"].mission
    metrics = state["metrics"]
    peak_cl = float(metrics["peak_cl"])
    vstall = math.sqrt(
        max(2.0 * mission.gross_weight_n / (mission.air_density_kgpm3 * mission.wing_area_m2 * max(peak_cl, 1e-9)), 0.0)
    )
    cm_required = state["cm_required"]
    drag_required = state["drag_required_n"]
    return {
        "airfoil_name": _display_airfoil_name(state["config"].airfoil_name),
        "rank": concept.rank,
        "n_props": concept.n_props,
        "prop_diameter_in": concept.prop_diameter_in,
        "prop_pitch_in": concept.prop_pitch_in,
        "prop_family": concept.prop_family,
        "blade_count_metadata": concept.blade_count_metadata,
        "speed_mps": float(state["speed_mps"]),
        "flap_deflection_deg": float(state["flap_deflection_deg"]),
        "is_feasible": int(bool(state["feasible"])),
        "rpm": float(state["rpm"]),
        "veff_mps": float(state["veff_mps"]),
        "induced_velocity_mps": float(state["induced_velocity_mps"]),
        "re_inf": float(state["re_inf"]),
        "re_blown": float(state["re_blown"]),
        "cl_required": float(state["cl_required"]),
        "clmax": peak_cl,
        "equivalent_vstall_mps": vstall,
        "lift_margin": peak_cl / max(float(state["cl_required"]), 1e-9) - 1.0,
        "alpha_at_clmax_deg": float(metrics["alpha_at_peak_deg"]),
        "alpha_required_deg": "" if state["alpha_required_deg"] is None else float(state["alpha_required_deg"]),
        "alpha_poststall_90pct_deg": (
            "" if metrics["alpha_poststall_90pct_deg"] is None else float(metrics["alpha_poststall_90pct_deg"])
        ),
        "poststall_drop_5deg": float(metrics["poststall_drop_5deg"]),
        "cd_at_required_alpha": "" if state["cd_required"] is None else float(state["cd_required"]),
        "cm_at_required_alpha": "" if cm_required is None else float(cm_required),
        "drag_required_n": "" if drag_required is None else float(drag_required),
        "thrust_available_n": float(state["op"]["thrust_total_n"]),
        "thrust_minus_drag_n": "" if drag_required is None else float(state["op"]["thrust_total_n"] - drag_required),
        "power_electric_total_w": float(state["op"]["power_electric_total_w"]),
        "power_shaft_total_w": float(state["op"]["power_shaft_total_w"]),
        "torque_per_prop_nm": float(state["op"]["torque_per_prop_nm"]),
        "tip_mach": float(state["op"]["tip_mach"]),
        "flap_span_fraction": flap.flap_span_fraction,
        "flap_chord_fraction": flap.flap_chord_fraction,
        "aileron_span_fraction": aileron.aileron_span_fraction,
        "aileron_chord_fraction": aileron.aileron_chord_fraction,
        "prop_drop_fraction_of_chord": flap.prop_drop_fraction_of_chord,
        "prop_drop_m": flap.prop_drop_m,
    }


def _curve_rows(
    state: dict[str, Any],
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in state["curve_rows"]:
        rows.append(
            {
                "speed_mps": float(state["speed_mps"]),
                "flap_deflection_deg": float(state["flap_deflection_deg"]),
                "rpm": float(state["rpm"]),
                "veff_mps": float(state["veff_mps"]),
                "alpha_deg": float(row["alpha_deg"]),
                "cl_total": float(row["cl_all_high_lift"]),
                "cd_total": float(row["cd_all_high_lift"]),
                "cm_total": float(row["cm_all_high_lift"]),
                "cambridge_clean_immersion_factor": float(row["cambridge_clean_immersion_factor"]),
                "cambridge_flap_immersion_factor": float(row["cambridge_flap_immersion_factor"]),
            }
        )
    return rows


def _render_performance_plot(rows: list[dict[str, Any]], output_path: Path, *, airfoil_name: str) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    fig, axs = plt.subplots(2, 2, figsize=(12.6, 8.6))
    axs = axs.ravel()

    for flap_deflection_deg in (0.0, 20.0, 40.0):
        state_rows = [row for row in rows if abs(float(row["flap_deflection_deg"]) - flap_deflection_deg) < 1e-9]
        state_rows.sort(key=lambda row: float(row["speed_mps"]))
        color = flap_state_color(flap_deflection_deg)
        speeds = [float(row["speed_mps"]) for row in state_rows]
        axs[0].plot(speeds, [float(row["clmax"]) for row in state_rows], color=color, linewidth=2.2, label=rf"$\delta_f={flap_deflection_deg:.0f}^\circ$")
        axs[0].plot(speeds, [float(row["cl_required"]) for row in state_rows], linestyle="--", color=color, linewidth=1.2, alpha=0.85)
        axs[1].plot(speeds, [100.0 * float(row["lift_margin"]) for row in state_rows], color=color, linewidth=2.2)
        axs[2].plot(speeds, [float(row["power_electric_total_w"]) for row in state_rows], color=color, linewidth=2.2)
        axs[3].plot(speeds, [float(row["rpm"]) for row in state_rows], color=color, linewidth=2.2)

    axs[0].set_title(f"{_display_airfoil_name(airfoil_name)} | CLmax and CL Required")
    axs[0].set_ylabel("Freestream-referenced CL")
    axs[1].set_title("Lift Margin vs Speed")
    axs[1].set_ylabel("Lift margin [%]")
    axs[2].set_title("Electrical Power vs Speed")
    axs[2].set_ylabel("Electrical power [W]")
    axs[3].set_title("Solved RPM vs Speed")
    axs[3].set_ylabel("RPM")

    for ax in axs:
        ax.set_xlabel("Speed [m/s]")
        ax.grid(True, alpha=0.25)
    axs[0].legend()

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_operating_plot(rows: list[dict[str, Any]], output_path: Path, *, airfoil_name: str) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    fig, axs = plt.subplots(2, 2, figsize=(12.6, 8.6))
    axs = axs.ravel()

    for flap_deflection_deg in (0.0, 20.0, 40.0):
        state_rows = [row for row in rows if abs(float(row["flap_deflection_deg"]) - flap_deflection_deg) < 1e-9]
        state_rows.sort(key=lambda row: float(row["speed_mps"]))
        color = flap_state_color(flap_deflection_deg)
        speeds = [float(row["speed_mps"]) for row in state_rows]
        axs[0].plot(speeds, [float(row["veff_mps"]) for row in state_rows], color=color, linewidth=2.2, label=rf"$V_{{eff}}, \delta_f={flap_deflection_deg:.0f}^\circ$")
        axs[0].plot(speeds, [float(row["induced_velocity_mps"]) for row in state_rows], linestyle="--", color=color, linewidth=1.2, alpha=0.85)
        alpha_req = [np.nan if row["alpha_required_deg"] == "" else float(row["alpha_required_deg"]) for row in state_rows]
        cm_req = [np.nan if row["cm_at_required_alpha"] == "" else float(row["cm_at_required_alpha"]) for row in state_rows]
        axs[1].plot(speeds, alpha_req, color=color, linewidth=2.2)
        axs[2].plot(speeds, cm_req, color=color, linewidth=2.2)
        axs[3].plot(speeds, [float(row["tip_mach"]) for row in state_rows], color=color, linewidth=2.2)

    axs[0].set_title(f"{_display_airfoil_name(airfoil_name)} | Effective and Induced Velocity")
    axs[0].set_ylabel("Velocity [m/s]")
    axs[1].set_title("Alpha Required for Level Flight")
    axs[1].set_ylabel("Alpha required [deg]")
    axs[2].set_title("Wing CM at Required Alpha")
    axs[2].set_ylabel("Freestream-referenced C_M")
    axs[3].set_title("Tip Mach vs Speed")
    axs[3].set_ylabel("Tip Mach")

    for ax in axs:
        ax.set_xlabel("Speed [m/s]")
        ax.grid(True, alpha=0.25)
    axs[0].legend(fontsize=8)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_selected_curve_plot(curve_rows: list[dict[str, Any]], output_path: Path, *, airfoil_name: str) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    selected_speeds = (5.0, 10.0, 15.0)
    fig, axs = plt.subplots(1, len(selected_speeds), figsize=(15.0, 4.6), sharey=True)

    for ax, speed_mps in zip(axs, selected_speeds):
        for flap_deflection_deg in (0.0, 20.0, 40.0):
            state_rows = [
                row for row in curve_rows
                if abs(float(row["speed_mps"]) - speed_mps) < 1e-9
                and abs(float(row["flap_deflection_deg"]) - flap_deflection_deg) < 1e-9
            ]
            state_rows.sort(key=lambda row: float(row["alpha_deg"]))
            color = flap_state_color(flap_deflection_deg)
            ax.plot(
                [float(row["alpha_deg"]) for row in state_rows],
                [float(row["cl_total"]) for row in state_rows],
                color=color,
                linewidth=2.1,
                label=rf"$\delta_f={flap_deflection_deg:.0f}^\circ$",
            )
        ax.set_title(f"{speed_mps:.0f} m/s")
        ax.set_xlabel("Alpha [deg]")
        ax.grid(True, alpha=0.25)
        ax.legend(fontsize=8)

    axs[0].set_ylabel("Freestream-referenced wing C_L")
    fig.suptitle(f"{_display_airfoil_name(airfoil_name)} Whole-Wing CL Curves at Selected Speeds")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _write_summary_markdown(
    summary_rows: list[dict[str, Any]],
    output: WingSpeedSweepOutput,
    *,
    airfoil_name: str,
    flap: FlapSizingResult,
    aileron: AileronSizingResult,
) -> None:
    lines = [
        f"# Frozen-Geometry Speed Sweep | {_display_airfoil_name(airfoil_name)}",
        "",
        f"- Flap geometry held fixed at `span={flap.flap_span_fraction:.2f}` semispan and `c_f/c={flap.flap_chord_fraction:.2f}`.",
        f"- Aileron geometry held fixed at `span={aileron.aileron_span_fraction:.2f}` semispan and `c_a/c={aileron.aileron_chord_fraction:.2f}`.",
        f"- Motor drop held fixed at `{1000.0 * flap.prop_drop_m:.0f} mm` (`{flap.prop_drop_fraction_of_chord:.2f} c`).",
        "- RPM is re-solved at each speed for each flap state using trimmed level-flight thrust closure.",
        "",
        "## Artifacts",
        "",
        f"- Summary CSV: [{output.summary_csv.name}]({output.summary_csv.name})",
        f"- Curve CSV: [{output.curve_csv.name}]({output.curve_csv.name})",
        f"- Performance plot: ![]({output.performance_plot.name})",
        "",
        f"- Operating-point plot: ![]({output.operating_plot.name})",
        "",
        f"- Selected-speed CL curves: ![]({output.selected_curve_plot.name})",
        "",
    ]
    best_rows = [row for row in summary_rows if abs(float(row["flap_deflection_deg"]) - 40.0) < 1e-9]
    if best_rows:
        best_low = min(best_rows, key=lambda row: float(row["equivalent_vstall_mps"]))
        lines.extend(
            [
                "## Notes",
                "",
                (
                    f"- The strongest high-lift branch in the sweep occurred near `{float(best_low['speed_mps']):.0f} m/s` "
                    f"with `CLmax = {float(best_low['clmax']):.3f}` and `Vstall = {float(best_low['equivalent_vstall_mps']):.3f} m/s`."
                ),
                "- The `0°`, `20°`, and `40°` branches are all evaluated with the same frozen wing/control geometry so only the operating point and flap deflection change across the sweep.",
            ]
        )
    output.summary_md.parent.mkdir(parents=True, exist_ok=True)
    output.summary_md.write_text("\n".join(lines), encoding="utf-8")


def run_wing_speed_sweep(
    *,
    rank: int = 6,
    blade_count_metadata: int = 3,
    airfoil_name: str = "dae51",
    speeds_mps: tuple[float, ...] = default_speed_sweep_grid(),
    flap_deflections_deg: tuple[float, ...] = (0.0, 20.0, 40.0),
    prop_drop_fraction_of_chord: float = 0.12,
    fixed_flap_span_fraction: float | None = None,
    fixed_flap_chord_fraction: float | None = None,
    fixed_aileron_span_fraction: float | None = None,
    fixed_aileron_chord_fraction: float | None = None,
    output_root: Path = Path("outputs/wing_speed_sweep"),
) -> WingSpeedSweepOutput:
    base_config = RectangularWingControlConfig(
        blade_count_metadata=blade_count_metadata,
        airfoil_name=airfoil_name.lower(),
        prop_drop_fraction_of_chord=prop_drop_fraction_of_chord,
        prop_drop_fraction_of_diameter=None,
    )
    concept = _load_selected_concept(rank=rank, blade_count_metadata=blade_count_metadata)
    if fixed_flap_span_fraction is None or fixed_flap_chord_fraction is None:
        flap, aileron, _, _ = _pick_flap_and_aileron(base_config, concept)
    else:
        flap = evaluate_flap_candidate(
            base_config,
            concept,
            fixed_flap_span_fraction,
            fixed_flap_chord_fraction,
            base_config.max_slotted_flap_deflection_deg,
            polar_cache={},
        )
        if fixed_aileron_span_fraction is None or fixed_aileron_chord_fraction is None:
            aileron = evaluate_aileron_candidate(
                base_config,
                concept,
                flap,
                base_config.aileron_span_fraction_values[-1],
                base_config.aileron_chord_fraction_values[-1],
            )
        else:
            aileron = evaluate_aileron_candidate(
                base_config,
                concept,
                flap,
                fixed_aileron_span_fraction,
                fixed_aileron_chord_fraction,
            )
        if aileron is None:
            raise ValueError("The fixed aileron geometry is infeasible for the selected frozen-geometry speed sweep.")

    output_dir = output_root / _slugify_concept(concept)
    if base_config.airfoil_name != "s1210":
        output_dir = output_dir / _slugify_airfoil(base_config.airfoil_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    output = WingSpeedSweepOutput(
        output_dir=output_dir,
        summary_csv=output_dir / "speed_sweep_summary.csv",
        summary_md=output_dir / "speed_sweep_summary.md",
        curve_csv=output_dir / "speed_sweep_curves.csv",
        performance_plot=output_dir / "speed_sweep_performance.png",
        operating_plot=output_dir / "speed_sweep_operating_points.png",
        selected_curve_plot=output_dir / "speed_sweep_selected_curves.png",
    )

    summary_rows: list[dict[str, Any]] = []
    curve_rows: list[dict[str, Any]] = []
    for speed_mps in speeds_mps:
        for flap_deflection_deg in flap_deflections_deg:
            state = _solve_speed_state(
                base_config,
                concept,
                flap,
                speed_mps=float(speed_mps),
                flap_deflection_deg=float(flap_deflection_deg),
            )
            summary_rows.append(_summary_row(concept, flap, aileron, state))
            curve_rows.extend(_curve_rows(state))

    _write_csv(output.summary_csv, summary_rows)
    _write_csv(output.curve_csv, curve_rows)
    _render_performance_plot(summary_rows, output.performance_plot, airfoil_name=base_config.airfoil_name)
    _render_operating_plot(summary_rows, output.operating_plot, airfoil_name=base_config.airfoil_name)
    _render_selected_curve_plot(curve_rows, output.selected_curve_plot, airfoil_name=base_config.airfoil_name)
    _write_summary_markdown(summary_rows, output, airfoil_name=base_config.airfoil_name, flap=flap, aileron=aileron)
    return output
