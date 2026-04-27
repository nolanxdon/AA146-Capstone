from __future__ import annotations

import csv
from pathlib import Path

from optimizer.core.data_models import Stage1MissionConfig
from optimizer.core.stage3_refinement import (
    STAGE3_GALLERY_MD,
    STAGE3_CONSTRAINTS_YAML,
    STAGE3_ENGINEERING_TEX,
    STAGE3_READABLE_RESULTS_MD,
    STAGE3_REPORT_MD,
    STAGE3_TRADE_PLOT,
    STAGE3_VISUAL_DIR,
    Stage3SizingConfig,
    candidate_key,
    load_csv_rows,
    load_selected_wing_context,
    load_stage1_lookup,
    load_stage3_sizing_config,
    refine_stage3_candidate,
    write_gallery,
    write_stage3_engineering_tex,
    write_stage3_readable_results,
    write_stage3_report,
    write_trade_space_plot,
)


STAGE2_INPUT_CSV = Path("outputs/stage2_prop_span_report.csv")
STAGE1_PARETO_INPUT_CSV = Path("outputs/stage1_pareto_front.csv")
QUEUE_OUTPUT_CSV = Path("outputs/stage3_aerosandbox_queue.csv")
RESULTS_OUTPUT_CSV = Path("outputs/stage3_aerosandbox_results.csv")
TOP_RESULTS_OUTPUT_CSV = Path("outputs/stage3_aerosandbox_top_designs.csv")

STAGE3_FIELDNAMES = [
    "n_props",
    "prop_diameter_in",
    "prop_pitch_ratio",
    "prop_family",
    "fixed_wing_span_m",
    "fixed_wing_chord_m",
    "fixed_main_airfoil",
    "fixed_total_mass_kg",
    "seed_low_speed_rpm",
    "seed_cruise_rpm",
    "refine_variables",
    "material_model",
    "status",
]

STAGE3_RESULTS_FIELDNAMES = [
    "rank",
    "status",
    "design_warnings",
    "n_props",
    "prop_diameter_in",
    "prop_pitch_ratio",
    "prop_family",
    "seed_low_speed_rpm",
    "seed_cruise_rpm",
    "selected_airfoil",
    "tail_airfoil",
    "wing_context_source",
    "wing_span_m",
    "wing_chord_m",
    "wing_area_m2",
    "wing_aspect_ratio",
    "main_wing_incidence_deg",
    "main_wing_washout_deg",
    "prop_axial_location_fraction_of_chord",
    "prop_axial_x_m",
    "propulsion_model_source",
    "ecalc_static_csv",
    "ecalc_dynamic_csv",
    "wiring_power_loss_factor",
    "avionics_power_loss_factor",
    "avionics_power_base_w",
    "avionics_power_with_losses_w",
    "fuselage_nose_x_m",
    "fuselage_width_m",
    "fuselage_height_m",
    "stage3_slow_flight_speed_mps",
    "upstream_stage12_low_speed_mps",
    "flap_span_fraction",
    "flap_chord_fraction",
    "flap_deflection_slow_deg",
    "aileron_span_fraction",
    "aileron_chord_fraction",
    "htail_span_m",
    "htail_root_chord_m",
    "htail_tip_chord_m",
    "htail_area_m2",
    "htail_aspect_ratio",
    "htail_taper",
    "htail_incidence_deg",
    "elevator_chord_fraction",
    "elevator_max_deflection_deg",
    "target_slow_pitch_control_cm",
    "slow_pitch_control_cm_authority",
    "slow_pitch_control_margin_percent",
    "elevator_trim_cruise_deg",
    "vtail_span_m",
    "vtail_root_chord_m",
    "vtail_tip_chord_m",
    "vtail_area_m2",
    "vtail_aspect_ratio",
    "vtail_taper",
    "vertical_tail_incidence_deg",
    "rudder_chord_fraction",
    "rudder_max_deflection_deg",
    "target_slow_yaw_control_cn",
    "slow_yaw_control_cn_authority",
    "slow_yaw_control_margin_percent",
    "tail_arm_m",
    "horizontal_tail_volume",
    "vertical_tail_volume",
    "horizontal_tail_volume_min",
    "horizontal_tail_volume_max",
    "vertical_tail_volume_min",
    "vertical_tail_volume_max",
    "cg_x_m",
    "cg_percent_mac",
    "cg_target_x_m",
    "cg_target_percent_mac",
    "cg_error_m",
    "cg_error_percent_mac",
    "stage1_baseline_cg_used_percent_mac",
    "stage1_baseline_cg_required_percent_mac",
    "neutral_point_x_m",
    "static_margin_mac",
    "vlm_cm_alpha_per_deg",
    "main_wing_foam_mass_kg",
    "htail_foam_mass_kg",
    "vtail_foam_mass_kg",
    "total_tail_foam_mass_kg",
    "stage1_total_built_mass_kg",
    "stage3_total_built_mass_kg",
    "gross_flight_mass_kg",
    "payload_mass_lb",
    "payload_mass_kg",
    "payload_on_mass_kg",
    "post_delivery_mass_kg",
    "post_delivery_mass_delta_kg",
    "battery_sizing_assumes_payload_on",
    "remaining_mass_to_gross_kg",
    "mass_budget_margin_kg",
    "blown_span_fraction",
    "clmax_source",
    "clmax_curve_source",
    "stage12_high_lift_polar_source",
    "physical_clmax_source",
    "stage12_clean_section_clmax_unblown",
    "stage12_clean_section_clmax_blown",
    "stage12_flapped_section_clmax_unblown",
    "stage12_flapped_section_clmax_blown",
    "stage12_weighted_blown_physical_clmax",
    "no_flap_clmax",
    "flap_only_clmax",
    "clean_blowing_clmax",
    "flap_down_blown_clmax",
    "raw_no_flap_clmax",
    "raw_flap_only_clmax",
    "raw_clean_blowing_clmax",
    "raw_flap_down_blown_clmax",
    "clmax_was_capped",
    "no_flap_clmax_alpha_deg",
    "flap_only_clmax_alpha_deg",
    "clean_blowing_clmax_alpha_deg",
    "flap_down_blown_clmax_alpha_deg",
    "fallback_slotted_flap_section_clmax",
    "slow_flight_stall_margin_factor",
    "main_wing_pitch_up_stall_margin_factor",
    "main_wing_clean_design_clmax_for_pitch_up",
    "main_wing_flapped_design_local_clmax",
    "payload_on_cruise_pitch_up_stall_margin_percent",
    "post_delivery_cruise_pitch_up_stall_margin_percent",
    "payload_on_climb_pitch_up_stall_margin_percent",
    "post_delivery_climb_pitch_up_stall_margin_percent",
    "slow_flight_unblown_clmax",
    "slow_flight_blown_clmax",
    "slow_flight_blown_local_clmax",
    "slow_flight_unblown_equivalent_clmax",
    "slow_flight_blown_equivalent_clmax",
    "slow_flight_design_unblown_clmax",
    "slow_flight_design_blown_clmax",
    "slow_flight_design_blown_local_clmax",
    "low_speed_required_veff_mps",
    "low_speed_actual_veff_mps",
    "low_speed_drag_n",
    "low_speed_natural_drag_n",
    "low_speed_added_drag_required_n",
    "low_speed_drag_delta_vs_cruise_n",
    "low_speed_blown_lift_thrust_n",
    "low_speed_stage1_baseline_drag_n",
    "low_speed_wing_profile_drag_n",
    "low_speed_wing_induced_drag_n",
    "low_speed_wing_induced_local_drag_n",
    "low_speed_wing_induced_freestream_equiv_drag_n",
    "low_speed_unblown_induced_drag_n",
    "low_speed_blown_induced_drag_n",
    "low_speed_htail_drag_n",
    "low_speed_vtail_drag_n",
    "low_speed_fuselage_drag_n",
    "slow_flight_lift_available_n",
    "slow_flight_lift_target_n",
    "slow_flight_lift_margin_n",
    "slow_flight_lift_margin_percent",
    "slow_flight_equiv_stall_speed_mps",
    "slow_flight_freestream_cl_required",
    "slow_flight_uniform_local_cl_required",
    "slow_flight_unblown_dynamic_pressure_pa",
    "slow_flight_blown_dynamic_pressure_pa",
    "slow_flight_effective_dynamic_pressure_pa",
    "slow_flight_blown_to_unblown_q_ratio",
    "slow_flight_blown_area_m2",
    "slow_flight_unblown_area_m2",
    "slow_flight_unblown_local_cl",
    "slow_flight_blown_local_cl",
    "slow_flight_local_cl_margin",
    "slow_flight_required_veff_margin_mps",
    "slow_flight_feasible",
    "low_speed_power_w",
    "low_speed_propulsion_electric_power_without_aircraft_losses_w",
    "low_speed_propulsion_wiring_loss_w",
    "low_speed_avionics_power_loss_w",
    "low_speed_aircraft_power_losses_w",
    "low_speed_energy_wh",
    "battery_cruise_segment_min",
    "battery_slow_segment_min",
    "battery_cruise_energy_wh",
    "battery_slow_energy_wh",
    "battery_usable_energy_wh",
    "battery_reserve_fraction",
    "battery_capacity_required_wh",
    "battery_pack_voltage_v",
    "battery_capacity_required_mah",
    "battery_mass_required_kg",
    "mission_profile_battery_cell_count",
    "mission_profile_cell_nominal_voltage_v",
    "mission_profile_pack_voltage_v",
    "mission_profile_usable_energy_wh",
    "mission_profile_used_capacity_mah",
    "mission_profile_capacity_required_wh",
    "mission_profile_capacity_required_mah",
    "mission_profile_reserve_fraction",
    "low_speed_rpm",
    "low_speed_rpm_feasible",
    "low_speed_thrust_required_n",
    "low_speed_throttle_percent",
    "low_speed_ct",
    "low_speed_cp",
    "cruise_drag_n",
    "cruise_wing_profile_drag_n",
    "cruise_wing_induced_drag_n",
    "cruise_htail_drag_n",
    "cruise_vtail_drag_n",
    "cruise_fuselage_drag_n",
    "cruise_power_w",
    "cruise_propulsion_electric_power_without_aircraft_losses_w",
    "cruise_propulsion_wiring_loss_w",
    "cruise_avionics_power_loss_w",
    "cruise_aircraft_power_losses_w",
    "cruise_rpm",
    "cruise_rpm_feasible",
    "cruise_thrust_required_n",
    "cruise_throttle_percent",
    "cruise_ct",
    "cruise_cp",
    "cruise_blown_effective_velocity_mps",
    "cruise_unblown_dynamic_pressure_pa",
    "cruise_blown_dynamic_pressure_pa",
    "cruise_effective_dynamic_pressure_pa",
    "cruise_blown_to_unblown_q_ratio",
    "cruise_uniform_local_cl_required",
    "cruise_alpha_deg",
    "cruise_body_alpha_proxy_deg",
    "cruise_section_alpha_proxy_deg",
    "cruise_lift_n",
    "cruise_vlm_induced_drag_n",
    "cruise_cl_required",
    "cruise_cl",
    "cruise_cd",
    "cruise_l_over_d",
    "cruise_cm",
    "trim_residual_lift_n",
    "trim_residual_cm",
    "payload_on_cruise_cl_required",
    "payload_on_cruise_uniform_local_cl_required",
    "payload_on_cruise_drag_n",
    "payload_on_cruise_thrust_required_n",
    "payload_on_cruise_throttle_percent",
    "payload_on_cruise_rpm",
    "payload_on_cruise_power_w",
    "payload_on_cruise_blown_effective_velocity_mps",
    "payload_on_cruise_effective_dynamic_pressure_pa",
    "payload_on_slow_cl_required",
    "payload_on_slow_uniform_local_cl_required",
    "payload_on_slow_drag_n",
    "payload_on_slow_natural_drag_n",
    "payload_on_slow_added_drag_required_n",
    "payload_on_slow_thrust_required_n",
    "payload_on_slow_throttle_percent",
    "payload_on_slow_rpm",
    "payload_on_slow_power_w",
    "payload_on_slow_blown_effective_velocity_mps",
    "payload_on_slow_effective_dynamic_pressure_pa",
    "payload_on_slow_unblown_local_cl",
    "payload_on_slow_blown_local_cl",
    "payload_on_slow_local_cl_margin",
    "payload_on_approach_cl_required",
    "payload_on_approach_uniform_local_cl_required",
    "payload_on_approach_drag_n",
    "payload_on_approach_thrust_required_n",
    "payload_on_approach_throttle_percent",
    "payload_on_approach_rpm",
    "payload_on_approach_power_w",
    "payload_on_approach_blown_effective_velocity_mps",
    "payload_on_approach_effective_dynamic_pressure_pa",
    "payload_on_approach_elevator_trim_deg",
    "payload_on_approach_rpm_feasible",
    "payload_on_climb_cl_required",
    "payload_on_climb_uniform_local_cl_required",
    "payload_on_climb_drag_n",
    "payload_on_climb_thrust_required_n",
    "payload_on_climb_throttle_percent",
    "payload_on_climb_rpm",
    "payload_on_climb_power_w",
    "payload_on_climb_blown_effective_velocity_mps",
    "payload_on_climb_effective_dynamic_pressure_pa",
    "payload_on_climb_elevator_trim_deg",
    "payload_on_climb_rpm_feasible",
    "post_delivery_cruise_cl_required",
    "post_delivery_cruise_uniform_local_cl_required",
    "post_delivery_cruise_drag_n",
    "post_delivery_cruise_thrust_required_n",
    "post_delivery_cruise_throttle_percent",
    "post_delivery_cruise_rpm",
    "post_delivery_cruise_power_w",
    "post_delivery_cruise_blown_effective_velocity_mps",
    "post_delivery_cruise_effective_dynamic_pressure_pa",
    "post_delivery_slow_cl_required",
    "post_delivery_slow_uniform_local_cl_required",
    "post_delivery_slow_drag_n",
    "post_delivery_slow_natural_drag_n",
    "post_delivery_slow_added_drag_required_n",
    "post_delivery_slow_thrust_required_n",
    "post_delivery_slow_throttle_percent",
    "post_delivery_slow_rpm",
    "post_delivery_slow_power_w",
    "post_delivery_slow_blown_effective_velocity_mps",
    "post_delivery_slow_effective_dynamic_pressure_pa",
    "post_delivery_slow_unblown_local_cl",
    "post_delivery_slow_blown_local_cl",
    "post_delivery_slow_local_cl_margin",
    "post_delivery_approach_cl_required",
    "post_delivery_approach_uniform_local_cl_required",
    "post_delivery_approach_drag_n",
    "post_delivery_approach_thrust_required_n",
    "post_delivery_approach_throttle_percent",
    "post_delivery_approach_rpm",
    "post_delivery_approach_power_w",
    "post_delivery_approach_blown_effective_velocity_mps",
    "post_delivery_approach_effective_dynamic_pressure_pa",
    "post_delivery_approach_elevator_trim_deg",
    "post_delivery_approach_rpm_feasible",
    "post_delivery_climb_cl_required",
    "post_delivery_climb_uniform_local_cl_required",
    "post_delivery_climb_drag_n",
    "post_delivery_climb_thrust_required_n",
    "post_delivery_climb_throttle_percent",
    "post_delivery_climb_rpm",
    "post_delivery_climb_power_w",
    "post_delivery_climb_blown_effective_velocity_mps",
    "post_delivery_climb_effective_dynamic_pressure_pa",
    "post_delivery_climb_elevator_trim_deg",
    "post_delivery_climb_rpm_feasible",
    "section_re_cruise",
    "section_re_low_speed",
    "main_wing_re_cruise",
    "main_wing_re_low_speed",
    "htail_re_cruise",
    "htail_re_low_speed",
    "vtail_re_cruise",
    "vtail_re_low_speed",
    "clean_section_clmax",
    "flapped_section_clmax",
    "approach_target_angle_deg",
    "approach_target_sink_rate_fps",
    "approach_target_sink_rate_mps",
    "approach_speed_mps",
    "approach_alpha_deg",
    "approach_cl_required",
    "approach_uniform_local_cl_required",
    "approach_lift_required_n",
    "approach_drag_n",
    "approach_thrust_required_n",
    "approach_power_w",
    "approach_rpm",
    "approach_throttle_percent",
    "approach_max_descent_rate_fps",
    "approach_descent_rate_limited",
    "approach_sink_rate_target_met",
    "approach_sink_rate_mps",
    "approach_sink_rate_fps",
    "approach_glide_ratio",
    "approach_elevator_trim_deg",
    "approach_rudder_trim_deg",
    "approach_flap_deflection_deg",
    "approach_wing_profile_drag_n",
    "approach_wing_induced_drag_n",
    "approach_htail_drag_n",
    "approach_vtail_drag_n",
    "approach_fuselage_drag_n",
    "approach_rpm_feasible",
    "approach_blown_effective_velocity_mps",
    "approach_unblown_dynamic_pressure_pa",
    "approach_blown_dynamic_pressure_pa",
    "approach_effective_dynamic_pressure_pa",
    "climb_target_rate_fps",
    "climb_target_rate_mps",
    "climb_angle_deg",
    "climb_speed_mps",
    "climb_cl_required",
    "climb_uniform_local_cl_required",
    "climb_drag_n",
    "climb_thrust_required_n",
    "climb_power_w",
    "climb_rpm",
    "climb_throttle_percent",
    "climb_elevator_trim_deg",
    "climb_rpm_feasible",
    "climb_blown_effective_velocity_mps",
    "climb_unblown_dynamic_pressure_pa",
    "climb_blown_dynamic_pressure_pa",
    "climb_effective_dynamic_pressure_pa",
    "geometry_score",
    "penalty_score",
    "top_view_png",
    "three_view_png",
    "wireframe_png",
    "render_3d_png",
    "polar_png",
    "performance_sweep_csv",
    "drag_power_sweeps_png",
    "drag_components_png",
    "mesh_npz",
    "report_md",
]


def _write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        if rows:
            writer.writerows(rows)


def _stage3_queue_rows(
    mission: Stage1MissionConfig,
    stage2_rows: list[dict[str, str]],
    wing_context: dict[str, object],
    config: Stage3SizingConfig,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for row in stage2_rows:
        rows.append(
            {
                "n_props": row["n_props"],
                "prop_diameter_in": row["prop_diameter_in"],
                "prop_pitch_ratio": row["prop_pitch_ratio"],
                "prop_family": row["prop_family"],
                "fixed_wing_span_m": mission.span_m,
                "fixed_wing_chord_m": mission.chord_m,
                "fixed_main_airfoil": wing_context["selected_airfoil"],
                "fixed_total_mass_kg": mission.gross_mass_kg,
                "seed_low_speed_rpm": row["solved_low_speed_rpm"],
                "seed_cruise_rpm": row["solved_cruise_rpm"],
                "refine_variables": "horizontal_tail_area,horizontal_tail_AR,horizontal_tail_taper,vertical_tail_area,vertical_tail_AR,vertical_tail_taper,tail_arm,tail_incidence",
                "material_model": f"NGX250 foam density {config.foam_density_kgpm3:.1f} kg/m^3",
                "status": "QUEUED_FOR_TAIL_REFINEMENT",
            }
        )
    return rows


def _empty_failure_row(stage2_row: dict[str, str], status: str) -> dict[str, object]:
    row: dict[str, object] = {field: "" for field in STAGE3_RESULTS_FIELDNAMES}
    row.update(
        {
            "rank": 0,
            "status": status,
            "n_props": int(float(stage2_row["n_props"])),
            "prop_diameter_in": float(stage2_row["prop_diameter_in"]),
            "prop_pitch_ratio": float(stage2_row["prop_pitch_ratio"]),
            "prop_family": stage2_row["prop_family"],
        }
    )
    return row


def _propulsion_matches_context(
    stage2_row: dict[str, str],
    wing_context: dict[str, object],
) -> bool:
    fixed_n_props = wing_context.get("fixed_n_props")
    fixed_diameter = wing_context.get("fixed_prop_diameter_in")
    fixed_pitch_ratio = wing_context.get("fixed_prop_pitch_ratio")
    fixed_family = wing_context.get("fixed_prop_family")
    if fixed_n_props is None or fixed_diameter is None or fixed_pitch_ratio is None or fixed_family is None:
        return False
    return all(
        (
            int(float(stage2_row["n_props"])) == int(fixed_n_props),
            abs(float(stage2_row["prop_diameter_in"]) - float(fixed_diameter)) <= 5e-3,
            abs(float(stage2_row["prop_pitch_ratio"]) - float(fixed_pitch_ratio)) <= 5e-3,
            stage2_row["prop_family"] == str(fixed_family),
        )
    )


def _fixed_stage2_rows(
    stage2_rows: list[dict[str, str]],
    wing_context: dict[str, object],
    config: Stage3SizingConfig,
) -> list[dict[str, str]]:
    """Return only the frozen Stage 1/2 propulsion row for Stage 3 refinement."""

    if config.fixed_propulsion_enabled:
        matches = [
            row
            for row in stage2_rows
            if all(
                (
                    int(float(row["n_props"])) == int(config.fixed_n_props),
                    abs(float(row["prop_diameter_in"]) - config.fixed_prop_diameter_in) <= 5e-3,
                    abs(float(row["prop_pitch_ratio"]) - config.fixed_prop_pitch_ratio) <= 5e-3,
                    row["prop_family"] == config.fixed_prop_family,
                )
            )
        ]
        if matches:
            return matches[:1]
        raise ValueError(
            "Stage 3 fixed propulsion was requested, but no matching Stage 2 row was found: "
            f"N={config.fixed_n_props}, D={config.fixed_prop_diameter_in:.3f} in, "
            f"P/D={config.fixed_prop_pitch_ratio:.3f}, family={config.fixed_prop_family!r}."
        )

    matches = [row for row in stage2_rows if _propulsion_matches_context(row, wing_context)]
    if matches:
        return matches[:1]
    return stage2_rows[:1]


def _artifact_slug(row: dict[str, object]) -> str:
    return (
        f"rank{int(row['rank']):02d}_n{int(row['n_props'])}"
        f"_d{float(row['prop_diameter_in']):.1f}"
        f"_pd{float(row['prop_pitch_ratio']):.2f}"
        f"_{row['prop_family']}"
    ).replace(".", "p")


def _rename_stage3_artifacts(rows: list[dict[str, object]]) -> None:
    suffix_map = {
        "top_view_png": "top_view.png",
        "three_view_png": "three_view.png",
        "wireframe_png": "wireframe.png",
        "render_3d_png": "3d_wireframe.png",
        "polar_png": "polars.png",
        "performance_sweep_csv": "performance_sweep.csv",
        "drag_power_sweeps_png": "drag_power_sweeps.png",
        "drag_components_png": "drag_components.png",
        "mesh_npz": "mesh.npz",
    }
    for row in rows:
        if row["status"] != "SUCCESS":
            continue
        slug = _artifact_slug(row)
        for field, suffix in suffix_map.items():
            old_path = Path(str(row[field]))
            if not old_path.exists():
                continue
            new_path = old_path.with_name(f"{slug}_{suffix}")
            if old_path != new_path:
                if new_path.exists():
                    new_path.unlink()
                old_path.rename(new_path)
            row[field] = str(new_path)


def _clear_stale_stage3_rank_artifacts() -> None:
    """Remove old rank-labeled Stage 3 visuals before regenerating the fixed layout."""

    if not STAGE3_VISUAL_DIR.exists():
        return
    for pattern in (
        "rank*_top_view.png",
        "rank*_three_view.png",
        "rank*_wireframe.png",
        "rank*_3d_wireframe.png",
        "rank*_polars.png",
        "rank*_performance_sweep.csv",
        "rank*_drag_power_sweeps.png",
        "rank*_drag_components.png",
        "rank*_mesh.npz",
    ):
        for path in STAGE3_VISUAL_DIR.glob(pattern):
            path.unlink()


def run_stage3(
    mission: Stage1MissionConfig | None = None,
    config: Stage3SizingConfig | None = None,
) -> list[dict[str, object]]:
    mission = mission or Stage1MissionConfig()
    config = config or load_stage3_sizing_config()
    wing_context = load_selected_wing_context(config)
    stage2_rows = load_csv_rows(STAGE2_INPUT_CSV)
    fixed_stage2_rows = _fixed_stage2_rows(stage2_rows, wing_context, config)
    stage1_lookup = load_stage1_lookup(STAGE1_PARETO_INPUT_CSV)

    queue_rows = _stage3_queue_rows(mission, fixed_stage2_rows, wing_context, config)
    _write_csv(QUEUE_OUTPUT_CSV, queue_rows, STAGE3_FIELDNAMES)
    _clear_stale_stage3_rank_artifacts()

    results: list[dict[str, object]] = []
    for index, stage2_row in enumerate(fixed_stage2_rows, start=1):
        key = candidate_key(stage2_row)
        stage1_row = stage1_lookup.get(key)
        if stage1_row is None:
            results.append(_empty_failure_row(stage2_row, "MISSING_STAGE1_CONTEXT"))
            continue

        try:
            results.append(
                refine_stage3_candidate(
                    mission,
                    stage2_row,
                    stage1_row,
                    rank_seed=index,
                    wing_context=wing_context,
                    config=config,
                )
            )
        except Exception as exc:
            failed = _empty_failure_row(stage2_row, "STAGE3_EXCEPTION")
            failed["design_warnings"] = repr(exc)
            results.append(failed)

    successful = sorted(
        [row for row in results if row["status"] == "SUCCESS"],
        key=lambda row: float(row["geometry_score"]),
    )
    for rank, row in enumerate(successful, start=1):
        row["rank"] = rank
    _rename_stage3_artifacts(successful)

    failed = [row for row in results if row["status"] != "SUCCESS"]
    ordered_results = successful + failed

    _write_csv(RESULTS_OUTPUT_CSV, ordered_results, STAGE3_RESULTS_FIELDNAMES)
    _write_csv(TOP_RESULTS_OUTPUT_CSV, ordered_results[:5], STAGE3_RESULTS_FIELDNAMES)

    STAGE3_VISUAL_DIR.mkdir(parents=True, exist_ok=True)
    write_trade_space_plot(ordered_results, STAGE3_TRADE_PLOT)
    write_gallery(ordered_results, STAGE3_GALLERY_MD)
    write_stage3_report(ordered_results, STAGE3_REPORT_MD, mission, config)
    write_stage3_readable_results(ordered_results, STAGE3_READABLE_RESULTS_MD, mission, config)
    write_stage3_engineering_tex(ordered_results, STAGE3_ENGINEERING_TEX, mission, config)

    print("Stage 3 AeroSandbox fixed-wing tail/material refinement")
    print(f"  Stage 2 input:            {STAGE2_INPUT_CSV}")
    print(f"  Stage 3 constraints:      {STAGE3_CONSTRAINTS_YAML}")
    print(f"  Wing context:             {wing_context['source']}")
    print(f"  Frozen airfoil:           {wing_context['selected_airfoil']}")
    if fixed_stage2_rows:
        frozen = fixed_stage2_rows[0]
        print(
            "  Frozen propulsion:        "
            f"N={int(float(frozen['n_props']))}, "
            f"D={float(frozen['prop_diameter_in']):.1f} in, "
            f"P/D={float(frozen['prop_pitch_ratio']):.2f}, "
            f"family={frozen['prop_family']}"
        )
    print(f"  Queue output:             {QUEUE_OUTPUT_CSV}")
    print(f"  Refined result count:     {len(ordered_results)}")
    print(f"  Results CSV:              {RESULTS_OUTPUT_CSV}")
    print(f"  Top designs CSV:          {TOP_RESULTS_OUTPUT_CSV}")
    print(f"  Readable results:         {STAGE3_READABLE_RESULTS_MD}")
    print(f"  Design report:            {STAGE3_REPORT_MD}")
    print(f"  LaTeX report:             {STAGE3_ENGINEERING_TEX}")
    print(f"  Visual gallery:           {STAGE3_GALLERY_MD}")
    print(f"  Trade-space plot:         {STAGE3_TRADE_PLOT}")
    if successful:
        best = successful[0]
        print(
            f"  Best concept: rank #{best['rank']} | N={best['n_props']} | "
            f"D={float(best['prop_diameter_in']):.1f} in | "
            f"H-tail span={float(best['htail_span_m']):.3f} m | "
            f"V-tail height={float(best['vtail_span_m']):.3f} m | "
            f"cruise power={float(best['cruise_power_w']):.1f} W | "
            f"slow power={float(best['low_speed_power_w']):.1f} W"
        )

    return ordered_results


if __name__ == "__main__":
    run_stage3()
