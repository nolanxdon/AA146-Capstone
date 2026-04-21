from __future__ import annotations

import csv
from pathlib import Path

from optimizer.core.data_models import Stage1MissionConfig, Stage1SweepConfig
from optimizer.core.pareto import pareto_front
from optimizer.core.physics import evaluate_stage1_candidate, generate_stage1_candidates


OUTPUT_DIR = Path("outputs")
ALL_RESULTS_CSV = OUTPUT_DIR / "stage1_screen_results.csv"
PARETO_RESULTS_CSV = OUTPUT_DIR / "stage1_pareto_front.csv"


def write_results_csv(path: Path, results) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "is_feasible",
                "n_props",
                "prop_diameter_in",
                "prop_pitch_ratio",
                "prop_pitch_in",
                "prop_family",
                "solved_low_speed_rpm",
                "solved_cruise_rpm",
                "low_speed_power_w",
                "cruise_power_w",
                "loiter_energy_wh",
                "low_speed_thrust_n",
                "low_speed_drag_n",
                "cruise_thrust_n",
                "cruise_drag_n",
                "low_speed_veff_mps",
                "low_speed_required_veff_mps",
                "low_speed_blown_span_fraction",
                "low_speed_cl_required",
                "cruise_cl_required",
                "low_speed_clmax_blown_section",
                "low_speed_thrust_to_drag",
                "cruise_thrust_to_drag",
                "low_speed_disk_loading_npm2",
                "low_speed_momentum_coefficient",
                "per_prop_low_speed_torque_nm",
                "per_prop_cruise_torque_nm",
                "per_prop_peak_shaft_power_w",
                "per_prop_peak_current_a",
                "motor_required_current_a",
                "motor_required_kv_rpm_per_v",
                "low_speed_tip_mach",
                "cruise_tip_mach",
                "blade_reynolds_low_speed",
                "prop_reynolds_penalty_factor",
                "prop_mass_kg_per_unit",
                "motor_mass_kg_per_unit",
                "esc_mass_kg_per_unit",
                "propulsion_mass_kg_total",
                "battery_mass_kg_estimate",
                "total_built_mass_kg",
                "mass_budget_margin_kg",
                "packing_margin_m",
                "fuselage_margin_m",
                "inter_prop_margin_m",
                "tip_margin_m",
                "low_speed_thrust_margin_n",
                "cruise_thrust_margin_n",
                "low_speed_veff_margin_mps",
                "low_speed_lift_margin",
                "cruise_lift_margin",
                "disk_loading_lower_margin_npm2",
                "disk_loading_upper_margin_npm2",
            ]
        )
        for result in results:
            writer.writerow(
                [
                    int(result.is_feasible),
                    result.candidate.n_props,
                    f"{result.candidate.prop_diameter_in:.3f}",
                    f"{result.candidate.prop_pitch_ratio:.3f}",
                    f"{result.candidate.prop_pitch_in:.3f}",
                    result.candidate.prop_family,
                    f"{result.solved_low_speed_rpm:.1f}",
                    f"{result.solved_cruise_rpm:.1f}",
                    f"{result.low_speed_power_w:.3f}",
                    f"{result.cruise_power_w:.3f}",
                    f"{result.loiter_energy_wh:.3f}",
                    f"{result.low_speed_thrust_n:.3f}",
                    f"{result.low_speed_drag_n:.3f}",
                    f"{result.cruise_thrust_n:.3f}",
                    f"{result.cruise_drag_n:.3f}",
                    f"{result.low_speed_veff_mps:.3f}",
                    f"{result.low_speed_required_veff_mps:.3f}",
                    f"{result.low_speed_blown_span_fraction:.3f}",
                    f"{result.low_speed_cl_required:.3f}",
                    f"{result.cruise_cl_required:.3f}",
                    f"{result.low_speed_clmax_blown_section:.3f}",
                    f"{result.low_speed_thrust_to_drag:.3f}",
                    f"{result.cruise_thrust_to_drag:.3f}",
                    f"{result.low_speed_disk_loading_npm2:.3f}",
                    f"{result.low_speed_momentum_coefficient:.3f}",
                    f"{result.per_prop_low_speed_torque_nm:.5f}",
                    f"{result.per_prop_cruise_torque_nm:.5f}",
                    f"{result.per_prop_peak_shaft_power_w:.3f}",
                    f"{result.per_prop_peak_current_a:.3f}",
                    f"{result.motor_required_current_a:.3f}",
                    f"{result.motor_required_kv_rpm_per_v:.3f}",
                    f"{result.low_speed_tip_mach:.3f}",
                    f"{result.cruise_tip_mach:.3f}",
                    f"{result.blade_reynolds_low_speed:.0f}",
                    f"{result.prop_reynolds_penalty_factor:.3f}",
                    f"{result.prop_mass_kg_per_unit:.4f}",
                    f"{result.motor_mass_kg_per_unit:.4f}",
                    f"{result.esc_mass_kg_per_unit:.4f}",
                    f"{result.propulsion_mass_kg_total:.4f}",
                    f"{result.battery_mass_kg_estimate:.4f}",
                    f"{result.total_built_mass_kg:.4f}",
                    f"{result.mass_budget_margin_kg:.4f}",
                    f"{result.constraints['packing_margin_m']:.5f}",
                    f"{result.constraints['fuselage_margin_m']:.5f}",
                    f"{result.constraints['inter_prop_margin_m']:.5f}",
                    f"{result.constraints['tip_margin_m']:.5f}",
                    f"{result.constraints['low_speed_thrust_margin_n']:.3f}",
                    f"{result.constraints['cruise_thrust_margin_n']:.3f}",
                    f"{result.constraints['low_speed_veff_margin_mps']:.3f}",
                    f"{result.constraints['low_speed_lift_margin']:.3f}",
                    f"{result.constraints['cruise_lift_margin']:.3f}",
                    f"{result.constraints['disk_loading_lower_margin_npm2']:.3f}",
                    f"{result.constraints['disk_loading_upper_margin_npm2']:.3f}",
                ]
            )


def run_stage1(
    mission: Stage1MissionConfig | None = None,
    sweep: Stage1SweepConfig | None = None,
) -> tuple[list, list]:
    mission = mission or Stage1MissionConfig()
    sweep = sweep or Stage1SweepConfig()

    print("Stage 1 V2 variable ranges")
    print(f"  n_props: {sweep.n_props_values}")
    print(f"  prop_diameter_in: {sweep.prop_diameter_in_values}")
    print(f"  prop_pitch_ratio: {sweep.prop_pitch_ratio_values}")
    print(f"  prop_family: {sweep.prop_family_values}")
    print(f"  low_speed_rpm solved in [{mission.low_speed_rpm_bounds[0]}, {mission.low_speed_rpm_bounds[1]}]")
    print(f"  cruise_rpm solved in [{mission.cruise_rpm_bounds[0]}, {mission.cruise_rpm_bounds[1]}]")
    print("")
    print("Stage 1 objectives (3-D Pareto)")
    print(f"  1. Minimize low-speed electrical power at {mission.low_speed_mps:.1f} m/s")
    print(f"  2. Minimize {mission.loiter_time_min:.0f}-minute loiter energy at {mission.cruise_speed_mps:.1f} m/s")
    print("  3. Minimize total propulsion mass (prop + motor + ESC, per-prop x N)")
    print("")
    print("Stage 1 constraints")
    print("  - Spanwise packing margin >= 0")
    print(
        f"  - Fuselage clearance = {mission.fuselage_prop_clearance_in:.1f} in, "
        f"inter-prop clearance = {mission.inter_prop_clearance_in:.1f} in, "
        f"tip margin = {mission.tip_margin_in:.1f} in"
    )
    print("  - Low-speed lift feasible with Cμ blown-wing model")
    print(f"  - Low-speed section CL ceiling with margin = {mission.cl_section_ceiling_flapped:.2f}")
    print("  - Cruise lift feasible with clean-wing CLmax")
    print("  - Low-speed thrust >= drag * margin")
    print("  - Cruise thrust >= drag * margin")
    print("  - Tip Mach <= configured limit")
    print("  - solved_low_speed_rpm >= solved_cruise_rpm")
    print(
        f"  - Disk loading in [{mission.disk_loading_min_npm2:.0f}, "
        f"{mission.disk_loading_max_npm2:.0f}] N/m^2"
    )
    print(
        f"  - Total built mass = fixed ({mission.fixed_system_mass_kg:.2f} kg) + "
        f"propulsion + battery <= {mission.max_mass_kg:.2f} kg"
    )
    print("")

    candidates = generate_stage1_candidates(sweep)
    results = [evaluate_stage1_candidate(mission, candidate) for candidate in candidates]
    pareto = pareto_front(results)

    write_results_csv(ALL_RESULTS_CSV, results)
    write_results_csv(PARETO_RESULTS_CSV, pareto)

    feasible_count = sum(result.is_feasible for result in results)
    print(f"Evaluated candidates: {len(results)}")
    print(f"Feasible candidates:  {feasible_count}")
    print(f"Pareto candidates:    {len(pareto)}")
    print(f"All results:          {ALL_RESULTS_CSV}")
    print(f"Pareto front:         {PARETO_RESULTS_CSV}")

    if pareto:
        print("")
        print("Top 10 Pareto candidates")
        for idx, result in enumerate(pareto[:10], start=1):
            print(
                f"{idx:2d}) N={result.candidate.n_props:2d} | "
                f"D={result.candidate.prop_diameter_in:.1f} in | "
                f"P/D={result.candidate.prop_pitch_ratio:.2f} | "
                f"family={result.candidate.prop_family:11s} | "
                f"RPM_low={result.solved_low_speed_rpm:5.0f} | "
                f"RPM_cruise={result.solved_cruise_rpm:5.0f} | "
                f"P_low={result.low_speed_power_w:6.1f} W | "
                f"E_loit={result.loiter_energy_wh:5.1f} Wh | "
                f"m_prop={result.propulsion_mass_kg_total:4.2f} kg | "
                f"m_bat={result.battery_mass_kg_estimate:4.2f} kg | "
                f"m_tot={result.total_built_mass_kg:4.2f} kg | "
                f"T/A={result.low_speed_disk_loading_npm2:5.1f} N/m^2"
            )

    return results, pareto


if __name__ == "__main__":
    run_stage1()
