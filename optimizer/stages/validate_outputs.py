from __future__ import annotations

import csv
from pathlib import Path

from optimizer.core.data_models import Stage1MissionConfig, Stage1SweepConfig
from optimizer.stages.stage1_screen import STAGE1_FIELDNAMES
from optimizer.stages.stage2_prop_span import STAGE2_FIELDNAMES
from optimizer.stages.stage3_aerosandbox import STAGE3_FIELDNAMES, STAGE3_RESULTS_FIELDNAMES


STAGE1_CSV = Path("outputs/stage1_screen_results.csv")
PARETO_CSV = Path("outputs/stage1_pareto_front.csv")
STAGE2_CSV = Path("outputs/stage2_prop_span_report.csv")
STAGE3_QUEUE_CSV = Path("outputs/stage3_aerosandbox_queue.csv")
STAGE3_RESULTS_CSV = Path("outputs/stage3_aerosandbox_results.csv")
MOTOR_TARGET_ROOT = Path("outputs/motor_targeting")


def _load_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _load_header(path: Path) -> list[str]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            return next(reader)
        except StopIteration:
            return []


def validate_outputs(
    mission: Stage1MissionConfig | None = None,
    sweep: Stage1SweepConfig | None = None,
) -> list[str]:
    mission = mission or Stage1MissionConfig()
    sweep = sweep or Stage1SweepConfig()
    issues: list[str] = []

    stage1_rows = _load_rows(STAGE1_CSV)
    pareto_rows = _load_rows(PARETO_CSV)
    stage2_rows = _load_rows(STAGE2_CSV)
    stage3_queue_rows = _load_rows(STAGE3_QUEUE_CSV)
    stage3_result_rows = _load_rows(STAGE3_RESULTS_CSV)
    stage1_header = _load_header(STAGE1_CSV)
    pareto_header = _load_header(PARETO_CSV)
    stage2_header = _load_header(STAGE2_CSV)
    stage3_queue_header = _load_header(STAGE3_QUEUE_CSV)
    stage3_results_header = _load_header(STAGE3_RESULTS_CSV)

    if stage1_header and stage1_header != STAGE1_FIELDNAMES:
        issues.append("Stage 1 results header does not match the current writer schema.")
    if pareto_header and pareto_header != STAGE1_FIELDNAMES:
        issues.append("Stage 1 Pareto header does not match the current writer schema.")
    if stage2_header and stage2_header != STAGE2_FIELDNAMES:
        issues.append("Stage 2 header does not match the current writer schema.")
    if stage3_queue_header and stage3_queue_header != STAGE3_FIELDNAMES:
        issues.append("Stage 3 queue header does not match the current writer schema.")
    if stage3_results_header and stage3_results_header != STAGE3_RESULTS_FIELDNAMES:
        issues.append("Stage 3 results header does not match the current writer schema.")

    expected_count = (
        len(sweep.n_props_values)
        * len(sweep.prop_diameter_in_values)
        * len(sweep.prop_pitch_ratio_values)
        * len(sweep.prop_family_values)
    )
    if len(stage1_rows) != expected_count:
        issues.append(
            f"Stage 1 row count mismatch: expected {expected_count}, found {len(stage1_rows)}."
        )

    for idx, row in enumerate(stage1_rows, start=1):
        low_rpm = float(row["solved_low_speed_rpm"])
        cruise_rpm = float(row["solved_cruise_rpm"])
        clmax_b = float(row["low_speed_clmax_blown_section"])
        loiter_wh = float(row["loiter_energy_wh"])
        cruise_power = float(row["cruise_power_w"])
        is_feasible = row["is_feasible"] == "1"

        if is_feasible:
            if low_rpm < cruise_rpm - 1e-9:
                issues.append(f"Row {idx}: feasible point has low-speed RPM below cruise RPM.")
            if float(row["packing_margin_m"]) < -1e-9:
                issues.append(f"Row {idx}: feasible point has negative packing margin.")
            if float(row["low_speed_thrust_margin_n"]) < -1e-9:
                issues.append(f"Row {idx}: feasible point has negative low-speed thrust margin.")
            if float(row["cruise_thrust_margin_n"]) < -1e-9:
                issues.append(f"Row {idx}: feasible point has negative cruise thrust margin.")
            if float(row["mass_budget_margin_kg"]) < -1e-9:
                issues.append(f"Row {idx}: feasible point violates mass budget.")
            if float(row["low_speed_veff_margin_mps"]) < -1e-9:
                issues.append(f"Row {idx}: feasible point misses required blown velocity.")

        if clmax_b > mission.cl_section_ceiling_flapped + 1e-6:
            issues.append(
                f"Row {idx}: blown-section CLmax {clmax_b:.3f} exceeds configured ceiling "
                f"{mission.cl_section_ceiling_flapped:.3f}."
            )

        expected_loiter = cruise_power * mission.loiter_time_min / 60.0
        if abs(loiter_wh - expected_loiter) > 0.05:
            issues.append(
                f"Row {idx}: loiter energy mismatch, expected {expected_loiter:.3f} Wh "
                f"from cruise power and loiter time."
            )

    if pareto_rows and stage2_rows and len(stage2_rows) != len(pareto_rows):
        issues.append(
            f"Stage 2 report row count {len(stage2_rows)} does not match Pareto row count {len(pareto_rows)}."
        )

    if stage2_rows and stage3_queue_rows and len(stage3_queue_rows) != len(stage2_rows):
        issues.append(
            f"Stage 3 queue row count {len(stage3_queue_rows)} does not match Stage 2 row count {len(stage2_rows)}."
        )

    if stage2_rows and stage3_result_rows and len(stage3_result_rows) != len(stage2_rows):
        issues.append(
            f"Stage 3 result row count {len(stage3_result_rows)} does not match Stage 2 row count {len(stage2_rows)}."
        )

    for idx, row in enumerate(stage3_result_rows, start=1):
        if row["status"] != "SUCCESS":
            issues.append(f"Stage 3 row {idx} has non-success status {row['status']}.")
            continue
        for field in ["top_view_png", "three_view_png", "wireframe_png", "polar_png", "mesh_npz"]:
            if not Path(row[field]).exists():
                issues.append(f"Stage 3 row {idx} is missing artifact {row[field]}.")

    if MOTOR_TARGET_ROOT.exists():
        for run_dir in sorted(path for path in MOTOR_TARGET_ROOT.iterdir() if path.is_dir()):
            summary_csv = run_dir / "motor_target_summary.csv"
            summary_md = run_dir / "motor_target_summary.md"
            candidate_csv = run_dir / "motor_target_candidates.csv"
            operating_plot = run_dir / "motor_target_operating_points.png"

            for path in [summary_csv, summary_md, candidate_csv, operating_plot]:
                if not path.exists():
                    issues.append(f"Motor targeting output is missing artifact {path}.")

            summary_rows = _load_rows(summary_csv)
            candidate_rows = _load_rows(candidate_csv)
            if len(summary_rows) != 1:
                issues.append(
                    f"Motor targeting summary in {run_dir} should contain exactly one row, found {len(summary_rows)}."
                )
            if not candidate_rows:
                issues.append(f"Motor targeting candidates in {run_dir} are empty.")

    return issues


def main() -> None:
    issues = validate_outputs()
    if issues:
        print("Output validation failed:")
        for issue in issues:
            print(f"  - {issue}")
        raise SystemExit(1)
    print("Output validation passed.")


if __name__ == "__main__":
    main()
