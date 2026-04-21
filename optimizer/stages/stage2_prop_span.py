from __future__ import annotations

import csv
from pathlib import Path

from optimizer.core.data_models import Stage1Candidate, Stage1MissionConfig
from optimizer.core.physics import (
    blown_span_fraction,
    packing_margins,
    prop_center_positions,
    prop_operating_point,
)


INPUT_CSV = Path("outputs/stage1_pareto_front.csv")
OUTPUT_CSV = Path("outputs/stage2_prop_span_report.csv")


def _parse_candidate(row: dict[str, str]) -> Stage1Candidate:
    return Stage1Candidate(
        n_props=int(row["n_props"]),
        prop_diameter_in=float(row["prop_diameter_in"]),
        prop_pitch_ratio=float(row["prop_pitch_ratio"]),
        prop_family=row["prop_family"],
    )


def load_stage1_shortlist(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def run_stage2(mission: Stage1MissionConfig | None = None) -> list[dict[str, str | float]]:
    mission = mission or Stage1MissionConfig()
    shortlist = load_stage1_shortlist(INPUT_CSV)

    reports: list[dict[str, str | float]] = []
    for row in shortlist:
        candidate = _parse_candidate(row)
        positions = prop_center_positions(mission, candidate)
        packing = packing_margins(mission, candidate)
        low_speed_rpm = float(row["solved_low_speed_rpm"])
        cruise_rpm = float(row["solved_cruise_rpm"])
        low_op = prop_operating_point(mission, candidate, low_speed_rpm, mission.low_speed_mps)
        cruise_op = prop_operating_point(mission, candidate, cruise_rpm, mission.cruise_speed_mps)

        reports.append(
            {
                "n_props": candidate.n_props,
                "prop_diameter_in": candidate.prop_diameter_in,
                "prop_pitch_ratio": candidate.prop_pitch_ratio,
                "prop_family": candidate.prop_family,
                "solved_low_speed_rpm": low_speed_rpm,
                "solved_cruise_rpm": cruise_rpm,
                "prop_centers_m": ";".join(f"{y:.4f}" for y in positions),
                "blown_span_fraction": blown_span_fraction(mission, candidate),
                "packing_margin_m": packing["packing_margin_m"],
                "fuselage_margin_m": packing["fuselage_margin_m"],
                "inter_prop_margin_m": packing["inter_prop_margin_m"],
                "tip_margin_m": packing["tip_margin_m"],
                "per_prop_low_speed_thrust_n": low_op["thrust_total_n"] / candidate.n_props,
                "per_prop_cruise_thrust_n": cruise_op["thrust_total_n"] / candidate.n_props,
                "per_prop_low_speed_power_w": low_op["power_shaft_total_w"] / candidate.n_props,
                "per_prop_cruise_power_w": cruise_op["power_shaft_total_w"] / candidate.n_props,
                "per_prop_low_speed_torque_nm": low_op["torque_per_prop_nm"],
                "per_prop_cruise_torque_nm": cruise_op["torque_per_prop_nm"],
            }
        )

    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_CSV.open("w", newline="", encoding="utf-8") as f:
        if reports:
            writer = csv.DictWriter(f, fieldnames=list(reports[0].keys()))
            writer.writeheader()
            writer.writerows(reports)

    print("Stage 2 prop-span report")
    print(f"  Stage 1 shortlist source: {INPUT_CSV}")
    print(f"  Candidate count:          {len(reports)}")
    print(f"  Output report:            {OUTPUT_CSV}")
    if reports:
        best = reports[0]
        print(
            f"  Lead concept: N={best['n_props']} | D={best['prop_diameter_in']:.1f} in | "
            f"RPM_low={best['solved_low_speed_rpm']:.0f} | centers={best['prop_centers_m']}"
        )

    return reports


if __name__ == "__main__":
    run_stage2()
