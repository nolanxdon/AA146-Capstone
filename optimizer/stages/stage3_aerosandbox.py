from __future__ import annotations

import csv
from pathlib import Path

from optimizer.core.data_models import Stage1MissionConfig


INPUT_CSV = Path("outputs/stage2_prop_span_report.csv")
OUTPUT_CSV = Path("outputs/stage3_aerosandbox_queue.csv")


def load_stage2_report(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def run_stage3(mission: Stage1MissionConfig | None = None) -> list[dict[str, str | float]]:
    mission = mission or Stage1MissionConfig()
    stage2_rows = load_stage2_report(INPUT_CSV)

    queue: list[dict[str, str | float]] = []
    for row in stage2_rows:
        queue.append(
            {
                "n_props": row["n_props"],
                "prop_diameter_in": row["prop_diameter_in"],
                "prop_pitch_ratio": row["prop_pitch_ratio"],
                "prop_family": row["prop_family"],
                "baseline_span_m": mission.span_m,
                "baseline_chord_m": mission.chord_m,
                "baseline_low_speed_mps": mission.low_speed_mps,
                "baseline_cruise_speed_mps": mission.cruise_speed_mps,
                "seed_low_speed_rpm": row["solved_low_speed_rpm"],
                "seed_cruise_rpm": row["solved_cruise_rpm"],
                "refine_variables": "chord,taper,washout,flap_span,flap_chord,flap_deflection,prop_y_shift",
                "status": "READY_FOR_AEROSANDBOX",
            }
        )

    OUTPUT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT_CSV.open("w", newline="", encoding="utf-8") as f:
        if queue:
            writer = csv.DictWriter(f, fieldnames=list(queue[0].keys()))
            writer.writeheader()
            writer.writerows(queue)

    print("Stage 3 AeroSandbox handoff")
    print(f"  Stage 2 input:            {INPUT_CSV}")
    print(f"  Refinement queue entries: {len(queue)}")
    print(f"  Output queue:             {OUTPUT_CSV}")
    print("  Next step: connect this queue to an AeroSandbox optimizer loop.")

    return queue


if __name__ == "__main__":
    run_stage3()
