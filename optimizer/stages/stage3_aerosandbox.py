from __future__ import annotations

import csv
from pathlib import Path

from optimizer.core.data_models import Stage1MissionConfig
from optimizer.core.stage3_refinement import (
    STAGE3_GALLERY_MD,
    STAGE3_TRADE_PLOT,
    STAGE3_VISUAL_DIR,
    candidate_key,
    load_csv_rows,
    load_stage1_lookup,
    refine_stage3_candidate,
    write_gallery,
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
    "baseline_span_m",
    "baseline_chord_m",
    "baseline_low_speed_mps",
    "baseline_cruise_speed_mps",
    "seed_low_speed_rpm",
    "seed_cruise_rpm",
    "refine_variables",
    "status",
]
STAGE3_RESULTS_FIELDNAMES = [
    "rank",
    "status",
    "n_props",
    "prop_diameter_in",
    "prop_pitch_ratio",
    "prop_family",
    "seed_low_speed_rpm",
    "seed_cruise_rpm",
    "blown_span_fraction",
    "baseline_required_veff_mps",
    "baseline_actual_veff_mps",
    "baseline_low_speed_power_w",
    "baseline_cruise_power_w",
    "baseline_cruise_drag_n",
    "optimized_root_chord_m",
    "optimized_tip_chord_m",
    "optimized_taper",
    "optimized_wing_area_m2",
    "optimized_mac_m",
    "optimized_aspect_ratio",
    "optimized_washout_deg",
    "optimized_flap_span_fraction",
    "optimized_flap_chord_fraction",
    "recommended_flap_deflection_deg",
    "optimized_aileron_span_fraction",
    "optimized_aileron_chord_fraction",
    "recommended_aileron_deflection_deg",
    "horizontal_tail_area_m2",
    "vertical_tail_area_m2",
    "tail_arm_m",
    "elevator_chord_fraction",
    "rudder_chord_fraction",
    "refined_low_speed_cl_required",
    "refined_low_speed_required_veff_mps",
    "refined_low_speed_power_proxy_w",
    "propulsor_veff_margin_mps",
    "cruise_alpha_deg",
    "cruise_lift_n",
    "cruise_drag_n",
    "cruise_cl",
    "cruise_cd",
    "cruise_cm",
    "static_cm_alpha_per_deg",
    "section_re_low_speed",
    "section_re_cruise",
    "clean_section_clmax_raw",
    "flapped_section_clmax_raw",
    "flap_delta_cl_raw",
    "aileron_delta_cl_per_deg",
    "elevator_delta_cm_per_deg",
    "rudder_delta_cn_per_deg",
    "geometry_score",
    "top_view_png",
    "three_view_png",
    "wireframe_png",
    "polar_png",
    "mesh_npz",
]


def _write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        if rows:
            writer.writerows(rows)


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
        "polar_png": "polars.png",
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


def _stage3_queue_rows(
    mission: Stage1MissionConfig,
    stage2_rows: list[dict[str, str]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for row in stage2_rows:
        rows.append(
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
                "refine_variables": "root_chord,taper,washout,flap_span,flap_chord,aileron_span,aileron_chord,tail_volume",
                "status": "QUEUED_FOR_REFINEMENT",
            }
        )
    return rows


def run_stage3(mission: Stage1MissionConfig | None = None) -> list[dict[str, object]]:
    mission = mission or Stage1MissionConfig()
    stage2_rows = load_csv_rows(STAGE2_INPUT_CSV)
    stage1_lookup = load_stage1_lookup(STAGE1_PARETO_INPUT_CSV)

    queue_rows = _stage3_queue_rows(mission, stage2_rows)
    _write_csv(QUEUE_OUTPUT_CSV, queue_rows, STAGE3_FIELDNAMES)

    results: list[dict[str, object]] = []
    for index, stage2_row in enumerate(stage2_rows, start=1):
        key = candidate_key(stage2_row)
        stage1_row = stage1_lookup.get(key)
        if stage1_row is None:
            results.append(
                {
                    "rank": 0,
                    "status": "MISSING_STAGE1_CONTEXT",
                    "n_props": int(float(stage2_row["n_props"])),
                    "prop_diameter_in": float(stage2_row["prop_diameter_in"]),
                    "prop_pitch_ratio": float(stage2_row["prop_pitch_ratio"]),
                    "prop_family": stage2_row["prop_family"],
                    **{field: "" for field in STAGE3_RESULTS_FIELDNAMES if field not in {"rank", "status", "n_props", "prop_diameter_in", "prop_pitch_ratio", "prop_family"}},
                }
            )
            continue

        results.append(
            refine_stage3_candidate(
                mission,
                stage2_row,
                stage1_row,
                rank_seed=index,
            )
        )

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

    print("Stage 3 AeroSandbox refinement")
    print(f"  Stage 2 input:            {STAGE2_INPUT_CSV}")
    print(f"  Queue output:             {QUEUE_OUTPUT_CSV}")
    print(f"  Refined result count:     {len(ordered_results)}")
    print(f"  Results CSV:              {RESULTS_OUTPUT_CSV}")
    print(f"  Top designs CSV:          {TOP_RESULTS_OUTPUT_CSV}")
    print(f"  Visual gallery:           {STAGE3_GALLERY_MD}")
    print(f"  Trade-space plot:         {STAGE3_TRADE_PLOT}")
    if successful:
        best = successful[0]
        print(
            f"  Best concept: rank #{best['rank']} | N={best['n_props']} | "
            f"D={best['prop_diameter_in']:.1f} in | "
            f"root chord={best['optimized_root_chord_m']:.3f} m | "
            f"taper={best['optimized_taper']:.2f} | "
            f"Veff,req={best['refined_low_speed_required_veff_mps']:.2f} m/s | "
            f"cruise drag={best['cruise_drag_n']:.2f} N"
        )

    return ordered_results


if __name__ == "__main__":
    run_stage3()
