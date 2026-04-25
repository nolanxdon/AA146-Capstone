from __future__ import annotations

import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from optimizer.core.control_surface_sizing import _load_selected_concept, ensure_stage3_runtime
from optimizer.core.data_models import Stage1MissionConfig
from optimizer.core.mass_model import (
    MassModel,
    load_esc_mass_records,
    load_motor_mass_records,
)


STAGE1_PARETO_INPUT_CSV = Path("outputs/stage1_pareto_front.csv")


@dataclass(frozen=True)
class MotorTargetingOutput:
    output_dir: Path
    summary_csv: Path
    summary_md: Path
    candidate_csv: Path
    operating_plot: Path


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


def _find_stage1_row_for_concept(
    *,
    rank: int,
    n_props: int,
    diameter_in: float,
    pitch_ratio: float,
    prop_family: str,
) -> dict[str, str]:
    rows = _load_rows(STAGE1_PARETO_INPUT_CSV)
    for row in rows:
        if (
            int(float(row["n_props"])) == n_props
            and abs(float(row["prop_diameter_in"]) - diameter_in) < 1e-9
            and abs(float(row["prop_pitch_ratio"]) - pitch_ratio) < 1e-9
            and row["prop_family"] == prop_family
        ):
            return row
    raise ValueError(f"Could not find Stage 1 Pareto row for rank {rank}.")


def _parse_example_kv(example: str) -> float | None:
    match = re.search(r"_(\d+)Kv", example)
    if match:
        return float(match.group(1))
    return None


def _select_motor_candidates(
    required_kv: float,
    target_peak_power_w: float,
    target_mass_g: float,
) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for record in load_motor_mass_records():
        kv = _parse_example_kv(record.example)
        kv_penalty = 2.5 if kv is None else abs(math.log(max(kv, 1e-9) / max(required_kv, 1e-9)))
        power_shortfall = max(target_peak_power_w - record.peak_power_w, 0.0) / max(target_peak_power_w, 1e-9)
        mass_penalty = record.mass_g / max(target_mass_g, 1e-9)
        score = 4.0 * power_shortfall + 1.5 * kv_penalty + 0.15 * mass_penalty
        candidates.append(
            {
                "example": record.example,
                "kv_rpm_per_v": kv,
                "peak_power_w": record.peak_power_w,
                "mass_g": record.mass_g,
                "power_margin_ratio": record.peak_power_w / max(target_peak_power_w, 1e-9),
                "kv_ratio": (kv / required_kv) if kv is not None else "",
                "selection_score": score,
            }
        )
    candidates.sort(key=lambda row: float(row["selection_score"]))
    return candidates[:3]


def _select_esc_candidates(target_cont_current_a: float) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for record in load_esc_mass_records():
        current_margin_ratio = record.current_a / max(target_cont_current_a, 1e-9)
        current_shortfall = max(target_cont_current_a - record.current_a, 0.0) / max(target_cont_current_a, 1e-9)
        score = 4.0 * current_shortfall + 0.05 * current_margin_ratio
        rows.append(
            {
                "example": record.example,
                "continuous_current_a": record.current_a,
                "mass_g": record.mass_g,
                "current_margin_ratio": current_margin_ratio,
                "selection_score": score,
            }
        )
    rows.sort(key=lambda row: float(row["selection_score"]))
    return rows[:3]


def _render_operating_plot(
    *,
    low_speed_rpm: float,
    low_speed_torque_nm: float,
    cruise_rpm: float,
    cruise_torque_nm: float,
    required_kv: float,
    target_kv_low: float,
    target_kv_high: float,
    motor_candidates: list[dict[str, Any]],
    target_peak_power_w: float,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    fig, axs = plt.subplots(1, 2, figsize=(12.0, 4.8))

    axs[0].scatter([cruise_rpm, low_speed_rpm], [cruise_torque_nm, low_speed_torque_nm], color=["#1d4ed8", "#d97706"], s=70)
    axs[0].annotate("Cruise", (cruise_rpm, cruise_torque_nm), textcoords="offset points", xytext=(8, 8))
    axs[0].annotate("Low speed", (low_speed_rpm, low_speed_torque_nm), textcoords="offset points", xytext=(8, 8))
    axs[0].set_xlabel("Motor speed [rpm]")
    axs[0].set_ylabel("Per-motor shaft torque [N m]")
    axs[0].set_title("Required Operating Points")
    axs[0].grid(True, alpha=0.25)

    names = [row["example"].replace("_", "\n") for row in motor_candidates]
    peak_power = [float(row["peak_power_w"]) for row in motor_candidates]
    kv_vals = [float(row["kv_rpm_per_v"]) if row["kv_rpm_per_v"] not in {"", None} else float("nan") for row in motor_candidates]
    x = list(range(len(motor_candidates)))
    axs[1].bar(x, peak_power, color="#9ca3af", alpha=0.85, label="Peak power class [W]")
    axs[1].axhline(target_peak_power_w, color="#d97706", linestyle="--", linewidth=1.3, label="Target peak power")
    axs[1].set_xticks(x, names)
    axs[1].set_ylabel("Peak power [W]")
    axs[1].set_title("Representative Motor Classes")
    axs[1].grid(True, axis="y", alpha=0.25)
    ax2 = axs[1].twinx()
    ax2.plot(x, kv_vals, marker="o", color="#1d4ed8", linewidth=2.0, label="Kv class")
    ax2.axhline(required_kv, color="#1d4ed8", linestyle=":", linewidth=1.2, label="Required Kv")
    ax2.axhspan(target_kv_low, target_kv_high, color="#93c5fd", alpha=0.22, label="Target Kv band")
    ax2.set_ylabel("Kv [rpm/V]")
    lines1, labels1 = axs[1].get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    axs[1].legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _write_summary_markdown(
    output: MotorTargetingOutput,
    summary_row: dict[str, Any],
    motor_candidates: list[dict[str, Any]],
    esc_candidates: list[dict[str, Any]],
) -> None:
    best_motor = motor_candidates[0]
    best_esc = esc_candidates[0]
    lines = [
        f"# Motor Targeting: Rank {summary_row['rank']}",
        "",
        "## Selected propulsion concept",
        "",
        f"- Prop layout: `{summary_row['n_props']} x {summary_row['prop_diameter_in']:.1f} x {summary_row['prop_pitch_in']:.1f} in`",
        f"- Prop family: `{summary_row['prop_family']}`",
        f"- Low-speed RPM: `{summary_row['low_speed_rpm']:.1f}`",
        f"- Cruise RPM: `{summary_row['cruise_rpm']:.1f}`",
        f"- Low-speed torque per motor: `{summary_row['per_prop_low_speed_torque_nm']:.5f} N m`",
        f"- Cruise torque per motor: `{summary_row['per_prop_cruise_torque_nm']:.5f} N m`",
        "",
        "## Motor target specification",
        "",
        f"- Required Kv from current Stage 1 model: `{summary_row['motor_required_kv_rpm_per_v']:.1f} rpm/V`",
        f"- Recommended Kv band (heuristic): `{summary_row['target_kv_low_rpm_per_v']:.1f}` to `{summary_row['target_kv_high_rpm_per_v']:.1f} rpm/V`",
        f"- Required peak electrical power per motor: `{summary_row['required_peak_electrical_power_w']:.1f} W`",
        f"- Target peak electrical power per motor (1.25x): `{summary_row['target_peak_electrical_power_w']:.1f} W`",
        f"- Required peak shaft power per motor: `{summary_row['per_prop_peak_shaft_power_w']:.1f} W`",
        f"- Required peak current per motor: `{summary_row['motor_required_current_a']:.2f} A`",
        f"- Target continuous current per motor (1.25x): `{summary_row['target_continuous_current_a']:.2f} A`",
        f"- Estimated motor mass from fit at target power: `{1000.0 * summary_row['estimated_motor_mass_kg']:.1f} g`",
        "",
        "## Recommended representative classes",
        "",
        f"- Best motor class match: `{best_motor['example']}`",
        f"  Peak power = `{best_motor['peak_power_w']:.0f} W`, Kv = `{best_motor['kv_rpm_per_v']:.0f} rpm/V`, mass = `{best_motor['mass_g']:.1f} g`",
        f"- Best ESC class match: `{best_esc['example']}`",
        f"  Continuous current = `{best_esc['continuous_current_a']:.0f} A`, mass = `{best_esc['mass_g']:.1f} g`",
        "",
        "## Candidate files",
        "",
        f"- Summary plot: ![]({output.operating_plot.name})",
        "",
        "## Notes",
        "",
        "- These are representative motor and ESC classes selected from the local sizing datasets, not final vendor picks.",
        "- The motor class dataset is based on representative published specifications from T-Motor, SunnySky, Turnigy, Tiger Motor, and EMAX, as documented in `data/motors/motor_mass.csv`.",
        "- The ESC class dataset is based on representative current/mass points documented in `data/motors/esc_mass.csv`.",
        "- The Kv band and the 1.25x power/current factors are engineering sizing heuristics used to turn the Stage 1 operating point into a procurement target sheet.",
    ]
    output.summary_md.write_text("\n".join(lines), encoding="utf-8")


def run_motor_targeting(
    *,
    rank: int = 6,
    blade_count_metadata: int = 3,
    output_root: Path = Path("outputs/motor_targeting"),
    mission: Stage1MissionConfig | None = None,
) -> MotorTargetingOutput:
    mission = mission or Stage1MissionConfig()
    concept = _load_selected_concept(rank=rank, blade_count_metadata=blade_count_metadata)
    stage1_row = _find_stage1_row_for_concept(
        rank=rank,
        n_props=concept.n_props,
        diameter_in=concept.prop_diameter_in,
        pitch_ratio=concept.prop_pitch_ratio,
        prop_family=concept.prop_family,
    )
    mass_model = MassModel()

    required_kv = float(stage1_row["motor_required_kv_rpm_per_v"])
    required_current = float(stage1_row["motor_required_current_a"])
    required_peak_electric = required_current * mission.battery_voltage_v
    required_peak_shaft = float(stage1_row["per_prop_peak_shaft_power_w"])
    target_peak_power = 1.25 * required_peak_electric
    target_cont_current = 1.25 * required_current
    target_kv_low = 0.85 * required_kv
    target_kv_high = 1.15 * required_kv
    estimated_motor_mass_kg = mass_model.motor_mass_kg(target_peak_power)

    motor_candidates = _select_motor_candidates(required_kv, target_peak_power, 1000.0 * estimated_motor_mass_kg)
    esc_candidates = _select_esc_candidates(target_cont_current)

    output_dir = output_root / (
        f"rank{concept.rank:02d}_n{concept.n_props}_d{concept.prop_diameter_in:.1f}"
        f"_p{concept.prop_pitch_in:.1f}_{concept.prop_family}_b{concept.blade_count_metadata}"
    ).replace(".", "p")
    output_dir.mkdir(parents=True, exist_ok=True)
    output = MotorTargetingOutput(
        output_dir=output_dir,
        summary_csv=output_dir / "motor_target_summary.csv",
        summary_md=output_dir / "motor_target_summary.md",
        candidate_csv=output_dir / "motor_target_candidates.csv",
        operating_plot=output_dir / "motor_target_operating_points.png",
    )

    summary_row = {
        "rank": concept.rank,
        "n_props": concept.n_props,
        "prop_diameter_in": concept.prop_diameter_in,
        "prop_pitch_in": concept.prop_pitch_in,
        "prop_family": concept.prop_family,
        "low_speed_rpm": concept.low_speed_rpm,
        "cruise_rpm": concept.cruise_rpm,
        "per_prop_low_speed_torque_nm": float(stage1_row["per_prop_low_speed_torque_nm"]),
        "per_prop_cruise_torque_nm": float(stage1_row["per_prop_cruise_torque_nm"]),
        "per_prop_peak_shaft_power_w": required_peak_shaft,
        "motor_required_current_a": required_current,
        "motor_required_kv_rpm_per_v": required_kv,
        "required_peak_electrical_power_w": required_peak_electric,
        "target_peak_electrical_power_w": target_peak_power,
        "target_continuous_current_a": target_cont_current,
        "target_kv_low_rpm_per_v": target_kv_low,
        "target_kv_high_rpm_per_v": target_kv_high,
        "estimated_motor_mass_kg": estimated_motor_mass_kg,
        "selected_motor_example": motor_candidates[0]["example"],
        "selected_motor_peak_power_w": motor_candidates[0]["peak_power_w"],
        "selected_motor_kv_rpm_per_v": motor_candidates[0]["kv_rpm_per_v"],
        "selected_motor_mass_g": motor_candidates[0]["mass_g"],
        "selected_esc_example": esc_candidates[0]["example"],
        "selected_esc_current_a": esc_candidates[0]["continuous_current_a"],
        "selected_esc_mass_g": esc_candidates[0]["mass_g"],
        "operating_plot": str(output.operating_plot),
    }

    candidate_rows: list[dict[str, Any]] = []
    for idx, row in enumerate(motor_candidates, start=1):
        candidate_rows.append(
            {
                "candidate_type": "motor",
                "rank": idx,
                "example": row["example"],
                "kv_rpm_per_v": row["kv_rpm_per_v"],
                "peak_power_w": row["peak_power_w"],
                "continuous_current_a": "",
                "mass_g": row["mass_g"],
                "power_margin_ratio": row["power_margin_ratio"],
                "current_margin_ratio": "",
                "kv_ratio": row["kv_ratio"],
                "selection_score": row["selection_score"],
            }
        )
    for idx, row in enumerate(esc_candidates, start=1):
        candidate_rows.append(
            {
                "candidate_type": "esc",
                "rank": idx,
                "example": row["example"],
                "kv_rpm_per_v": "",
                "peak_power_w": "",
                "continuous_current_a": row["continuous_current_a"],
                "mass_g": row["mass_g"],
                "power_margin_ratio": "",
                "current_margin_ratio": row["current_margin_ratio"],
                "kv_ratio": "",
                "selection_score": row["selection_score"],
            }
        )

    _write_csv(output.summary_csv, [summary_row])
    _write_csv(output.candidate_csv, candidate_rows)
    _render_operating_plot(
        low_speed_rpm=concept.low_speed_rpm,
        low_speed_torque_nm=float(stage1_row["per_prop_low_speed_torque_nm"]),
        cruise_rpm=concept.cruise_rpm,
        cruise_torque_nm=float(stage1_row["per_prop_cruise_torque_nm"]),
        required_kv=required_kv,
        target_kv_low=target_kv_low,
        target_kv_high=target_kv_high,
        motor_candidates=motor_candidates,
        target_peak_power_w=target_peak_power,
        output_path=output.operating_plot,
    )
    _write_summary_markdown(output, summary_row, motor_candidates, esc_candidates)
    return output
