from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from optimizer.core.data_models import Stage1MissionConfig
from optimizer.core.high_lift_model import (
    apply_slotted_flap_high_lift_corrections,
    cambridge_uniform_jet_immersion,
    resolve_prop_drop_m,
)
from optimizer.core.stage3_refinement import (
    candidate_key,
    ensure_stage3_runtime,
    load_csv_rows,
    load_stage1_lookup,
    parse_prop_centers,
)


STAGE1_PARETO_INPUT_CSV = Path("outputs/stage1_pareto_front.csv")
STAGE2_INPUT_CSV = Path("outputs/stage2_prop_span_report.csv")
STAGE3_RESULTS_INPUT_CSV = Path("outputs/stage3_aerosandbox_results.csv")


@dataclass(frozen=True)
class RectangularWingControlConfig:
    mission: Stage1MissionConfig = Stage1MissionConfig()
    blade_count_metadata: int = 3
    flap_gap_to_aileron_m: float = 0.06
    minimum_aileron_span_fraction: float = 0.16
    target_positive_prop_overlap_fraction: float = 0.80
    target_positive_disk_overlap_fraction: float = 0.55
    prop_overlap_penalty_weight: float = 4.0
    disk_overlap_penalty_weight: float = 4.0
    prop_drop_fraction_of_chord: float | None = 0.12
    prop_drop_fraction_of_diameter: float | None = None
    slot_lift_gain_base: float = 1.12
    slot_drag_gain_base: float = 1.08
    blown_section_reference_cl_limit: float = 9.0
    prop_axial_location_fraction_of_chord: float = -0.25
    jet_core_height_factor: float = 1.0
    cambridge_base_streamline_fraction_of_chord: float = 0.08
    cambridge_alpha_streamline_gain: float = 0.55
    cambridge_flap_streamline_gain: float = 0.18
    cambridge_jet_softness_fraction_of_chord: float = 0.02
    use_max_flap_deflection_for_low_speed: bool = True
    max_slotted_flap_deflection_deg: float = 40.0
    aileron_target_roll_rate_degps: float = 30.0
    aileron_nominal_deflection_deg: float = 14.0
    flap_span_fraction_values: tuple[float, ...] = (0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65)
    flap_chord_fraction_values: tuple[float, ...] = (0.22, 0.25, 0.28, 0.31, 0.34)
    flap_deflection_values_deg: tuple[float, ...] = (20.0, 25.0, 30.0, 35.0, 40.0)
    aileron_span_fraction_values: tuple[float, ...] = (0.16, 0.20, 0.24, 0.28, 0.32)
    aileron_chord_fraction_values: tuple[float, ...] = (0.16, 0.19, 0.22, 0.25, 0.28)
    aileron_deflection_sweep_deg: tuple[float, ...] = (-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0)
    alpha_grid_deg: tuple[float, float, int] = (-6.0, 28.0, 121)
    roll_damping_p_hat_test: float = 0.05
    reference_velocity_lines_mps: tuple[float, ...] = (3.0, 4.0, 5.0, 6.0, 8.0, 10.0)


@dataclass(frozen=True)
class SelectedPropConcept:
    rank: int
    n_props: int
    prop_diameter_in: float
    prop_pitch_ratio: float
    prop_family: str
    blade_count_metadata: int
    prop_centers_m: tuple[float, ...]
    blown_span_fraction: float
    low_speed_rpm: float
    cruise_rpm: float
    low_speed_veff_mps: float
    low_speed_required_veff_mps: float
    low_speed_power_w: float
    cruise_power_w: float

    @property
    def prop_diameter_m(self) -> float:
        return self.prop_diameter_in * 0.0254

    @property
    def prop_pitch_in(self) -> float:
        return self.prop_diameter_in * self.prop_pitch_ratio


@dataclass(frozen=True)
class FlapSizingResult:
    flap_span_fraction: float
    flap_chord_fraction: float
    flap_deflection_deg: float
    flap_end_y_m: float
    flap_area_m2: float
    flap_area_ratio: float
    blown_flap_area_m2: float
    blown_clean_area_m2: float
    overlapping_props_count: int
    overlapping_positive_props_count: int
    positive_prop_overlap_fraction: float
    positive_disk_overlap_fraction: float
    actual_veff_mps: float
    equivalent_reference_clmax: float
    equivalent_vstall_mps: float
    low_speed_lift_margin: float
    clean_clmax_unblown: float
    clean_clmax_blown: float
    flapped_clmax_unblown: float
    flapped_clmax_blown: float
    slot_gain: float
    prop_drop_m: float
    prop_drop_fraction_of_chord: float
    prop_drop_fraction_of_diameter: float
    cambridge_clean_immersion_at_clmax: float
    cambridge_flap_immersion_at_clmax: float
    cambridge_flap_margin_m_at_clmax: float
    score: float


@dataclass(frozen=True)
class AileronSizingResult:
    aileron_span_fraction: float
    aileron_chord_fraction: float
    aileron_start_y_m: float
    aileron_area_m2: float
    aileron_area_ratio: float
    cruise_trim_alpha_deg: float
    cl_delta_per_deg: float
    cl_p_hat: float
    roll_rate_at_nominal_degps: float
    roll_rate_at_max_degps: float
    score: float


@dataclass(frozen=True)
class ControlSurfaceSizingOutput:
    concept: SelectedPropConcept
    flap: FlapSizingResult
    aileron: AileronSizingResult
    output_dir: Path
    summary_csv: Path
    summary_md: Path
    flap_sweep_csv: Path
    aileron_sweep_csv: Path
    layout_plot: Path
    flap_heatmap_plot: Path
    flap_curves_plot: Path
    flap_section_polar_plot: Path
    total_cl_curve_csv: Path
    total_cl_curve_plot: Path
    aileron_curves_plot: Path


def _safe_float(value: Any, fallback: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _load_selected_concept(rank: int, blade_count_metadata: int) -> SelectedPropConcept:
    stage3_rows = load_csv_rows(STAGE3_RESULTS_INPUT_CSV)
    stage2_rows = load_csv_rows(STAGE2_INPUT_CSV)
    stage1_lookup = load_stage1_lookup(STAGE1_PARETO_INPUT_CSV)

    ranked_row = None
    for row in stage3_rows:
        if row.get("status") == "SUCCESS" and int(float(row["rank"])) == rank:
            ranked_row = row
            break
    if ranked_row is None:
        raise ValueError(f"Could not find a successful Stage 3 result for rank {rank}.")

    selected_key = candidate_key(ranked_row)
    stage2_row = None
    for row in stage2_rows:
        if candidate_key(row) == selected_key:
            stage2_row = row
            break
    if stage2_row is None:
        raise ValueError(f"Could not find Stage 2 context for rank {rank}.")

    stage1_row = stage1_lookup.get(selected_key)
    if stage1_row is None:
        raise ValueError(f"Could not find Stage 1 context for rank {rank}.")

    return SelectedPropConcept(
        rank=rank,
        n_props=int(float(stage2_row["n_props"])),
        prop_diameter_in=float(stage2_row["prop_diameter_in"]),
        prop_pitch_ratio=float(stage2_row["prop_pitch_ratio"]),
        prop_family=stage2_row["prop_family"],
        blade_count_metadata=blade_count_metadata,
        prop_centers_m=tuple(parse_prop_centers(stage2_row["prop_centers_m"])),
        blown_span_fraction=float(stage2_row["blown_span_fraction"]),
        low_speed_rpm=float(stage2_row["solved_low_speed_rpm"]),
        cruise_rpm=float(stage2_row["solved_cruise_rpm"]),
        low_speed_veff_mps=float(stage1_row["low_speed_veff_mps"]),
        low_speed_required_veff_mps=float(stage1_row["low_speed_required_veff_mps"]),
        low_speed_power_w=float(stage1_row["low_speed_power_w"]),
        cruise_power_w=float(stage1_row["cruise_power_w"]),
    )


def _positive_blow_intervals(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
) -> list[tuple[float, float]]:
    semispan = config.mission.span_m / 2.0
    k_span_expansion = getattr(config.mission, "k_span_expansion", getattr(config.mission, "k_span_e8xpansion"))
    radius_blow = 0.5 * k_span_expansion * concept.prop_diameter_m
    positive_centers = [y for y in concept.prop_centers_m if y > 0.0]
    intervals: list[tuple[float, float]] = []
    for center in positive_centers:
        start = max(0.0, center - radius_blow)
        end = min(semispan, center + radius_blow)
        if end > start:
            intervals.append((start, end))

    if not intervals:
        return []

    intervals.sort()
    merged = [intervals[0]]
    for start, end in intervals[1:]:
        prev_start, prev_end = merged[-1]
        if start <= prev_end:
            merged[-1] = (prev_start, max(prev_end, end))
        else:
            merged.append((start, end))
    return merged


def _interval_overlap(intervals: list[tuple[float, float]], start: float, end: float) -> float:
    overlap = 0.0
    for interval_start, interval_end in intervals:
        overlap += max(0.0, min(end, interval_end) - max(start, interval_start))
    return overlap


def _prop_flap_overlap_metrics(
    concept: SelectedPropConcept,
    flap_end_y_m: float,
) -> dict[str, float]:
    prop_radius = 0.5 * concept.prop_diameter_m
    positive_centers = [y for y in concept.prop_centers_m if y > 0.0]
    overlapping_positive = 0
    total_positive_disk_fraction = 0.0
    for y_center in positive_centers:
        overlap = max(0.0, min(flap_end_y_m, y_center + prop_radius) - max(0.0, y_center - prop_radius))
        if overlap > 1e-9:
            overlapping_positive += 1
        total_positive_disk_fraction += overlap / max(2.0 * prop_radius, 1e-9)

    positive_count = len(positive_centers)
    positive_prop_overlap_fraction = overlapping_positive / max(positive_count, 1)
    positive_disk_overlap_fraction = total_positive_disk_fraction / max(positive_count, 1)
    overlapping_total = 2 * overlapping_positive
    if any(abs(y) < 1e-9 for y in concept.prop_centers_m):
        overlapping_total += 1

    return {
        "overlapping_props_count": overlapping_total,
        "overlapping_positive_props_count": overlapping_positive,
        "positive_prop_overlap_fraction": positive_prop_overlap_fraction,
        "positive_disk_overlap_fraction": positive_disk_overlap_fraction,
    }


def _slot_lift_gain(config: RectangularWingControlConfig, flap_chord_fraction: float, deflection_deg: float) -> float:
    chord_factor = np.clip((flap_chord_fraction - 0.20) / 0.14, 0.0, 1.0)
    deflection_factor = np.clip(deflection_deg / 35.0, 0.5, 1.15)
    return 1.0 + (config.slot_lift_gain_base - 1.0) * (0.65 + 0.35 * deflection_factor) * (0.75 + 0.25 * chord_factor)


def _slot_drag_gain(config: RectangularWingControlConfig, flap_chord_fraction: float, deflection_deg: float) -> float:
    chord_factor = np.clip((flap_chord_fraction - 0.20) / 0.14, 0.0, 1.0)
    deflection_factor = np.clip(deflection_deg / 35.0, 0.5, 1.20)
    return 1.0 + (config.slot_drag_gain_base - 1.0) * (0.70 + 0.30 * deflection_factor) * (0.80 + 0.20 * chord_factor)


def _prop_drop_m(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
) -> float:
    return resolve_prop_drop_m(
        chord_m=config.mission.chord_m,
        prop_diameter_m=concept.prop_diameter_m,
        drop_fraction_of_chord=config.prop_drop_fraction_of_chord,
        drop_fraction_of_diameter=config.prop_drop_fraction_of_diameter,
    )


def _saturate_blown_section_reference(
    raw_reference_value: float,
    cl_limit: float,
) -> tuple[float, float]:
    if cl_limit <= 0.0:
        return raw_reference_value, 1.0
    limited = cl_limit * math.tanh(raw_reference_value / cl_limit)
    scale = limited / raw_reference_value if abs(raw_reference_value) > 1e-9 else 1.0
    return limited, scale


def _flapped_section_polars(
    config: RectangularWingControlConfig,
    reynolds: float,
    flap_chord_fraction: float,
    flap_deflection_deg: float,
    cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]],
) -> dict[str, np.ndarray]:
    key = (round(reynolds, 2), flap_chord_fraction, flap_deflection_deg, 1.0)
    if key in cache:
        return cache[key]

    runtime = ensure_stage3_runtime()
    asb = runtime.asb
    alpha_min, alpha_max, alpha_count = config.alpha_grid_deg
    alphas = np.linspace(alpha_min, alpha_max, int(alpha_count))
    airfoil = asb.Airfoil("s1210")
    clean = airfoil.get_aero_from_neuralfoil(alpha=alphas, Re=reynolds)
    raw_flap = airfoil.get_aero_from_neuralfoil(
        alpha=alphas,
        Re=reynolds,
        control_surfaces=[
            asb.ControlSurface(
                name="flap",
                symmetric=True,
                deflection=flap_deflection_deg,
                hinge_point=1.0 - flap_chord_fraction,
            )
        ],
    )

    clean_cl = np.asarray(clean["CL"], dtype=float)
    clean_cd = np.asarray(clean["CD"], dtype=float)
    clean_cm = np.asarray(clean["CM"], dtype=float)
    raw_cl = np.asarray(raw_flap["CL"], dtype=float)
    raw_cd = np.asarray(raw_flap["CD"], dtype=float)
    raw_cm = np.asarray(raw_flap["CM"], dtype=float)

    slot_gain = _slot_lift_gain(config, flap_chord_fraction, flap_deflection_deg)
    slot_drag = _slot_drag_gain(config, flap_chord_fraction, flap_deflection_deg)
    corrected = apply_slotted_flap_high_lift_corrections(
        alpha_deg=alphas,
        clean_cl=clean_cl,
        clean_cd=clean_cd,
        clean_cm=clean_cm,
        raw_flap_cl=raw_cl,
        raw_flap_cd=raw_cd,
        raw_flap_cm=raw_cm,
        flap_chord_fraction=flap_chord_fraction,
        flap_deflection_deg=flap_deflection_deg,
        slot_gain=slot_gain,
        slot_drag=slot_drag,
        prop_drop_bonus=1.0,
        reynolds=reynolds,
        reference_reynolds=max(1.2e5, 0.55 * reynolds),
    )
    adjusted_cl = corrected["adjusted_cl"]
    adjusted_cd = corrected["adjusted_cd"]
    adjusted_cm = corrected["adjusted_cm"]

    result = {
        "alpha_deg": alphas,
        "clean_cl": clean_cl,
        "clean_cd": clean_cd,
        "clean_cm": clean_cm,
        "flapped_cl": adjusted_cl,
        "flapped_cd": adjusted_cd,
        "flapped_cm": adjusted_cm,
        "clean_clmax": np.array([float(clean_cl.max())]),
        "flapped_clmax": np.array([float(adjusted_cl.max())]),
        "slot_gain": np.array([slot_gain]),
    }
    cache[key] = result
    return result


def _flap_area_breakdown(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap_span_fraction: float,
) -> dict[str, float]:
    mission = config.mission
    semispan = mission.span_m / 2.0
    chord = mission.chord_m
    wing_area = mission.wing_area_m2
    flap_end_y = flap_span_fraction * semispan

    intervals = _positive_blow_intervals(config, concept)
    flap_blow_span_half = _interval_overlap(intervals, 0.0, flap_end_y)
    clean_blow_span_half = _interval_overlap(intervals, flap_end_y, semispan)

    flap_area = 2.0 * chord * flap_end_y
    blown_flap_area = 2.0 * chord * flap_blow_span_half
    blown_clean_area = 2.0 * chord * clean_blow_span_half
    flap_unblown_area = max(0.0, flap_area - blown_flap_area)
    clean_total_area = wing_area - flap_area
    clean_unblown_area = max(0.0, clean_total_area - blown_clean_area)
    prop_overlap = _prop_flap_overlap_metrics(concept, flap_end_y)

    return {
        "flap_end_y_m": flap_end_y,
        "flap_area_m2": flap_area,
        "blown_flap_area_m2": blown_flap_area,
        "blown_clean_area_m2": blown_clean_area,
        "flap_unblown_area_m2": flap_unblown_area,
        "clean_total_area_m2": clean_total_area,
        "clean_unblown_area_m2": clean_unblown_area,
        **prop_overlap,
    }


def _cambridge_immersion_data(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap_chord_fraction: float,
    flap_deflection_deg: float,
    alpha_deg: np.ndarray,
) -> dict[str, np.ndarray]:
    return cambridge_uniform_jet_immersion(
        alpha_deg=alpha_deg,
        flap_deflection_deg=flap_deflection_deg,
        flap_chord_fraction=flap_chord_fraction,
        prop_diameter_m=concept.prop_diameter_m,
        prop_drop_m=_prop_drop_m(config, concept),
        chord_m=config.mission.chord_m,
        prop_axial_location_fraction_of_chord=config.prop_axial_location_fraction_of_chord,
        jet_core_height_factor=config.jet_core_height_factor,
        base_streamline_fraction_of_chord=config.cambridge_base_streamline_fraction_of_chord,
        alpha_streamline_gain=config.cambridge_alpha_streamline_gain,
        flap_streamline_gain=config.cambridge_flap_streamline_gain,
        jet_softness_fraction_of_chord=config.cambridge_jet_softness_fraction_of_chord,
    )


def _combined_high_lift_curve_data(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    areas: dict[str, float],
    flap_chord_fraction: float,
    flap_deflection_deg: float,
    polars_unblown: dict[str, np.ndarray],
    polars_blown: dict[str, np.ndarray],
) -> list[dict[str, float]]:
    mission = config.mission
    wing_area = mission.wing_area_m2
    q_inf = 0.5 * mission.air_density_kgpm3 * mission.low_speed_mps**2
    q_b = 0.5 * mission.air_density_kgpm3 * concept.low_speed_veff_mps**2
    q_ratio = q_b / max(q_inf, 1e-9)

    clean_total_area = areas["clean_total_area_m2"]
    flap_area = areas["flap_area_m2"]
    blown_clean_area = areas["blown_clean_area_m2"]
    blown_flap_area = areas["blown_flap_area_m2"]
    clean_unblown_area = areas["clean_unblown_area_m2"]
    flap_unblown_area = areas["flap_unblown_area_m2"]
    clean_total_fraction = clean_total_area / max(wing_area, 1e-9)
    flap_fraction = flap_area / max(wing_area, 1e-9)
    blown_clean_fraction = blown_clean_area / max(wing_area, 1e-9)
    blown_flap_fraction = blown_flap_area / max(wing_area, 1e-9)
    clean_unblown_fraction = clean_unblown_area / max(wing_area, 1e-9)
    flap_unblown_fraction = flap_unblown_area / max(wing_area, 1e-9)

    rows: list[dict[str, float]] = []
    alphas = np.asarray(polars_unblown["alpha_deg"], dtype=float)
    immersion = _cambridge_immersion_data(
        config,
        concept,
        flap_chord_fraction,
        flap_deflection_deg,
        alphas,
    )
    for idx, alpha_deg in enumerate(alphas):
        clean_unblown_cl = float(polars_unblown["clean_cl"][idx])
        clean_blown_cl = float(polars_blown["clean_cl"][idx])
        flap_unblown_cl = float(polars_unblown["flapped_cl"][idx])
        flap_blown_cl = float(polars_blown["flapped_cl"][idx])
        clean_unblown_cd = float(polars_unblown["clean_cd"][idx])
        clean_blown_cd = float(polars_blown["clean_cd"][idx])
        flap_unblown_cd = float(polars_unblown["flapped_cd"][idx])
        flap_blown_cd = float(polars_blown["flapped_cd"][idx])
        clean_unblown_cm = float(polars_unblown["clean_cm"][idx])
        clean_blown_cm = float(polars_blown["clean_cm"][idx])
        flap_unblown_cm = float(polars_unblown["flapped_cm"][idx])
        flap_blown_cm = float(polars_blown["flapped_cm"][idx])

        clean_immersion = float(immersion["clean_immersion_factor"][idx])
        flap_immersion = float(immersion["flap_immersion_factor"][idx])

        clean_blown_cl_ref_raw = clean_unblown_cl + clean_immersion * (q_ratio * clean_blown_cl - clean_unblown_cl)
        flap_blown_cl_ref_raw = flap_unblown_cl + flap_immersion * (q_ratio * flap_blown_cl - flap_unblown_cl)
        clean_blown_cl_ref, clean_blown_scale = _saturate_blown_section_reference(
            clean_blown_cl_ref_raw,
            config.blown_section_reference_cl_limit,
        )
        flap_blown_cl_ref, flap_blown_scale = _saturate_blown_section_reference(
            flap_blown_cl_ref_raw,
            config.blown_section_reference_cl_limit,
        )
        clean_blown_cd_ref = clean_unblown_cd + clean_immersion * (q_ratio * clean_blown_cd - clean_unblown_cd)
        flap_blown_cd_ref = flap_unblown_cd + flap_immersion * (q_ratio * flap_blown_cd - flap_unblown_cd)
        clean_blown_cm_ref = clean_unblown_cm + clean_immersion * (q_ratio * clean_blown_cm - clean_unblown_cm)
        flap_blown_cm_ref = flap_unblown_cm + flap_immersion * (q_ratio * flap_blown_cm - flap_unblown_cm)
        clean_blown_cd_ref *= clean_blown_scale
        flap_blown_cd_ref *= flap_blown_scale
        clean_blown_cm_ref *= clean_blown_scale
        flap_blown_cm_ref *= flap_blown_scale

        cl_clean_baseline = clean_unblown_cl
        cl_flap_only = (
            clean_total_fraction * clean_unblown_cl
            + flap_fraction * flap_unblown_cl
        )
        cl_blow_only = (
            clean_unblown_fraction * clean_unblown_cl
            + (blown_clean_fraction + blown_flap_fraction) * clean_blown_cl_ref
            + flap_unblown_fraction * clean_unblown_cl
        )
        cl_all_high_lift = (
            clean_unblown_fraction * clean_unblown_cl
            + blown_clean_fraction * clean_blown_cl_ref
            + flap_unblown_fraction * flap_unblown_cl
            + blown_flap_fraction * flap_blown_cl_ref
        )

        cd_clean_baseline = clean_unblown_cd
        cd_flap_only = (
            clean_total_fraction * clean_unblown_cd
            + flap_fraction * flap_unblown_cd
        )
        cd_blow_only = (
            clean_unblown_fraction * clean_unblown_cd
            + (blown_clean_fraction + blown_flap_fraction) * clean_blown_cd_ref
            + flap_unblown_fraction * clean_unblown_cd
        )
        cd_all_high_lift = (
            clean_unblown_fraction * clean_unblown_cd
            + blown_clean_fraction * clean_blown_cd_ref
            + flap_unblown_fraction * flap_unblown_cd
            + blown_flap_fraction * flap_blown_cd_ref
        )

        cm_clean_baseline = clean_unblown_cm
        cm_flap_only = (
            clean_total_fraction * clean_unblown_cm
            + flap_fraction * flap_unblown_cm
        )
        cm_blow_only = (
            clean_unblown_fraction * clean_unblown_cm
            + (blown_clean_fraction + blown_flap_fraction) * clean_blown_cm_ref
            + flap_unblown_fraction * clean_unblown_cm
        )
        cm_all_high_lift = (
            clean_unblown_fraction * clean_unblown_cm
            + blown_clean_fraction * clean_blown_cm_ref
            + flap_unblown_fraction * flap_unblown_cm
            + blown_flap_fraction * flap_blown_cm_ref
        )

        rows.append(
            {
                "alpha_deg": float(alpha_deg),
                "cl_clean_baseline": cl_clean_baseline,
                "cl_flap_only": cl_flap_only,
                "cl_blow_only": cl_blow_only,
                "cl_all_high_lift": cl_all_high_lift,
                "cd_clean_baseline": cd_clean_baseline,
                "cd_flap_only": cd_flap_only,
                "cd_blow_only": cd_blow_only,
                "cd_all_high_lift": cd_all_high_lift,
                "cm_clean_baseline": cm_clean_baseline,
                "cm_flap_only": cm_flap_only,
                "cm_blow_only": cm_blow_only,
                "cm_all_high_lift": cm_all_high_lift,
                "cambridge_clean_immersion_factor": clean_immersion,
                "cambridge_flap_immersion_factor": flap_immersion,
                "cambridge_clean_margin_m": float(immersion["clean_margin_m"][idx]),
                "cambridge_flap_margin_m": float(immersion["flap_margin_m"][idx]),
            }
        )
    return rows


def evaluate_flap_candidate(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap_span_fraction: float,
    flap_chord_fraction: float,
    flap_deflection_deg: float,
    *,
    polar_cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]],
) -> FlapSizingResult:
    mission = config.mission
    chord = mission.chord_m
    wing_area = mission.wing_area_m2
    areas = _flap_area_breakdown(config, concept, flap_span_fraction)

    q_inf = 0.5 * mission.air_density_kgpm3 * mission.low_speed_mps**2
    re_inf = mission.air_density_kgpm3 * mission.low_speed_mps * chord / mission.dynamic_viscosity_pas
    re_blown = mission.air_density_kgpm3 * concept.low_speed_veff_mps * chord / mission.dynamic_viscosity_pas

    polars_inf = _flapped_section_polars(
        config,
        re_inf,
        flap_chord_fraction,
        flap_deflection_deg,
        cache=polar_cache,
    )
    polars_blown = _flapped_section_polars(
        config,
        re_blown,
        flap_chord_fraction,
        flap_deflection_deg,
        cache=polar_cache,
    )

    clean_clmax_unblown = float(polars_inf["clean_clmax"][0])
    clean_clmax_blown = float(polars_blown["clean_clmax"][0])
    flapped_clmax_unblown = float(polars_inf["flapped_clmax"][0])
    flapped_clmax_blown = float(polars_blown["flapped_clmax"][0])
    slot_gain = float(polars_inf["slot_gain"][0])

    total_curve_rows = _combined_high_lift_curve_data(
        config,
        concept,
        areas,
        flap_chord_fraction,
        flap_deflection_deg,
        polars_inf,
        polars_blown,
    )
    clmax_row = max(
        total_curve_rows,
        key=lambda row: float(row["cl_all_high_lift"]),
    )
    equivalent_reference_clmax = float(clmax_row["cl_all_high_lift"])
    equivalent_vstall = math.sqrt(
        max(2.0 * mission.gross_weight_n / (mission.air_density_kgpm3 * wing_area * max(equivalent_reference_clmax, 1e-9)), 0.0)
    )
    required_cl = mission.gross_weight_n / max(q_inf * wing_area, 1e-9)
    low_speed_lift_margin = equivalent_reference_clmax / max(required_cl, 1e-9) - 1.0
    flap_area_ratio = areas["flap_area_m2"] / wing_area
    prop_overlap_penalty = config.prop_overlap_penalty_weight * max(
        config.target_positive_prop_overlap_fraction - areas["positive_prop_overlap_fraction"],
        0.0,
    )
    disk_overlap_penalty = config.disk_overlap_penalty_weight * max(
        config.target_positive_disk_overlap_fraction - areas["positive_disk_overlap_fraction"],
        0.0,
    )
    prop_drop_m = _prop_drop_m(config, concept)

    score = (
        equivalent_vstall
        + 0.18 * flap_area_ratio
        + 0.002 * abs(flap_deflection_deg - 32.0)
        + prop_overlap_penalty
        + disk_overlap_penalty
    )
    return FlapSizingResult(
        flap_span_fraction=flap_span_fraction,
        flap_chord_fraction=flap_chord_fraction,
        flap_deflection_deg=flap_deflection_deg,
        flap_end_y_m=areas["flap_end_y_m"],
        flap_area_m2=areas["flap_area_m2"],
        flap_area_ratio=flap_area_ratio,
        blown_flap_area_m2=areas["blown_flap_area_m2"],
        blown_clean_area_m2=areas["blown_clean_area_m2"],
        overlapping_props_count=int(areas["overlapping_props_count"]),
        overlapping_positive_props_count=int(areas["overlapping_positive_props_count"]),
        positive_prop_overlap_fraction=areas["positive_prop_overlap_fraction"],
        positive_disk_overlap_fraction=areas["positive_disk_overlap_fraction"],
        actual_veff_mps=concept.low_speed_veff_mps,
        equivalent_reference_clmax=equivalent_reference_clmax,
        equivalent_vstall_mps=equivalent_vstall,
        low_speed_lift_margin=low_speed_lift_margin,
        clean_clmax_unblown=clean_clmax_unblown,
        clean_clmax_blown=clean_clmax_blown,
        flapped_clmax_unblown=flapped_clmax_unblown,
        flapped_clmax_blown=flapped_clmax_blown,
        slot_gain=slot_gain,
        prop_drop_m=prop_drop_m,
        prop_drop_fraction_of_chord=prop_drop_m / max(mission.chord_m, 1e-9),
        prop_drop_fraction_of_diameter=prop_drop_m / max(concept.prop_diameter_m, 1e-9),
        cambridge_clean_immersion_at_clmax=float(clmax_row["cambridge_clean_immersion_factor"]),
        cambridge_flap_immersion_at_clmax=float(clmax_row["cambridge_flap_immersion_factor"]),
        cambridge_flap_margin_m_at_clmax=float(clmax_row["cambridge_flap_margin_m"]),
        score=score,
    )


def _build_rectangular_control_airplane(
    config: RectangularWingControlConfig,
    flap: FlapSizingResult,
    aileron_span_fraction: float,
    aileron_chord_fraction: float,
    *,
    aileron_deflection_deg: float = 0.0,
) -> Any:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb
    mission = config.mission
    semispan = mission.span_m / 2.0
    aileron_start_y = semispan * (1.0 - aileron_span_fraction)
    chord = mission.chord_m
    airfoil = asb.Airfoil("s1210")

    wing = asb.Wing(
        name="Rectangular Wing",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[0.0, 0.0, 0.0],
                chord=chord,
                airfoil=airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="flap",
                        symmetric=True,
                        hinge_point=1.0 - flap.flap_chord_fraction,
                        deflection=0.0,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[0.0, flap.flap_end_y_m, 0.0],
                chord=chord,
                airfoil=airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="flap",
                        symmetric=True,
                        hinge_point=1.0 - flap.flap_chord_fraction,
                        deflection=0.0,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[0.0, aileron_start_y, 0.0],
                chord=chord,
                airfoil=airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="aileron",
                        symmetric=False,
                        hinge_point=1.0 - aileron_chord_fraction,
                        deflection=aileron_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[0.0, semispan, 0.0],
                chord=chord,
                airfoil=airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="aileron",
                        symmetric=False,
                        hinge_point=1.0 - aileron_chord_fraction,
                        deflection=aileron_deflection_deg,
                    )
                ],
            ),
        ],
    )
    return asb.Airplane(
        name="Rectangular Control Surface Sizing Model",
        wings=[wing],
        xyz_ref=[0.25 * chord, 0.0, 0.0],
        s_ref=wing.area(),
        c_ref=wing.mean_aerodynamic_chord(),
        b_ref=wing.span(),
    )


def _run_vlm(
    airplane: Any,
    velocity_mps: float,
    alpha_deg: float,
    *,
    p_rad_s: float = 0.0,
) -> dict[str, float]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb
    result = asb.VortexLatticeMethod(
        airplane=airplane,
        op_point=asb.OperatingPoint(
            velocity=velocity_mps,
            alpha=float(alpha_deg),
            p=float(p_rad_s),
        ),
        spanwise_resolution=12,
        chordwise_resolution=8,
        run_symmetric_if_possible=False,
        verbose=False,
    ).run()
    normalized: dict[str, float] = {}
    for key, value in result.items():
        try:
            normalized[key] = float(value)
        except (TypeError, ValueError):
            continue
    return normalized


def _solve_trim_alpha(config: RectangularWingControlConfig, airplane: Any) -> tuple[float, dict[str, float]]:
    mission = config.mission
    lower_alpha = -2.0
    upper_alpha = 14.0
    lower_res = _run_vlm(airplane, mission.cruise_speed_mps, lower_alpha)["L"] - mission.gross_weight_n
    upper_out = _run_vlm(airplane, mission.cruise_speed_mps, upper_alpha)
    upper_res = upper_out["L"] - mission.gross_weight_n

    while lower_res * upper_res > 0.0 and upper_alpha < 22.0:
        upper_alpha += 2.0
        upper_out = _run_vlm(airplane, mission.cruise_speed_mps, upper_alpha)
        upper_res = upper_out["L"] - mission.gross_weight_n

    if lower_res * upper_res > 0.0:
        if abs(lower_res) <= abs(upper_res):
            return lower_alpha, _run_vlm(airplane, mission.cruise_speed_mps, lower_alpha)
        return upper_alpha, upper_out

    for _ in range(28):
        mid = 0.5 * (lower_alpha + upper_alpha)
        mid_out = _run_vlm(airplane, mission.cruise_speed_mps, mid)
        mid_res = mid_out["L"] - mission.gross_weight_n
        if abs(mid_res) < 0.2 or abs(upper_alpha - lower_alpha) < 0.05:
            return mid, mid_out
        if lower_res * mid_res <= 0.0:
            upper_alpha = mid
            upper_res = mid_res
        else:
            lower_alpha = mid
            lower_res = mid_res

    return mid, mid_out


def evaluate_aileron_candidate(
    config: RectangularWingControlConfig,
    flap: FlapSizingResult,
    aileron_span_fraction: float,
    aileron_chord_fraction: float,
) -> AileronSizingResult | None:
    mission = config.mission
    semispan = mission.span_m / 2.0
    aileron_start_y = semispan * (1.0 - aileron_span_fraction)
    if aileron_start_y < flap.flap_end_y_m + config.flap_gap_to_aileron_m:
        return None

    neutral_airplane = _build_rectangular_control_airplane(
        config,
        flap,
        aileron_span_fraction,
        aileron_chord_fraction,
        aileron_deflection_deg=0.0,
    )
    trim_alpha_deg, neutral = _solve_trim_alpha(config, neutral_airplane)

    deflections = np.asarray(config.aileron_deflection_sweep_deg, dtype=float)
    cl_values = []
    for deflection in deflections:
        deflected_airplane = _build_rectangular_control_airplane(
            config,
            flap,
            aileron_span_fraction,
            aileron_chord_fraction,
            aileron_deflection_deg=float(deflection),
        )
        cl_values.append(_run_vlm(deflected_airplane, mission.cruise_speed_mps, trim_alpha_deg)["Cl"])
    cl_values = np.asarray(cl_values, dtype=float)
    cl_delta_per_deg = float(np.polyfit(deflections, cl_values, 1)[0])

    p_hat = config.roll_damping_p_hat_test
    p_test = p_hat * 2.0 * mission.cruise_speed_mps / mission.span_m
    cl_p_plus = _run_vlm(neutral_airplane, mission.cruise_speed_mps, trim_alpha_deg, p_rad_s=p_test)["Cl"]
    cl_p_minus = _run_vlm(neutral_airplane, mission.cruise_speed_mps, trim_alpha_deg, p_rad_s=-p_test)["Cl"]
    cl_p_hat = float((cl_p_plus - cl_p_minus) / (2.0 * p_hat))

    if abs(cl_delta_per_deg) < 1e-6:
        wing_area = mission.wing_area_m2
        y_bar = 0.5 * (aileron_start_y + semispan)
        cl_delta_per_deg = (
            (2.4 / 57.29577951308232)
            * ((2.0 * mission.chord_m * (semispan - aileron_start_y) * aileron_chord_fraction) / max(wing_area, 1e-9))
            * (y_bar / max(semispan, 1e-9))
            * (0.30 + 1.10 * aileron_chord_fraction)
        )

    nominal_delta = config.aileron_nominal_deflection_deg
    cl_nominal = cl_delta_per_deg * nominal_delta
    if abs(cl_p_hat) < 1e-9:
        roll_rate_nominal = 0.0
        roll_rate_max = 0.0
    else:
        p_hat_ss_nominal = -cl_nominal / cl_p_hat
        roll_rate_nominal = p_hat_ss_nominal * 2.0 * mission.cruise_speed_mps / mission.span_m * 57.29577951308232
        p_hat_ss_max = -(cl_delta_per_deg * max(abs(deflections))) / cl_p_hat
        roll_rate_max = p_hat_ss_max * 2.0 * mission.cruise_speed_mps / mission.span_m * 57.29577951308232

    aileron_span_m = aileron_span_fraction * semispan
    aileron_area = 2.0 * mission.chord_m * aileron_span_m * aileron_chord_fraction
    aileron_area_ratio = aileron_area / mission.wing_area_m2
    target_gap = max(config.aileron_target_roll_rate_degps - roll_rate_nominal, 0.0)
    score = aileron_area_ratio + 0.0025 * target_gap

    return AileronSizingResult(
        aileron_span_fraction=aileron_span_fraction,
        aileron_chord_fraction=aileron_chord_fraction,
        aileron_start_y_m=aileron_start_y,
        aileron_area_m2=aileron_area,
        aileron_area_ratio=aileron_area_ratio,
        cruise_trim_alpha_deg=trim_alpha_deg,
        cl_delta_per_deg=cl_delta_per_deg,
        cl_p_hat=cl_p_hat,
        roll_rate_at_nominal_degps=roll_rate_nominal,
        roll_rate_at_max_degps=roll_rate_max,
        score=score,
    )


def _slugify_concept(concept: SelectedPropConcept) -> str:
    return (
        f"rank{concept.rank:02d}_n{concept.n_props}_d{concept.prop_diameter_in:.1f}"
        f"_p{concept.prop_pitch_in:.1f}_{concept.prop_family}_b{concept.blade_count_metadata}"
    ).replace(".", "p")


def _pick_flap_and_aileron(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
) -> tuple[FlapSizingResult, AileronSizingResult, list[FlapSizingResult], list[AileronSizingResult]]:
    polar_cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]] = {}
    flap_candidates: list[FlapSizingResult] = []
    if config.use_max_flap_deflection_for_low_speed:
        flap_deflection_values = (config.max_slotted_flap_deflection_deg,)
    else:
        flap_deflection_values = config.flap_deflection_values_deg
    for flap_span_fraction in config.flap_span_fraction_values:
        flap_end_y = 0.5 * config.mission.span_m * flap_span_fraction
        remaining = config.mission.span_m / 2.0 - flap_end_y - config.flap_gap_to_aileron_m
        if remaining <= config.minimum_aileron_span_fraction * 0.5 * config.mission.span_m:
            continue
        for flap_chord_fraction in config.flap_chord_fraction_values:
            for flap_deflection_deg in flap_deflection_values:
                flap_candidates.append(
                    evaluate_flap_candidate(
                        config,
                        concept,
                        flap_span_fraction,
                        flap_chord_fraction,
                        flap_deflection_deg,
                        polar_cache=polar_cache,
                    )
                )

    if not flap_candidates:
        raise ValueError("No feasible flap candidates were generated under the current rectangular-wing constraints.")
    flap_candidates.sort(key=lambda result: result.score)

    best_pair: tuple[FlapSizingResult, AileronSizingResult] | None = None
    best_pair_score = float("inf")
    aileron_records: list[AileronSizingResult] = []
    shortlisted_flaps = flap_candidates[: min(8, len(flap_candidates))]
    for flap in shortlisted_flaps:
        local_best: AileronSizingResult | None = None
        local_best_score = float("inf")
        for aileron_span_fraction in config.aileron_span_fraction_values:
            for aileron_chord_fraction in config.aileron_chord_fraction_values:
                aileron = evaluate_aileron_candidate(
                    config,
                    flap,
                    aileron_span_fraction,
                    aileron_chord_fraction,
                )
                if aileron is None:
                    continue
                aileron_records.append(aileron)
                target_penalty = max(config.aileron_target_roll_rate_degps - aileron.roll_rate_at_nominal_degps, 0.0)
                combined_score = flap.score + aileron.score + 0.02 * target_penalty
                if combined_score < local_best_score:
                    local_best_score = combined_score
                    local_best = aileron
        if local_best is None:
            continue
        if local_best_score < best_pair_score:
            best_pair = (flap, local_best)
            best_pair_score = local_best_score

    if best_pair is None:
        raise ValueError("No feasible aileron candidates were found for the shortlisted flap layouts.")

    return best_pair[0], best_pair[1], flap_candidates, aileron_records


def _flap_curve_data(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
) -> list[dict[str, float]]:
    polar_cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]] = {}
    curves = []
    for deflection in config.flap_deflection_values_deg:
        curves.append(
            asdict(
                evaluate_flap_candidate(
                    config,
                    concept,
                    flap.flap_span_fraction,
                    flap.flap_chord_fraction,
                    deflection,
                    polar_cache=polar_cache,
                )
            )
        )
    return curves


def _selected_flap_section_polars(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
) -> dict[str, np.ndarray]:
    mission = config.mission
    chord = mission.chord_m
    re_inf = mission.air_density_kgpm3 * mission.low_speed_mps * chord / mission.dynamic_viscosity_pas
    re_blown = mission.air_density_kgpm3 * concept.low_speed_veff_mps * chord / mission.dynamic_viscosity_pas
    induced_velocity_mps = max(0.0, 0.5 * (concept.low_speed_veff_mps - mission.low_speed_mps))
    cache: dict[tuple[float, float, float, float], dict[str, np.ndarray]] = {}
    unblown = _flapped_section_polars(
        config,
        re_inf,
        flap.flap_chord_fraction,
        flap.flap_deflection_deg,
        cache=cache,
    )
    blown = _flapped_section_polars(
        config,
        re_blown,
        flap.flap_chord_fraction,
        flap.flap_deflection_deg,
        cache=cache,
    )
    return {
        "alpha_deg": unblown["alpha_deg"],
        "clean_cl_unblown": unblown["clean_cl"],
        "flapped_cl_unblown": unblown["flapped_cl"],
        "clean_cd_unblown": unblown["clean_cd"],
        "flapped_cd_unblown": unblown["flapped_cd"],
        "clean_cm_unblown": unblown["clean_cm"],
        "flapped_cm_unblown": unblown["flapped_cm"],
        "clean_cl_blown": blown["clean_cl"],
        "flapped_cl_blown": blown["flapped_cl"],
        "clean_cd_blown": blown["clean_cd"],
        "flapped_cd_blown": blown["flapped_cd"],
        "clean_cm_blown": blown["clean_cm"],
        "flapped_cm_blown": blown["flapped_cm"],
        "freestream_velocity_mps": np.asarray([mission.low_speed_mps], dtype=float),
        "blown_effective_velocity_mps": np.asarray([concept.low_speed_veff_mps], dtype=float),
        "required_effective_velocity_mps": np.asarray([concept.low_speed_required_veff_mps], dtype=float),
        "induced_velocity_mps": np.asarray([induced_velocity_mps], dtype=float),
        "re_inf": np.asarray([re_inf], dtype=float),
        "re_blown": np.asarray([re_blown], dtype=float),
    }


def _total_high_lift_curve_data(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    polars: dict[str, np.ndarray],
) -> list[dict[str, float]]:
    unblown_polars = {
        "alpha_deg": polars["alpha_deg"],
        "clean_cl": polars["clean_cl_unblown"],
        "flapped_cl": polars["flapped_cl_unblown"],
        "clean_cd": polars["clean_cd_unblown"],
        "flapped_cd": polars["flapped_cd_unblown"],
        "clean_cm": polars["clean_cm_unblown"],
        "flapped_cm": polars["flapped_cm_unblown"],
    }
    blown_polars = {
        "alpha_deg": polars["alpha_deg"],
        "clean_cl": polars["clean_cl_blown"],
        "flapped_cl": polars["flapped_cl_blown"],
        "clean_cd": polars["clean_cd_blown"],
        "flapped_cd": polars["flapped_cd_blown"],
        "clean_cm": polars["clean_cm_blown"],
        "flapped_cm": polars["flapped_cm_blown"],
    }
    areas = {
        "flap_area_m2": flap.flap_area_m2,
        "blown_flap_area_m2": flap.blown_flap_area_m2,
        "blown_clean_area_m2": flap.blown_clean_area_m2,
        "flap_unblown_area_m2": max(0.0, flap.flap_area_m2 - flap.blown_flap_area_m2),
        "clean_total_area_m2": max(0.0, config.mission.wing_area_m2 - flap.flap_area_m2),
        "clean_unblown_area_m2": max(
            0.0,
            config.mission.wing_area_m2 - flap.flap_area_m2 - flap.blown_clean_area_m2,
        ),
        "flap_end_y_m": flap.flap_end_y_m,
    }
    return _combined_high_lift_curve_data(
        config,
        concept,
        areas,
        flap.flap_chord_fraction,
        flap.flap_deflection_deg,
        unblown_polars,
        blown_polars,
    )


def _aileron_curve_data(
    config: RectangularWingControlConfig,
    flap: FlapSizingResult,
    aileron: AileronSizingResult,
) -> list[dict[str, float]]:
    mission = config.mission
    neutral_airplane = _build_rectangular_control_airplane(
        config,
        flap,
        aileron.aileron_span_fraction,
        aileron.aileron_chord_fraction,
        aileron_deflection_deg=0.0,
    )
    trim_alpha_deg, _ = _solve_trim_alpha(config, neutral_airplane)

    curves = []
    for deflection in config.aileron_deflection_sweep_deg:
        airplane = _build_rectangular_control_airplane(
            config,
            flap,
            aileron.aileron_span_fraction,
            aileron.aileron_chord_fraction,
            aileron_deflection_deg=float(deflection),
        )
        result = _run_vlm(airplane, mission.cruise_speed_mps, trim_alpha_deg)
        if abs(aileron.cl_p_hat) < 1e-9:
            roll_rate_degps = 0.0
        else:
            p_hat_ss = -(result["Cl"] / aileron.cl_p_hat)
            roll_rate_degps = p_hat_ss * 2.0 * mission.cruise_speed_mps / mission.span_m * 57.29577951308232
        curves.append(
            {
                "deflection_deg": float(deflection),
                "cl": float(result["Cl"]),
                "cd": float(result["CD"]),
                "lift_n": float(result["L"]),
                "drag_n": float(result["D"]),
                "roll_rate_degps": float(roll_rate_degps),
            }
        )
    return curves


def _selected_flap_aileron_sweep(
    config: RectangularWingControlConfig,
    flap: FlapSizingResult,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    for aileron_span_fraction in config.aileron_span_fraction_values:
        for aileron_chord_fraction in config.aileron_chord_fraction_values:
            result = evaluate_aileron_candidate(
                config,
                flap,
                aileron_span_fraction,
                aileron_chord_fraction,
            )
            if result is None:
                continue
            row = asdict(result)
            row["selected_flap_span_fraction"] = flap.flap_span_fraction
            row["selected_flap_chord_fraction"] = flap.flap_chord_fraction
            row["selected_flap_deflection_deg"] = flap.flap_deflection_deg
            rows.append(row)
    return rows


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        if not rows:
            return
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _render_layout_plot(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    aileron: AileronSizingResult,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    patches = runtime.patches
    mission = config.mission
    semispan = mission.span_m / 2.0
    chord = mission.chord_m
    prop_radius = concept.prop_diameter_m / 2.0
    prop_drop = _prop_drop_m(config, concept)

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw={"width_ratios": [1.55, 1.0]})
    ax = axs[0]
    wing = patches.Rectangle((0.0, -semispan), chord, mission.span_m, facecolor="#dbe4ee", edgecolor="#1f2937", linewidth=1.2)
    flap_rect_r = patches.Rectangle(
        (chord * (1.0 - flap.flap_chord_fraction), 0.0),
        chord * flap.flap_chord_fraction,
        flap.flap_end_y_m,
        facecolor="#f4a261",
        edgecolor="#1f2937",
        alpha=0.85,
        label="Slotted flap",
    )
    flap_rect_l = patches.Rectangle(
        (chord * (1.0 - flap.flap_chord_fraction), -flap.flap_end_y_m),
        chord * flap.flap_chord_fraction,
        flap.flap_end_y_m,
        facecolor="#f4a261",
        edgecolor="#1f2937",
        alpha=0.85,
    )
    aileron_span = semispan - aileron.aileron_start_y_m
    aileron_rect_r = patches.Rectangle(
        (chord * (1.0 - aileron.aileron_chord_fraction), aileron.aileron_start_y_m),
        chord * aileron.aileron_chord_fraction,
        aileron_span,
        facecolor="#457b9d",
        edgecolor="#1f2937",
        alpha=0.85,
        label="Aileron",
    )
    aileron_rect_l = patches.Rectangle(
        (chord * (1.0 - aileron.aileron_chord_fraction), -semispan),
        chord * aileron.aileron_chord_fraction,
        aileron_span,
        facecolor="#457b9d",
        edgecolor="#1f2937",
        alpha=0.85,
    )
    for patch in [wing, flap_rect_r, flap_rect_l, aileron_rect_r, aileron_rect_l]:
        ax.add_patch(patch)

    overlapping_props = 0
    for y_center in concept.prop_centers_m:
        is_flap_prop = max(0.0, abs(y_center) - prop_radius) <= flap.flap_end_y_m + 1e-9
        if is_flap_prop:
            overlapping_props += 1
        circle = patches.Circle(
            (0.15, y_center),
            prop_radius,
            facecolor="none",
            edgecolor="#7c3aed" if is_flap_prop else "#6b7280",
            linestyle="--",
            linewidth=1.2,
            alpha=0.9,
        )
        ax.add_patch(circle)

    ax.text(
        0.02,
        0.98,
        (
            f"Rectangular wing: {mission.span_m:.2f} m x {mission.chord_m:.2f} m\n"
            f"Flap span = {flap.flap_span_fraction:.2f} semispan\n"
            f"Aileron span = {aileron.aileron_span_fraction:.2f} semispan\n"
            f"Disk-overlap flap props = {overlapping_props}/{concept.n_props}\n"
            f"Positive-half prop overlap = {100.0 * flap.positive_prop_overlap_fraction:.0f}%"
        ),
        transform=ax.transAxes,
        ha="left",
        va="top",
        bbox=dict(facecolor="white", edgecolor="#cbd5e1", alpha=0.92),
    )
    ax.set_aspect("equal")
    ax.set_xlim(-0.04, chord + 0.12)
    ax.set_ylim(-1.05, 1.05)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Rectangular Planform, Flaps, Ailerons, and Prop Disks")
    ax.legend(loc="lower right")
    ax.grid(True, alpha=0.2)

    side = axs[1]
    side.plot([0.0, chord], [0.0, 0.0], color="#1f2937", linewidth=2.0, label="Wing chord line")
    flap_hinge_x = chord * (1.0 - flap.flap_chord_fraction)
    flap_te_x = chord
    flap_deflection_rad = math.radians(flap.flap_deflection_deg)
    flap_len = chord * flap.flap_chord_fraction
    flap_tip_x = flap_hinge_x + flap_len * math.cos(flap_deflection_rad)
    flap_tip_z = -flap_len * math.sin(flap_deflection_rad)
    side.plot([flap_hinge_x, flap_tip_x], [0.0, flap_tip_z], color="#f4a261", linewidth=3.0, label="Slotted flap")
    side.add_patch(
        patches.Circle(
            (0.15, -prop_drop),
            prop_radius,
            facecolor="none",
            edgecolor="#7c3aed",
            linestyle="--",
            linewidth=1.2,
            alpha=0.9,
        )
    )
    side.annotate(
        f"Prop center dropped by {prop_drop:.3f} m",
        xy=(0.15, -prop_drop),
        xytext=(0.04, -0.12),
        arrowprops=dict(arrowstyle="->", color="#4b5563"),
        fontsize=9,
    )
    side.set_xlim(-0.05, chord + 0.12)
    side.set_ylim(min(-0.18, flap_tip_z - 0.05), 0.10)
    side.set_xlabel("x [m]")
    side.set_ylabel("z [m]")
    side.set_title("Flap/Prop Side View Assumption")
    side.grid(True, alpha=0.2)
    side.legend(loc="lower left")

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_flap_heatmap(
    config: RectangularWingControlConfig,
    flap_candidates: list[FlapSizingResult],
    flap: FlapSizingResult,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    span_vals = sorted({result.flap_span_fraction for result in flap_candidates})
    chord_vals = sorted({result.flap_chord_fraction for result in flap_candidates})
    heatmap = np.full((len(chord_vals), len(span_vals)), np.nan)
    for i, chord_fraction in enumerate(chord_vals):
        for j, span_fraction in enumerate(span_vals):
            matches = [
                result
                for result in flap_candidates
                if abs(result.flap_span_fraction - span_fraction) < 1e-9
                and abs(result.flap_chord_fraction - chord_fraction) < 1e-9
                and abs(result.flap_deflection_deg - flap.flap_deflection_deg) < 1e-9
            ]
            if matches:
                heatmap[i, j] = matches[0].equivalent_vstall_mps

    fig, ax = plt.subplots(figsize=(7.2, 5.6))
    image = ax.imshow(
        heatmap,
        origin="lower",
        aspect="auto",
        extent=(min(span_vals), max(span_vals), min(chord_vals), max(chord_vals)),
        cmap="viridis_r",
    )
    ax.scatter(
        [flap.flap_span_fraction],
        [flap.flap_chord_fraction],
        color="white",
        edgecolor="#111827",
        s=90,
        marker="*",
        label="Selected flap",
    )
    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label("Equivalent stall speed [m/s]")
    ax.set_xlabel("Flap span fraction of semispan")
    ax.set_ylabel("Flap chord fraction")
    ax.set_title(f"Rectangular-Wing Flap Sweep at {flap.flap_deflection_deg:.0f} deg")
    ax.legend(loc="upper right")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_flap_curves(
    flap_curve_rows: list[dict[str, float]],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    deflections = [row["flap_deflection_deg"] for row in flap_curve_rows]
    clmax = [row["equivalent_reference_clmax"] for row in flap_curve_rows]
    vstall = [row["equivalent_vstall_mps"] for row in flap_curve_rows]
    margin = [100.0 * row["low_speed_lift_margin"] for row in flap_curve_rows]

    fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.6))
    axs[0].plot(deflections, clmax, marker="o", color="#1d3557", label="Equivalent reference CLmax")
    axs[0].plot(deflections, margin, marker="s", color="#2a9d8f", label="Lift margin at 4 m/s [%]")
    axs[0].set_xlabel("Flap deflection [deg]")
    axs[0].set_title("Low-Speed Lift Metrics vs Flap Deflection")
    axs[0].grid(True, alpha=0.25)
    axs[0].legend()

    axs[1].plot(deflections, vstall, marker="o", color="#e76f51")
    axs[1].set_xlabel("Flap deflection [deg]")
    axs[1].set_ylabel("Equivalent stall speed [m/s]")
    axs[1].set_title("Equivalent Stall Speed vs Flap Deflection")
    axs[1].grid(True, alpha=0.25)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_flap_section_polars(polars: dict[str, np.ndarray], output_path: Path) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    fig, axs = plt.subplots(1, 2, figsize=(12.0, 4.8))

    v_inf = float(polars["freestream_velocity_mps"][0])
    v_eff = float(polars["blown_effective_velocity_mps"][0])
    v_eff_req = float(polars["required_effective_velocity_mps"][0])
    v_induced = float(polars["induced_velocity_mps"][0])
    re_inf = float(polars["re_inf"][0])
    re_blown = float(polars["re_blown"][0])

    axs[0].plot(
        polars["alpha_deg"],
        polars["clean_cl_unblown"],
        label=rf"Clean @ $V_\infty$ = {v_inf:.2f} m/s",
        color="#1d3557",
    )
    axs[0].plot(
        polars["alpha_deg"],
        polars["flapped_cl_unblown"],
        label=rf"Slotted flap @ $V_\infty$ = {v_inf:.2f} m/s",
        color="#e76f51",
    )
    axs[0].plot(
        polars["alpha_deg"],
        polars["clean_cl_blown"],
        label=rf"Clean @ $V_{{eff}}$ = {v_eff:.2f} m/s",
        color="#2a9d8f",
    )
    axs[0].plot(
        polars["alpha_deg"],
        polars["flapped_cl_blown"],
        label=rf"Slotted flap @ $V_{{eff}}$ = {v_eff:.2f} m/s",
        color="#7c3aed",
    )
    axs[0].set_xlabel("Alpha [deg]")
    axs[0].set_ylabel("Section CL")
    axs[0].set_title("Section Lift Curves Used in Flap Sizing")
    axs[0].grid(True, alpha=0.25)
    axs[0].legend()

    axs[1].plot(
        polars["alpha_deg"],
        polars["clean_cd_unblown"],
        label=rf"Clean @ $V_\infty$ = {v_inf:.2f} m/s",
        color="#1d3557",
    )
    axs[1].plot(
        polars["alpha_deg"],
        polars["flapped_cd_unblown"],
        label=rf"Slotted flap @ $V_\infty$ = {v_inf:.2f} m/s",
        color="#e76f51",
    )
    axs[1].plot(
        polars["alpha_deg"],
        polars["clean_cd_blown"],
        label=rf"Clean @ $V_{{eff}}$ = {v_eff:.2f} m/s",
        color="#2a9d8f",
    )
    axs[1].plot(
        polars["alpha_deg"],
        polars["flapped_cd_blown"],
        label=rf"Slotted flap @ $V_{{eff}}$ = {v_eff:.2f} m/s",
        color="#7c3aed",
    )
    axs[1].set_xlabel("Alpha [deg]")
    axs[1].set_ylabel("Section CD")
    axs[1].set_title("Section Drag Curves Used in Flap Sizing")
    axs[1].grid(True, alpha=0.25)
    axs[1].legend()

    fig.text(
        0.5,
        0.985,
        (
            rf"$V_\infty$ = {v_inf:.2f} m/s, "
            rf"$V_{{eff}}$ = {v_eff:.2f} m/s, "
            rf"$v_i$ = {v_induced:.2f} m/s, "
            rf"$V_{{eff,req}}$ = {v_eff_req:.2f} m/s"
            "\n"
            rf"$Re_\infty$ = {re_inf:.2e}, "
            rf"$Re_{{eff}}$ = {re_blown:.2e}"
        ),
        ha="center",
        va="top",
        fontsize=10,
        bbox=dict(facecolor="white", edgecolor="#cbd5e1", alpha=0.92),
    )

    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.89))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_total_cl_curve(
    total_curve_rows: list[dict[str, float]],
    config: RectangularWingControlConfig,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    mission = config.mission

    alpha = [row["alpha_deg"] for row in total_curve_rows]
    cl_clean = [row["cl_clean_baseline"] for row in total_curve_rows]
    cl_flap = [row["cl_flap_only"] for row in total_curve_rows]
    cl_blow = [row["cl_blow_only"] for row in total_curve_rows]
    cl_all = [row["cl_all_high_lift"] for row in total_curve_rows]
    cd_all = [row["cd_all_high_lift"] for row in total_curve_rows]

    fig, axs = plt.subplots(1, 2, figsize=(12.2, 4.8))
    axs[0].plot(alpha, cl_clean, color="#1d3557", label="Clean wing")
    axs[0].plot(alpha, cl_flap, color="#e76f51", label="Flap only")
    axs[0].plot(alpha, cl_blow, color="#2a9d8f", label="Blowing only")
    axs[0].plot(alpha, cl_all, color="#7c3aed", linewidth=2.3, label="All high-lift active")
    speed_colors = plt.cm.Greys(np.linspace(0.85, 0.35, len(config.reference_velocity_lines_mps)))
    for color, speed_mps in zip(speed_colors, config.reference_velocity_lines_mps):
        cl_required = mission.gross_weight_n / max(
            0.5 * mission.air_density_kgpm3 * speed_mps**2 * mission.wing_area_m2,
            1e-9,
        )
        emphasis = abs(speed_mps - mission.low_speed_mps) < 1e-9
        axs[0].axhline(
            cl_required,
            color=color,
            linestyle="--",
            linewidth=1.5 if emphasis else 1.0,
            alpha=0.95 if emphasis else 0.75,
            label=f"Required at {speed_mps:.0f} m/s",
        )
    axs[0].set_xlabel("Alpha [deg]")
    axs[0].set_ylabel("Freestream-referenced wing C_L")
    axs[0].set_title("Whole-Wing High-Lift C_L Curves")
    axs[0].grid(True, alpha=0.25)
    axs[0].legend(ncol=2, fontsize=9)

    axs[1].plot(alpha, cd_all, color="#7c3aed", linewidth=2.3)
    axs[1].set_xlabel("Alpha [deg]")
    axs[1].set_ylabel("Freestream-referenced wing C_D")
    axs[1].set_title("All-High-Lift C_D Curve")
    axs[1].grid(True, alpha=0.25)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _render_aileron_curves(
    config: RectangularWingControlConfig,
    aileron_curve_rows: list[dict[str, float]],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    deflections = [row["deflection_deg"] for row in aileron_curve_rows]
    cl_values = [row["cl"] for row in aileron_curve_rows]
    roll_rates = [row["roll_rate_degps"] for row in aileron_curve_rows]
    drag = [row["drag_n"] for row in aileron_curve_rows]

    fig, axs = plt.subplots(1, 2, figsize=(11.5, 4.6))
    axs[0].plot(deflections, cl_values, marker="o", color="#1d3557", label="Rolling moment coefficient Cl")
    axs[0].plot(deflections, drag, marker="s", color="#e76f51", label="Total drag [N]")
    axs[0].axhline(0.0, color="#6b7280", linewidth=1.0)
    axs[0].set_xlabel("Aileron deflection [deg]")
    axs[0].set_title("Cruise Aileron Control Curve")
    axs[0].grid(True, alpha=0.25)
    axs[0].legend()

    axs[1].plot(deflections, roll_rates, marker="o", color="#2a9d8f")
    axs[1].axhline(
        config.aileron_target_roll_rate_degps,
        color="#7c3aed",
        linestyle="--",
        linewidth=1.0,
        label="Target roll rate",
    )
    axs[1].set_xlabel("Aileron deflection [deg]")
    axs[1].set_ylabel("Estimated steady roll rate [deg/s]")
    axs[1].set_title("Estimated Cruise Roll Response")
    axs[1].grid(True, alpha=0.25)
    axs[1].legend()

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _summary_row(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    aileron: AileronSizingResult,
    output: ControlSurfaceSizingOutput,
) -> dict[str, object]:
    induced_velocity_mps = 0.5 * (concept.low_speed_veff_mps - config.mission.low_speed_mps)
    return {
        "rank": concept.rank,
        "n_props": concept.n_props,
        "prop_diameter_in": concept.prop_diameter_in,
        "prop_pitch_in": concept.prop_pitch_in,
        "prop_pitch_ratio": concept.prop_pitch_ratio,
        "prop_family": concept.prop_family,
        "blade_count_metadata": concept.blade_count_metadata,
        "low_speed_rpm": concept.low_speed_rpm,
        "cruise_rpm": concept.cruise_rpm,
        "low_speed_veff_mps": concept.low_speed_veff_mps,
        "low_speed_induced_velocity_mps": induced_velocity_mps,
        "low_speed_required_veff_mps": concept.low_speed_required_veff_mps,
        "recommended_flap_span_fraction": flap.flap_span_fraction,
        "recommended_flap_end_y_m": flap.flap_end_y_m,
        "recommended_flap_chord_fraction": flap.flap_chord_fraction,
        "recommended_flap_deflection_deg": flap.flap_deflection_deg,
        "flap_area_m2": flap.flap_area_m2,
        "blown_flap_area_m2": flap.blown_flap_area_m2,
        "overlapping_props_count": flap.overlapping_props_count,
        "overlapping_positive_props_count": flap.overlapping_positive_props_count,
        "positive_prop_overlap_fraction": flap.positive_prop_overlap_fraction,
        "positive_disk_overlap_fraction": flap.positive_disk_overlap_fraction,
        "prop_drop_m": flap.prop_drop_m,
        "prop_drop_fraction_of_chord": flap.prop_drop_fraction_of_chord,
        "prop_drop_fraction_of_diameter": flap.prop_drop_fraction_of_diameter,
        "cambridge_clean_immersion_at_clmax": flap.cambridge_clean_immersion_at_clmax,
        "cambridge_flap_immersion_at_clmax": flap.cambridge_flap_immersion_at_clmax,
        "cambridge_flap_margin_m_at_clmax": flap.cambridge_flap_margin_m_at_clmax,
        "equivalent_reference_clmax": flap.equivalent_reference_clmax,
        "equivalent_vstall_mps": flap.equivalent_vstall_mps,
        "low_speed_lift_margin": flap.low_speed_lift_margin,
        "recommended_aileron_span_fraction": aileron.aileron_span_fraction,
        "recommended_aileron_start_y_m": aileron.aileron_start_y_m,
        "recommended_aileron_chord_fraction": aileron.aileron_chord_fraction,
        "aileron_area_m2": aileron.aileron_area_m2,
        "cruise_trim_alpha_deg": aileron.cruise_trim_alpha_deg,
        "aileron_cl_delta_per_deg": aileron.cl_delta_per_deg,
        "aileron_cl_p_hat": aileron.cl_p_hat,
        "roll_rate_at_nominal_degps": aileron.roll_rate_at_nominal_degps,
        "roll_rate_at_max_degps": aileron.roll_rate_at_max_degps,
        "layout_plot": str(output.layout_plot),
        "flap_heatmap_plot": str(output.flap_heatmap_plot),
        "flap_curves_plot": str(output.flap_curves_plot),
        "flap_section_polar_plot": str(output.flap_section_polar_plot),
        "total_cl_curve_csv": str(output.total_cl_curve_csv),
        "total_cl_curve_plot": str(output.total_cl_curve_plot),
        "aileron_curves_plot": str(output.aileron_curves_plot),
    }


def _write_summary_markdown(
    config: RectangularWingControlConfig,
    concept: SelectedPropConcept,
    flap: FlapSizingResult,
    aileron: AileronSizingResult,
    output: ControlSurfaceSizingOutput,
) -> None:
    prop_drop_m = flap.prop_drop_m
    induced_velocity_mps = 0.5 * (concept.low_speed_veff_mps - config.mission.low_speed_mps)
    lines = [
        f"# Rectangular-Wing Control Surface Sizing: Rank {concept.rank}",
        "",
        "## Selected propulsion concept",
        "",
        f"- Rank: `{concept.rank}`",
        f"- Prop layout: `{concept.n_props} x {concept.prop_diameter_in:.1f} x {concept.prop_pitch_in:.1f} in`",
        f"- Prop family: `{concept.prop_family}`",
        f"- Blade count metadata: `{concept.blade_count_metadata}`",
        f"- Low-speed RPM: `{concept.low_speed_rpm:.1f}`",
        f"- Cruise RPM: `{concept.cruise_rpm:.1f}`",
        f"- Available low-speed blown velocity: `{concept.low_speed_veff_mps:.2f} m/s`",
        f"- Low-speed induced velocity from actuator-disk relation: `{induced_velocity_mps:.2f} m/s`",
        f"- Required low-speed blown velocity from Stage 1: `{concept.low_speed_required_veff_mps:.2f} m/s`",
        "",
        "## Rectangular-wing assumptions",
        "",
        f"- Span: `{config.mission.span_m:.2f} m`",
        f"- Chord: `{config.mission.chord_m:.2f} m`",
        f"- Wing area: `{config.mission.wing_area_m2:.3f} m^2`",
        f"- Slotted flap modeling: `NeuralFoil flap polar + staged slotted-flap correction`",
        f"- Low-speed flap assumption: `max slotted flap deflection = {config.max_slotted_flap_deflection_deg:.1f} deg`",
        f"- Prop drop ahead of flap: `{prop_drop_m:.3f} m` (`{flap.prop_drop_fraction_of_chord:.3f} c`, `{flap.prop_drop_fraction_of_diameter:.3f} D_p`) for props that lie inside the flap span",
        f"- Cambridge-style jet model: `uniform 2D jet with immersion criterion` at `x_p/c = {config.prop_axial_location_fraction_of_chord:.2f}`",
        "",
        "## Recommended flap",
        "",
        f"- Span fraction of semispan: `{flap.flap_span_fraction:.2f}`",
        f"- End station: `{flap.flap_end_y_m:.3f} m`",
        f"- Chord fraction: `{flap.flap_chord_fraction:.2f}`",
        f"- Deflection: `{flap.flap_deflection_deg:.1f} deg`",
        f"- Flap area: `{flap.flap_area_m2:.3f} m^2`",
        f"- Blown flap area: `{flap.blown_flap_area_m2:.3f} m^2`",
        f"- Props with disk overlap over flap: `{flap.overlapping_props_count}/{concept.n_props}`",
        f"- Positive-half prop overlap fraction: `{100.0 * flap.positive_prop_overlap_fraction:.0f}%`",
        f"- Positive-half disk overlap fraction: `{100.0 * flap.positive_disk_overlap_fraction:.0f}%`",
        f"- Common-alpha freestream-referenced wing CLmax: `{flap.equivalent_reference_clmax:.3f}`",
        f"- Equivalent stall speed: `{flap.equivalent_vstall_mps:.3f} m/s`",
        f"- Lift margin at 4 m/s: `{100.0 * flap.low_speed_lift_margin:.1f} %`",
        f"- Cambridge clean-strip immersion at CLmax: `{flap.cambridge_clean_immersion_at_clmax:.3f}`",
        f"- Cambridge flap-strip immersion at CLmax: `{flap.cambridge_flap_immersion_at_clmax:.3f}`",
        f"- Cambridge flap-strip jet margin at CLmax: `{1000.0 * flap.cambridge_flap_margin_m_at_clmax:.1f} mm`",
        "",
        "## Recommended aileron",
        "",
        f"- Span fraction of semispan: `{aileron.aileron_span_fraction:.2f}`",
        f"- Start station: `{aileron.aileron_start_y_m:.3f} m`",
        f"- Chord fraction: `{aileron.aileron_chord_fraction:.2f}`",
        f"- Aileron area: `{aileron.aileron_area_m2:.3f} m^2`",
        f"- Trim alpha at 10 m/s: `{aileron.cruise_trim_alpha_deg:.2f} deg`",
        f"- Rolling moment derivative: `{aileron.cl_delta_per_deg:.6f} /deg`",
        f"- Roll damping derivative: `{aileron.cl_p_hat:.4f}`",
        f"- Estimated roll rate at {config.aileron_nominal_deflection_deg:.0f} deg: `{aileron.roll_rate_at_nominal_degps:.1f} deg/s`",
        f"- Estimated roll rate at 20 deg: `{aileron.roll_rate_at_max_degps:.1f} deg/s`",
        f"- Internal target roll rate for sizing: `{config.aileron_target_roll_rate_degps:.1f} deg/s`",
        "",
        "## Artifacts",
        "",
        f"- Layout plot: ![]({output.layout_plot.name})",
        "",
        f"- Flap heatmap: ![]({output.flap_heatmap_plot.name})",
        "",
        f"- Flap curves: ![]({output.flap_curves_plot.name})",
        "",
        f"- Section polars: ![]({output.flap_section_polar_plot.name})",
        "",
        f"- Whole-wing CL/CD curve: ![]({output.total_cl_curve_plot.name})",
        "",
        f"- Aileron curves: ![]({output.aileron_curves_plot.name})",
        "",
        "## Notes",
        "",
        "- The blade-count input is tracked as metadata only in this script; the current Stage 1/2 prop surrogate does not explicitly model blade count.",
        "- The slotted-flap correction remains a concept-level surrogate layered on top of NeuralFoil flap polars; however, motor height is no longer represented by a purely empirical Gaussian bonus.",
        "- The blown-strip contribution now follows a Cambridge-style uniform-jet immersion model: the blown benefit remains strong only while the wing stays submerged in the jet, and it collapses rapidly as the jet rides above the wing.",
        f"- The blown-section freestream-referenced lift contribution is smoothly limited to about `{config.blown_section_reference_cl_limit:.1f}` to stay consistent with the order of magnitude reported by 2D blown-flap wind-tunnel data.",
        f"- Flap candidates are now penalized if they fail to cover at least `{100.0 * config.target_positive_prop_overlap_fraction:.0f}%` of positive-half prop disks and `{100.0 * config.target_positive_disk_overlap_fraction:.0f}%` of positive-half disk span.",
        "- The reported wing CLmax is now extracted from a common-alpha whole-wing lift curve, not from independently maximized section values.",
        "- The aileron sizing is evaluated at cruise trim using AeroSandbox VLM on the rectangular wing. The report uses an estimated steady roll rate based on VLM aileron effectiveness and roll-damping derivative.",
    ]
    output.summary_md.parent.mkdir(parents=True, exist_ok=True)
    output.summary_md.write_text("\n".join(lines), encoding="utf-8")


def run_rectangular_control_surface_sizing(
    *,
    rank: int = 6,
    blade_count_metadata: int = 3,
    output_root: Path = Path("outputs/control_surface_sizing"),
) -> ControlSurfaceSizingOutput:
    config = RectangularWingControlConfig(blade_count_metadata=blade_count_metadata)
    concept = _load_selected_concept(rank=rank, blade_count_metadata=blade_count_metadata)
    flap, aileron, flap_candidates, _ = _pick_flap_and_aileron(config, concept)

    slug = _slugify_concept(concept)
    output_dir = output_root / slug
    output_dir.mkdir(parents=True, exist_ok=True)

    output = ControlSurfaceSizingOutput(
        concept=concept,
        flap=flap,
        aileron=aileron,
        output_dir=output_dir,
        summary_csv=output_dir / "control_surface_summary.csv",
        summary_md=output_dir / "control_surface_summary.md",
        flap_sweep_csv=output_dir / "flap_sweep.csv",
        aileron_sweep_csv=output_dir / "aileron_sweep.csv",
        layout_plot=output_dir / "layout.png",
        flap_heatmap_plot=output_dir / "flap_heatmap.png",
        flap_curves_plot=output_dir / "flap_curves.png",
        flap_section_polar_plot=output_dir / "flap_section_polars.png",
        total_cl_curve_csv=output_dir / "total_cl_curve.csv",
        total_cl_curve_plot=output_dir / "total_cl_curve.png",
        aileron_curves_plot=output_dir / "aileron_curves.png",
    )

    flap_curve_rows = _flap_curve_data(config, concept, flap)
    section_polars = _selected_flap_section_polars(config, concept, flap)
    total_curve_rows = _total_high_lift_curve_data(config, concept, flap, section_polars)
    aileron_curve_rows = _aileron_curve_data(config, flap, aileron)
    aileron_sweep_rows = _selected_flap_aileron_sweep(config, flap)

    _write_csv(output.flap_sweep_csv, [asdict(result) for result in flap_candidates])
    _write_csv(output.aileron_sweep_csv, aileron_sweep_rows)
    _write_csv(output.total_cl_curve_csv, total_curve_rows)
    _render_layout_plot(config, concept, flap, aileron, output.layout_plot)
    _render_flap_heatmap(config, flap_candidates, flap, output.flap_heatmap_plot)
    _render_flap_curves(flap_curve_rows, output.flap_curves_plot)
    _render_flap_section_polars(section_polars, output.flap_section_polar_plot)
    _render_total_cl_curve(total_curve_rows, config, output.total_cl_curve_plot)
    _render_aileron_curves(config, aileron_curve_rows, output.aileron_curves_plot)

    summary = _summary_row(config, concept, flap, aileron, output)
    _write_csv(output.summary_csv, [summary])
    _write_summary_markdown(config, concept, flap, aileron, output)
    return output
