from __future__ import annotations

import math

import numpy as np


def resolve_prop_drop_m(
    *,
    chord_m: float,
    prop_diameter_m: float,
    drop_fraction_of_chord: float | None,
    drop_fraction_of_diameter: float | None,
) -> float:
    """
    Resolve the propeller vertical drop in metres.

    Newer blown-wing studies in this repository use a chord-referenced motor
    height to align with the Cambridge configuration study. A diameter-based
    fallback is kept for backward compatibility with earlier artifacts.
    """
    if drop_fraction_of_chord is not None:
        return max(float(drop_fraction_of_chord), 0.0) * max(float(chord_m), 0.0)
    if drop_fraction_of_diameter is not None:
        return max(float(drop_fraction_of_diameter), 0.0) * max(float(prop_diameter_m), 0.0)
    return 0.0


def cambridge_uniform_jet_immersion(
    *,
    alpha_deg: np.ndarray | float,
    flap_deflection_deg: float,
    flap_chord_fraction: float,
    prop_diameter_m: float,
    prop_drop_m: float,
    chord_m: float,
    prop_axial_location_fraction_of_chord: float = -0.25,
    jet_core_height_factor: float = 1.0,
    base_streamline_fraction_of_chord: float = 0.08,
    alpha_streamline_gain: float = 0.55,
    flap_streamline_gain: float = 0.18,
    jet_softness_fraction_of_chord: float = 0.02,
) -> dict[str, np.ndarray]:
    """
    Estimate whether the wing remains immersed in the propeller jet using a
    low-order interpretation of the methodology described by Hawkswell et al.

    The Cambridge study models the blown region as a uniform two-dimensional
    jet-covered strip and highlights a sharp loss in benefit once the jet rides
    above the wing. This helper applies that idea as a smooth but steep
    immersion factor:

    - `available_submergence_m` is set by jet half-height plus prop drop.
    - `required_submergence_m` is an effective stagnation-streamline height
      proxy that rises with angle of attack and flap deflection.
    - `immersion_factor` tends to one when the wing remains submerged and tends
      rapidly to zero once the jet clears the wing.

    The result is intentionally a concept-level surrogate rather than a CFD
    solution, but it preserves the first-order mechanism emphasized by the
    Cambridge paper.
    """
    alpha = np.asarray(alpha_deg, dtype=float)
    alpha_pos_rad = np.deg2rad(np.clip(alpha, 0.0, 30.0))
    flap_deflection_rad = math.radians(max(float(flap_deflection_deg), 0.0))

    x_over_c = abs(float(prop_axial_location_fraction_of_chord))
    axial_scale = 1.0 + 0.35 * max(x_over_c - 0.25, 0.0) / 0.25
    available_submergence_m = 0.5 * float(jet_core_height_factor) * float(prop_diameter_m) + float(prop_drop_m)

    clean_required_m = float(chord_m) * (
        float(base_streamline_fraction_of_chord)
        + float(alpha_streamline_gain) * np.sin(alpha_pos_rad)
    ) * axial_scale
    flap_required_m = clean_required_m + float(chord_m) * (
        float(flap_streamline_gain)
        * float(np.clip(flap_chord_fraction, 0.05, 0.60))
        * math.sin(flap_deflection_rad)
    )

    softness_m = max(float(jet_softness_fraction_of_chord) * float(chord_m), 1e-6)
    clean_margin_m = available_submergence_m - clean_required_m
    flap_margin_m = available_submergence_m - flap_required_m

    clean_immersion_factor = 0.5 * (1.0 + np.tanh(clean_margin_m / softness_m))
    flap_immersion_factor = 0.5 * (1.0 + np.tanh(flap_margin_m / softness_m))

    return {
        "available_submergence_m": np.full_like(alpha, available_submergence_m, dtype=float),
        "clean_required_submergence_m": np.asarray(clean_required_m, dtype=float),
        "flap_required_submergence_m": np.asarray(flap_required_m, dtype=float),
        "clean_margin_m": np.asarray(clean_margin_m, dtype=float),
        "flap_margin_m": np.asarray(flap_margin_m, dtype=float),
        "clean_immersion_factor": np.asarray(clean_immersion_factor, dtype=float),
        "flap_immersion_factor": np.asarray(flap_immersion_factor, dtype=float),
    }


def apply_slotted_flap_high_lift_corrections(
    *,
    alpha_deg: np.ndarray,
    clean_cl: np.ndarray,
    clean_cd: np.ndarray,
    clean_cm: np.ndarray,
    raw_flap_cl: np.ndarray,
    raw_flap_cd: np.ndarray,
    raw_flap_cm: np.ndarray,
    flap_chord_fraction: float,
    flap_deflection_deg: float,
    slot_gain: float,
    slot_drag: float,
    prop_drop_bonus: float,
    reynolds: float,
    reference_reynolds: float,
) -> dict[str, np.ndarray]:
    """
    Build a concept-level slotted-flap surrogate that separates:
    - low-alpha camber / lift-curve shift
    - high-alpha / near-stall lift augmentation
    - additional nose-down pitching moment

    This is intentionally a mid-fidelity engineering model rather than a pure
    NeuralFoil output. The goal is to produce better concept trends for large
    flap deflections used in blown-lift studies.
    """
    alpha = np.asarray(alpha_deg, dtype=float)
    clean_cl = np.asarray(clean_cl, dtype=float)
    clean_cd = np.asarray(clean_cd, dtype=float)
    clean_cm = np.asarray(clean_cm, dtype=float)
    raw_flap_cl = np.asarray(raw_flap_cl, dtype=float)
    raw_flap_cd = np.asarray(raw_flap_cd, dtype=float)
    raw_flap_cm = np.asarray(raw_flap_cm, dtype=float)

    flap_deflection_rad = math.radians(max(flap_deflection_deg, 0.0))
    cf = float(np.clip(flap_chord_fraction, 0.05, 0.50))

    base_cl = clean_cl + slot_gain * (raw_flap_cl - clean_cl)
    base_cd = clean_cd + slot_drag * (raw_flap_cd - clean_cd)
    base_cm = clean_cm + slot_gain * (raw_flap_cm - clean_cm)

    re_ratio = max(reynolds, 1e4) / max(reference_reynolds, 1e4)
    re_scale = 1.0 + 0.12 * np.clip(math.log10(re_ratio), -0.4, 0.5)
    blowing_scale = 1.0 + 0.35 * max(prop_drop_bonus - 1.0, 0.0)

    low_alpha_gate = 1.0 / (1.0 + np.exp((alpha - 11.0) / 2.4))
    peak_alpha = 11.0 + 0.08 * flap_deflection_deg
    peak_width = 3.8 + 3.0 * cf
    post_stall_gate = np.exp(-0.5 * ((alpha - peak_alpha) / peak_width) ** 2)

    delta_cl0 = 0.95 * cf * math.sin(flap_deflection_rad) * re_scale * blowing_scale
    delta_clmax = 1.85 * cf**0.80 * math.sin(flap_deflection_rad) ** 0.95 * re_scale * blowing_scale
    delta_cl = delta_cl0 * low_alpha_gate + delta_clmax * post_stall_gate

    # Ensure the corrected flap curve remains meaningfully above the clean curve
    # through the pre-stall range, but let the raw/base solver shape dominate
    # elsewhere.
    minimum_increment = 0.45 * delta_cl0 * low_alpha_gate + 0.20 * delta_clmax * post_stall_gate
    adjusted_cl = np.maximum(base_cl + delta_cl, clean_cl + minimum_increment)

    delta_cd0 = 0.10 * cf * math.sin(flap_deflection_rad)
    delta_cd_peak = 0.42 * cf * math.sin(flap_deflection_rad) ** 1.15
    adjusted_cd = np.maximum(
        base_cd + delta_cd0 * (0.6 + 0.4 * low_alpha_gate) + delta_cd_peak * post_stall_gate,
        clean_cd,
    )

    delta_cm0 = 0.22 * cf * math.sin(flap_deflection_rad) * blowing_scale
    adjusted_cm = base_cm - delta_cm0 * (0.75 * low_alpha_gate + 0.35 * post_stall_gate)

    return {
        "adjusted_cl": adjusted_cl,
        "adjusted_cd": adjusted_cd,
        "adjusted_cm": adjusted_cm,
        "delta_cl": delta_cl,
        "delta_cd": adjusted_cd - base_cd,
        "delta_cm": adjusted_cm - base_cm,
    }
