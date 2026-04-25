from __future__ import annotations

import csv
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from optimizer.core.data_models import Stage1MissionConfig
from optimizer.core.high_lift_model import (
    apply_slotted_flap_high_lift_corrections,
    cambridge_uniform_jet_immersion,
    resolve_prop_drop_m,
)
from optimizer.core.stage3_refinement import ensure_stage3_runtime
from optimizer.core.workflow_style import airfoil_red_styles


@dataclass(frozen=True)
class AirfoilPolarComparisonConfig:
    mission: Stage1MissionConfig = Stage1MissionConfig()
    airfoils: tuple[str, ...] = ("s1210", "e423", "dae51", "naca0012", "naca2412")
    flap_deflections_deg: tuple[float, float] = (0.0, 40.0)
    cmu_levels: tuple[float, ...] = (0.0, 0.5, 1.0, 1.5, 2.0, 2.5)
    alpha_min_deg: float = -6.0
    alpha_max_deg: float = 16.0
    alpha_count: int = 111
    sampled_alpha_points_deg: tuple[float, ...] = (-5.0, 0.0, 5.0, 10.0, 15.0)
    representative_n_props: int = 10
    representative_prop_diameter_in: float = 5.5
    representative_flap_span_fraction: float = 0.60
    flap_chord_fraction: float = 0.22
    flap_hinge_point: float = 0.78
    slot_lift_gain_base: float = 1.12
    slot_drag_gain_base: float = 1.08
    blown_section_reference_cl_limit: float = 9.0
    prop_drop_fraction_of_chord: float | None = 0.12
    prop_drop_fraction_of_diameter: float | None = None
    prop_axial_location_fraction_of_chord: float = -0.25
    jet_core_height_factor: float = 1.0
    cambridge_base_streamline_fraction_of_chord: float = 0.08
    cambridge_alpha_streamline_gain: float = 0.55
    cambridge_flap_streamline_gain: float = 0.18
    cambridge_jet_softness_fraction_of_chord: float = 0.02
    legacy_reference_dir: Path = Path("Blown Wing Sizing/S1210")
    legacy_output_root: Path = Path("outputs/airfoil_comparison/legacy_style_polars")
    output_root: Path = Path("outputs/airfoil_comparison")
    display_names: dict[str, str] = field(
        default_factory=lambda: {
            "s1210": "S1210",
            "dae51": "DAE51",
            "naca0012": "NACA 0012",
            "naca2412": "NACA 2412",
            "e423": "Epler E423",
        }
    )

    @property
    def representative_prop_diameter_m(self) -> float:
        return self.representative_prop_diameter_in * 0.0254

    @property
    def representative_disk_area_m2(self) -> float:
        radius = 0.5 * self.representative_prop_diameter_m
        return self.representative_n_props * math.pi * radius**2


def _slugify(name: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


def _display_name(config: AirfoilPolarComparisonConfig, airfoil_name: str) -> str:
    return config.display_names.get(airfoil_name.lower(), airfoil_name)


def _sample_indices(alpha_deg: np.ndarray, sample_alpha_points_deg: tuple[float, ...]) -> np.ndarray:
    points = []
    for alpha_target in sample_alpha_points_deg:
        idx = int(np.argmin(np.abs(alpha_deg - alpha_target)))
        points.append(idx)
    return np.unique(np.asarray(points, dtype=int))


def _slot_lift_gain(config: AirfoilPolarComparisonConfig, flap_deflection_deg: float) -> float:
    deflection_factor = np.clip(flap_deflection_deg / 35.0, 0.5, 1.15)
    chord_factor = np.clip((config.flap_chord_fraction - 0.20) / 0.14, 0.0, 1.0)
    return 1.0 + (config.slot_lift_gain_base - 1.0) * (0.65 + 0.35 * deflection_factor) * (0.75 + 0.25 * chord_factor)


def _slot_drag_gain(config: AirfoilPolarComparisonConfig, flap_deflection_deg: float) -> float:
    deflection_factor = np.clip(flap_deflection_deg / 35.0, 0.5, 1.20)
    chord_factor = np.clip((config.flap_chord_fraction - 0.20) / 0.14, 0.0, 1.0)
    return 1.0 + (config.slot_drag_gain_base - 1.0) * (0.70 + 0.30 * deflection_factor) * (0.80 + 0.20 * chord_factor)


def _representative_prop_drop_m(config: AirfoilPolarComparisonConfig) -> float:
    return resolve_prop_drop_m(
        chord_m=config.mission.chord_m,
        prop_diameter_m=config.representative_prop_diameter_m,
        drop_fraction_of_chord=config.prop_drop_fraction_of_chord,
        drop_fraction_of_diameter=config.prop_drop_fraction_of_diameter,
    )


def _veff_from_cmu(config: AirfoilPolarComparisonConfig, cmu: float) -> float:
    if cmu <= 0.0:
        return config.mission.low_speed_mps

    q_inf = 0.5 * config.mission.air_density_kgpm3 * config.mission.low_speed_mps**2
    thrust_total_n = cmu * q_inf * config.mission.wing_area_m2
    disk_area = max(config.representative_disk_area_m2, 1e-9)
    induced_velocity = math.sqrt(thrust_total_n / (2.0 * config.mission.air_density_kgpm3 * disk_area))
    return config.mission.low_speed_mps + 2.0 * induced_velocity


def _reynolds(config: AirfoilPolarComparisonConfig, velocity_mps: float) -> float:
    return (
        config.mission.air_density_kgpm3
        * velocity_mps
        * config.mission.chord_m
        / max(config.mission.dynamic_viscosity_pas, 1e-12)
    )


def _equivalent_coefficients(
    alpha_deg: np.ndarray,
    cl_local: np.ndarray,
    cd_local: np.ndarray,
    cm_local: np.ndarray,
    q_ratio: float,
    cl_limit: float | None = None,
) -> dict[str, np.ndarray]:
    alpha_rad = np.deg2rad(alpha_deg)
    cl_equiv = q_ratio * cl_local
    if cl_limit is not None and q_ratio > 1.0 + 1e-9:
        cl_limited = cl_limit * np.tanh(cl_equiv / cl_limit)
        scale = np.divide(
            cl_limited,
            cl_equiv,
            out=np.ones_like(cl_equiv),
            where=np.abs(cl_equiv) > 1e-9,
        )
        cl_equiv = cl_limited
    else:
        scale = np.ones_like(cl_equiv)
    cd_equiv = q_ratio * cd_local * scale
    cm_equiv = q_ratio * cm_local * scale
    cx_equiv = -(cd_equiv * np.cos(alpha_rad) + cl_equiv * np.sin(alpha_rad))
    return {
        "cl_equiv": cl_equiv,
        "cd_equiv": cd_equiv,
        "cm_equiv": cm_equiv,
        "cx_equiv": cx_equiv,
    }


def _representative_area_fractions(config: AirfoilPolarComparisonConfig) -> dict[str, float]:
    k_span_expansion = getattr(
        config.mission,
        "k_span_expansion",
        getattr(config.mission, "k_span_e8xpansion"),
    )
    eta = min(
        1.0,
        (
            k_span_expansion
            * config.representative_n_props
            * config.representative_prop_diameter_m
            / max(config.mission.span_m, 1e-9)
        ),
    )
    flap_fraction = np.clip(config.representative_flap_span_fraction, 0.0, 1.0)
    clean_fraction = max(0.0, 1.0 - flap_fraction)
    blown_flap_fraction = eta * flap_fraction
    blown_clean_fraction = eta * clean_fraction
    return {
        "blown_total_fraction": eta,
        "flap_fraction": flap_fraction,
        "clean_fraction": clean_fraction,
        "blown_flap_fraction": blown_flap_fraction,
        "blown_clean_fraction": blown_clean_fraction,
        "flap_unblown_fraction": max(0.0, flap_fraction - blown_flap_fraction),
        "clean_unblown_fraction": max(0.0, clean_fraction - blown_clean_fraction),
    }


def _wing_reference_coefficients(
    config: AirfoilPolarComparisonConfig,
    q_ratio: float,
    alpha_deg: np.ndarray,
    flap_deflection_deg: float,
    clean_cl_unblown: np.ndarray,
    clean_cd_unblown: np.ndarray,
    clean_cm_unblown: np.ndarray,
    clean_cl_blown: np.ndarray,
    clean_cd_blown: np.ndarray,
    clean_cm_blown: np.ndarray,
    flap_cl_unblown: np.ndarray,
    flap_cd_unblown: np.ndarray,
    flap_cm_unblown: np.ndarray,
    flap_cl_blown: np.ndarray,
    flap_cd_blown: np.ndarray,
    flap_cm_blown: np.ndarray,
) -> dict[str, np.ndarray]:
    fractions = _representative_area_fractions(config)
    alpha_rad = np.deg2rad(alpha_deg)
    immersion = cambridge_uniform_jet_immersion(
        alpha_deg=alpha_deg,
        flap_deflection_deg=flap_deflection_deg,
        flap_chord_fraction=config.flap_chord_fraction,
        prop_diameter_m=config.representative_prop_diameter_m,
        prop_drop_m=_representative_prop_drop_m(config),
        chord_m=config.mission.chord_m,
        prop_axial_location_fraction_of_chord=config.prop_axial_location_fraction_of_chord,
        jet_core_height_factor=config.jet_core_height_factor,
        base_streamline_fraction_of_chord=config.cambridge_base_streamline_fraction_of_chord,
        alpha_streamline_gain=config.cambridge_alpha_streamline_gain,
        flap_streamline_gain=config.cambridge_flap_streamline_gain,
        jet_softness_fraction_of_chord=config.cambridge_jet_softness_fraction_of_chord,
    )
    clean_immersion = immersion["clean_immersion_factor"]
    flap_immersion = immersion["flap_immersion_factor"]

    clean_blown_cl_ref_raw = clean_cl_unblown + clean_immersion * (q_ratio * clean_cl_blown - clean_cl_unblown)
    flap_blown_cl_ref_raw = flap_cl_unblown + flap_immersion * (q_ratio * flap_cl_blown - flap_cl_unblown)
    clean_blown_cl_ref = config.blown_section_reference_cl_limit * np.tanh(
        clean_blown_cl_ref_raw / config.blown_section_reference_cl_limit
    )
    flap_blown_cl_ref = config.blown_section_reference_cl_limit * np.tanh(
        flap_blown_cl_ref_raw / config.blown_section_reference_cl_limit
    )
    clean_scale = np.divide(
        clean_blown_cl_ref,
        clean_blown_cl_ref_raw,
        out=np.ones_like(clean_blown_cl_ref_raw),
        where=np.abs(clean_blown_cl_ref_raw) > 1e-9,
    )
    flap_scale = np.divide(
        flap_blown_cl_ref,
        flap_blown_cl_ref_raw,
        out=np.ones_like(flap_blown_cl_ref_raw),
        where=np.abs(flap_blown_cl_ref_raw) > 1e-9,
    )
    clean_blown_cd_ref = (clean_cd_unblown + clean_immersion * (q_ratio * clean_cd_blown - clean_cd_unblown)) * clean_scale
    clean_blown_cm_ref = (clean_cm_unblown + clean_immersion * (q_ratio * clean_cm_blown - clean_cm_unblown)) * clean_scale
    flap_blown_cd_ref = (flap_cd_unblown + flap_immersion * (q_ratio * flap_cd_blown - flap_cd_unblown)) * flap_scale
    flap_blown_cm_ref = (flap_cm_unblown + flap_immersion * (q_ratio * flap_cm_blown - flap_cm_unblown)) * flap_scale

    cl_wing_ref = (
        fractions["clean_unblown_fraction"] * clean_cl_unblown
        + fractions["blown_clean_fraction"] * clean_blown_cl_ref
        + fractions["flap_unblown_fraction"] * flap_cl_unblown
        + fractions["blown_flap_fraction"] * flap_blown_cl_ref
    )
    cd_wing_ref = (
        fractions["clean_unblown_fraction"] * clean_cd_unblown
        + fractions["blown_clean_fraction"] * clean_blown_cd_ref
        + fractions["flap_unblown_fraction"] * flap_cd_unblown
        + fractions["blown_flap_fraction"] * flap_blown_cd_ref
    )
    cm_wing_ref = (
        fractions["clean_unblown_fraction"] * clean_cm_unblown
        + fractions["blown_clean_fraction"] * clean_blown_cm_ref
        + fractions["flap_unblown_fraction"] * flap_cm_unblown
        + fractions["blown_flap_fraction"] * flap_blown_cm_ref
    )
    cx_wing_ref = -(cd_wing_ref * np.cos(alpha_rad) + cl_wing_ref * np.sin(alpha_rad))
    return {
        "cl_wing_ref": cl_wing_ref,
        "cd_wing_ref": cd_wing_ref,
        "cm_wing_ref": cm_wing_ref,
        "cx_wing_ref": cx_wing_ref,
        "clean_immersion_factor": clean_immersion,
        "flap_immersion_factor": flap_immersion,
    }


def _evaluate_case(
    config: AirfoilPolarComparisonConfig,
    airfoil_name: str,
    flap_deflection_deg: float,
    cmu: float,
) -> dict[str, Any]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb

    airfoil = asb.Airfoil(airfoil_name)
    alpha_deg = np.linspace(config.alpha_min_deg, config.alpha_max_deg, config.alpha_count)
    reynolds_inf = _reynolds(config, config.mission.low_speed_mps)
    veff_mps = _veff_from_cmu(config, cmu)
    reynolds = _reynolds(config, veff_mps)
    q_ratio = (veff_mps / config.mission.low_speed_mps) ** 2

    clean_inf = airfoil.get_aero_from_neuralfoil(alpha=alpha_deg, Re=reynolds_inf)
    clean_blown = airfoil.get_aero_from_neuralfoil(alpha=alpha_deg, Re=reynolds)
    clean_cl_inf = np.asarray(clean_inf["CL"], dtype=float)
    clean_cd_inf = np.asarray(clean_inf["CD"], dtype=float)
    clean_cm_inf = np.asarray(clean_inf["CM"], dtype=float)
    clean_cl_blown = np.asarray(clean_blown["CL"], dtype=float)
    clean_cd_blown = np.asarray(clean_blown["CD"], dtype=float)
    clean_cm_blown = np.asarray(clean_blown["CM"], dtype=float)

    if flap_deflection_deg > 0.0:
        raw_flap_inf = airfoil.get_aero_from_neuralfoil(
            alpha=alpha_deg,
            Re=reynolds_inf,
            control_surfaces=[
                asb.ControlSurface(
                    name="flap",
                    symmetric=True,
                    deflection=flap_deflection_deg,
                    hinge_point=config.flap_hinge_point,
                )
            ],
        )
        raw_flap_blown = airfoil.get_aero_from_neuralfoil(
            alpha=alpha_deg,
            Re=reynolds,
            control_surfaces=[
                asb.ControlSurface(
                    name="flap",
                    symmetric=True,
                    deflection=flap_deflection_deg,
                    hinge_point=config.flap_hinge_point,
                )
            ],
        )
        raw_flap_cl_inf = np.asarray(raw_flap_inf["CL"], dtype=float)
        raw_flap_cd_inf = np.asarray(raw_flap_inf["CD"], dtype=float)
        raw_flap_cm_inf = np.asarray(raw_flap_inf["CM"], dtype=float)
        raw_flap_cl_blown = np.asarray(raw_flap_blown["CL"], dtype=float)
        raw_flap_cd_blown = np.asarray(raw_flap_blown["CD"], dtype=float)
        raw_flap_cm_blown = np.asarray(raw_flap_blown["CM"], dtype=float)

        slot_gain_unblown = _slot_lift_gain(config, flap_deflection_deg)
        slot_gain_blown = slot_gain_unblown
        slot_drag = _slot_drag_gain(config, flap_deflection_deg)

        unblown_corrected = apply_slotted_flap_high_lift_corrections(
            alpha_deg=alpha_deg,
            clean_cl=clean_cl_inf,
            clean_cd=clean_cd_inf,
            clean_cm=clean_cm_inf,
            raw_flap_cl=raw_flap_cl_inf,
            raw_flap_cd=raw_flap_cd_inf,
            raw_flap_cm=raw_flap_cm_inf,
            flap_chord_fraction=config.flap_chord_fraction,
            flap_deflection_deg=flap_deflection_deg,
            slot_gain=slot_gain_unblown,
            slot_drag=slot_drag,
            prop_drop_bonus=1.0,
            reynolds=reynolds_inf,
            reference_reynolds=max(1.2e5, 0.55 * reynolds_inf),
        )
        blown_corrected = apply_slotted_flap_high_lift_corrections(
            alpha_deg=alpha_deg,
            clean_cl=clean_cl_blown,
            clean_cd=clean_cd_blown,
            clean_cm=clean_cm_blown,
            raw_flap_cl=raw_flap_cl_blown,
            raw_flap_cd=raw_flap_cd_blown,
            raw_flap_cm=raw_flap_cm_blown,
            flap_chord_fraction=config.flap_chord_fraction,
            flap_deflection_deg=flap_deflection_deg,
            slot_gain=slot_gain_blown,
            slot_drag=slot_drag,
            prop_drop_bonus=1.0,
            reynolds=reynolds,
            reference_reynolds=max(1.2e5, 0.55 * reynolds),
        )

        flap_cl_unblown = unblown_corrected["adjusted_cl"]
        flap_cd_unblown = unblown_corrected["adjusted_cd"]
        flap_cm_unblown = unblown_corrected["adjusted_cm"]
        flap_cl_blown = blown_corrected["adjusted_cl"]
        flap_cd_blown = blown_corrected["adjusted_cd"]
        flap_cm_blown = blown_corrected["adjusted_cm"]
    else:
        flap_cl_unblown = clean_cl_inf
        flap_cd_unblown = clean_cd_inf
        flap_cm_unblown = clean_cm_inf
        flap_cl_blown = clean_cl_blown
        flap_cd_blown = clean_cd_blown
        flap_cm_blown = clean_cm_blown

    cl_local = flap_cl_blown if cmu > 0.0 else flap_cl_unblown
    cd_local = flap_cd_blown if cmu > 0.0 else flap_cd_unblown
    cm_local = flap_cm_blown if cmu > 0.0 else flap_cm_unblown

    section_reference = _equivalent_coefficients(
        alpha_deg=alpha_deg,
        cl_local=cl_local,
        cd_local=cd_local,
        cm_local=cm_local,
        q_ratio=q_ratio,
        cl_limit=config.blown_section_reference_cl_limit,
    )
    wing_reference = _wing_reference_coefficients(
        config,
        q_ratio,
        alpha_deg,
        flap_deflection_deg,
        clean_cl_inf,
        clean_cd_inf,
        clean_cm_inf,
        clean_cl_blown,
        clean_cd_blown,
        clean_cm_blown,
        flap_cl_unblown,
        flap_cd_unblown,
        flap_cm_unblown,
        flap_cl_blown,
        flap_cd_blown,
        flap_cm_blown,
    )

    return {
        "airfoil": airfoil_name,
        "flap_deflection_deg": flap_deflection_deg,
        "cmu": cmu,
        "veff_mps": veff_mps,
        "reynolds": reynolds,
        "reynolds_inf": reynolds_inf,
        "q_ratio": q_ratio,
        "alpha_deg": alpha_deg,
        "cl_local": cl_local,
        "cd_local": cd_local,
        "cm_local": cm_local,
        **section_reference,
        **wing_reference,
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _render_response_grid(
    config: AirfoilPolarComparisonConfig,
    airfoil_name: str,
    cases: list[dict[str, Any]],
    output_path: Path,
    *,
    middle_quantity: str,
    middle_label: str,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    plt.rcParams["font.family"] = "serif"
    plt.rcParams["mathtext.fontset"] = "stix"

    case_lookup = {(row["flap_deflection_deg"], row["cmu"]): row for row in cases}
    colors = plt.cm.Reds(np.linspace(0.20, 0.95, len(config.cmu_levels)))
    if len(colors) > 0:
        colors[0] = np.array([0.12, 0.12, 0.14, 1.0])

    fig, axs = plt.subplots(3, 2, figsize=(9.2, 12.4), sharex="col")
    fig.suptitle(
        f"{_display_name(config, airfoil_name)} | section response vs. blowing level",
        fontsize=17,
        y=0.995,
    )

    y_keys = ("cl_equiv", middle_quantity, "cm_equiv")
    y_labels = (r"$c_\ell$", middle_label, r"$c_m$")
    titles = (
        r"$\delta_f = 0^\circ$",
        r"$\delta_f = 40^\circ$",
    )

    for col, flap_deflection_deg in enumerate(config.flap_deflections_deg):
        axs[0, col].set_title(titles[col], fontsize=22)
        for color, cmu in zip(colors, config.cmu_levels):
            row = case_lookup[(flap_deflection_deg, cmu)]
            alpha_deg = row["alpha_deg"]
            sample_idx = _sample_indices(alpha_deg, config.sampled_alpha_points_deg)
            for ax, y_key in zip(axs[:, col], y_keys):
                y = np.asarray(row[y_key], dtype=float)
                ax.plot(alpha_deg, y, color=color, linewidth=1.8)
                ax.scatter(alpha_deg[sample_idx], y[sample_idx], color=color, s=14, zorder=3)

    for row_idx in range(3):
        for col in range(2):
            axs[row_idx, col].grid(True, alpha=0.22)
            axs[row_idx, col].tick_params(labelsize=16)
            axs[row_idx, col].set_ylabel(y_labels[row_idx], fontsize=28)
            if row_idx == 2:
                axs[row_idx, col].set_xlabel(r"$\alpha$", fontsize=28)

    handles = [
        plt.Line2D([0], [0], color=color, lw=2.0, label=rf"$C_\mu={cmu:.1f}$")
        for color, cmu in zip(colors, config.cmu_levels)
    ]
    fig.legend(handles=handles, loc="upper center", ncol=min(len(handles), 4), fontsize=12, frameon=False, bbox_to_anchor=(0.5, 0.975))
    fig.tight_layout(rect=(0.03, 0.03, 0.97, 0.955))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _render_airfoil_comparison_summary(
    config: AirfoilPolarComparisonConfig,
    metric_rows: list[dict[str, Any]],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    plt.rcParams["font.family"] = "serif"
    plt.rcParams["mathtext.fontset"] = "stix"

    fig, axs = plt.subplots(2, 2, figsize=(12.4, 8.8))
    styles = airfoil_red_styles(config.airfoils)

    for airfoil_name in config.airfoils:
        style = styles.get(airfoil_name, {"color": "#111827", "marker": "o"})
        for flap_deflection_deg, ax_cl, ax_cd in (
            (0.0, axs[0, 0], axs[1, 0]),
            (40.0, axs[0, 1], axs[1, 1]),
        ):
            rows = [
                row for row in metric_rows
                if row["airfoil"] == airfoil_name and abs(row["flap_deflection_deg"] - flap_deflection_deg) < 1e-9
            ]
            rows.sort(key=lambda row: row["cmu"])
            cmu = [row["cmu"] for row in rows]
            clmax = [row["wing_cl_ref_max"] for row in rows]
            cd_alpha0 = [row["wing_cd_ref_at_alpha0"] for row in rows]
            label = _display_name(config, airfoil_name)
            ax_cl.plot(cmu, clmax, marker=style["marker"], color=style["color"], linewidth=2.0, label=label)
            ax_cd.plot(cmu, cd_alpha0, marker=style["marker"], color=style["color"], linewidth=2.0, label=label)

    axs[0, 0].set_title(r"$\delta_f = 0^\circ$ | representative wing max $C_L$")
    axs[0, 1].set_title(r"$\delta_f = 40^\circ$ | representative wing max $C_L$")
    axs[1, 0].set_title(r"$\delta_f = 0^\circ$ | representative wing $C_D$ at $\alpha = 0^\circ$")
    axs[1, 1].set_title(r"$\delta_f = 40^\circ$ | representative wing $C_D$ at $\alpha = 0^\circ$")

    for ax in axs[0, :]:
        ax.set_ylabel(r"max $C_L$")
    for ax in axs[1, :]:
        ax.set_ylabel(r"$C_D(\alpha=0^\circ)$")
        ax.set_xlabel(r"$C_\mu$")

    for ax in axs.flat:
        ax.grid(True, alpha=0.25)
        ax.legend()

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def _metric_summary_rows(cases: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in cases:
        alpha_deg = np.asarray(row["alpha_deg"], dtype=float)
        cl_section = np.asarray(row["cl_equiv"], dtype=float)
        cd_section = np.asarray(row["cd_equiv"], dtype=float)
        cm_section = np.asarray(row["cm_equiv"], dtype=float)
        cx_section = np.asarray(row["cx_equiv"], dtype=float)
        cl_wing = np.asarray(row["cl_wing_ref"], dtype=float)
        cd_wing = np.asarray(row["cd_wing_ref"], dtype=float)
        cm_wing = np.asarray(row["cm_wing_ref"], dtype=float)
        cx_wing = np.asarray(row["cx_wing_ref"], dtype=float)
        idx_cl_max_section = int(np.nanargmax(cl_section))
        idx_cx_min_section = int(np.nanargmin(cx_section))
        idx_cl_max_wing = int(np.nanargmax(cl_wing))
        idx_cx_min_wing = int(np.nanargmin(cx_wing))
        idx_alpha0 = int(np.argmin(np.abs(alpha_deg)))
        rows.append(
            {
                "airfoil": row["airfoil"],
                "flap_deflection_deg": row["flap_deflection_deg"],
                "cmu": row["cmu"],
                "veff_mps": row["veff_mps"],
                "reynolds": row["reynolds"],
                "reynolds_inf": row["reynolds_inf"],
                "q_ratio": row["q_ratio"],
                "cl_equiv_max": float(cl_section[idx_cl_max_section]),
                "alpha_at_clmax_deg": float(alpha_deg[idx_cl_max_section]),
                "cd_equiv_at_alpha0": float(cd_section[idx_alpha0]),
                "cm_equiv_at_alpha0": float(cm_section[idx_alpha0]),
                "cx_equiv_min": float(cx_section[idx_cx_min_section]),
                "alpha_at_cxmin_deg": float(alpha_deg[idx_cx_min_section]),
                "wing_cl_ref_max": float(cl_wing[idx_cl_max_wing]),
                "wing_alpha_at_clmax_deg": float(alpha_deg[idx_cl_max_wing]),
                "wing_cd_ref_at_alpha0": float(cd_wing[idx_alpha0]),
                "wing_cm_ref_at_alpha0": float(cm_wing[idx_alpha0]),
                "wing_cx_ref_min": float(cx_wing[idx_cx_min_wing]),
                "wing_alpha_at_cxmin_deg": float(alpha_deg[idx_cx_min_wing]),
            }
        )
    return rows


def _flatten_curve_rows(cases: list[dict[str, Any]]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for row in cases:
        alpha_deg = np.asarray(row["alpha_deg"], dtype=float)
        for idx, alpha in enumerate(alpha_deg):
            rows.append(
                {
                    "airfoil": row["airfoil"],
                    "flap_deflection_deg": row["flap_deflection_deg"],
                    "cmu": row["cmu"],
                    "veff_mps": row["veff_mps"],
                    "reynolds": row["reynolds"],
                    "reynolds_inf": row["reynolds_inf"],
                    "q_ratio": row["q_ratio"],
                    "alpha_deg": float(alpha),
                    "cl_local": float(row["cl_local"][idx]),
                    "cd_local": float(row["cd_local"][idx]),
                    "cm_local": float(row["cm_local"][idx]),
                    "cl_equiv": float(row["cl_equiv"][idx]),
                    "cd_equiv": float(row["cd_equiv"][idx]),
                    "cm_equiv": float(row["cm_equiv"][idx]),
                    "cx_equiv": float(row["cx_equiv"][idx]),
                    "cl_wing_ref": float(row["cl_wing_ref"][idx]),
                    "cd_wing_ref": float(row["cd_wing_ref"][idx]),
                    "cm_wing_ref": float(row["cm_wing_ref"][idx]),
                    "cx_wing_ref": float(row["cx_wing_ref"][idx]),
                }
            )
    return rows


def _reference_reynolds_list(config: AirfoilPolarComparisonConfig) -> list[float]:
    if not config.legacy_reference_dir.exists():
        return []
    re_values: set[float] = set()
    for path in config.legacy_reference_dir.glob("T1-Re*.txt"):
        match = re.search(r"Re([0-9]+(?:\.[0-9]+)?)", path.name)
        if match:
            re_values.add(float(match.group(1)) * 1.0e6)
    return sorted(re_values)


def _render_legacy_style_clean_polars(
    config: AirfoilPolarComparisonConfig,
    airfoil_name: str,
) -> list[Path]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb

    re_values = _reference_reynolds_list(config)
    if not re_values:
        return []

    airfoil = asb.Airfoil(airfoil_name)
    alpha_deg = np.arange(-24.0, 29.0, 1.0)
    output_dir = config.legacy_output_root / _slugify(airfoil_name)
    output_dir.mkdir(parents=True, exist_ok=True)

    written_paths: list[Path] = []
    for re_value in re_values:
        aero = airfoil.get_aero_from_neuralfoil(alpha=alpha_deg, Re=re_value)
        cl = np.asarray(aero["CL"], dtype=float)
        cd = np.asarray(aero["CD"], dtype=float)
        cm = np.asarray(aero["CM"], dtype=float)
        file_name = f"T1-Re{re_value / 1.0e6:0.3f}-N9.0.txt"
        path = output_dir / file_name
        header = [
            "flow5 v7.54",
            "",
            f" Calculated polar for: {_display_name(config, airfoil_name)}",
            "",
            " 1 1 Reynolds number fixed          Mach number fixed         ",
            "",
            f" xtrf =   1.000 (top)        1.000 (bottom)",
            f" Mach =   0.000     Re =     {re_value / 1.0e6:0.3f} e 6     Ncrit =   9.000",
            "",
            "  alpha     CL        CD        Cm",
            " ------- -------- --------- ---------",
        ]
        rows = [
            f" {alpha:7.3f} {cl_i:8.4f} {cd_i:9.5f} {cm_i:9.4f}"
            for alpha, cl_i, cd_i, cm_i in zip(alpha_deg, cl, cd, cm)
        ]
        path.write_text("\n".join(header + rows) + "\n", encoding="utf-8")
        written_paths.append(path)
    return written_paths


def run_airfoil_polar_comparison(
    config: AirfoilPolarComparisonConfig | None = None,
) -> dict[str, Any]:
    config = config or AirfoilPolarComparisonConfig()
    cases: list[dict[str, Any]] = []
    for airfoil_name in config.airfoils:
        for flap_deflection_deg in config.flap_deflections_deg:
            for cmu in config.cmu_levels:
                cases.append(_evaluate_case(config, airfoil_name, flap_deflection_deg, cmu))

    curve_rows = _flatten_curve_rows(cases)
    metric_rows = _metric_summary_rows(cases)

    output_root = config.output_root
    output_root.mkdir(parents=True, exist_ok=True)
    curve_csv = output_root / "airfoil_curve_data.csv"
    metric_csv = output_root / "airfoil_metric_summary.csv"
    _write_csv(curve_csv, curve_rows)
    _write_csv(metric_csv, metric_rows)

    airfoil_artifacts: dict[str, dict[str, Any]] = {}
    for airfoil_name in config.airfoils:
        airfoil_slug = _slugify(airfoil_name)
        airfoil_cases = [row for row in cases if row["airfoil"] == airfoil_name]
        output_dir = output_root / airfoil_slug
        figure_cd = output_dir / "response_grid_cd.png"
        figure_cx = output_dir / "response_grid_cx.png"
        _render_response_grid(
            config,
            airfoil_name,
            airfoil_cases,
            figure_cd,
            middle_quantity="cd_equiv",
            middle_label=r"$c_d$",
        )
        _render_response_grid(
            config,
            airfoil_name,
            airfoil_cases,
            figure_cx,
            middle_quantity="cx_equiv",
            middle_label=r"$c_x$",
        )
        legacy_paths = _render_legacy_style_clean_polars(config, airfoil_name)
        airfoil_artifacts[airfoil_name] = {
            "response_grid_cd_png": figure_cd,
            "response_grid_cx_png": figure_cx,
            "legacy_paths": legacy_paths,
        }

    comparison_png = output_root / "airfoil_comparison_summary.png"
    _render_airfoil_comparison_summary(config, metric_rows, comparison_png)

    summary_md = output_root / "README.md"
    lines = [
        "# Airfoil Blowing Comparison",
        "",
        (
            "This study compares "
            + ", ".join(f"`{_display_name(config, airfoil)}`" for airfoil in config.airfoils)
            + " under the current blown-wing low-speed assumptions."
        ),
        "",
        "## Assumptions",
        "",
        f"- Freestream speed: `{config.mission.low_speed_mps:.1f} m/s`",
        f"- Chord used for section Reynolds number: `{config.mission.chord_m:.2f} m`",
        f"- Representative prop layout for mapping `C_mu -> V_eff`: `{config.representative_n_props} x {config.representative_prop_diameter_in:.1f} in`",
        f"- Flap deflections compared: `{config.flap_deflections_deg[0]:.0f} deg` and `{config.flap_deflections_deg[1]:.0f} deg`",
        f"- Blowing levels: `{', '.join(f'{value:.1f}' for value in config.cmu_levels)}` in `C_mu` increments",
        f"- Slotted flap model: `NeuralFoil flap polar + staged slotted-flap correction`",
        f"- Representative motor height for wing-level summary: `{_representative_prop_drop_m(config):.3f} m` (`{_representative_prop_drop_m(config) / max(config.mission.chord_m, 1e-9):.3f} c`)",
        f"- Freestream-referenced blown section `c_l` is smoothly limited to about `{config.blown_section_reference_cl_limit:.1f}` to stay within the order of magnitude reported by 2D blown-flap wind-tunnel data",
        f"- Representative wing summary uses a Cambridge-style uniform-jet immersion model with a blown-area fraction of about `{100.0 * _representative_area_fractions(config)['blown_total_fraction']:.1f}%` and flap span fraction `{100.0 * config.representative_flap_span_fraction:.0f}%`",
        "",
        "## Outputs",
        "",
        f"- Curve CSV: [{curve_csv.name}]({curve_csv.name})",
        f"- Summary CSV: [{metric_csv.name}]({metric_csv.name})",
        f"- Cross-airfoil summary: ![]({comparison_png.name})",
        "- The per-airfoil response grids are 2D section response plots.",
        "- The cross-airfoil summary plot uses a representative wing-area integration so airfoil ranking is not driven only by section-max scaling.",
        "",
    ]
    for airfoil_name in config.airfoils:
        slug = _slugify(airfoil_name)
        display = _display_name(config, airfoil_name)
        lines.extend(
            [
                f"## {display}",
                "",
                f"- `c_l/c_d/c_m` grid: ![]({slug}/response_grid_cd.png)",
                "",
                f"- paper-style `c_l/c_x/c_m` grid: ![]({slug}/response_grid_cx.png)",
                "",
                f"- legacy-style clean polar folder: `{config.legacy_output_root / slug}`",
                "",
            ]
        )
    summary_md.write_text("\n".join(lines), encoding="utf-8")

    return {
        "curve_csv": curve_csv,
        "metric_csv": metric_csv,
        "comparison_png": comparison_png,
        "summary_md": summary_md,
        "airfoil_artifacts": airfoil_artifacts,
    }
