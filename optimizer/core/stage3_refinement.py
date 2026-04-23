from __future__ import annotations

import csv
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from optimizer.core.data_models import Stage1MissionConfig


STAGE3_VISUAL_DIR = Path("outputs/stage3_visuals")
STAGE3_TRADE_PLOT = STAGE3_VISUAL_DIR / "stage3_trade_space.png"
STAGE3_GALLERY_MD = STAGE3_VISUAL_DIR / "STAGE3_GALLERY.md"


@dataclass(frozen=True)
class RuntimeModules:
    asb: Any
    anp: Any
    onp: Any
    plt: Any
    patches: Any


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def ensure_stage3_runtime() -> RuntimeModules:
    """Load AeroSandbox and plotting libraries, falling back to the repo-local .venv."""

    repo_root = _repo_root()
    mpl_dir = repo_root / "outputs" / ".mplconfig"
    mpl_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(mpl_dir))

    try:
        import aerosandbox as asb
    except ModuleNotFoundError:
        for site_packages in (repo_root / ".venv" / "lib").glob("python*/site-packages"):
            if str(site_packages) not in sys.path:
                sys.path.insert(0, str(site_packages))
            break
        import aerosandbox as asb

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import patches
    import numpy as onp
    import aerosandbox.numpy as anp

    return RuntimeModules(asb=asb, anp=anp, onp=onp, plt=plt, patches=patches)


def candidate_key(row: dict[str, Any]) -> str:
    return (
        f"{int(float(row['n_props']))}|"
        f"{float(row['prop_diameter_in']):.3f}|"
        f"{float(row['prop_pitch_ratio']):.3f}|"
        f"{row['prop_family']}"
    )


def parse_prop_centers(serialized: str) -> list[float]:
    if not serialized:
        return []
    return [float(chunk) for chunk in serialized.split(";") if chunk]


def load_csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def load_stage1_lookup(path: Path) -> dict[str, dict[str, str]]:
    return {candidate_key(row): row for row in load_csv_rows(path)}


def _safe_float(value: Any, fallback: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _geometry_seeds(mission: Stage1MissionConfig, stage2_row: dict[str, str]) -> dict[str, float]:
    blown_fraction = _safe_float(stage2_row["blown_span_fraction"], 0.35)
    n_props = int(float(stage2_row["n_props"]))
    return {
        "root_chord_m": min(0.44, max(0.31, mission.chord_m * (1.0 + 0.22 * (0.45 - blown_fraction)))),
        "taper": 0.78 if n_props >= 6 else 0.66,
        "washout_deg": -2.5,                  # -2.5° washout seed; typical for stall-delay on tapered wings (Raymer 6th ed. Ch. 4)
        "flap_span_fraction": 0.52,           # flap spans ~52% of semispan; Raymer 6th ed. Table 27.1: optimum flap span 60-70% of total span → ~52% semispan
        "flap_chord_fraction": 0.27,          # plain/slotted flap chord ≈ 27% of local wing chord; Raymer 6th ed. Ch. 27: typical 25-30%
        "aileron_span_fraction": 0.23,        # aileron spans outer ~23% of semispan; Raymer 6th ed.: ailerons typically on outer 30-40% of span
        "aileron_chord_fraction": 0.22,       # aileron chord ≈ 22% of local chord; Raymer 6th ed. Ch. 27: typical 20-25%
        "horizontal_tail_volume": 0.68,       # VH = 0.68; Raymer 6th ed. Table 6.4: conventional GA/UAV VH = 0.4-0.7; 0.68 is high end to handle large CG travel with blown wing
        "vertical_tail_volume": 0.055,        # VV = 0.055; Raymer 6th ed. Table 6.4: conventional GA/UAV VV = 0.02-0.06; 0.055 provides yaw authority under asymmetric-thrust failure
    }


def optimize_stage3_geometry(
    mission: Stage1MissionConfig,
    stage2_row: dict[str, str],
    stage1_row: dict[str, str],
) -> dict[str, float]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb
    anp = runtime.anp

    seeds = _geometry_seeds(mission, stage2_row)
    baseline_required_veff = max(_safe_float(stage1_row["low_speed_required_veff_mps"], mission.low_speed_mps), mission.low_speed_mps)
    baseline_actual_veff = max(_safe_float(stage1_row["low_speed_veff_mps"], baseline_required_veff), baseline_required_veff)
    baseline_low_speed_power = max(_safe_float(stage1_row["low_speed_power_w"], 1.0), 1.0)
    baseline_cruise_power = max(_safe_float(stage1_row["cruise_power_w"], 1.0), 1.0)
    baseline_cruise_drag = max(_safe_float(stage1_row["cruise_drag_n"], 1.0), 1.0)
    blown_fraction = max(0.05, min(0.95, _safe_float(stage2_row["blown_span_fraction"], 0.35)))

    q_low = 0.5 * mission.air_density_kgpm3 * mission.low_speed_mps**2
    q_cruise = 0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2

    opti = asb.Opti()

    c_root = opti.variable(
        init_guess=seeds["root_chord_m"],
        lower_bound=0.24,
        upper_bound=0.46,
    )
    taper = opti.variable(
        init_guess=seeds["taper"],
        lower_bound=0.40,
        upper_bound=0.95,
    )
    washout_deg = opti.variable(
        init_guess=seeds["washout_deg"],
        lower_bound=-5.0,
        upper_bound=0.0,
    )
    flap_span_fraction = opti.variable(
        init_guess=seeds["flap_span_fraction"],
        lower_bound=0.42,
        upper_bound=0.68,
    )
    flap_chord_fraction = opti.variable(
        init_guess=seeds["flap_chord_fraction"],
        lower_bound=0.20,
        upper_bound=0.34,
    )
    aileron_span_fraction = opti.variable(
        init_guess=seeds["aileron_span_fraction"],
        lower_bound=0.16,
        upper_bound=0.34,
    )
    aileron_chord_fraction = opti.variable(
        init_guess=seeds["aileron_chord_fraction"],
        lower_bound=0.16,
        upper_bound=0.28,
    )
    horizontal_tail_volume = opti.variable(
        init_guess=seeds["horizontal_tail_volume"],
        lower_bound=0.50,
        upper_bound=0.85,
    )
    vertical_tail_volume = opti.variable(
        init_guess=seeds["vertical_tail_volume"],
        lower_bound=0.03,
        upper_bound=0.08,
    )

    c_tip = c_root * taper
    wing_area = 0.5 * mission.span_m * (c_root + c_tip)
    aspect_ratio = mission.span_m**2 / wing_area
    mean_aerodynamic_chord = (2.0 / 3.0) * c_root * (1.0 + taper + taper**2) / (1.0 + taper)
    semispan = mission.span_m / 2.0
    flap_end_y = semispan * flap_span_fraction
    aileron_start_y = semispan * (1.0 - aileron_span_fraction)
    opti.subject_to(aileron_start_y >= flap_end_y + 0.08)

    area_unblown = (1.0 - blown_fraction) * wing_area
    area_blown = blown_fraction * wing_area
    lift_remaining = anp.maximum(
        mission.gross_weight_n - q_low * area_unblown * mission.cl_section_ceiling_flapped,
        0.0,
    )
    q_b_required = lift_remaining / (area_blown * mission.cl_section_ceiling_flapped + 1e-9)
    required_veff = anp.sqrt(
        anp.maximum(
            2.0 * q_b_required / mission.air_density_kgpm3,
            mission.low_speed_mps**2,
        )
    )

    cl_required_cruise = mission.gross_weight_n / (q_cruise * wing_area)
    # Oswald efficiency proxy: base 0.80 from Raymer 6th ed. Table 12.6 for rectangular/
    # mildly-tapered wings.  Taper penalty -0.06*(taper-0.45)^2 and washout bonus
    # +0.03*(-washout/5°) are curve-fits to Lifting Line Theory results for similar AR
    # (Prandtl 1918; Glauert 1926); the optimal taper for minimum induced drag at this AR
    # is ~0.4-0.5 (Raymer Fig. 12.5), hence reference taper 0.45.
    oswald_efficiency = (
        0.80                               # Raymer 6th ed. Table 12.6: baseline e for tapered wings
        - 0.06 * (taper - 0.45) ** 2      # taper deviation penalty; optimal taper ≈ 0.45 at AR~6 (Raymer Fig. 12.5)
        + 0.03 * (-washout_deg / 5.0)     # washout bonus; ~5° washout improves spanload toward elliptic (Anderson "Fundamentals of Aerodynamics" 6th ed. Ch. 5)
    )
    induced_drag_factor = 1.0 / (anp.pi * oswald_efficiency * aspect_ratio)
    # Tail arm approximation: fixed offset + MAC-scaling is a proxy for fuselage length;
    # coefficient 0.55 gives arm ≈ 1.19 m at MAC ≈ 0.35 m, consistent with 1.4 m fuselage
    # layout; TODO: update from actual CG/NP analysis
    horizontal_tail_arm = 1.0 + 0.55 * mean_aerodynamic_chord
    horizontal_tail_area = horizontal_tail_volume * wing_area * mean_aerodynamic_chord / horizontal_tail_arm
    vertical_tail_area = vertical_tail_volume * wing_area * mission.span_m / horizontal_tail_arm

    # CD0 proxy: base 0.048 from Drela "Flight Vehicle Aerodynamics" MIT 2014 and
    # Raymer 6th ed. for 2-m span / 5-kg class UAV with fixed landing gear.
    # Area and taper penalties are first-order wetted-area corrections.
    # Taper reference 0.55 minimizes profile drag build-up for this chord distribution
    # (Hoerner "Fluid Dynamic Drag" 1965 Ch. 5).
    # TODO: replace proxy with AeroSandbox viscous panel sweep once geometry is locked.
    cd0_proxy = (
        0.048                                              # base CD0 for 5-kg fixed-wing UAV; Drela (2014) / Raymer 6th ed.
        + 0.020 * (wing_area / mission.wing_area_m2 - 1.0) ** 2  # wetted-area penalty for larger wing; Hoerner (1965)
        + 0.003 * (taper - 0.55) ** 2                     # taper deviation from drag-minimum taper; Hoerner (1965) Ch. 5
        + 0.002 * (washout_deg / 4.0) ** 2                # washout profile drag penalty (viscous); empirical estimate
    )
    # Tail parasite drag coefficient 0.0015 × (S_H + S_V) / S_W is consistent with
    # Raymer 6th ed. Eq. 12.21 component buildup for thin horizontal/vertical tail surfaces.
    cd_proxy = (
        cd0_proxy
        + induced_drag_factor * cl_required_cruise**2
        + 0.0015 * (horizontal_tail_area + vertical_tail_area) / wing_area  # Raymer 6th ed. Eq. 12.21
    )
    cruise_drag_proxy = q_cruise * wing_area * cd_proxy

    flap_average_chord = 0.5 * (c_root + (c_root + (c_tip - c_root) * (flap_end_y / semispan)))
    aileron_average_chord = 0.5 * (
        (c_root + (c_tip - c_root) * (aileron_start_y / semispan))
        + c_tip
    )
    flap_area_proxy = 2.0 * flap_span_fraction * flap_average_chord * flap_chord_fraction
    aileron_area_proxy = 2.0 * aileron_span_fraction * aileron_average_chord * aileron_chord_fraction
    control_area_ratio = (flap_area_proxy + aileron_area_proxy) / wing_area

    # Power scaling exponents: for induced/profile power at low speed, P ∝ T^1.5 (actuator
    # disk theory, Leishman "Principles of Helicopter Aerodynamics" 2nd ed. Ch. 2), which
    # maps to P ∝ Veff^~1.5 near hover; exponent 1.6 is slightly steeper to account for
    # profile drag growth with velocity.  At cruise, P = D × V and D ∝ CL^2/AR + CD0, so
    # P ∝ Drag with minor nonlinearity; exponent 1.12 captures the mild superlinear
    # sensitivity — empirical estimate, TODO: validate against Stage 1 sweep residuals.
    low_speed_power_proxy = baseline_low_speed_power * (required_veff / baseline_required_veff) ** 1.6
    cruise_power_proxy = baseline_cruise_power * (cruise_drag_proxy / baseline_cruise_drag) ** 1.12
    low_speed_margin_bonus = (baseline_actual_veff - required_veff) / baseline_actual_veff

    objective = (
        0.40 * (low_speed_power_proxy / baseline_low_speed_power)
        + 0.28 * (cruise_power_proxy / baseline_cruise_power)
        + 0.14 * (wing_area / mission.wing_area_m2)
        + 0.06 * control_area_ratio
        + 0.05 * (horizontal_tail_area / wing_area)
        + 0.03 * (vertical_tail_area / wing_area)
        + 0.02 * ((taper - 0.58) / 0.18) ** 2
        + 0.02 * ((washout_deg + 2.5) / 2.5) ** 2
        - 0.04 * low_speed_margin_bonus
    )

    opti.subject_to(
        [
            cl_required_cruise <= 1.35,              # cruise CL cap; clean CLmax ~1.4 (Abbott & Von Doenhoff 1959) with 5% margin for gust/maneuver
            required_veff <= baseline_actual_veff,   # geometry must not worsen the blown-lift requirement
            horizontal_tail_area >= 0.10 * wing_area,  # minimum VH constraint; Raymer 6th ed. Table 6.4: VH_min = 0.4 → S_H/S_W ≥ ~0.10 at typical tail arm
            vertical_tail_area >= 0.04 * wing_area,    # minimum VV constraint; Raymer 6th ed. Table 6.4: VV_min = 0.02 → S_V/S_W ≥ ~0.04 at typical tail arm
        ]
    )
    opti.minimize(objective)
    sol = opti.solve(max_iter=400, verbose=False)

    solved = {
        "geometry_score": float(sol(objective)),
        "root_chord_m": float(sol(c_root)),
        "tip_chord_m": float(sol(c_tip)),
        "taper": float(sol(taper)),
        "washout_deg": float(sol(washout_deg)),
        "flap_span_fraction": float(sol(flap_span_fraction)),
        "flap_chord_fraction": float(sol(flap_chord_fraction)),
        "aileron_span_fraction": float(sol(aileron_span_fraction)),
        "aileron_chord_fraction": float(sol(aileron_chord_fraction)),
        "horizontal_tail_volume": float(sol(horizontal_tail_volume)),
        "vertical_tail_volume": float(sol(vertical_tail_volume)),
        "wing_area_m2": float(sol(wing_area)),
        "aspect_ratio": float(sol(aspect_ratio)),
        "mac_m": float(sol(mean_aerodynamic_chord)),
        "flap_end_y_m": float(sol(flap_end_y)),
        "aileron_start_y_m": float(sol(aileron_start_y)),
        "horizontal_tail_area_m2": float(sol(horizontal_tail_area)),
        "vertical_tail_area_m2": float(sol(vertical_tail_area)),
        "tail_arm_m": float(sol(horizontal_tail_arm)),
        "required_veff_mps": float(sol(required_veff)),
        "low_speed_power_proxy_w": float(sol(low_speed_power_proxy)),
        "cruise_power_proxy_w": float(sol(cruise_power_proxy)),
        "cruise_drag_proxy_n": float(sol(cruise_drag_proxy)),
        "cruise_cl_required": float(sol(cl_required_cruise)),
        "control_area_ratio": float(sol(control_area_ratio)),
        "baseline_required_veff_mps": baseline_required_veff,
        "baseline_actual_veff_mps": baseline_actual_veff,
        "baseline_low_speed_power_w": baseline_low_speed_power,
        "baseline_cruise_power_w": baseline_cruise_power,
        "baseline_cruise_drag_n": baseline_cruise_drag,
        "blown_span_fraction": blown_fraction,
    }

    requirement_ratio = (
        (solved["required_veff_mps"] - mission.low_speed_mps)
        / max(baseline_required_veff - mission.low_speed_mps, 1e-6)
    )
    requirement_ratio = min(1.0, max(0.0, requirement_ratio))
    # Flap deflection: base 24° + up to 12° extra proportional to blown-lift requirement;
    # Raymer 6th ed. Ch. 27: plain flaps optimum deflection 20-35° (beyond ~40° drag rises
    # faster than lift); 24-36° range is appropriate for takeoff/low-speed operations.
    solved["recommended_flap_deflection_deg"] = 24.0 + 12.0 * requirement_ratio
    # Aileron ±14°: Raymer 6th ed. Ch. 27: minimum aileron deflection for roll authority
    # typically ±15-25°; 14° is conservative — TODO: verify roll rate meets mission spec
    solved["recommended_aileron_deflection_deg"] = 14.0
    # Elevator ±12°: Raymer 6th ed. Ch. 27: elevator deflection for pitch authority typically
    # ±15-25° for conventional tails; 12° may be insufficient — TODO: verify pitch authority
    # at CG aft limit with blown-wing nose-down pitch break
    solved["recommended_elevator_deflection_deg"] = 12.0
    # Rudder ±18°: Raymer 6th ed. Ch. 27: rudder deflection 20-35° typical; 18° may be
    # insufficient under one-engine-out with asymmetric thrust — TODO: verify with OEI analysis
    solved["recommended_rudder_deflection_deg"] = 18.0
    # Elevator chord fraction 28%: Raymer 6th ed. Table 27.1: elevator cf/c ≈ 25-35%
    solved["elevator_chord_fraction"] = 0.28
    # Rudder chord fraction 30%: Raymer 6th ed. Table 27.1: rudder cf/c ≈ 25-35%
    solved["rudder_chord_fraction"] = 0.30
    solved["status"] = "OPTIMIZED"
    return solved


def _wing_chord_at_y(geometry: dict[str, float], y_abs: float, semispan: float) -> float:
    return geometry["root_chord_m"] + (geometry["tip_chord_m"] - geometry["root_chord_m"]) * (y_abs / semispan)


def _wing_le_at_y(geometry: dict[str, float], y_abs: float, semispan: float) -> float:
    # LE sweep rate: base 0.04 m/m (≈2.3° at b/2=1m) + taper-dependent increment;
    # more-tapered wings have a more swept LE to maintain structural depth at the tip
    # (Raymer 6th ed. Ch. 4 taper/sweep relationship); TODO: compute from actual structural layout
    sweep_le_per_m = 0.04 + 0.06 * (1.0 - geometry["taper"])
    return sweep_le_per_m * y_abs


def build_airplane_geometry(
    mission: Stage1MissionConfig,
    geometry: dict[str, float],
    *,
    aileron_deflection_deg: float = 0.0,
    elevator_deflection_deg: float = 0.0,
    rudder_deflection_deg: float = 0.0,
) -> tuple[Any, dict[str, float]]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb

    airfoil = asb.Airfoil("s1210")
    semispan = mission.span_m / 2.0
    flap_end_y = geometry["flap_end_y_m"]
    aileron_start_y = geometry["aileron_start_y_m"]

    root_x = 0.0
    mid_x = _wing_le_at_y(geometry, flap_end_y, semispan)
    outer_x = _wing_le_at_y(geometry, aileron_start_y, semispan)
    tip_x = _wing_le_at_y(geometry, semispan, semispan)

    wing = asb.Wing(
        name="Main Wing",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[root_x, 0.0, 0.0],
                chord=geometry["root_chord_m"],
                twist=0.0,
                airfoil=airfoil,
            ),
            asb.WingXSec(
                xyz_le=[mid_x, flap_end_y, 0.0],
                chord=_wing_chord_at_y(geometry, flap_end_y, semispan),
                twist=geometry["washout_deg"] * flap_end_y / semispan,
                airfoil=airfoil,
            ),
            asb.WingXSec(
                xyz_le=[outer_x, aileron_start_y, 0.0],
                chord=_wing_chord_at_y(geometry, aileron_start_y, semispan),
                twist=geometry["washout_deg"] * aileron_start_y / semispan,
                airfoil=airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="aileron",
                        symmetric=False,
                        hinge_point=1.0 - geometry["aileron_chord_fraction"],
                        deflection=aileron_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[tip_x, semispan, 0.0],
                chord=geometry["tip_chord_m"],
                twist=geometry["washout_deg"],
                airfoil=airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="aileron",
                        symmetric=False,
                        hinge_point=1.0 - geometry["aileron_chord_fraction"],
                        deflection=aileron_deflection_deg,
                    )
                ],
            ),
        ],
    )

    htail_area = geometry["horizontal_tail_area_m2"]
    htail_aspect_ratio = 4.2   # Raymer 6th ed. Ch. 4: H-tail AR typically 3-6; 4.2 is mid-range for conventional UAV tails; TODO: update from detailed tail sizing
    htail_taper = 0.75         # Raymer 6th ed. Ch. 4: H-tail taper 0.6-0.9 typical; 0.75 is standard value for symmetric airfoil tails
    htail_span = math.sqrt(max(htail_aspect_ratio * htail_area, 1e-6))
    htail_root_chord = 2.0 * htail_area / (htail_span * (1.0 + htail_taper))
    htail_tip_chord = htail_root_chord * htail_taper
    htail_root_x = 0.25 * geometry["mac_m"] + geometry["tail_arm_m"] - 0.25 * htail_root_chord
    htail_tip_x = htail_root_x + 0.04 * (htail_span / 2.0)  # LE sweep rate 0.04 m/m; slight sweep for structural efficiency

    horizontal_tail = asb.Wing(
        name="Horizontal Tail",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[htail_root_x, 0.0, 0.0],
                chord=htail_root_chord,
                airfoil=asb.Airfoil("naca0012"),
                control_surfaces=[
                    asb.ControlSurface(
                        name="elevator",
                        symmetric=True,
                        hinge_point=1.0 - geometry["elevator_chord_fraction"],
                        deflection=elevator_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[htail_tip_x, htail_span / 2.0, 0.0],
                chord=htail_tip_chord,
                airfoil=asb.Airfoil("naca0012"),
                control_surfaces=[
                    asb.ControlSurface(
                        name="elevator",
                        symmetric=True,
                        hinge_point=1.0 - geometry["elevator_chord_fraction"],
                        deflection=elevator_deflection_deg,
                    )
                ],
            ),
        ],
    )

    vtail_area = geometry["vertical_tail_area_m2"]
    vtail_aspect_ratio = 1.6   # Raymer 6th ed. Ch. 4: V-tail AR typically 1.5-2.5; 1.6 is low end for a compact fuselage; TODO: update from detailed tail sizing
    vtail_taper = 0.70         # Raymer 6th ed. Ch. 4: V-tail taper 0.6-0.8 typical; 0.70 gives reasonable tip chord for rudder effectiveness
    vtail_height = math.sqrt(max(vtail_aspect_ratio * vtail_area, 1e-6))
    vtail_root_chord = 2.0 * vtail_area / (vtail_height * (1.0 + vtail_taper))
    vtail_tip_chord = vtail_root_chord * vtail_taper
    vtail_root_x = htail_root_x - 0.03  # V-tail root starts 30 mm ahead of H-tail root; accounts for fuselage attachment geometry
    vtail_tip_x = vtail_root_x + 0.06 * vtail_height  # LE sweep rate 0.06 m/m; slightly more swept than H-tail for structural depth

    vertical_tail = asb.Wing(
        name="Vertical Tail",
        symmetric=False,
        xsecs=[
            asb.WingXSec(
                xyz_le=[vtail_root_x, 0.0, 0.0],
                chord=vtail_root_chord,
                airfoil=asb.Airfoil("naca0012"),
                control_surfaces=[
                    asb.ControlSurface(
                        name="rudder",
                        symmetric=False,
                        hinge_point=1.0 - geometry["rudder_chord_fraction"],
                        deflection=rudder_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[vtail_tip_x, 0.0, vtail_height],
                chord=vtail_tip_chord,
                airfoil=asb.Airfoil("naca0012"),
                control_surfaces=[
                    asb.ControlSurface(
                        name="rudder",
                        symmetric=False,
                        hinge_point=1.0 - geometry["rudder_chord_fraction"],
                        deflection=rudder_deflection_deg,
                    )
                ],
            ),
        ],
    )

    fuselage_width = mission.fuselage_width_m
    fuselage_height = 0.14
    fuselage_end_x = max(vtail_root_x + 0.85 * vtail_root_chord, 1.40)
    fuselage = asb.Fuselage(
        name="Fuselage",
        xsecs=[
            asb.FuselageXSec(xyz_c=[-0.08, 0.0, 0.0], radius=0.025),
            asb.FuselageXSec(xyz_c=[0.22, 0.0, 0.0], width=fuselage_width, height=fuselage_height),
            asb.FuselageXSec(xyz_c=[0.95, 0.0, 0.0], width=0.11, height=0.10),
            asb.FuselageXSec(xyz_c=[fuselage_end_x, 0.0, 0.02], radius=0.016),
        ],
    )

    airplane = asb.Airplane(
        name="AA146 Stage 3 Refined Concept",
        xyz_ref=[0.25 * geometry["mac_m"], 0.0, 0.0],
        wings=[wing, horizontal_tail, vertical_tail],
        fuselages=[fuselage],
        s_ref=wing.area(),
        c_ref=wing.mean_aerodynamic_chord(),
        b_ref=wing.span(),
    )

    meta = {
        "htail_span_m": htail_span,
        "htail_root_chord_m": htail_root_chord,
        "htail_tip_chord_m": htail_tip_chord,
        "vtail_height_m": vtail_height,
        "vtail_root_chord_m": vtail_root_chord,
        "vtail_tip_chord_m": vtail_tip_chord,
        "fuselage_end_x_m": fuselage_end_x,
    }
    return airplane, meta


def _run_vlm(
    airplane: Any,
    *,
    velocity_mps: float,
    alpha_deg: float,
) -> dict[str, float]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb
    results = asb.VortexLatticeMethod(
        airplane=airplane,
        op_point=asb.OperatingPoint(
            velocity=velocity_mps,
            alpha=float(alpha_deg),
        ),
        spanwise_resolution=8,
        chordwise_resolution=6,
        run_symmetric_if_possible=False,
        verbose=False,
    ).run()
    normalized: dict[str, Any] = {}
    for key, value in results.items():
        try:
            normalized[key] = float(value)
        except (TypeError, ValueError):
            normalized[key] = runtime.onp.asarray(value, dtype=float).tolist()
    return normalized


def solve_cruise_alpha(
    mission: Stage1MissionConfig,
    airplane_builder,
) -> tuple[float, dict[str, float]]:
    lower_alpha = -2.0
    upper_alpha = 10.0

    def residual(alpha_deg: float) -> tuple[float, dict[str, float]]:
        airplane, _ = airplane_builder(
            aileron_deflection_deg=0.0,
            elevator_deflection_deg=0.0,
            rudder_deflection_deg=0.0,
        )
        results = _run_vlm(
            airplane,
            velocity_mps=mission.cruise_speed_mps,
            alpha_deg=alpha_deg,
        )
        return results["L"] - mission.gross_weight_n, results

    lower_residual, lower_results = residual(lower_alpha)
    upper_residual, upper_results = residual(upper_alpha)
    while lower_residual * upper_residual > 0.0 and upper_alpha < 18.0:
        upper_alpha += 2.0
        upper_residual, upper_results = residual(upper_alpha)

    if lower_residual * upper_residual > 0.0:
        if abs(lower_residual) < abs(upper_residual):
            return lower_alpha, lower_results
        return upper_alpha, upper_results

    for _ in range(24):
        mid_alpha = 0.5 * (lower_alpha + upper_alpha)
        mid_residual, mid_results = residual(mid_alpha)
        if abs(mid_residual) < 0.25:
            return mid_alpha, mid_results
        if lower_residual * mid_residual <= 0.0:
            upper_alpha = mid_alpha
            upper_residual = mid_residual
            upper_results = mid_results
        else:
            lower_alpha = mid_alpha
            lower_residual = mid_residual
            lower_results = mid_results

    return 0.5 * (lower_alpha + upper_alpha), mid_results


def evaluate_section_polars(
    mission: Stage1MissionConfig,
    geometry: dict[str, float],
) -> dict[str, Any]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb
    onp = runtime.onp

    airfoil = asb.Airfoil("s1210")
    alphas_low = onp.linspace(-4.0, 24.0, 90)
    alphas_cruise = onp.linspace(-4.0, 14.0, 70)
    re_low = (
        mission.air_density_kgpm3
        * max(geometry["required_veff_mps"], mission.low_speed_mps)
        * geometry["mac_m"]
        / mission.dynamic_viscosity_pas
    )
    re_cruise = (
        mission.air_density_kgpm3
        * mission.cruise_speed_mps
        * geometry["mac_m"]
        / mission.dynamic_viscosity_pas
    )

    clean_low = airfoil.get_aero_from_neuralfoil(alpha=alphas_low, Re=re_low)
    flap_low = airfoil.get_aero_from_neuralfoil(
        alpha=alphas_low,
        Re=re_low,
        control_surfaces=[
            asb.ControlSurface(
                name="flap",
                symmetric=True,
                deflection=geometry["recommended_flap_deflection_deg"],
                hinge_point=1.0 - geometry["flap_chord_fraction"],
            )
        ],
    )
    clean_cruise = airfoil.get_aero_from_neuralfoil(alpha=alphas_cruise, Re=re_cruise)

    clean_cl = runtime.onp.asarray(clean_low["CL"], dtype=float)
    flap_cl = runtime.onp.asarray(flap_low["CL"], dtype=float)
    return {
        "alphas_low_deg": alphas_low,
        "alphas_cruise_deg": alphas_cruise,
        "clean_low": clean_low,
        "flap_low": flap_low,
        "clean_cruise": clean_cruise,
        "re_low": float(re_low),
        "re_cruise": float(re_cruise),
        "clean_section_clmax_raw": float(clean_cl.max()),
        "flapped_section_clmax_raw": float(flap_cl.max()),
        "flap_delta_cl_raw": float(flap_cl.max() - clean_cl.max()),
    }


def render_top_view(
    mission: Stage1MissionConfig,
    geometry: dict[str, float],
    stage2_row: dict[str, str],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    patches = runtime.patches

    semispan = mission.span_m / 2.0
    y_vals = [
        0.0,
        geometry["flap_end_y_m"],
        geometry["aileron_start_y_m"],
        semispan,
    ]

    def le(y_abs: float) -> float:
        return _wing_le_at_y(geometry, y_abs, semispan)

    def te(y_abs: float) -> float:
        return le(y_abs) + _wing_chord_at_y(geometry, y_abs, semispan)

    wing_poly_right = [
        (le(0.0), 0.0),
        (le(semispan), semispan),
        (te(semispan), semispan),
        (te(0.0), 0.0),
    ]
    wing_poly_left = [(x, -y) for x, y in reversed(wing_poly_right)]

    flap_poly_right = [
        (te(0.0) - geometry["flap_chord_fraction"] * _wing_chord_at_y(geometry, 0.0, semispan), 0.0),
        (te(geometry["flap_end_y_m"]) - geometry["flap_chord_fraction"] * _wing_chord_at_y(geometry, geometry["flap_end_y_m"], semispan), geometry["flap_end_y_m"]),
        (te(geometry["flap_end_y_m"]), geometry["flap_end_y_m"]),
        (te(0.0), 0.0),
    ]
    flap_poly_left = [(x, -y) for x, y in reversed(flap_poly_right)]

    aileron_poly_right = [
        (
            te(geometry["aileron_start_y_m"]) - geometry["aileron_chord_fraction"] * _wing_chord_at_y(geometry, geometry["aileron_start_y_m"], semispan),
            geometry["aileron_start_y_m"],
        ),
        (
            te(semispan) - geometry["aileron_chord_fraction"] * _wing_chord_at_y(geometry, semispan, semispan),
            semispan,
        ),
        (te(semispan), semispan),
        (te(geometry["aileron_start_y_m"]), geometry["aileron_start_y_m"]),
    ]
    aileron_poly_left = [(x, -y) for x, y in reversed(aileron_poly_right)]

    fig, ax = plt.subplots(figsize=(10, 5))
    for poly, color, alpha, label in [
        (wing_poly_right, "#dbe4ee", 0.95, "Wing"),
        (wing_poly_left, "#dbe4ee", 0.95, None),
        (flap_poly_right, "#f4a261", 0.80, "Flap"),
        (flap_poly_left, "#f4a261", 0.80, None),
        (aileron_poly_right, "#457b9d", 0.80, "Aileron"),
        (aileron_poly_left, "#457b9d", 0.80, None),
    ]:
        ax.add_patch(
            patches.Polygon(
                poly,
                closed=True,
                facecolor=color,
                edgecolor="#1f2937",
                linewidth=1.2,
                alpha=alpha,
                label=label,
            )
        )

    # Horizontal and vertical tail projections.
    htail_span = geometry["htail_span_m"]
    htail_root_chord = geometry["htail_root_chord_m"]
    htail_tip_chord = geometry["htail_tip_chord_m"]
    htail_root_x = 0.25 * geometry["mac_m"] + geometry["tail_arm_m"] - 0.25 * htail_root_chord
    htail_tip_x = htail_root_x + 0.04 * (htail_span / 2.0)
    htail_poly_right = [
        (htail_root_x, 0.0),
        (htail_tip_x, htail_span / 2.0),
        (htail_tip_x + htail_tip_chord, htail_span / 2.0),
        (htail_root_x + htail_root_chord, 0.0),
    ]
    htail_poly_left = [(x, -y) for x, y in reversed(htail_poly_right)]
    for poly in [htail_poly_right, htail_poly_left]:
        ax.add_patch(
            patches.Polygon(
                poly,
                closed=True,
                facecolor="#b8e0d2",
                edgecolor="#1f2937",
                linewidth=1.0,
                alpha=0.80,
            )
        )

    fuselage_end_x = geometry["fuselage_end_x_m"]
    fuselage_half_width = mission.fuselage_width_m / 2.0
    ax.add_patch(
        patches.FancyBboxPatch(
            (-0.08, -fuselage_half_width),
            fuselage_end_x + 0.08,
            2.0 * fuselage_half_width,
            boxstyle="round,pad=0.02,rounding_size=0.03",
            linewidth=1.2,
            edgecolor="#2d3748",
            facecolor="#f7fafc",
            alpha=0.9,
        )
    )

    prop_radius = _safe_float(stage2_row["prop_diameter_in"]) * 0.0254 / 2.0
    for y_center in parse_prop_centers(stage2_row["prop_centers_m"]):
        disk = patches.Circle(
            (0.18, y_center),
            prop_radius,
            facecolor="none",
            edgecolor="#7c3aed",
            linestyle="--",
            linewidth=1.0,
            alpha=0.75,
        )
        ax.add_patch(disk)

    ax.text(
        0.02,
        0.98,
        (
            f"S={geometry['wing_area_m2']:.3f} m²\n"
            f"λ={geometry['taper']:.2f}\n"
            f"washout={geometry['washout_deg']:.1f}°\n"
            f"Veff,req={geometry['required_veff_mps']:.2f} m/s"
        ),
        transform=ax.transAxes,
        ha="left",
        va="top",
        bbox=dict(facecolor="white", edgecolor="#cbd5e1", alpha=0.9),
    )
    ax.set_aspect("equal")
    ax.set_xlim(-0.12, max(fuselage_end_x + 0.1, te(semispan) + 0.15))
    ax.set_ylim(-1.1, 1.1)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Stage 3 Refined Planform and Prop Layout")
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc="lower right")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def render_three_view_and_wireframe(
    airplane: Any,
    three_view_path: Path,
    wireframe_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    axs = airplane.draw_three_view(show=False, style="wireframe")
    axs[0, 0].figure.savefig(three_view_path, dpi=180, bbox_inches="tight")
    plt.close(axs[0, 0].figure)

    ax = airplane.draw_wireframe(show=False)
    ax.figure.savefig(wireframe_path, dpi=180, bbox_inches="tight")
    plt.close(ax.figure)


def render_polar_plot(
    polars: dict[str, Any],
    cruise_alpha_deg: float,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    onp = runtime.onp

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.5))

    axs[0].plot(polars["alphas_low_deg"], onp.asarray(polars["clean_low"]["CL"], dtype=float), label="Clean @ low Re", color="#1d3557")
    axs[0].plot(polars["alphas_low_deg"], onp.asarray(polars["flap_low"]["CL"], dtype=float), label="Flapped @ low Re", color="#e76f51")
    axs[0].plot(polars["alphas_cruise_deg"], onp.asarray(polars["clean_cruise"]["CL"], dtype=float), label="Clean @ cruise Re", color="#2a9d8f")
    axs[0].axvline(cruise_alpha_deg, color="#6b7280", linestyle="--", linewidth=1.0, label="Cruise alpha")
    axs[0].set_xlabel("Alpha [deg]")
    axs[0].set_ylabel("Section CL")
    axs[0].set_title("S1210 Section Lift Curves")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend()

    axs[1].plot(polars["alphas_low_deg"], onp.asarray(polars["clean_low"]["CD"], dtype=float), label="Clean @ low Re", color="#1d3557")
    axs[1].plot(polars["alphas_low_deg"], onp.asarray(polars["flap_low"]["CD"], dtype=float), label="Flapped @ low Re", color="#e76f51")
    axs[1].plot(polars["alphas_cruise_deg"], onp.asarray(polars["clean_cruise"]["CD"], dtype=float), label="Clean @ cruise Re", color="#2a9d8f")
    axs[1].set_xlabel("Alpha [deg]")
    axs[1].set_ylabel("Section CD")
    axs[1].set_title("S1210 Section Drag Curves")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend()

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def write_trade_space_plot(results: list[dict[str, Any]], output_path: Path) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    successful = [row for row in results if row["status"] == "SUCCESS"]
    if not successful:
        return

    fig, ax = plt.subplots(figsize=(7.5, 5.5))
    x = [_safe_float(row["cruise_drag_n"]) for row in successful]
    y = [_safe_float(row["refined_low_speed_required_veff_mps"]) for row in successful]
    s = [220.0 * _safe_float(row["optimized_wing_area_m2"]) / 0.70 for row in successful]
    c = [_safe_float(row["n_props"]) for row in successful]
    scatter = ax.scatter(x, y, s=s, c=c, cmap="viridis", alpha=0.85, edgecolor="#1f2937")

    for row in successful[:10]:
        ax.annotate(
            f"#{row['rank']}",
            (_safe_float(row["cruise_drag_n"]), _safe_float(row["refined_low_speed_required_veff_mps"])),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
        )

    colorbar = fig.colorbar(scatter, ax=ax)
    colorbar.set_label("Prop count")
    ax.set_xlabel("Cruise drag [N]")
    ax.set_ylabel("Required blown velocity [m/s]")
    ax.set_title("Stage 3 Trade Space")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def write_gallery(results: list[dict[str, Any]], output_path: Path) -> None:
    successful = [row for row in results if row["status"] == "SUCCESS"]
    lines = [
        "# Stage 3 Gallery",
        "",
        "This gallery is generated from the current Stage 3 AeroSandbox refinement pass.",
        "",
    ]
    for row in successful[:5]:
        lines.extend(
            [
                f"## Rank {row['rank']}: {row['n_props']} props, {row['prop_diameter_in']} in, {row['prop_family']}",
                "",
                f"- Wing area: `{row['optimized_wing_area_m2']} m^2`",
                f"- Taper: `{row['optimized_taper']}`",
                f"- Required blown velocity: `{row['refined_low_speed_required_veff_mps']} m/s`",
                f"- Cruise drag: `{row['cruise_drag_n']} N`",
                f"- Cruise alpha: `{row['cruise_alpha_deg']} deg`",
                "",
                f"![Top view]({Path(str(row['top_view_png'])).name})",
                "",
                f"![Three view]({Path(str(row['three_view_png'])).name})",
                "",
                f"![Section polars]({Path(str(row['polar_png'])).name})",
                "",
            ]
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")


def refine_stage3_candidate(
    mission: Stage1MissionConfig,
    stage2_row: dict[str, str],
    stage1_row: dict[str, str],
    *,
    rank_seed: int,
) -> dict[str, Any]:
    geometry = optimize_stage3_geometry(mission, stage2_row, stage1_row)

    def airplane_builder(
        *,
        aileron_deflection_deg: float = 0.0,
        elevator_deflection_deg: float = 0.0,
        rudder_deflection_deg: float = 0.0,
    ):
        return build_airplane_geometry(
            mission,
            geometry,
            aileron_deflection_deg=aileron_deflection_deg,
            elevator_deflection_deg=elevator_deflection_deg,
            rudder_deflection_deg=rudder_deflection_deg,
        )

    cruise_alpha_deg, neutral_results = solve_cruise_alpha(mission, airplane_builder)
    neutral_airplane, airplane_meta = airplane_builder(
        aileron_deflection_deg=0.0,
        elevator_deflection_deg=0.0,
        rudder_deflection_deg=0.0,
    )

    aileron_airplane, _ = airplane_builder(
        aileron_deflection_deg=geometry["recommended_aileron_deflection_deg"],
        elevator_deflection_deg=0.0,
        rudder_deflection_deg=0.0,
    )
    elevator_airplane, _ = airplane_builder(
        aileron_deflection_deg=0.0,
        elevator_deflection_deg=geometry["recommended_elevator_deflection_deg"],
        rudder_deflection_deg=0.0,
    )
    rudder_airplane, _ = airplane_builder(
        aileron_deflection_deg=0.0,
        elevator_deflection_deg=0.0,
        rudder_deflection_deg=geometry["recommended_rudder_deflection_deg"],
    )

    aileron_results = _run_vlm(
        aileron_airplane,
        velocity_mps=mission.cruise_speed_mps,
        alpha_deg=cruise_alpha_deg,
    )
    elevator_results = _run_vlm(
        elevator_airplane,
        velocity_mps=mission.cruise_speed_mps,
        alpha_deg=cruise_alpha_deg,
    )
    rudder_results = _run_vlm(
        rudder_airplane,
        velocity_mps=mission.cruise_speed_mps,
        alpha_deg=cruise_alpha_deg,
    )
    alpha_minus = _run_vlm(
        neutral_airplane,
        velocity_mps=mission.cruise_speed_mps,
        alpha_deg=cruise_alpha_deg - 1.0,
    )
    alpha_plus = _run_vlm(
        neutral_airplane,
        velocity_mps=mission.cruise_speed_mps,
        alpha_deg=cruise_alpha_deg + 1.0,
    )
    cm_alpha_per_deg = 0.5 * (alpha_plus["Cm"] - alpha_minus["Cm"])

    wing = neutral_airplane.wings[0]
    horizontal_tail = neutral_airplane.wings[1]
    vertical_tail = neutral_airplane.wings[2]
    semispan = mission.span_m / 2.0
    aileron_area = float(wing.control_surface_area(by_name="aileron"))
    elevator_area = float(horizontal_tail.control_surface_area(by_name="elevator"))
    rudder_area = float(vertical_tail.control_surface_area(by_name="rudder"))
    aileron_y_bar = 0.5 * (geometry["aileron_start_y_m"] + semispan)
    aileron_proxy = (
        (2.4 / 57.29577951308232)
        * (aileron_area / max(neutral_airplane.s_ref, 1e-9))
        * (aileron_y_bar / max(semispan, 1e-9))
        * (0.30 + 1.10 * geometry["aileron_chord_fraction"])
    )
    elevator_proxy = -(
        (1.9 / 57.29577951308232)
        * (elevator_area / max(neutral_airplane.s_ref, 1e-9))
        * (geometry["tail_arm_m"] / max(geometry["mac_m"], 1e-9))
        * (0.40 + 1.00 * geometry["elevator_chord_fraction"])
    )
    rudder_proxy = (
        (1.7 / 57.29577951308232)
        * (rudder_area / max(neutral_airplane.s_ref, 1e-9))
        * (geometry["tail_arm_m"] / max(mission.span_m, 1e-9))
        * (0.40 + 1.00 * geometry["rudder_chord_fraction"])
    )

    aileron_vlm = (aileron_results["Cl"] - neutral_results["Cl"]) / geometry["recommended_aileron_deflection_deg"]
    elevator_vlm = (elevator_results["Cm"] - neutral_results["Cm"]) / geometry["recommended_elevator_deflection_deg"]
    rudder_vlm = (rudder_results["Cn"] - neutral_results["Cn"]) / geometry["recommended_rudder_deflection_deg"]
    aileron_authority = aileron_vlm if abs(aileron_vlm) > 1e-6 else aileron_proxy
    elevator_authority = elevator_vlm if abs(elevator_vlm) > 1e-6 else elevator_proxy
    rudder_authority = rudder_vlm if abs(rudder_vlm) > 1e-6 else rudder_proxy

    polars = evaluate_section_polars(mission, geometry)

    slug = (
        f"rank{rank_seed:02d}_n{int(float(stage2_row['n_props']))}"
        f"_d{float(stage2_row['prop_diameter_in']):.1f}"
        f"_pd{float(stage2_row['prop_pitch_ratio']):.2f}"
        f"_{stage2_row['prop_family']}"
    ).replace(".", "p")
    top_view_path = STAGE3_VISUAL_DIR / f"{slug}_top_view.png"
    three_view_path = STAGE3_VISUAL_DIR / f"{slug}_three_view.png"
    wireframe_path = STAGE3_VISUAL_DIR / f"{slug}_wireframe.png"
    polar_path = STAGE3_VISUAL_DIR / f"{slug}_polars.png"
    mesh_path = STAGE3_VISUAL_DIR / f"{slug}_mesh.npz"

    render_top_view(mission, {**geometry, **airplane_meta}, stage2_row, top_view_path)
    render_three_view_and_wireframe(neutral_airplane, three_view_path, wireframe_path)
    render_polar_plot(polars, cruise_alpha_deg, polar_path)

    runtime = ensure_stage3_runtime()
    points, faces = neutral_airplane.mesh_body(method="quad", thin_wings=True, stack_meshes=True)
    runtime.onp.savez(mesh_path, points=runtime.onp.asarray(points), faces=runtime.onp.asarray(faces))

    return {
        "rank": 0,
        "status": "SUCCESS",
        "n_props": int(float(stage2_row["n_props"])),
        "prop_diameter_in": float(stage2_row["prop_diameter_in"]),
        "prop_pitch_ratio": float(stage2_row["prop_pitch_ratio"]),
        "prop_family": stage2_row["prop_family"],
        "seed_low_speed_rpm": _safe_float(stage2_row["solved_low_speed_rpm"]),
        "seed_cruise_rpm": _safe_float(stage2_row["solved_cruise_rpm"]),
        "blown_span_fraction": geometry["blown_span_fraction"],
        "baseline_required_veff_mps": geometry["baseline_required_veff_mps"],
        "baseline_actual_veff_mps": geometry["baseline_actual_veff_mps"],
        "baseline_low_speed_power_w": geometry["baseline_low_speed_power_w"],
        "baseline_cruise_power_w": geometry["baseline_cruise_power_w"],
        "baseline_cruise_drag_n": geometry["baseline_cruise_drag_n"],
        "optimized_root_chord_m": geometry["root_chord_m"],
        "optimized_tip_chord_m": geometry["tip_chord_m"],
        "optimized_taper": geometry["taper"],
        "optimized_wing_area_m2": geometry["wing_area_m2"],
        "optimized_mac_m": geometry["mac_m"],
        "optimized_aspect_ratio": geometry["aspect_ratio"],
        "optimized_washout_deg": geometry["washout_deg"],
        "optimized_flap_span_fraction": geometry["flap_span_fraction"],
        "optimized_flap_chord_fraction": geometry["flap_chord_fraction"],
        "recommended_flap_deflection_deg": geometry["recommended_flap_deflection_deg"],
        "optimized_aileron_span_fraction": geometry["aileron_span_fraction"],
        "optimized_aileron_chord_fraction": geometry["aileron_chord_fraction"],
        "recommended_aileron_deflection_deg": geometry["recommended_aileron_deflection_deg"],
        "horizontal_tail_area_m2": geometry["horizontal_tail_area_m2"],
        "vertical_tail_area_m2": geometry["vertical_tail_area_m2"],
        "tail_arm_m": geometry["tail_arm_m"],
        "elevator_chord_fraction": geometry["elevator_chord_fraction"],
        "rudder_chord_fraction": geometry["rudder_chord_fraction"],
        "refined_low_speed_cl_required": mission.gross_weight_n / (0.5 * mission.air_density_kgpm3 * mission.low_speed_mps**2 * geometry["wing_area_m2"]),
        "refined_low_speed_required_veff_mps": geometry["required_veff_mps"],
        "refined_low_speed_power_proxy_w": geometry["low_speed_power_proxy_w"],
        "propulsor_veff_margin_mps": geometry["baseline_actual_veff_mps"] - geometry["required_veff_mps"],
        "cruise_alpha_deg": cruise_alpha_deg,
        "cruise_lift_n": neutral_results["L"],
        "cruise_drag_n": neutral_results["D"],
        "cruise_cl": neutral_results["CL"],
        "cruise_cd": neutral_results["CD"],
        "cruise_cm": neutral_results["Cm"],
        "static_cm_alpha_per_deg": cm_alpha_per_deg,
        "section_re_low_speed": polars["re_low"],
        "section_re_cruise": polars["re_cruise"],
        "clean_section_clmax_raw": polars["clean_section_clmax_raw"],
        "flapped_section_clmax_raw": polars["flapped_section_clmax_raw"],
        "flap_delta_cl_raw": polars["flap_delta_cl_raw"],
        "aileron_delta_cl_per_deg": aileron_authority,
        "elevator_delta_cm_per_deg": elevator_authority,
        "rudder_delta_cn_per_deg": rudder_authority,
        "geometry_score": geometry["geometry_score"],
        "top_view_png": str(top_view_path),
        "three_view_png": str(three_view_path),
        "wireframe_png": str(wireframe_path),
        "polar_png": str(polar_path),
        "mesh_npz": str(mesh_path),
    }
