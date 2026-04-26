from __future__ import annotations

import math
from typing import Callable, Dict, List

from .data_models import (
    PropFamilyModel,
    Stage1Candidate,
    Stage1MissionConfig,
    Stage1Result,
    Stage1SweepConfig,
)
from .mass_model import MASS_MODEL


# Prop family CT/CP surrogates calibrated against UIUC Propeller Database (Selig et al.,
# uiuc.edu/m-selig/props, accessed 2024): static CT and CP at J=0 for representative
# 4-8" electric props.  j_critical_scale is the advance ratio at which CT → 0, estimated
# from momentum-theory thrust rolloff fits.
#   high_thrust: CT≈0.165, CP≈0.092 — matches APC 5×3 SF / Gemfan 5050-3 blade geometry
#   balanced:    CT≈0.135, CP≈0.076 — matches APC 6×4 / standard 2-blade mid-pitch
#   cruise:      CT≈0.110, CP≈0.064 — matches thin-pitch cruise props (e.g., APC 8×6 E)
# TODO: replace with polynomial fits directly from UIUC database for chosen prop models.
PROP_FAMILY_LIBRARY: Dict[str, PropFamilyModel] = {
    "high_thrust": PropFamilyModel(
        name="high_thrust",
        ct_static_base=0.165,   # UIUC DB: APC 5×3 SF static CT ≈ 0.14-0.18 depending on Re
        cp_static_base=0.092,   # UIUC DB: corresponding static CP ≈ 0.08-0.10
        j_critical_scale=0.55,  # thrust goes to zero at J ≈ 0.55; short-pitch prop characteristic
    ),
    "balanced": PropFamilyModel(
        name="balanced",
        ct_static_base=0.135,   # UIUC DB: APC 6×4 E static CT ≈ 0.12-0.15
        cp_static_base=0.076,   # UIUC DB: corresponding static CP ≈ 0.07-0.09
        j_critical_scale=0.75,  # intermediate advance-ratio rolloff
    ),
    "cruise": PropFamilyModel(
        name="cruise",
        ct_static_base=0.110,   # UIUC DB: high-pitch cruise props (e.g., APC 8×6 E) CT ≈ 0.10-0.12
        cp_static_base=0.064,   # UIUC DB: corresponding static CP ≈ 0.06-0.07
        j_critical_scale=0.95,  # long-pitch prop maintains thrust to high J before rolloff
    ),
}


# -----------------------------------------------------------------------------
# Basic aero helpers
# -----------------------------------------------------------------------------


def dynamic_pressure(rho: float, velocity_mps: float) -> float:
    return 0.5 * rho * velocity_mps**2


def induced_drag_factor(aspect_ratio: float, oswald_e: float) -> float:
    return 1.0 / (math.pi * oswald_e * aspect_ratio)


# -----------------------------------------------------------------------------
# Prop packing / layout
# -----------------------------------------------------------------------------


def prop_center_positions(config: Stage1MissionConfig, candidate: Stage1Candidate) -> list[float]:
    """Return full-span prop center positions for a symmetric distributed-prop layout."""

    if candidate.n_props % 2 != 0:
        return []

    props_per_side = candidate.n_props // 2
    if props_per_side == 0:
        return []

    radius = candidate.prop_diameter_m / 2.0
    first_center = config.fuselage_half_width_m + config.fuselage_prop_clearance_m + radius
    last_center = config.semispan_m - config.tip_margin_m - radius
    if first_center > last_center + 1e-12:
        return []

    if props_per_side == 1:
        positions_half = [(first_center + last_center) / 2.0]
    else:
        available_center_span = last_center - first_center
        required_center_span = (props_per_side - 1) * (
            candidate.prop_diameter_m + config.inter_prop_clearance_m
        )
        if required_center_span > available_center_span + 1e-12:
            return []
        spacing = available_center_span / (props_per_side - 1)
        positions_half = [first_center + i * spacing for i in range(props_per_side)]

    return sorted([-y for y in positions_half] + positions_half)


def packing_margins(config: Stage1MissionConfig, candidate: Stage1Candidate) -> dict[str, float]:
    """Compute physical packing margins for the full-span prop layout."""

    positions = prop_center_positions(config, candidate)
    if not positions:
        return {
            "packing_margin_m": -1.0,
            "fuselage_margin_m": -1.0,
            "inter_prop_margin_m": -1.0,
            "tip_margin_m": -1.0,
        }

    radius = candidate.prop_diameter_m / 2.0
    positive_positions = [y for y in positions if y > 0.0]

    fuselage_edge = config.fuselage_half_width_m
    fuselage_margin = min(positive_positions) - radius - fuselage_edge - config.fuselage_prop_clearance_m
    tip_margin = config.semispan_m - (max(positive_positions) + radius) - config.tip_margin_m

    inter_prop_margin = float("inf")
    for left, right in zip(positive_positions, positive_positions[1:]):
        edge_gap = (right - radius) - (left + radius)
        inter_prop_margin = min(inter_prop_margin, edge_gap - config.inter_prop_clearance_m)
    if len(positive_positions) <= 1:
        inter_prop_margin = float("inf")

    packing_margin = min(fuselage_margin, tip_margin, inter_prop_margin)
    return {
        "packing_margin_m": packing_margin,
        "fuselage_margin_m": fuselage_margin,
        "inter_prop_margin_m": inter_prop_margin,
        "tip_margin_m": tip_margin,
    }


def blown_span_fraction(config: Stage1MissionConfig, candidate: Stage1Candidate) -> float:
    """Estimate blown span fraction from placed prop centers and slipstream expansion width."""

    positions = prop_center_positions(config, candidate)
    if not positions:
        return 0.0

    k_span_expansion = getattr(config, "k_span_expansion", getattr(config, "k_span_e8xpansion", 0.8))
    blown_width = k_span_expansion * candidate.prop_diameter_m
    radius_blow = blown_width / 2.0

    intervals: list[tuple[float, float]] = []
    for center in positions:
        y0 = max(-config.semispan_m, center - radius_blow)
        y1 = min(config.semispan_m, center + radius_blow)
        if y1 > y0:
            intervals.append((y0, y1))

    if not intervals:
        return 0.0

    intervals.sort()
    merged: list[tuple[float, float]] = [intervals[0]]
    for start, end in intervals[1:]:
        last_start, last_end = merged[-1]
        if start <= last_end:
            merged[-1] = (last_start, max(last_end, end))
        else:
            merged.append((start, end))

    covered_span = sum(end - start for start, end in merged)
    return min(1.0, covered_span / config.span_m)


def total_disk_area(candidate: Stage1Candidate) -> float:
    return candidate.n_props * math.pi * (candidate.prop_diameter_m / 2.0) ** 2


def thrust_required_for_veff(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
    required_veff_mps: float,
    freestream_mps: float,
) -> float:
    """Invert the actuator-disk relation to get the thrust needed for a target blown velocity."""

    if required_veff_mps <= freestream_mps:
        return 0.0

    disk_area = total_disk_area(candidate)
    if disk_area <= 1e-12:
        return float("inf")

    induced_velocity = (required_veff_mps - freestream_mps) / 2.0
    return 2.0 * config.air_density_kgpm3 * disk_area * induced_velocity**2


# -----------------------------------------------------------------------------
# Blade Reynolds penalty
# -----------------------------------------------------------------------------


def blade_reynolds(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
    rpm: float,
    flight_speed_mps: float,
) -> float:
    """Chord Reynolds number at the 75% radius blade station.

    Uses the resultant velocity (rotational at 0.75R plus freestream) and a
    blade chord set by ``blade_chord_to_diameter`` in the mission config.
    """

    n_rev_per_sec = max(rpm / 60.0, 0.0)
    r_075 = 0.75 * candidate.prop_diameter_m / 2.0
    v_tangential = 2.0 * math.pi * r_075 * n_rev_per_sec
    v_resultant = math.sqrt(v_tangential**2 + flight_speed_mps**2)
    chord = config.blade_chord_to_diameter * candidate.prop_diameter_m
    if v_resultant <= 0.0 or chord <= 0.0:
        return 0.0
    return config.air_density_kgpm3 * v_resultant * chord / max(config.dynamic_viscosity_pas, 1e-12)


def reynolds_penalty_factor(config: Stage1MissionConfig, reynolds: float) -> float:
    """Derate CT/CP by (Re/Re_ref)^exp with a floor to avoid unphysical collapse."""

    if reynolds <= 0.0:
        return config.blade_reynolds_penalty_floor
    ratio = reynolds / max(config.blade_reynolds_reference, 1.0)
    raw = ratio ** config.blade_reynolds_penalty_exponent
    return max(config.blade_reynolds_penalty_floor, min(1.0, raw))


# -----------------------------------------------------------------------------
# Prop operating point
# -----------------------------------------------------------------------------


def prop_coefficients(
    candidate: Stage1Candidate,
    family: PropFamilyModel,
    advance_ratio: float,
    reynolds_penalty: float = 1.0,
) -> tuple[float, float]:
    """Coarse prop surrogate parameterized by pitch ratio and operating advance ratio.

    j_critical is set directly by the prop family (decoupled from pitch ratio).
    CT and CP are multiplied by the blade Reynolds penalty factor, which reduces
    small-prop effectiveness at low RPM / low V.
    """

    pitch_scale = max(candidate.prop_pitch_ratio / family.pitch_ratio_ref, 0.25)
    # CT scales weaker with pitch (exponent 0.35) than CP (exponent 0.80); consistent with
    # UIUC database trends: doubling P/D raises CT ~25% but CP ~60-70% (Brandt & Selig 2011).
    ct_static = family.ct_static_base * pitch_scale**0.35
    cp_static = family.cp_static_base * pitch_scale**0.80
    j_critical = max(family.j_critical_scale, 0.15)

    # Thrust rolloff exponent 1.45: empirical fit to UIUC measured CT vs J curves showing
    # faster-than-linear CT drop near j_critical (Deters et al. AIAA 2014-2151).
    thrust_factor = max(0.0, 1.0 - (advance_ratio / j_critical) ** 1.45)
    # Power maintains ~25% of static CP at zero thrust (windmilling); 1.15 × j_critical
    # accounts for the fact that CP peaks slightly beyond j_critical before dropping.
    # Exponent 1.60 from empirical fits to UIUC CP vs J data (Brandt & Selig 2011).
    power_factor = max(0.10, 1.0 - (advance_ratio / (1.15 * j_critical)) ** 1.60)

    ct = ct_static * thrust_factor * reynolds_penalty
    cp = cp_static * (0.25 + 0.75 * power_factor) * reynolds_penalty
    return ct, cp


def prop_operating_point(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
    rpm: float,
    flight_speed_mps: float,
) -> dict[str, float]:
    family = PROP_FAMILY_LIBRARY[candidate.prop_family]
    n_rev_per_sec = rpm / 60.0
    if n_rev_per_sec <= 1e-12:
        return {
            "rpm": rpm,
            "advance_ratio": float("inf"),
            "ct": 0.0,
            "cp": 0.0,
            "thrust_total_n": 0.0,
            "power_shaft_total_w": 0.0,
            "power_shaft_per_prop_w": 0.0,
            "power_electric_total_w": config.avionics_power_w,
            "torque_per_prop_nm": 0.0,
            "tip_mach": 0.0,
            "blade_reynolds": 0.0,
            "reynolds_penalty": config.blade_reynolds_penalty_floor,
        }

    re_blade = blade_reynolds(config, candidate, rpm, flight_speed_mps)
    re_penalty = reynolds_penalty_factor(config, re_blade)
    advance_ratio = flight_speed_mps / (n_rev_per_sec * candidate.prop_diameter_m)
    ct, cp = prop_coefficients(candidate, family, advance_ratio, reynolds_penalty=re_penalty)

    thrust_per_prop = ct * config.air_density_kgpm3 * n_rev_per_sec**2 * candidate.prop_diameter_m**4
    shaft_power_per_prop = cp * config.air_density_kgpm3 * n_rev_per_sec**3 * candidate.prop_diameter_m**5
    thrust_total = candidate.n_props * thrust_per_prop
    shaft_power_total = candidate.n_props * shaft_power_per_prop
    electric_power_total = shaft_power_total / max(config.electrical_chain_efficiency, 1e-9) + config.avionics_power_w
    torque_per_prop = shaft_power_per_prop / max(2.0 * math.pi * n_rev_per_sec, 1e-9)
    tip_speed = math.sqrt((math.pi * candidate.prop_diameter_m * n_rev_per_sec) ** 2 + flight_speed_mps**2)
    tip_mach = tip_speed / config.speed_of_sound_mps

    return {
        "rpm": rpm,
        "advance_ratio": advance_ratio,
        "ct": ct,
        "cp": cp,
        "thrust_total_n": thrust_total,
        "power_shaft_total_w": shaft_power_total,
        "power_shaft_per_prop_w": shaft_power_per_prop,
        "power_electric_total_w": electric_power_total,
        "torque_per_prop_nm": torque_per_prop,
        "tip_mach": tip_mach,
        "blade_reynolds": re_blade,
        "reynolds_penalty": re_penalty,
    }


def slipstream_velocity_after_prop(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
    thrust_total_n: float,
    freestream_mps: float,
) -> float:
    disk_area = total_disk_area(candidate)
    if disk_area <= 1e-12 or thrust_total_n <= 0.0:
        return freestream_mps
    induced_velocity = math.sqrt(thrust_total_n / (2.0 * config.air_density_kgpm3 * disk_area))
    return freestream_mps + 2.0 * induced_velocity


# -----------------------------------------------------------------------------
# Low-speed two-zone (blown / unblown) wing model with Cμ lift
# -----------------------------------------------------------------------------


def low_speed_zone_model(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
    thrust_total_n: float,
) -> dict[str, float]:
    eta = blown_span_fraction(config, candidate)
    s_total = config.wing_area_m2
    s_unblown = (1.0 - eta) * s_total
    s_blown = eta * s_total

    q_inf = dynamic_pressure(config.air_density_kgpm3, config.low_speed_mps)
    v_eff = slipstream_velocity_after_prop(config, candidate, thrust_total_n, config.low_speed_mps)
    q_b = dynamic_pressure(config.air_density_kgpm3, v_eff)

    # Momentum coefficient Cμ = T / (q_inf * S_total)
    cmu = thrust_total_n / max(q_inf * s_total, 1e-12)

    # Cμ-based ΔCL on the blown section (circulation control style).
    # ΔCL = k * sqrt(Cμ), clipped at the section ceiling above baseline CLmax.
    if cmu >= config.cmu_min_for_blowing:
        delta_cl_mu = config.cmu_lift_coefficient * math.sqrt(cmu)
    else:
        delta_cl_mu = 0.0
    clmax_b = min(config.clmax_section_flapped + delta_cl_mu, config.cl_section_ceiling_flapped)
    clmax_u = min(config.clmax_section_flapped, config.cl_section_ceiling_flapped)

    cl_req_ref = config.gross_weight_n / max(q_inf * s_total, 1e-12)

    l_u_max = q_inf * s_unblown * clmax_u
    lift_remaining = config.gross_weight_n - l_u_max

    if lift_remaining <= 0.0:
        required_veff = config.low_speed_mps
    elif s_blown <= 1e-12:
        required_veff = float("inf")
    else:
        q_b_required = lift_remaining / max(s_blown * clmax_b, 1e-12)
        required_veff = math.sqrt(max(2.0 * q_b_required / config.air_density_kgpm3, 0.0))
        required_veff = max(required_veff, config.low_speed_mps)

    if lift_remaining <= 0.0:
        cl_req_u = config.gross_weight_n / max(q_inf * s_unblown, 1e-12) if s_unblown > 1e-12 else 0.0
        cl_req_b = 0.0
        lift_feasible = True
    else:
        if s_blown <= 1e-12 or q_b <= 1e-12:
            cl_req_u = clmax_u
            cl_req_b = float("inf")
            lift_feasible = False
        else:
            cl_req_u = clmax_u
            cl_req_b = lift_remaining / (q_b * s_blown)
            lift_feasible = cl_req_b <= clmax_b

    drag_factor = induced_drag_factor(config.aspect_ratio, config.oswald_e)
    cd_u = config.trim_drag_factor * (
        config.unblown_profile_drag_factor * config.cd0_flapped + drag_factor * cl_req_u**2
    )
    cd_b = config.trim_drag_factor * (
        config.blown_profile_drag_factor * config.cd0_flapped + drag_factor * max(cl_req_b, 0.0) ** 2
    )

    drag_unblown = q_inf * s_unblown * cd_u
    drag_blown = q_b * s_blown * cd_b if math.isfinite(cl_req_b) else float("inf")
    drag_total = drag_unblown + drag_blown

    return {
        "eta": eta,
        "v_eff": v_eff,
        "required_veff": required_veff,
        "cmu": cmu,
        "delta_cl_mu": delta_cl_mu,
        "cl_req_ref": cl_req_ref,
        "cl_req_u": cl_req_u,
        "cl_req_b": cl_req_b,
        "clmax_b": clmax_b,
        "drag_total": drag_total,
        "lift_feasible": lift_feasible,
    }


def cruise_model(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
) -> dict[str, float]:
    q_cruise = dynamic_pressure(config.air_density_kgpm3, config.cruise_speed_mps)
    cl_req = config.gross_weight_n / max(q_cruise * config.wing_area_m2, 1e-12)
    drag_factor = induced_drag_factor(config.aspect_ratio, config.oswald_e)
    cd_total = config.cd0_clean + drag_factor * cl_req**2
    drag_total = q_cruise * config.wing_area_m2 * cd_total
    return {
        "cl_req": cl_req,
        "drag_total": drag_total,
        "lift_feasible": cl_req <= config.clmax_section_clean,
    }


# -----------------------------------------------------------------------------
# RPM solvers
# -----------------------------------------------------------------------------


def bisection_solve(
    lower: float,
    upper: float,
    residual_fn: Callable[[float], float],
    tolerance: float,
    max_iters: int = 80,
) -> float | None:
    """Solve for a residual root with bisection, assuming monotonic behavior over the bracket."""

    r_lower = residual_fn(lower)
    r_upper = residual_fn(upper)

    if math.isnan(r_lower) or math.isnan(r_upper):
        return None

    if r_lower >= 0.0:
        return lower
    if r_upper < 0.0:
        return None

    for _ in range(max_iters):
        mid = 0.5 * (lower + upper)
        r_mid = residual_fn(mid)
        if math.isnan(r_mid):
            return None
        if abs(upper - lower) <= tolerance or abs(r_mid) <= 1e-6:
            return mid
        if r_mid >= 0.0:
            upper = mid
        else:
            lower = mid

    return 0.5 * (lower + upper)


def solve_low_speed_operating_point(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
) -> tuple[dict[str, float], dict[str, float]] | None:
    """Solve the minimum low-speed RPM that closes both drag and required blown velocity."""

    rpm_lower, rpm_upper = config.low_speed_rpm_bounds

    def residual(rpm: float) -> float:
        op = prop_operating_point(config, candidate, rpm, config.low_speed_mps)
        zone = low_speed_zone_model(config, candidate, op["thrust_total_n"])
        thrust_req_drag = config.thrust_margin_low_speed * zone["drag_total"]
        thrust_req_veff = thrust_required_for_veff(
            config=config,
            candidate=candidate,
            required_veff_mps=zone["required_veff"],
            freestream_mps=config.low_speed_mps,
        )
        thrust_required = max(thrust_req_drag, thrust_req_veff)
        return op["thrust_total_n"] - thrust_required

    solved_rpm = bisection_solve(
        lower=float(rpm_lower),
        upper=float(rpm_upper),
        residual_fn=residual,
        tolerance=config.rpm_solver_tolerance_rpm,
    )
    if solved_rpm is None:
        return None

    op = prop_operating_point(config, candidate, solved_rpm, config.low_speed_mps)
    zone = low_speed_zone_model(config, candidate, op["thrust_total_n"])
    zone["thrust_required_from_drag"] = config.thrust_margin_low_speed * zone["drag_total"]
    zone["thrust_required_from_veff"] = thrust_required_for_veff(
        config=config,
        candidate=candidate,
        required_veff_mps=zone["required_veff"],
        freestream_mps=config.low_speed_mps,
    )
    zone["thrust_required_total"] = max(zone["thrust_required_from_drag"], zone["thrust_required_from_veff"])
    return op, zone


def solve_cruise_operating_point(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
) -> tuple[dict[str, float], dict[str, float]] | None:
    """Solve the minimum cruise RPM that closes the cruise drag condition."""

    rpm_lower, rpm_upper = config.cruise_rpm_bounds
    cruise_zone = cruise_model(config, candidate)

    def residual(rpm: float) -> float:
        op = prop_operating_point(config, candidate, rpm, config.cruise_speed_mps)
        thrust_required = config.thrust_margin_cruise * cruise_zone["drag_total"]
        return op["thrust_total_n"] - thrust_required

    solved_rpm = bisection_solve(
        lower=float(rpm_lower),
        upper=float(rpm_upper),
        residual_fn=residual,
        tolerance=config.rpm_solver_tolerance_rpm,
    )
    if solved_rpm is None:
        return None

    op = prop_operating_point(config, candidate, solved_rpm, config.cruise_speed_mps)
    cruise_zone["thrust_required_total"] = config.thrust_margin_cruise * cruise_zone["drag_total"]
    return op, cruise_zone


# -----------------------------------------------------------------------------
# Motor / ESC / mass sizing
# -----------------------------------------------------------------------------


def infer_motor_requirements(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
    low_speed_operating_point: dict[str, float],
    cruise_operating_point: dict[str, float],
) -> dict[str, float]:
    peak_electric_w_total = max(
        low_speed_operating_point["power_electric_total_w"],
        cruise_operating_point["power_electric_total_w"],
    )
    current_total = peak_electric_w_total / max(config.battery_voltage_v, 1e-9)
    current_per_motor = current_total / candidate.n_props

    peak_shaft_w_per_prop = max(
        low_speed_operating_point["power_shaft_per_prop_w"],
        cruise_operating_point["power_shaft_per_prop_w"],
    )

    kv_required = low_speed_operating_point["rpm"] / max(0.85 * config.battery_voltage_v, 1e-9)
    return {
        "current_per_motor_a": current_per_motor,
        "peak_shaft_power_per_prop_w": peak_shaft_w_per_prop,
        "kv_required_rpm_per_v": kv_required,
    }


def battery_mass_estimate_kg(
    config: Stage1MissionConfig,
    low_speed_power_w: float,
    cruise_power_w: float,
) -> float:
    """Battery mass from mission energy plus a reserve.

    Mission energy = low-speed climb segment + loiter-at-cruise. Both converted
    from electrical power via battery specific energy and a reserve fraction.
    """

    energy_climb_wh = low_speed_power_w * (config.climb_segment_min / 60.0)
    energy_cruise_wh = cruise_power_w * (config.loiter_time_min / 60.0)
    usable_energy_wh = energy_climb_wh + energy_cruise_wh
    required_pack_wh = usable_energy_wh * (1.0 + config.battery_reserve_fraction)
    return required_pack_wh / max(config.battery_specific_energy_wh_per_kg, 1e-9)


# -----------------------------------------------------------------------------
# Candidate evaluation
# -----------------------------------------------------------------------------


def evaluate_stage1_candidate(
    config: Stage1MissionConfig,
    candidate: Stage1Candidate,
) -> Stage1Result:
    low_speed_solution = solve_low_speed_operating_point(config, candidate)
    cruise_solution = solve_cruise_operating_point(config, candidate)

    if low_speed_solution is None:
        low_speed_op = prop_operating_point(config, candidate, float(config.low_speed_rpm_bounds[1]), config.low_speed_mps)
        low_speed_zone = low_speed_zone_model(config, candidate, low_speed_op["thrust_total_n"])
        low_speed_zone["thrust_required_from_drag"] = config.thrust_margin_low_speed * low_speed_zone["drag_total"]
        low_speed_zone["thrust_required_from_veff"] = thrust_required_for_veff(
            config=config,
            candidate=candidate,
            required_veff_mps=low_speed_zone["required_veff"],
            freestream_mps=config.low_speed_mps,
        )
        low_speed_zone["thrust_required_total"] = max(
            low_speed_zone["thrust_required_from_drag"],
            low_speed_zone["thrust_required_from_veff"],
        )
    else:
        low_speed_op, low_speed_zone = low_speed_solution

    if cruise_solution is None:
        cruise_op = prop_operating_point(config, candidate, float(config.cruise_rpm_bounds[1]), config.cruise_speed_mps)
        cruise_zone = cruise_model(config, candidate)
        cruise_zone["thrust_required_total"] = config.thrust_margin_cruise * cruise_zone["drag_total"]
    else:
        cruise_op, cruise_zone = cruise_solution

    motor_req = infer_motor_requirements(config, candidate, low_speed_op, cruise_op)

    loiter_energy_wh = cruise_op["power_electric_total_w"] * (config.loiter_time_min / 60.0)
    disk_area = total_disk_area(candidate)
    q_low = dynamic_pressure(config.air_density_kgpm3, config.low_speed_mps)
    disk_loading = low_speed_op["thrust_total_n"] / max(disk_area, 1e-9)

    # ---- Propulsion mass via power-law fits to commercial databases ----
    mass_breakdown = MASS_MODEL.propulsion_mass_breakdown_kg(
        n_props=candidate.n_props,
        diameter_in=candidate.prop_diameter_in,
        pitch_in=candidate.prop_pitch_in,
        peak_shaft_power_per_prop_w=motor_req["peak_shaft_power_per_prop_w"],
        peak_current_per_motor_a=motor_req["current_per_motor_a"],
    )

    battery_mass_kg = battery_mass_estimate_kg(
        config=config,
        low_speed_power_w=low_speed_op["power_electric_total_w"],
        cruise_power_w=cruise_op["power_electric_total_w"],
    )

    propulsion_mass_total = mass_breakdown["propulsion_mass_kg_total"]
    total_built_mass = config.fixed_system_mass_kg + propulsion_mass_total + battery_mass_kg
    mass_budget_margin = config.max_mass_kg - total_built_mass

    packing = packing_margins(config, candidate)

    constraints = {
        **packing,
        "low_speed_thrust_margin_n": low_speed_op["thrust_total_n"] - low_speed_zone["thrust_required_total"],
        "cruise_thrust_margin_n": cruise_op["thrust_total_n"] - cruise_zone["thrust_required_total"],
        "low_speed_lift_margin": low_speed_zone["clmax_b"] - max(low_speed_zone["cl_req_b"], 0.0),
        "cruise_lift_margin": config.clmax_section_clean - cruise_zone["cl_req"],
        "low_speed_veff_margin_mps": low_speed_zone["v_eff"] - low_speed_zone["required_veff"],
        "low_speed_tip_mach_margin": config.max_tip_mach - low_speed_op["tip_mach"],
        "cruise_tip_mach_margin": config.max_tip_mach - cruise_op["tip_mach"],
        "rpm_schedule_margin": low_speed_op["rpm"] - cruise_op["rpm"],
        "disk_loading_lower_margin_npm2": disk_loading - config.disk_loading_min_npm2,
        "disk_loading_upper_margin_npm2": config.disk_loading_max_npm2 - disk_loading,
        "mass_budget_margin_kg": mass_budget_margin,
    }

    is_feasible = all(
        (
            constraints["packing_margin_m"] >= 0.0,
            low_speed_zone["lift_feasible"],
            cruise_zone["lift_feasible"],
            constraints["low_speed_veff_margin_mps"] >= 0.0,
            constraints["low_speed_thrust_margin_n"] >= 0.0,
            constraints["cruise_thrust_margin_n"] >= 0.0,
            constraints["low_speed_tip_mach_margin"] >= 0.0,
            constraints["cruise_tip_mach_margin"] >= 0.0,
            constraints["rpm_schedule_margin"] >= 0.0,
            constraints["disk_loading_lower_margin_npm2"] >= 0.0,
            constraints["disk_loading_upper_margin_npm2"] >= 0.0,
            constraints["mass_budget_margin_kg"] >= 0.0,
        )
    )

    return Stage1Result(
        candidate=candidate,
        is_feasible=is_feasible,
        low_speed_power_w=low_speed_op["power_electric_total_w"],
        cruise_power_w=cruise_op["power_electric_total_w"],
        loiter_energy_wh=loiter_energy_wh,
        low_speed_thrust_n=low_speed_op["thrust_total_n"],
        cruise_thrust_n=cruise_op["thrust_total_n"],
        low_speed_drag_n=low_speed_zone["drag_total"],
        cruise_drag_n=cruise_zone["drag_total"],
        low_speed_veff_mps=low_speed_zone["v_eff"],
        low_speed_required_veff_mps=low_speed_zone["required_veff"],
        low_speed_blown_span_fraction=low_speed_zone["eta"],
        low_speed_cl_required=low_speed_zone["cl_req_ref"],
        cruise_cl_required=cruise_zone["cl_req"],
        solved_low_speed_rpm=low_speed_op["rpm"],
        solved_cruise_rpm=cruise_op["rpm"],
        low_speed_thrust_to_drag=low_speed_op["thrust_total_n"] / max(low_speed_zone["drag_total"], 1e-9),
        cruise_thrust_to_drag=cruise_op["thrust_total_n"] / max(cruise_zone["drag_total"], 1e-9),
        low_speed_disk_loading_npm2=disk_loading,
        low_speed_momentum_coefficient=low_speed_zone["cmu"],
        per_prop_low_speed_torque_nm=low_speed_op["torque_per_prop_nm"],
        per_prop_cruise_torque_nm=cruise_op["torque_per_prop_nm"],
        per_prop_peak_shaft_power_w=motor_req["peak_shaft_power_per_prop_w"],
        per_prop_peak_current_a=motor_req["current_per_motor_a"],
        motor_required_current_a=motor_req["current_per_motor_a"],
        motor_required_kv_rpm_per_v=motor_req["kv_required_rpm_per_v"],
        low_speed_tip_mach=low_speed_op["tip_mach"],
        cruise_tip_mach=cruise_op["tip_mach"],
        low_speed_clmax_blown_section=low_speed_zone["clmax_b"],
        blade_reynolds_low_speed=low_speed_op["blade_reynolds"],
        prop_reynolds_penalty_factor=low_speed_op["reynolds_penalty"],
        prop_mass_kg_per_unit=mass_breakdown["prop_mass_kg_per_unit"],
        motor_mass_kg_per_unit=mass_breakdown["motor_mass_kg_per_unit"],
        esc_mass_kg_per_unit=mass_breakdown["esc_mass_kg_per_unit"],
        propulsion_mass_kg_total=mass_breakdown["propulsion_mass_kg_total"],
        battery_mass_kg_estimate=battery_mass_kg,
        total_built_mass_kg=total_built_mass,
        mass_budget_margin_kg=mass_budget_margin,
        constraints=constraints,
    )


def generate_stage1_candidates(sweep: Stage1SweepConfig) -> List[Stage1Candidate]:
    candidates: List[Stage1Candidate] = []
    for n_props in sweep.n_props_values:
        for diameter_in in sweep.prop_diameter_in_values:
            for pitch_ratio in sweep.prop_pitch_ratio_values:
                for prop_family in sweep.prop_family_values:
                    candidates.append(
                        Stage1Candidate(
                            n_props=n_props,
                            prop_diameter_in=diameter_in,
                            prop_pitch_ratio=pitch_ratio,
                            prop_family=prop_family,
                        )
                    )
    return candidates
