from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass(frozen=True)
class Stage1MissionConfig:
    """Global assumptions for the V1 Pareto screener."""

    # ---------------- Aircraft gross mass envelope ----------------
    gross_mass_kg: float = 5.0               # flight mass used in lift balance
    max_mass_kg: float = 5.0                 # hard ceiling on as-built mass
    fixed_system_mass_kg: float = 2.0        # airframe + avionics + payload + servos
    battery_specific_energy_wh_per_kg: float = 180.0  # pack-level LiPo
    battery_reserve_fraction: float = 0.20   # reserve on top of mission energy
    climb_segment_min: float = 1.5           # low-speed segment duration charged to battery

    # ---------------- Airframe / geometry ----------------
    span_m: float = 2.0
    chord_m: float = 0.35
    battery_voltage_v: float = 14.8

    # ---------------- Mission flight conditions ----------------
    low_speed_mps: float = 4.0
    cruise_speed_mps: float = 10.0
    loiter_time_min: float = 18.0

    # ---------------- Atmosphere ----------------
    air_density_kgpm3: float = 1.225
    dynamic_viscosity_pas: float = 1.81e-5
    gravity_mps2: float = 9.80665
    speed_of_sound_mps: float = 343.0

    # ---------------- Aero constants ----------------
    oswald_e: float = 0.8
    clmax_section_flapped: float = 2.2
    cl_section_ceiling_flapped: float = 3.2  # raised so Cμ model can actually move
    clmax_section_clean: float = 1.4
    cd0_flapped: float = 0.11
    cd0_clean: float = 0.05
    trim_drag_factor: float = 1.05
    blown_profile_drag_factor: float = 1.05
    unblown_profile_drag_factor: float = 1.0
    k_span_expansion: float = 0.8

    # ---------------- Blown-lift (Cμ) model ----------------
    cmu_lift_coefficient: float = 1.8        # ΔCL ≈ k * sqrt(Cμ); k ~ 1.5–2.5 literature
    cmu_min_for_blowing: float = 0.02        # below this, treat as no blowing help

    # ---------------- Packing ----------------
    fuselage_width_m: float = 0.20
    fuselage_prop_clearance_in: float = 1.0
    inter_prop_clearance_in: float = 1.0
    tip_margin_in: float = 1.5

    # ---------------- Electrical / propulsion efficiency ----------------
    electrical_chain_efficiency: float = 0.72
    avionics_power_w: float = 10.0
    thrust_margin_low_speed: float = 1.05
    thrust_margin_cruise: float = 1.02

    # ---------------- Tip Mach + RPM solver ----------------
    max_tip_mach: float = 0.55
    low_speed_rpm_bounds: Tuple[int, int] = (4000, 14000)
    cruise_rpm_bounds: Tuple[int, int] = (3000, 11000)
    rpm_solver_tolerance_rpm: float = 5.0

    # ---------------- Blade Reynolds penalty ----------------
    blade_chord_to_diameter: float = 0.10    # typical c/D for small electric props
    blade_reynolds_reference: float = 80000.0
    blade_reynolds_penalty_exponent: float = 0.30
    blade_reynolds_penalty_floor: float = 0.60  # CT,CP never derated below 60 %

    # ---------------- Disk-loading bounds ----------------
    disk_loading_min_npm2: float = 25.0      # avoid absurdly under-loaded disks
    disk_loading_max_npm2: float = 500.0     # avoid unrealistic high-DL concepts

    @property
    def wing_area_m2(self) -> float:
        return self.span_m * self.chord_m

    @property
    def aspect_ratio(self) -> float:
        return self.span_m**2 / self.wing_area_m2

    @property
    def gross_weight_n(self) -> float:
        return self.gross_mass_kg * self.gravity_mps2

    @property
    def semispan_m(self) -> float:
        return self.span_m / 2.0

    @property
    def fuselage_half_width_m(self) -> float:
        return self.fuselage_width_m / 2.0

    @property
    def fuselage_prop_clearance_m(self) -> float:
        return self.fuselage_prop_clearance_in * 0.0254

    @property
    def inter_prop_clearance_m(self) -> float:
        return self.inter_prop_clearance_in * 0.0254

    @property
    def tip_margin_m(self) -> float:
        return self.tip_margin_in * 0.0254

    @property
    def max_propulsion_plus_battery_mass_kg(self) -> float:
        return max(0.0, self.max_mass_kg - self.fixed_system_mass_kg)


@dataclass(frozen=True)
class Stage1SweepConfig:
    """Design-variable ranges for the V1 screening pass."""

    n_props_values: Tuple[int, ...] = (4, 6, 8, 10, 12, 14, 16)
    prop_diameter_in_values: Tuple[float, ...] = (4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 9.0, 10.0)
    prop_pitch_ratio_values: Tuple[float, ...] = (0.40, 0.50, 0.60, 0.70, 0.80, 0.90)
    prop_family_values: Tuple[str, ...] = ("high_thrust", "balanced", "cruise")


@dataclass(frozen=True)
class PropFamilyModel:
    """Coarse prop-family surrogate used in Stage 1 before detailed sizing."""

    name: str
    ct_static_base: float
    cp_static_base: float
    j_critical_scale: float
    pitch_ratio_ref: float = 0.65


@dataclass(frozen=True)
class Stage1Candidate:
    """One candidate architecture in the coarse Pareto screen."""

    n_props: int
    prop_diameter_in: float
    prop_pitch_ratio: float
    prop_family: str

    @property
    def prop_diameter_m(self) -> float:
        return self.prop_diameter_in * 0.0254

    @property
    def prop_pitch_in(self) -> float:
        return self.prop_diameter_in * self.prop_pitch_ratio


@dataclass
class Stage1Result:
    """Stored result for one screen candidate."""

    candidate: Stage1Candidate
    is_feasible: bool
    low_speed_power_w: float
    cruise_power_w: float
    loiter_energy_wh: float
    low_speed_thrust_n: float
    cruise_thrust_n: float
    low_speed_drag_n: float
    cruise_drag_n: float
    low_speed_veff_mps: float
    low_speed_required_veff_mps: float
    low_speed_blown_span_fraction: float
    low_speed_cl_required: float
    cruise_cl_required: float
    solved_low_speed_rpm: float
    solved_cruise_rpm: float
    low_speed_thrust_to_drag: float
    cruise_thrust_to_drag: float
    low_speed_disk_loading_npm2: float
    low_speed_momentum_coefficient: float
    per_prop_low_speed_torque_nm: float
    per_prop_cruise_torque_nm: float
    per_prop_peak_shaft_power_w: float
    per_prop_peak_current_a: float
    motor_required_current_a: float
    motor_required_kv_rpm_per_v: float
    low_speed_tip_mach: float
    cruise_tip_mach: float
    low_speed_clmax_blown_section: float
    blade_reynolds_low_speed: float
    prop_reynolds_penalty_factor: float
    prop_mass_kg_per_unit: float
    motor_mass_kg_per_unit: float
    esc_mass_kg_per_unit: float
    propulsion_mass_kg_total: float
    battery_mass_kg_estimate: float
    total_built_mass_kg: float
    mass_budget_margin_kg: float
    constraints: Dict[str, float] = field(default_factory=dict)
