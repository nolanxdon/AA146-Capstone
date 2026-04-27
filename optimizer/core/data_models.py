from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass(frozen=True)
class Stage1MissionConfig:
    """Global assumptions for the current Stage 1 V2 Pareto screener."""

    # ---------------- Aircraft gross mass envelope ----------------
    gross_mass_kg: float = 5.0               # flight mass used in lift balance; mission requirement
    max_mass_kg: float = 5.0                 # hard ceiling on as-built mass; mission requirement
    fixed_system_mass_kg: float = 2.0        # airframe + avionics + payload + servos; TODO: break down with component mass estimates from vendor datasheets
    battery_specific_energy_wh_per_kg: float = 180.0  # pack-level LiPo; Traub (2011) J. Aircraft 48(2) cites ~150-200 Wh/kg; Tattu 4S 5000mAh measured ~182 Wh/kg
    battery_reserve_fraction: float = 0.20   # 20% reserve is standard UAV practice; consistent with NATO STANAG 4671 UAV airworthiness margin guidance
    climb_segment_min: float = 1.5           # low-speed segment duration charged to battery; TODO: derive from climb performance or mission profile

    # ---------------- Airframe / geometry ----------------
    span_m: float = 2.0                      # mission requirement; drives wing loading and prop packing
    chord_m: float = 0.35                    # rectangular baseline; gives AR ≈ 5.7 (Raymer 6th ed. recommends AR 5-8 for UAVs at this scale)
    battery_voltage_v: float = 14.8          # 4S LiPo nominal (4 × 3.7 V); standard for 5-kg-class UAVs

    # ---------------- Mission flight conditions ----------------
    low_speed_mps: float = 4.0               # blown-lift operating point used by Stage 1/2; keep frozen so upstream wing/prop sizing remains unchanged
    cruise_speed_mps: float = 10.0           # mission requirement; corresponds to ~36 km/h
    loiter_time_min: float = 18.0            # endurance requirement; mission requirement

    # ---------------- Atmosphere ----------------
    air_density_kgpm3: float = 1.225         # ISA sea level; ICAO Doc 7488/3 standard atmosphere
    dynamic_viscosity_pas: float = 1.81e-5   # ISA sea level; Sutherland's law at 15°C (ICAO Doc 7488/3)
    gravity_mps2: float = 9.80665            # standard gravity; ISO 80000-3
    speed_of_sound_mps: float = 343.0        # ISA sea level at 15°C; ICAO Doc 7488/3

    # ---------------- Aero constants ----------------
    oswald_e: float = 0.6591                 # Effective Oswald factor for the 2.0 m x 0.35 m DAE51 rectangular wing, fit from an AeroSandbox VLM + NeuralFoil clean-wing drag polar over the 7.5-10 m/s cruise/loiter range
    clmax_section_flapped: float = 2.2       # Raymer 6th ed. Ch. 12: plain flap adds ΔCLmax ≈ 0.8 above clean baseline; 1.4 + 0.8 = 2.2; consistent with thin-airfoil flap theory (Glauert 1926)
    cl_section_ceiling_flapped: float = 2.0  # enforce the wing-system working ceiling; conservative cap below section CLmax to account for 3-D and installation losses
    clmax_section_clean: float = 1.4         # Abbott & Von Doenhoff "Theory of Wing Sections" (1959): NACA 4-digit profiles at Re ≈ 5×10⁵ give CLmax ≈ 1.3-1.5; 1.4 is a reasonable mid-estimate
    cd0_flapped: float = 0.11               # Raymer 6th ed. Table 12.5: flap penalty ΔCD0 ≈ 0.05-0.08 above clean; 0.11 total is consistent with simple flap at 5-kg UAV scale; TODO: refine with panel-code or wind-tunnel data
    cd0_clean: float = 0.05                 # Drela "Flight Vehicle Aerodynamics" MIT 2014: small UAV CD0 ≈ 0.03-0.06 for 2-m span class; 0.05 is mid-range estimate; TODO: refine with AeroSandbox viscous panel analysis
    trim_drag_factor: float = 1.05          # Raymer 6th ed.: trim drag typically 3-8% of total drag; 5% is standard conservative first-order estimate
    blown_profile_drag_factor: float = 1.05  # 5% penalty on blown section profile drag; Englar (2000) AIAA 2000-2541 observed slight drag rise with circulation control; TODO: validate with blown-section polar
    unblown_profile_drag_factor: float = 1.0 # no penalty on unblown sections
    k_span_e8xpansion: float = 0.8          # slipstream width at wing LE as fraction of prop diameter; Patterson & German (2016) AIAA 2016-3920: measured 0.7-0.85 D depending on advance ratio; 0.8 is mid-range; NOTE: field name has typo (e8xpansion), keep for backward compatibility

    # ---------------- Blown-lift (Cμ) model ----------------
    cmu_lift_coefficient: float = 1.8        # k in ΔCL ≈ k√Cμ; Englar (2000) AIAA 2000-2541: k ≈ 1.5-2.5 depending on slot geometry; Seele et al. (2013) AIAA 2013-0411: k ≈ 1.8-2.1 for discrete blown slots; 1.8 is conservative; TODO: calibrate against blown-wing wind-tunnel or CFD data
    cmu_min_for_blowing: float = 0.02        # below Cμ ≈ 0.02 the incremental lift is negligible; consistent with Williams & Alexander (2012) blown-slot threshold observations

    # ---------------- Packing ----------------
    fuselage_width_m: float = 0.20           # fuselage internal width; TODO: update from structural layout drawing
    fuselage_prop_clearance_in: float = 1.0  # 1 in (~25 mm) clearance from fuselage edge to nearest prop disk; standard fabrication margin for vibration isolation
    inter_prop_clearance_in: float = 1.0     # 1 in (~25 mm) minimum gap between adjacent prop disks; standard to prevent aerodynamic interaction and structural contact risk
    tip_margin_in: float = 1.5               # 1.5 in (~38 mm) from outermost prop edge to wing tip; conservative ground-strike margin; TODO: confirm from structural load analysis

    # ---------------- Electrical / propulsion efficiency ----------------
    electrical_chain_efficiency: float = 0.72  # combined motor × ESC efficiency; motor η ≈ 0.85 (T-Motor F20 II datasheet at rated load) × ESC η ≈ 0.95 = 0.81 at design point; derated to 0.72 for partial-throttle and wiring losses; Bohorquez et al. (2010) J. Am. Helicopter Soc. 55(1) measured small-UAS motor efficiency 0.75-0.85; TODO: replace with operating-point-dependent motor map
    avionics_power_w: float = 10.0           # fixed avionics/electronics power draw; typical Pixhawk + telemetry + receiver ≈ 5-15 W; 10 W is conservative mid-estimate; TODO: measure from component datasheets
    thrust_margin_low_speed: float = 1.05    # thrust must exceed drag by 5% at low speed; standard 5% climb-margin practice for electric UAV sizing
    thrust_margin_cruise: float = 1.02       # thrust must exceed drag by 2% at cruise; tighter margin acceptable in level flight

    # ---------------- Tip Mach + RPM solver ----------------
    max_tip_mach: float = 0.55              # compressibility effects on prop efficiency/noise onset at ~Mach 0.6 tip; Sinnige et al. AIAA 2018-3022 recommend < 0.55 for low-noise small UAV props; McCormick "Aerodynamics" 2nd ed. Ch. 3 corroborates this limit
    low_speed_rpm_bounds: Tuple[int, int] = (4000, 14000)  # bisection bracket; covers practical range for 4-10" electric props at low speed
    cruise_rpm_bounds: Tuple[int, int] = (3000, 11000)     # bisection bracket; upper bound set by tip-Mach limit at cruise
    rpm_solver_tolerance_rpm: float = 5.0   # convergence tolerance; < 0.05% of typical operating RPM

    # ---------------- Blade Reynolds penalty ----------------
    blade_chord_to_diameter: float = 0.10    # c/D at 0.75R station; Brandt & Selig AIAA 2011-1255 measured c/D ≈ 0.08-0.12 for 4-6" electric props; APC 5×4.7 c/D ≈ 0.10 at 0.75R
    blade_reynolds_reference: float = 80000.0  # Re at which CT/CP surrogates are calibrated; UIUC Propeller Database (Selig et al. uiuc.edu/m-selig/props): data at 40k-200k Re; 80k representative for 5-7" props at design RPM
    blade_reynolds_penalty_exponent: float = 0.30  # Lissaman (1983) "Low-Reynolds-Number Airfoils" AIAA: CD ∝ Re^-0.3 for laminar-separation regime Re < 2×10⁵; Brandt & Deters UIUC database corroborates ~0.25-0.35
    blade_reynolds_penalty_floor: float = 0.60  # CT/CP never derated below 60%; prevents unphysical collapse at very low Re; conservative bound consistent with Bohorquez (2010) rotor test data

    # ---------------- Disk-loading bounds ----------------
    disk_loading_min_npm2: float = 25.0      # below ~25 N/m² disk area becomes impractically large; Raymer 6th ed.: typical DEP/multirotor DL > 25 N/m²
    disk_loading_max_npm2: float = 500.0     # above ~500 N/m² actuator-disk hover FOM degrades severely; Leishman "Principles of Helicopter Aerodynamics" 2nd ed. Ch. 2: hover FOM drops sharply at high DL

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
    """Design-variable ranges for the current Stage 1 V2 screening pass."""

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
