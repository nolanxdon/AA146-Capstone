from __future__ import annotations

import csv
import math
import os
import sys
from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any

from optimizer.core.data_models import Stage1Candidate, Stage1MissionConfig
from optimizer.core.physics import (
    prop_operating_point,
    slipstream_velocity_after_prop,
    thrust_required_for_veff,
)


STAGE3_VISUAL_DIR = Path("outputs/stage3_visuals")
STAGE3_TRADE_PLOT = STAGE3_VISUAL_DIR / "stage3_trade_space.png"
STAGE3_GALLERY_MD = STAGE3_VISUAL_DIR / "STAGE3_GALLERY.md"
STAGE3_REPORT_MD = Path("outputs/stage3_aerosandbox_report.md")
STAGE3_READABLE_RESULTS_MD = Path("outputs/stage3_readable_results.md")
STAGE3_CONSTRAINTS_YAML = Path("optimizer/config/stage3_constraints.yaml")
STAGE3_ENGINEERING_TEX = Path("optimizer/STAGE3_ENGINEERING_WRITEUP.tex")


@dataclass(frozen=True)
class Stage3SizingConfig:
    """Editable assumptions for the Stage 3 fixed-wing tail refinement."""

    main_airfoil_default: str = "DAE51"
    tail_airfoil: str = "naca0008"
    foam_density_kgpm3: float = 25.0

    main_wing_incidence_deg: float = 0.0
    main_wing_incidence_bounds_deg: tuple[float, float] = (-2.0, 8.0)
    target_cruise_body_alpha_deg: float = 0.0
    main_wing_washout_deg: float = 0.0
    vertical_tail_incidence_deg: float = 0.0

    flap_span_fraction_default: float = 0.65
    flap_chord_fraction_default: float = 0.34
    flap_deflection_slow_deg: float = 40.0       # Frozen control-sizing result: slotted flaps deployed for all slow-flight sizing.
    slotted_flap_section_clmax: float = 2.00     # Fallback only; Stage 3 normally reads Stage 1/2 NeuralFoil slotted-flap section CLmax.
    clean_outer_section_clmax: float = 1.60      # Fallback only; Stage 3 normally reads Stage 1/2 clean section CLmax.
    physical_clean_clmax_cap: float = 1.60
    physical_flapped_clmax_cap: float = 2.00
    flap_down_3d_efficiency: float = 0.90        # Fallback conversion from section CLmax to finite-wing CLmax when workflow CL curves are unavailable.
    blown_flap_clmax_multiplier: float = 1.00    # Kept for fallback compatibility; propwash should raise q, not the airfoil CLmax.
    slow_flight_stall_margin_factor: float = 1.20  # Require 20% lift margin at slow flight; common early-design buffer for gusts, build errors, and CLmax uncertainty.
    min_slow_flight_lift_margin: float = 0.10    # Optimization warning threshold: at least 10% extra lift after the 20% sizing factor is preferred.
    main_wing_pitch_up_stall_margin_factor: float = 1.25  # Require 25% local-CL headroom in review tables so emergency pitch-up does not immediately hit stall.
    aileron_span_fraction_default: float = 0.28
    aileron_chord_fraction_default: float = 0.28
    elevator_chord_fraction: float = 0.30        # Fallback/control-surface reference; final value is sized for slow-flight pitch authority.
    rudder_chord_fraction: float = 0.32          # Fallback/control-surface reference; final value is sized for slow-flight yaw authority.
    elevator_chord_fraction_bounds: tuple[float, float] = (0.22, 0.45)
    rudder_chord_fraction_bounds: tuple[float, float] = (0.25, 0.50)
    elevator_max_deflection_deg: float = 25.0
    rudder_max_deflection_deg: float = 25.0
    target_slow_pitch_control_cm: float = 0.18   # Required elevator moment authority coefficient at slow flight, referenced to q*S*c.
    target_slow_yaw_control_cn: float = 0.055    # Required rudder yaw authority coefficient at slow flight, referenced to q*S*b.
    slow_tail_dynamic_pressure_ratio: float = 0.85
    control_surface_effectiveness_exponent: float = 0.70

    h_tail_area_bounds_m2: tuple[float, float] = (0.055, 0.240)
    h_tail_aspect_ratio_bounds: tuple[float, float] = (3.0, 6.0)
    h_tail_taper_bounds: tuple[float, float] = (0.55, 1.00)
    h_tail_root_chord_bounds_m: tuple[float, float] = (0.10, 0.34)

    v_tail_area_bounds_m2: tuple[float, float] = (0.025, 0.140)
    v_tail_aspect_ratio_bounds: tuple[float, float] = (1.15, 2.60)
    v_tail_taper_bounds: tuple[float, float] = (0.55, 1.00)
    v_tail_root_chord_bounds_m: tuple[float, float] = (0.10, 0.38)

    tail_arm_bounds_m: tuple[float, float] = (0.80, 1.55)
    h_tail_incidence_bounds_deg: tuple[float, float] = (-6.0, 6.0)

    horizontal_tail_volume_range: tuple[float, float] = (0.50, 0.95)
    vertical_tail_volume_range: tuple[float, float] = (0.035, 0.085)
    min_horizontal_tail_volume: float = 0.50
    max_horizontal_tail_volume: float = 0.95
    min_vertical_tail_volume: float = 0.035
    max_vertical_tail_volume: float = 0.085
    target_static_margin_mac: float = 0.13
    min_static_margin_mac: float = 0.08
    max_static_margin_mac: float = 0.24

    baseline_cg_fraction_mac: float = 0.30
    enforce_target_cg: bool = True
    target_cg_fraction_mac: float = 0.25
    baseline_cg_fraction_bounds_mac: tuple[float, float] = (0.05, 0.45)
    wing_aerodynamic_center_fraction_mac: float = 0.25
    prop_axial_location_fraction_of_chord: float = -0.25  # Prop disk x-location relative to the wing leading edge; negative means ahead of the leading edge, matching Stage 1/2 high-lift assumptions.
    tail_dynamic_pressure_ratio: float = 0.90
    downwash_gradient: float = 0.35
    fuselage_nose_x_m: float = -0.300             # Nose station relative to wing leading edge; -0.300 is 300 mm ahead of the LE.
    fuselage_width_m: float = 0.170              # Current structural cross-section width: 170 mm.
    fuselage_height_m: float = 0.130             # Current structural cross-section height: 130 mm.
    fuselage_tail_width_m: float = 0.095
    fuselage_tail_height_m: float = 0.075
    fuselage_cd_area_m2: float = 0.011           # Equivalent parasite drag area, D_fuse = q*CdA; roughly 0.5 Cd on a 170 mm x 130 mm frontal box with small protuberance allowance.
    tail_zero_lift_cd: float = 0.012             # Thin tail-section profile drag at low Re with small trim loads; intentionally a little above clean airfoil CDmin.

    cruise_weight: float = 0.68
    slow_flight_weight: float = 0.06
    material_mass_weight: float = 0.16
    stability_weight: float = 0.10
    reference_cruise_power_w: float = 120.0
    reference_slow_flight_power_w: float = 220.0

    fixed_propulsion_enabled: bool = True
    fixed_n_props: int = 10
    fixed_prop_diameter_in: float = 5.5
    fixed_prop_pitch_ratio: float = 0.40
    fixed_prop_family: str = "balanced"

    ecalc_propulsion_enabled: bool = True
    ecalc_static_csv: str = "outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_static_partial_load.csv"
    ecalc_dynamic_csv: str = "outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_dynamic_design_point.csv"
    ecalc_reference_n_props: int = 10
    ecalc_reference_prop_diameter_in: float = 5.5
    ecalc_match_diameter_tolerance_in: float = 0.05
    wiring_power_loss_factor: float = 1.08       # Multiplies motor/ESC input power to account for harness, connector, and distribution losses.
    avionics_power_loss_factor: float = 1.10     # Multiplies the fixed avionics draw to account for BEC/regulator and avionics wiring losses.

    performance_sweep_velocity_min_mps: float = 3.5
    performance_sweep_velocity_max_mps: float = 16.0
    performance_sweep_velocity_points: int = 51
    performance_sweep_alpha_min_deg: float = -4.0
    performance_sweep_alpha_max_deg: float = 18.0
    performance_sweep_alpha_points: int = 89
    slow_flight_speed_mps: float = 4.0
    clean_sweep_speed_mps: float = 10.0
    flaps_down_sweep_speed_mps: float = 4.0

    approach_target_angle_deg: float = 6.0       # Fallback only if approach_target_sink_rate_fps is nonpositive.
    approach_target_sink_rate_fps: float = 6.0
    approach_speed_mps: float = 5.0
    approach_flap_deflection_deg: float = 40.0
    approach_max_elevator_trim_deg: float = 20.0
    approach_tail_trim_effectiveness: float = 0.85
    max_climb_rate_fps: float = 6.0
    max_descent_rate_fps: float = 6.0
    climb_trim_speed_mps: float = 10.0
    climb_flap_deflection_deg: float = 0.0
    battery_cruise_segment_min: float = 20.0
    battery_slow_segment_min: float = 1.5
    mission_profile_battery_cell_count: int = 4
    mission_profile_cell_nominal_voltage_v: float = 3.7
    mission_profile_takeoff_time_min: float = 0.75
    mission_profile_loiter_pre_drop_time_min: float = 10.0
    mission_profile_drop_low_speed_time_min: float = 1.0
    mission_profile_loiter_post_drop_time_min: float = 5.0
    mission_profile_landing_time_min: float = 0.75
    payload_mass_lb: float = 2.5


@dataclass(frozen=True)
class RuntimeModules:
    asb: Any
    onp: Any
    plt: Any
    patches: Any


def _coerce_config_value(value: Any, default: Any) -> Any:
    if isinstance(default, tuple):
        if not isinstance(value, (list, tuple)) or len(value) != len(default):
            return default
        return tuple(type(item_default)(item) for item, item_default in zip(value, default))
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, int) and not isinstance(default, bool):
        return int(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, str):
        return str(value)
    return value


def load_stage3_sizing_config(path: Path = STAGE3_CONSTRAINTS_YAML) -> Stage3SizingConfig:
    """Load editable Stage 3 sizing constraints from YAML, falling back to defaults."""

    default = Stage3SizingConfig()
    if not path.exists():
        return default

    try:
        import yaml
    except ModuleNotFoundError:
        return default

    with path.open(encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    if not isinstance(raw, dict):
        return default

    data = raw.get("stage3_sizing", raw)
    if not isinstance(data, dict):
        return default

    defaults = {field.name: getattr(default, field.name) for field in fields(Stage3SizingConfig)}
    updates: dict[str, Any] = {}
    for key, value in data.items():
        if key not in defaults:
            continue
        updates[key] = _coerce_config_value(value, defaults[key])
    return Stage3SizingConfig(**{**defaults, **updates})


def _tex_escape(value: Any) -> str:
    text = str(value)
    replacements = {
        "\\": r"\textbackslash{}",
        "&": r"\&",
        "%": r"\%",
        "$": r"\$",
        "#": r"\#",
        "_": r"\_",
        "{": r"\{",
        "}": r"\}",
        "~": r"\textasciitilde{}",
        "^": r"\textasciicircum{}",
    }
    return "".join(replacements.get(char, char) for char in text)


def _tex_path(path_value: Any) -> str:
    path = Path(str(path_value))
    if path.parts and path.parts[0] == "outputs":
        return "../" + path.as_posix()
    return path.as_posix()


def _tex_float(row: dict[str, Any], key: str, default: float = 0.0) -> float:
    return _safe_float(row.get(key), default)


def _mission_profile_pack_voltage_v(
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> float:
    configured_voltage = (
        max(float(config.mission_profile_battery_cell_count), 0.0)
        * max(config.mission_profile_cell_nominal_voltage_v, 0.0)
    )
    return configured_voltage if configured_voltage > 1e-9 else max(mission.battery_voltage_v, 1e-9)


def _mission_regime_energy_rows(
    row: dict[str, Any],
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> list[dict[str, float | str]]:
    voltage = _mission_profile_pack_voltage_v(mission, config)
    regime_specs = [
        (
            "Takeoff",
            "payload_on_climb_rpm",
            "payload_on_climb_power_w",
            config.mission_profile_takeoff_time_min,
        ),
        (
            "Loiter pre-drop",
            "payload_on_cruise_rpm",
            "payload_on_cruise_power_w",
            config.mission_profile_loiter_pre_drop_time_min,
        ),
        (
            "Drop / low-speed",
            "payload_on_slow_rpm",
            "payload_on_slow_power_w",
            config.mission_profile_drop_low_speed_time_min,
        ),
        (
            "Loiter post-drop",
            "post_delivery_cruise_rpm",
            "post_delivery_cruise_power_w",
            config.mission_profile_loiter_post_drop_time_min,
        ),
        (
            "Landing",
            "post_delivery_slow_rpm",
            "post_delivery_slow_power_w",
            config.mission_profile_landing_time_min,
        ),
    ]
    rows: list[dict[str, float | str]] = []
    for regime, rpm_key, power_key, time_min in regime_specs:
        power_w = _tex_float(row, power_key)
        energy_wh = power_w * max(time_min, 0.0) / 60.0
        current_a = power_w / voltage
        used_mah = energy_wh / voltage * 1000.0
        rows.append(
            {
                "regime": regime,
                "rpm": _tex_float(row, rpm_key),
                "time_min": time_min,
                "power_w": power_w,
                "current_a": current_a,
                "energy_wh": energy_wh,
                "used_mah": used_mah,
                "pack_percent": 0.0,
            }
        )
    total_energy_wh = sum(float(item["energy_wh"]) for item in rows)
    total_used_mah = sum(float(item["used_mah"]) for item in rows)
    capacity_required_mah = total_used_mah / max(1.0 - mission.battery_reserve_fraction, 1e-9)
    capacity_required_mah = max(capacity_required_mah, 1e-9)
    for item in rows:
        item["pack_percent"] = 100.0 * float(item["used_mah"]) / capacity_required_mah
    rows.append(
        {
            "regime": "Total",
            "rpm": float("nan"),
            "time_min": sum(float(item["time_min"]) for item in rows),
            "power_w": float("nan"),
            "current_a": float("nan"),
            "energy_wh": total_energy_wh,
            "used_mah": total_used_mah,
            "pack_percent": 100.0 * total_used_mah / capacity_required_mah,
        }
    )
    return rows


def _mission_regime_energy_summary(
    row: dict[str, Any],
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> dict[str, float]:
    rows = _mission_regime_energy_rows(row, mission, config)
    total = rows[-1]
    voltage = _mission_profile_pack_voltage_v(mission, config)
    usable_energy_wh = float(total["energy_wh"])
    used_capacity_mah = float(total["used_mah"])
    capacity_required_wh = usable_energy_wh / max(1.0 - mission.battery_reserve_fraction, 1e-9)
    capacity_required_mah = capacity_required_wh / voltage * 1000.0
    return {
        "mission_profile_battery_cell_count": float(config.mission_profile_battery_cell_count),
        "mission_profile_cell_nominal_voltage_v": config.mission_profile_cell_nominal_voltage_v,
        "mission_profile_pack_voltage_v": voltage,
        "mission_profile_usable_energy_wh": usable_energy_wh,
        "mission_profile_used_capacity_mah": used_capacity_mah,
        "mission_profile_capacity_required_wh": capacity_required_wh,
        "mission_profile_capacity_required_mah": capacity_required_mah,
        "mission_profile_reserve_fraction": mission.battery_reserve_fraction,
    }


def _mission_regime_markdown_lines(
    row: dict[str, Any],
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> list[str]:
    lines = [
        "| Regime | RPM | Time [min] | Total P [W] | Total I [A] | Energy [Wh] | Used [mAh] | % of sized 4S pack |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for item in _mission_regime_energy_rows(row, mission, config):
        is_total = item["regime"] == "Total"
        rpm = "--" if is_total else f"{float(item['rpm']):.0f}"
        power = "--" if is_total else f"{float(item['power_w']):.0f}"
        current = "--" if is_total else f"{float(item['current_a']):.1f}"
        lines.append(
            f"| {item['regime']} | {rpm} | {float(item['time_min']):.2f} | {power} | "
            f"{current} | {float(item['energy_wh']):.2f} | {float(item['used_mah']):.0f} | "
            f"{float(item['pack_percent']):.2f}% |"
        )
    return lines


def _mission_regime_tex_lines(
    row: dict[str, Any],
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> list[str]:
    lines: list[str] = []
    for item in _mission_regime_energy_rows(row, mission, config):
        if item["regime"] == "Total":
            lines.append(
                rf"\textbf{{Total}} & -- & \textbf{{{float(item['time_min']):.2f}}} & -- & -- & "
                rf"\textbf{{{float(item['energy_wh']):.2f}}} & \textbf{{{float(item['used_mah']):.0f}}} & "
                rf"\textbf{{{float(item['pack_percent']):.2f}\%}} \\"
            )
            continue
        lines.append(
            rf"{_tex_escape(str(item['regime']))} & \num{{{float(item['rpm']):.0f}}} & "
            rf"{float(item['time_min']):.2f} & {float(item['power_w']):.0f} & "
            rf"{float(item['current_a']):.1f} & {float(item['energy_wh']):.2f} & "
            rf"{float(item['used_mah']):.0f} & {float(item['pack_percent']):.2f}\% \\"
        )
    return lines


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def ensure_stage3_runtime() -> RuntimeModules:
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

    return RuntimeModules(asb=asb, onp=onp, plt=plt, patches=patches)


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
        out = float(value)
    except (TypeError, ValueError):
        return fallback
    if math.isnan(out) or math.isinf(out):
        return fallback
    return out


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def _stage1_candidate_from_row(row: dict[str, str]) -> Stage1Candidate:
    return Stage1Candidate(
        n_props=int(float(row["n_props"])),
        prop_diameter_in=float(row["prop_diameter_in"]),
        prop_pitch_ratio=float(row["prop_pitch_ratio"]),
        prop_family=row["prop_family"],
    )


def _resolve_repo_path(path_value: str | Path) -> Path:
    path = Path(path_value)
    if path.exists():
        return path
    return _repo_root() / path


def _csv_path(value: Any) -> Path | None:
    if not value:
        return None
    path = Path(str(value))
    if path.exists():
        return path
    repo_path = _repo_root() / path
    if repo_path.exists():
        return repo_path
    return path


def _first_csv_row(path: Path | None) -> dict[str, str] | None:
    if path is None:
        return None
    rows = load_csv_rows(path)
    return rows[0] if rows else None


_ECALC_PROP_CACHE: dict[tuple[str, str], dict[str, Any] | None] = {}


def _load_ecalc_prop_calibration(config: Stage3SizingConfig) -> dict[str, Any] | None:
    static_path = _resolve_repo_path(config.ecalc_static_csv)
    dynamic_path = _resolve_repo_path(config.ecalc_dynamic_csv)
    cache_key = (str(static_path), str(dynamic_path))
    if cache_key in _ECALC_PROP_CACHE:
        return _ECALC_PROP_CACHE[cache_key]

    static_rows = load_csv_rows(static_path)
    if not static_rows:
        _ECALC_PROP_CACHE[cache_key] = None
        return None

    table: list[dict[str, float]] = []
    for row in static_rows:
        rpm = _safe_float(row.get("rpm"), float("nan"))
        if math.isnan(rpm):
            continue
        table.append(
            {
                "rpm": rpm,
                "thrust_n": _safe_float(row.get("thrust_n"), 0.0),
                "electric_power_w": _safe_float(row.get("electric_power_w"), 0.0),
                "shaft_power_w": _safe_float(row.get("shaft_power_w"), 0.0),
                "ct_static": _safe_float(row.get("ct_static"), 0.0),
                "cp_static": _safe_float(row.get("cp_static"), 0.0),
            }
        )
    table.sort(key=lambda item: item["rpm"])
    if not table:
        _ECALC_PROP_CACHE[cache_key] = None
        return None

    dynamic_rows = load_csv_rows(dynamic_path)
    dynamic = dynamic_rows[0] if dynamic_rows else {}
    calibration = {
        "source": f"eCalc static table: {static_path}",
        "static_path": str(static_path),
        "dynamic_path": str(dynamic_path) if dynamic_rows else "",
        "table": table,
        "dynamic_advance_ratio": _safe_float(dynamic.get("advance_ratio"), 0.0),
        "dynamic_ct": _safe_float(dynamic.get("ct"), 0.0),
        "dynamic_cp": _safe_float(dynamic.get("cp"), 0.0),
        "dynamic_rpm": _safe_float(dynamic.get("rpm"), 0.0),
    }
    _ECALC_PROP_CACHE[cache_key] = calibration
    return calibration


def _interp_table_value(table: list[dict[str, float]], rpm: float, key: str) -> float:
    if rpm <= table[0]["rpm"]:
        return table[0][key]
    if rpm >= table[-1]["rpm"]:
        return table[-1][key]
    for lower, upper in zip(table[:-1], table[1:]):
        if lower["rpm"] <= rpm <= upper["rpm"]:
            span = max(upper["rpm"] - lower["rpm"], 1e-12)
            frac = (rpm - lower["rpm"]) / span
            return lower[key] + frac * (upper[key] - lower[key])
    return table[-1][key]


def _candidate_matches_ecalc(config: Stage3SizingConfig, candidate: Stage1Candidate) -> bool:
    return (
        int(candidate.n_props) == int(config.ecalc_reference_n_props)
        and abs(candidate.prop_diameter_in - config.ecalc_reference_prop_diameter_in)
        <= config.ecalc_match_diameter_tolerance_in
    )


def _effective_rpm_bounds_for_prop_model(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    rpm_bounds: tuple[int, int],
    config: Stage3SizingConfig,
) -> tuple[float, float]:
    lower, upper = [float(v) for v in rpm_bounds]
    if config.ecalc_propulsion_enabled and _candidate_matches_ecalc(config, candidate):
        calibration = _load_ecalc_prop_calibration(config)
        if calibration is not None and calibration.get("table"):
            table = calibration["table"]
            lower = max(lower, float(table[0]["rpm"]))
            upper = min(upper, float(table[-1]["rpm"]))
    if upper < lower:
        upper = lower
    return lower, upper


def _stage3_power_loss_breakdown(
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
    propulsion_electric_power_w: float,
) -> dict[str, float]:
    propulsion_base_w = max(propulsion_electric_power_w, 0.0)
    avionics_base_w = max(mission.avionics_power_w, 0.0)
    wiring_factor = max(config.wiring_power_loss_factor, 0.0)
    avionics_factor = max(config.avionics_power_loss_factor, 0.0)
    propulsion_with_wiring_w = propulsion_base_w * wiring_factor
    avionics_with_losses_w = avionics_base_w * avionics_factor
    total_power_w = propulsion_with_wiring_w + avionics_with_losses_w
    return {
        "propulsion_electric_power_without_aircraft_losses_w": propulsion_base_w,
        "propulsion_wiring_loss_w": propulsion_with_wiring_w - propulsion_base_w,
        "avionics_power_without_losses_w": avionics_base_w,
        "avionics_power_with_losses_w": avionics_with_losses_w,
        "avionics_power_loss_w": avionics_with_losses_w - avionics_base_w,
        "aircraft_power_losses_w": total_power_w - propulsion_base_w - avionics_base_w,
        "power_electric_total_w": total_power_w,
    }


def _apply_stage3_power_losses_to_operating_point(
    op: dict[str, float],
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> dict[str, float]:
    propulsion_power_w = max(op.get("power_electric_total_w", 0.0) - mission.avionics_power_w, 0.0)
    updated = dict(op)
    updated.update(_stage3_power_loss_breakdown(mission, config, propulsion_power_w))
    return updated


def _stage3_prop_operating_point(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    rpm: float,
    flight_speed_mps: float,
    config: Stage3SizingConfig,
) -> dict[str, float]:
    if not config.ecalc_propulsion_enabled or not _candidate_matches_ecalc(config, candidate):
        op = prop_operating_point(mission, candidate, rpm, flight_speed_mps)
        op = _apply_stage3_power_losses_to_operating_point(op, mission, config)
        op["propulsion_model_source"] = "Stage 1 generic prop surrogate"
        return op

    calibration = _load_ecalc_prop_calibration(config)
    if calibration is None:
        op = prop_operating_point(mission, candidate, rpm, flight_speed_mps)
        op = _apply_stage3_power_losses_to_operating_point(op, mission, config)
        op["propulsion_model_source"] = "Stage 1 generic prop surrogate; eCalc files unavailable"
        return op

    n_rev_per_sec = rpm / 60.0
    if n_rev_per_sec <= 1e-12:
        op = prop_operating_point(mission, candidate, rpm, flight_speed_mps)
        op = _apply_stage3_power_losses_to_operating_point(op, mission, config)
        op["propulsion_model_source"] = calibration["source"]
        return op

    table = calibration["table"]
    static_thrust_per_prop = _interp_table_value(table, rpm, "thrust_n")
    electric_power_per_prop = _interp_table_value(table, rpm, "electric_power_w")
    shaft_power_per_prop = _interp_table_value(table, rpm, "shaft_power_w")
    ct_static = _interp_table_value(table, rpm, "ct_static")
    cp_static = _interp_table_value(table, rpm, "cp_static")

    advance_ratio = flight_speed_mps / max(n_rev_per_sec * candidate.prop_diameter_m, 1e-12)
    dynamic_j = calibration["dynamic_advance_ratio"]
    dynamic_ct = calibration["dynamic_ct"]
    if dynamic_j > 1e-9 and dynamic_ct > 1e-9:
        dynamic_ct_static = _interp_table_value(table, calibration["dynamic_rpm"], "ct_static")
        ct_ratio_at_reference = _clamp(dynamic_ct / max(dynamic_ct_static, 1e-12), 0.20, 1.20)
        thrust_factor = _clamp(
            1.0 - (1.0 - ct_ratio_at_reference) * advance_ratio / dynamic_j,
            0.0,
            1.15,
        )
    else:
        thrust_factor = 1.0

    thrust_per_prop = static_thrust_per_prop * thrust_factor
    thrust_total = candidate.n_props * thrust_per_prop
    shaft_power_total = candidate.n_props * shaft_power_per_prop
    propulsion_electric_power_total = candidate.n_props * electric_power_per_prop
    power_losses = _stage3_power_loss_breakdown(mission, config, propulsion_electric_power_total)
    torque_per_prop = shaft_power_per_prop / max(2.0 * math.pi * n_rev_per_sec, 1e-9)
    tip_speed = math.sqrt((math.pi * candidate.prop_diameter_m * n_rev_per_sec) ** 2 + flight_speed_mps**2)
    tip_mach = tip_speed / mission.speed_of_sound_mps
    blade_re = (
        mission.air_density_kgpm3
        * math.sqrt((0.75 * math.pi * candidate.prop_diameter_m * n_rev_per_sec) ** 2 + flight_speed_mps**2)
        * mission.blade_chord_to_diameter
        * candidate.prop_diameter_m
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )

    return {
        "rpm": rpm,
        "advance_ratio": advance_ratio,
        "ct": ct_static * thrust_factor,
        "cp": max(calibration["dynamic_cp"], cp_static) if calibration["dynamic_cp"] > 0.0 else cp_static,
        "thrust_total_n": thrust_total,
        "power_shaft_total_w": shaft_power_total,
        "power_shaft_per_prop_w": shaft_power_per_prop,
        "power_electric_total_w": power_losses["power_electric_total_w"],
        **power_losses,
        "torque_per_prop_nm": torque_per_prop,
        "tip_mach": tip_mach,
        "blade_reynolds": blade_re,
        "reynolds_penalty": 1.0,
        "propulsion_model_source": calibration["source"],
    }



def _curve_column_peak(
    rows: list[dict[str, str]],
    cl_column: str,
) -> tuple[float | None, float | None]:
    best_cl: float | None = None
    best_alpha: float | None = None
    for row in rows:
        if cl_column not in row:
            continue
        value = _safe_float(row.get(cl_column), float("nan"))
        if math.isnan(value):
            continue
        if best_cl is None or value > best_cl:
            best_cl = value
            best_alpha = _safe_float(row.get("alpha_deg"), float("nan"))
    return best_cl, best_alpha


def _matching_flap_sweep_row(
    rows: list[dict[str, str]],
    flap_span_fraction: float,
    flap_chord_fraction: float,
    flap_deflection_deg: float,
) -> dict[str, str] | None:
    if not rows:
        return None
    return min(
        rows,
        key=lambda row: (
            abs(_safe_float(row.get("flap_span_fraction"), flap_span_fraction) - flap_span_fraction)
            + abs(_safe_float(row.get("flap_chord_fraction"), flap_chord_fraction) - flap_chord_fraction)
            + 0.02 * abs(_safe_float(row.get("flap_deflection_deg"), flap_deflection_deg) - flap_deflection_deg)
        ),
    )


def _matching_motor_trade_row(
    rows: list[dict[str, str]],
    scenario: str,
    prop_drop_fraction: float,
) -> dict[str, str] | None:
    matching_scenario = [row for row in rows if row.get("scenario") == scenario]
    if not matching_scenario:
        return None
    return min(
        matching_scenario,
        key=lambda row: abs(_safe_float(row.get("prop_drop_fraction_of_chord"), 0.0) - prop_drop_fraction),
    )


def _apply_stage_high_lift_context(
    context: dict[str, Any],
    workflow_row: dict[str, str],
) -> None:
    control_path = _csv_path(workflow_row.get("control_summary_csv"))
    control_row = _first_csv_row(control_path)
    motor_path = _csv_path(workflow_row.get("motor_summary_csv"))

    context.update(
        {
            "clmax_source": str(control_path) if control_path else "Stage3SizingConfig fallback",
            "clmax_curve_source": "",
            "no_flap_clmax": None,
            "flap_only_clmax": None,
            "clean_blowing_clmax": None,
            "flap_down_blown_clmax": None,
            "no_flap_clmax_alpha_deg": None,
            "flap_only_clmax_alpha_deg": None,
            "clean_blowing_clmax_alpha_deg": None,
            "flap_down_blown_clmax_alpha_deg": None,
            "total_cl_curve_rows": [],
            "flap_sweep_source": "",
            "stage12_clean_section_clmax_unblown": None,
            "stage12_clean_section_clmax_blown": None,
            "stage12_flapped_section_clmax_unblown": None,
            "stage12_flapped_section_clmax_blown": None,
            "stage12_slot_gain": None,
            "stage12_flap_area_m2": None,
            "stage12_blown_flap_area_m2": None,
            "stage12_blown_clean_area_m2": None,
        }
    )

    selected_drop = _safe_float(workflow_row.get("optimized_motor_drop_fraction_of_chord"), 0.0)
    curve_path: Path | None = None
    if control_row:
        selected_drop = _safe_float(control_row.get("prop_drop_fraction_of_chord"), selected_drop)
        context["flap_deflection_slow_deg"] = _safe_float(
            control_row.get("recommended_flap_deflection_deg"),
            context.get("flap_deflection_slow_deg", 40.0),
        )
        context["flap_down_blown_clmax"] = _safe_float(
            control_row.get("equivalent_reference_clmax"),
            context["flap_down_blown_clmax"],
        )
        context["flap_down_blown_clmax_alpha_deg"] = _safe_float(
            control_row.get("alpha_at_clmax_deg"),
            context["flap_down_blown_clmax_alpha_deg"],
        )
        curve_path = _csv_path(control_row.get("total_cl_curve_csv"))

        flap_sweep_path = control_path.with_name("flap_sweep.csv") if control_path else None
        flap_sweep_rows = load_csv_rows(flap_sweep_path) if flap_sweep_path and flap_sweep_path.exists() else []
        flap_sweep_row = _matching_flap_sweep_row(
            flap_sweep_rows,
            _safe_float(control_row.get("recommended_flap_span_fraction"), context["flap_span_fraction"]),
            _safe_float(control_row.get("recommended_flap_chord_fraction"), context["flap_chord_fraction"]),
            _safe_float(control_row.get("recommended_flap_deflection_deg"), context["flap_deflection_slow_deg"]),
        )
        if flap_sweep_row:
            context["flap_sweep_source"] = str(flap_sweep_path)
            context["stage12_clean_section_clmax_unblown"] = _safe_float(
                flap_sweep_row.get("clean_clmax_unblown"),
                context["stage12_clean_section_clmax_unblown"],
            )
            context["stage12_clean_section_clmax_blown"] = _safe_float(
                flap_sweep_row.get("clean_clmax_blown"),
                context["stage12_clean_section_clmax_blown"],
            )
            context["stage12_flapped_section_clmax_unblown"] = _safe_float(
                flap_sweep_row.get("flapped_clmax_unblown"),
                context["stage12_flapped_section_clmax_unblown"],
            )
            context["stage12_flapped_section_clmax_blown"] = _safe_float(
                flap_sweep_row.get("flapped_clmax_blown"),
                context["stage12_flapped_section_clmax_blown"],
            )
            context["stage12_slot_gain"] = _safe_float(
                flap_sweep_row.get("slot_gain"),
                context["stage12_slot_gain"],
            )
            context["stage12_flap_area_m2"] = _safe_float(
                flap_sweep_row.get("flap_area_m2"),
                context["stage12_flap_area_m2"],
            )
            context["stage12_blown_flap_area_m2"] = _safe_float(
                flap_sweep_row.get("blown_flap_area_m2"),
                context["stage12_blown_flap_area_m2"],
            )
            context["stage12_blown_clean_area_m2"] = _safe_float(
                flap_sweep_row.get("blown_clean_area_m2"),
                context["stage12_blown_clean_area_m2"],
            )

    curve_rows = load_csv_rows(curve_path) if curve_path else []
    if curve_rows:
        context["clmax_curve_source"] = str(curve_path)
        context["total_cl_curve_rows"] = curve_rows
        curve_columns = {
            "no_flap_clmax": "cl_clean_baseline",
            "flap_only_clmax": "cl_flap_only",
            "clean_blowing_clmax": "cl_blow_only",
            "flap_down_blown_clmax": "cl_all_high_lift",
        }
        for context_key, curve_column in curve_columns.items():
            peak_cl, peak_alpha = _curve_column_peak(curve_rows, curve_column)
            if peak_cl is not None and (
                context.get(context_key) is None or context_key != "flap_down_blown_clmax"
            ):
                context[context_key] = peak_cl
            if peak_alpha is not None and (
                context.get(f"{context_key}_alpha_deg") is None or context_key != "flap_down_blown_clmax"
            ):
                context[f"{context_key}_alpha_deg"] = peak_alpha

    motor_rows = load_csv_rows(motor_path) if motor_path else []
    if motor_rows:
        clean_blowing = _matching_motor_trade_row(motor_rows, "clean_blowing", selected_drop)
        flap_blowing = _matching_motor_trade_row(motor_rows, "slotted_flap_blowing", selected_drop)
        if clean_blowing and context.get("clean_blowing_clmax") is None:
            context["clean_blowing_clmax"] = _safe_float(
                clean_blowing.get("equivalent_reference_clmax"),
                context["clean_blowing_clmax"],
            )
            context["clean_blowing_clmax_alpha_deg"] = _safe_float(
                clean_blowing.get("alpha_at_peak_total_cl_deg"),
                context["clean_blowing_clmax_alpha_deg"],
            )
        if flap_blowing and context.get("flap_down_blown_clmax") is None:
            context["flap_down_blown_clmax"] = _safe_float(
                flap_blowing.get("equivalent_reference_clmax"),
                context["flap_down_blown_clmax"],
            )
            context["flap_down_blown_clmax_alpha_deg"] = _safe_float(
                flap_blowing.get("alpha_at_peak_total_cl_deg"),
                context["flap_down_blown_clmax_alpha_deg"],
            )


def load_selected_wing_context(
    config: Stage3SizingConfig,
) -> dict[str, Any]:
    """Load frozen airfoil and control geometry from the wing workflow if present."""

    context: dict[str, Any] = {
        "selected_airfoil": config.main_airfoil_default,
        "flap_span_fraction": config.flap_span_fraction_default,
        "flap_chord_fraction": config.flap_chord_fraction_default,
        "aileron_span_fraction": config.aileron_span_fraction_default,
        "aileron_chord_fraction": config.aileron_chord_fraction_default,
        "fixed_n_props": None,
        "fixed_prop_diameter_in": None,
        "fixed_prop_pitch_ratio": None,
        "fixed_prop_family": None,
        "flap_deflection_slow_deg": config.flap_deflection_slow_deg,
        "clmax_source": "Stage3SizingConfig fallback",
        "clmax_curve_source": "",
        "no_flap_clmax": None,
        "flap_only_clmax": None,
        "clean_blowing_clmax": None,
        "flap_down_blown_clmax": None,
        "no_flap_clmax_alpha_deg": None,
        "flap_only_clmax_alpha_deg": None,
        "clean_blowing_clmax_alpha_deg": None,
        "flap_down_blown_clmax_alpha_deg": None,
        "total_cl_curve_rows": [],
        "flap_sweep_source": "",
        "stage12_clean_section_clmax_unblown": None,
        "stage12_clean_section_clmax_blown": None,
        "stage12_flapped_section_clmax_unblown": None,
        "stage12_flapped_section_clmax_blown": None,
        "stage12_slot_gain": None,
        "stage12_flap_area_m2": None,
        "stage12_blown_flap_area_m2": None,
        "stage12_blown_clean_area_m2": None,
        "source": "Stage3SizingConfig defaults",
    }

    summary_paths = sorted(Path("outputs/wing_workflow").glob("*/wing_workflow_summary.csv"))
    for path in summary_paths:
        rows = load_csv_rows(path)
        if not rows:
            continue
        row = rows[0]
        prop_diameter_in = _safe_float(row.get("prop_diameter_in"), 0.0)
        prop_pitch_in = _safe_float(row.get("prop_pitch_in"), 0.0)
        prop_pitch_ratio = (
            prop_pitch_in / prop_diameter_in
            if prop_diameter_in > 1e-9 and prop_pitch_in > 1e-9
            else None
        )
        if config.fixed_propulsion_enabled:
            workflow_matches_fixed_propulsion = all(
                (
                    int(float(row["n_props"])) == int(config.fixed_n_props),
                    abs(prop_diameter_in - config.fixed_prop_diameter_in) <= 5e-3,
                    prop_pitch_ratio is not None
                    and abs(prop_pitch_ratio - config.fixed_prop_pitch_ratio) <= 5e-3,
                    row.get("prop_family") == config.fixed_prop_family,
                )
            )
            if not workflow_matches_fixed_propulsion:
                continue
        context.update(
            {
                "selected_airfoil": row.get("airfoil", context["selected_airfoil"]),
                "fixed_n_props": int(float(row["n_props"])) if row.get("n_props") else None,
                "fixed_prop_diameter_in": prop_diameter_in if prop_diameter_in > 0.0 else None,
                "fixed_prop_pitch_ratio": prop_pitch_ratio,
                "fixed_prop_family": row.get("prop_family") or None,
                "flap_span_fraction": _safe_float(
                    row.get("control_flap_span_fraction"),
                    context["flap_span_fraction"],
                ),
                "flap_chord_fraction": _safe_float(
                    row.get("control_flap_chord_fraction"),
                    context["flap_chord_fraction"],
                ),
                "aileron_span_fraction": _safe_float(
                    row.get("control_aileron_span_fraction"),
                    context["aileron_span_fraction"],
                ),
                "aileron_chord_fraction": _safe_float(
                    row.get("control_aileron_chord_fraction"),
                    context["aileron_chord_fraction"],
                ),
                "source": str(path),
            }
        )
        _apply_stage_high_lift_context(context, row)
        break

    return context


def _trapezoid_area(span: float, root_chord: float, tip_chord: float) -> float:
    return 0.5 * span * (root_chord + tip_chord)


def _trapezoid_mac(root_chord: float, tip_chord: float) -> float:
    taper = tip_chord / max(root_chord, 1e-9)
    return (2.0 / 3.0) * root_chord * (1.0 + taper + taper**2) / max(1.0 + taper, 1e-9)


def _finite_wing_lift_slope_per_rad(aspect_ratio: float) -> float:
    return 2.0 * math.pi * aspect_ratio / max(aspect_ratio + 2.0, 1e-9)


def _workflow_curve_polar(
    wing_context: dict[str, Any],
    mode: str,
) -> dict[str, Any] | None:
    rows = wing_context.get("total_cl_curve_rows") or []
    if not rows:
        return None
    if mode == "flaps_down":
        columns = {
            "CL": "cl_all_high_lift",
            "CD": "cd_all_high_lift",
            "CM": "cm_all_high_lift",
        }
    elif mode == "clean":
        columns = {
            "CL": "cl_clean_baseline",
            "CD": "cd_clean_baseline",
            "CM": "cm_clean_baseline",
        }
    else:
        return None

    alpha: list[float] = []
    cl: list[float] = []
    cd: list[float] = []
    cm: list[float] = []
    for row in rows:
        a = _safe_float(row.get("alpha_deg"), float("nan"))
        values = {
            key: _safe_float(row.get(column), float("nan"))
            for key, column in columns.items()
        }
        if math.isnan(a) or any(math.isnan(value) for value in values.values()):
            continue
        alpha.append(a)
        cl.append(values["CL"])
        cd.append(values["CD"])
        cm.append(values["CM"])
    if not alpha:
        return None
    runtime = ensure_stage3_runtime()
    order = runtime.onp.argsort(runtime.onp.asarray(alpha, dtype=float))
    return {
        "alpha_deg": runtime.onp.asarray(alpha, dtype=float)[order],
        "CL": runtime.onp.asarray(cl, dtype=float)[order],
        "CD": runtime.onp.asarray(cd, dtype=float)[order],
        "CM": runtime.onp.asarray(cm, dtype=float)[order],
        "source": wing_context.get("clmax_curve_source", ""),
    }


def _airfoil_area_coefficient(airfoil_name: str) -> float:
    runtime = ensure_stage3_runtime()
    coords = runtime.onp.asarray(runtime.asb.Airfoil(airfoil_name).coordinates, dtype=float)
    x = coords[:, 0]
    y = coords[:, 1]
    return float(abs(0.5 * runtime.onp.sum(x * runtime.onp.roll(y, -1) - y * runtime.onp.roll(x, -1))))


def _foam_volume_trapezoid(
    span: float,
    root_chord: float,
    tip_chord: float,
    section_area_coefficient: float,
) -> float:
    return section_area_coefficient * span * (
        root_chord**2 + root_chord * tip_chord + tip_chord**2
    ) / 3.0


def _make_tail_geometry_from_variables(values: list[float] | tuple[float, ...]) -> dict[str, float]:
    (
        h_area,
        h_aspect_ratio,
        h_taper,
        v_area,
        v_aspect_ratio,
        v_taper,
        tail_arm,
        main_wing_incidence_deg,
    ) = [float(v) for v in values]

    h_span = math.sqrt(max(h_area * h_aspect_ratio, 1e-9))
    h_root_chord = 2.0 * h_area / max(h_span * (1.0 + h_taper), 1e-9)
    h_tip_chord = h_root_chord * h_taper

    v_height = math.sqrt(max(v_area * v_aspect_ratio, 1e-9))
    v_root_chord = 2.0 * v_area / max(v_height * (1.0 + v_taper), 1e-9)
    v_tip_chord = v_root_chord * v_taper

    return {
        "htail_area_m2": h_area,
        "htail_aspect_ratio": h_aspect_ratio,
        "htail_taper": h_taper,
        "htail_span_m": h_span,
        "htail_root_chord_m": h_root_chord,
        "htail_tip_chord_m": h_tip_chord,
        "htail_mac_m": _trapezoid_mac(h_root_chord, h_tip_chord),
        "vtail_area_m2": v_area,
        "vtail_aspect_ratio": v_aspect_ratio,
        "vtail_taper": v_taper,
        "vtail_span_m": v_height,
        "vtail_root_chord_m": v_root_chord,
        "vtail_tip_chord_m": v_tip_chord,
        "vtail_mac_m": _trapezoid_mac(v_root_chord, v_tip_chord),
        "tail_arm_m": tail_arm,
        "main_wing_incidence_deg": main_wing_incidence_deg,
    }


def _optimization_bounds(config: Stage3SizingConfig) -> list[tuple[float, float]]:
    return [
        config.h_tail_area_bounds_m2,
        config.h_tail_aspect_ratio_bounds,
        config.h_tail_taper_bounds,
        config.v_tail_area_bounds_m2,
        config.v_tail_aspect_ratio_bounds,
        config.v_tail_taper_bounds,
        config.tail_arm_bounds_m,
        config.main_wing_incidence_bounds_deg,
    ]


def _seed_vectors(
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> list[list[float]]:
    bounds = _optimization_bounds(config)
    seeds: list[list[float]] = []
    for tail_arm in (0.95, 1.15, 1.35):
        for vh, vv, h_ar, v_ar in (
            (0.52, 0.040, 4.0, 1.5),
            (0.62, 0.050, 4.5, 1.7),
            (0.74, 0.060, 5.0, 1.9),
            (0.86, 0.072, 5.5, 2.1),
        ):
            h_area = vh * mission.wing_area_m2 * mission.chord_m / tail_arm
            v_area = vv * mission.wing_area_m2 * mission.span_m / tail_arm
            raw = [
                h_area,
                h_ar,
                0.75,
                v_area,
                v_ar,
                0.72,
                tail_arm,
                config.main_wing_incidence_deg,
            ]
            seeds.append([
                _clamp(value, lower, upper)
                for value, (lower, upper) in zip(raw, bounds)
            ])
    return seeds


def _main_airfoil_polar(
    airfoil_name: str,
    reynolds: float,
) -> dict[str, Any]:
    runtime = ensure_stage3_runtime()
    alphas = runtime.onp.linspace(-8.0, 18.0, 131)
    airfoil = runtime.asb.Airfoil(airfoil_name)
    try:
        data = airfoil.get_aero_from_neuralfoil(alpha=alphas, Re=reynolds)
        cl = runtime.onp.asarray(data["CL"], dtype=float)
        cd = runtime.onp.asarray(data["CD"], dtype=float)
        cm = runtime.onp.asarray(data["CM"], dtype=float)
    except Exception:
        cl = 0.10 * alphas + 0.2
        cd = 0.012 + 0.008 * cl**2
        cm = -0.06 + 0.0 * alphas
    return {"alpha_deg": alphas, "CL": cl, "CD": cd, "CM": cm}


def _slow_flap_deflection_deg(
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
) -> float:
    return _safe_float(wing_context.get("flap_deflection_slow_deg"), config.flap_deflection_slow_deg)


def _stage3_slow_flight_speed_mps(config: Stage3SizingConfig) -> float:
    return max(config.slow_flight_speed_mps, 1e-6)


def _prop_axial_x_m(
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> float:
    return config.prop_axial_location_fraction_of_chord * mission.chord_m


def _horizontal_tail_volume_range(config: Stage3SizingConfig) -> tuple[float, float]:
    lower, upper = config.horizontal_tail_volume_range
    if upper <= lower:
        return config.min_horizontal_tail_volume, config.max_horizontal_tail_volume
    return lower, upper


def _vertical_tail_volume_range(config: Stage3SizingConfig) -> tuple[float, float]:
    lower, upper = config.vertical_tail_volume_range
    if upper <= lower:
        return config.min_vertical_tail_volume, config.max_vertical_tail_volume
    return lower, upper


def _flap_down_clmax_values(
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
) -> dict[str, Any]:
    """Physical CLmax values for the selected Stage 1/2 high-lift workflow.

    Stage 1/2 already generates NeuralFoil/slotted-flap CL-alpha curves and
    combined whole-wing high-lift curves. Use those first. The YAML values are
    only fallback caps if the workflow artifacts are not present.
    """

    flap_span_fraction = _clamp(_safe_float(wing_context.get("flap_span_fraction"), 0.65), 0.0, 1.0)
    fallback_unblown = config.flap_down_3d_efficiency * (
        flap_span_fraction * config.slotted_flap_section_clmax
        + (1.0 - flap_span_fraction) * config.clean_outer_section_clmax
    )
    fallback_blown = config.blown_flap_clmax_multiplier * fallback_unblown
    raw_no_flap = _safe_float(wing_context.get("no_flap_clmax"), config.clean_outer_section_clmax)
    raw_flap_only = _safe_float(wing_context.get("flap_only_clmax"), fallback_unblown)
    raw_clean_blowing = _safe_float(wing_context.get("clean_blowing_clmax"), raw_no_flap)
    raw_blown = _safe_float(wing_context.get("flap_down_blown_clmax"), fallback_blown)

    clean_section_unblown = _safe_float(
        wing_context.get("stage12_clean_section_clmax_unblown"),
        config.physical_clean_clmax_cap,
    )
    clean_section_blown = _safe_float(
        wing_context.get("stage12_clean_section_clmax_blown"),
        clean_section_unblown,
    )
    flapped_section_unblown = _safe_float(
        wing_context.get("stage12_flapped_section_clmax_unblown"),
        config.physical_flapped_clmax_cap,
    )
    flapped_section_blown = _safe_float(
        wing_context.get("stage12_flapped_section_clmax_blown"),
        flapped_section_unblown,
    )
    stage12_section_available = bool(wing_context.get("flap_sweep_source")) and all(
        value > 0.0 and not math.isnan(value)
        for value in (
            clean_section_unblown,
            clean_section_blown,
            flapped_section_unblown,
            flapped_section_blown,
        )
    )

    blown_clean_area = _safe_float(wing_context.get("stage12_blown_clean_area_m2"), float("nan"))
    blown_flap_area = _safe_float(wing_context.get("stage12_blown_flap_area_m2"), float("nan"))
    if (
        stage12_section_available
        and not math.isnan(blown_clean_area)
        and not math.isnan(blown_flap_area)
        and blown_clean_area + blown_flap_area > 1e-9
    ):
        physical_blown_cap = (
            blown_clean_area * clean_section_blown
            + blown_flap_area * flapped_section_blown
        ) / (blown_clean_area + blown_flap_area)
    else:
        physical_blown_cap = flapped_section_blown if stage12_section_available else config.physical_flapped_clmax_cap

    no_flap_cap = clean_section_unblown if stage12_section_available else config.physical_clean_clmax_cap
    flap_only_cap = flapped_section_unblown if stage12_section_available else config.physical_flapped_clmax_cap
    clean_blowing_cap = clean_section_blown if stage12_section_available else config.physical_clean_clmax_cap
    cap_source = (
        wing_context.get("flap_sweep_source")
        if stage12_section_available
        else "Stage3SizingConfig physical CLmax fallback"
    )

    no_flap = min(raw_no_flap, no_flap_cap)
    flap_only = min(raw_flap_only, flap_only_cap)
    clean_blowing = min(raw_clean_blowing, clean_blowing_cap)
    blown = min(raw_blown, physical_blown_cap)
    unblown = flap_only
    design_unblown = unblown / max(config.slow_flight_stall_margin_factor, 1e-9)
    design_blown = blown / max(config.slow_flight_stall_margin_factor, 1e-9)
    was_capped = any(
        raw > capped + 1e-9
        for raw, capped in (
            (raw_no_flap, no_flap),
            (raw_flap_only, flap_only),
            (raw_clean_blowing, clean_blowing),
            (raw_blown, blown),
        )
    )
    return {
        "flap_span_fraction": flap_span_fraction,
        "source": wing_context.get("clmax_source", "Stage3SizingConfig fallback"),
        "curve_source": wing_context.get("clmax_curve_source", ""),
        "physical_clmax_source": cap_source,
        "stage12_clean_section_clmax_unblown": clean_section_unblown,
        "stage12_clean_section_clmax_blown": clean_section_blown,
        "stage12_flapped_section_clmax_unblown": flapped_section_unblown,
        "stage12_flapped_section_clmax_blown": flapped_section_blown,
        "stage12_weighted_blown_physical_clmax": physical_blown_cap,
        "raw_no_flap_clmax": raw_no_flap,
        "raw_flap_only_clmax": raw_flap_only,
        "raw_clean_blowing_clmax": raw_clean_blowing,
        "raw_flap_down_blown_clmax": raw_blown,
        "no_flap_clmax": no_flap,
        "flap_only_clmax": flap_only,
        "clean_blowing_clmax": clean_blowing,
        "flap_down_blown_clmax": blown,
        "no_flap_clmax_alpha_deg": _safe_float(wing_context.get("no_flap_clmax_alpha_deg"), float("nan")),
        "flap_only_clmax_alpha_deg": _safe_float(wing_context.get("flap_only_clmax_alpha_deg"), float("nan")),
        "clean_blowing_clmax_alpha_deg": _safe_float(wing_context.get("clean_blowing_clmax_alpha_deg"), float("nan")),
        "flap_down_blown_clmax_alpha_deg": _safe_float(wing_context.get("flap_down_blown_clmax_alpha_deg"), float("nan")),
        "unblown_clmax": unblown,
        "blown_clmax": blown,
        "design_unblown_clmax": design_unblown,
        "design_blown_clmax": design_blown,
        "clmax_was_capped": 1.0 if was_capped else 0.0,
    }


def _interp_polar(polar: dict[str, Any], key: str, alpha_deg: float) -> float:
    runtime = ensure_stage3_runtime()
    return float(runtime.onp.interp(alpha_deg, polar["alpha_deg"], polar[key]))


def _alpha_for_cl(polar: dict[str, Any], cl_target: float) -> float:
    runtime = ensure_stage3_runtime()
    cl = runtime.onp.asarray(polar["CL"], dtype=float)
    alpha = runtime.onp.asarray(polar["alpha_deg"], dtype=float)
    peak_index = int(runtime.onp.argmax(cl))
    cl_branch = cl[: peak_index + 1]
    alpha_branch = alpha[: peak_index + 1]
    order = runtime.onp.argsort(cl_branch)
    return float(runtime.onp.interp(cl_target, cl_branch[order], alpha_branch[order]))


def _solve_prop_power_for_thrust(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    velocity_mps: float,
    thrust_required_n: float,
    rpm_bounds: tuple[int, int],
    config: Stage3SizingConfig,
) -> dict[str, float]:
    lower, upper = _effective_rpm_bounds_for_prop_model(mission, candidate, rpm_bounds, config)

    def residual(rpm: float) -> float:
        return _stage3_prop_operating_point(mission, candidate, rpm, velocity_mps, config)["thrust_total_n"] - thrust_required_n

    r_lower = residual(lower)
    if r_lower >= 0.0:
        op = _stage3_prop_operating_point(mission, candidate, lower, velocity_mps, config)
        op["thrust_required_n"] = thrust_required_n
        op["thrust_margin_n"] = op["thrust_total_n"] - thrust_required_n
        op["rpm_feasible"] = 1.0
        return op

    r_upper = residual(upper)
    feasible = r_upper >= 0.0
    if not feasible:
        op = _stage3_prop_operating_point(mission, candidate, upper, velocity_mps, config)
        op["thrust_required_n"] = thrust_required_n
        op["thrust_margin_n"] = op["thrust_total_n"] - thrust_required_n
        op["rpm_feasible"] = 0.0
        return op

    for _ in range(72):
        mid = 0.5 * (lower + upper)
        r_mid = residual(mid)
        if abs(upper - lower) <= mission.rpm_solver_tolerance_rpm:
            break
        if r_mid >= 0.0:
            upper = mid
        else:
            lower = mid

    rpm = 0.5 * (lower + upper)
    op = _stage3_prop_operating_point(mission, candidate, rpm, velocity_mps, config)
    op["thrust_required_n"] = thrust_required_n
    op["thrust_margin_n"] = op["thrust_total_n"] - thrust_required_n
    op["rpm_feasible"] = 1.0
    return op


def _evaluate_tail_proxy(
    values: list[float] | tuple[float, ...],
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
    candidate: Stage1Candidate,
    stage1_row: dict[str, str],
    wing_context: dict[str, Any],
    main_polar: dict[str, Any],
    tail_area_coeff: float,
) -> dict[str, Any]:
    geometry = _make_tail_geometry_from_variables(values)
    q_cruise = 0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2
    slow_flight_speed = _stage3_slow_flight_speed_mps(config)
    q_low = 0.5 * mission.air_density_kgpm3 * slow_flight_speed**2
    s_wing = mission.wing_area_m2
    mac = mission.chord_m
    aspect_ratio = mission.aspect_ratio
    weight_n = mission.gross_weight_n

    h_volume = geometry["htail_area_m2"] * geometry["tail_arm_m"] / max(s_wing * mac, 1e-9)
    v_volume = geometry["vtail_area_m2"] * geometry["tail_arm_m"] / max(s_wing * mission.span_m, 1e-9)

    h_volume_mass = _foam_volume_trapezoid(
        geometry["htail_span_m"],
        geometry["htail_root_chord_m"],
        geometry["htail_tip_chord_m"],
        tail_area_coeff,
    ) * config.foam_density_kgpm3
    v_volume_mass = _foam_volume_trapezoid(
        geometry["vtail_span_m"],
        geometry["vtail_root_chord_m"],
        geometry["vtail_tip_chord_m"],
        tail_area_coeff,
    ) * config.foam_density_kgpm3
    tail_foam_mass = h_volume_mass + v_volume_mass

    stage1_total_mass = _safe_float(stage1_row.get("total_built_mass_kg"), mission.fixed_system_mass_kg)
    stage3_total_mass = stage1_total_mass + tail_foam_mass

    wing_ac_x = config.wing_aerodynamic_center_fraction_mac * mac
    htail_ac_x = wing_ac_x + geometry["tail_arm_m"]
    vtail_ac_x = htail_ac_x
    target_cg_x = config.target_cg_fraction_mac * mac
    required_base_cg_x = (
        target_cg_x * stage3_total_mass
        - h_volume_mass * htail_ac_x
        - v_volume_mass * vtail_ac_x
    ) / max(stage1_total_mass, 1e-9)
    if config.enforce_target_cg:
        lower_cg_x = config.baseline_cg_fraction_bounds_mac[0] * mac
        upper_cg_x = config.baseline_cg_fraction_bounds_mac[1] * mac
        base_cg_x = _clamp(required_base_cg_x, lower_cg_x, upper_cg_x)
    else:
        base_cg_x = config.baseline_cg_fraction_mac * mac
    base_cg_fraction = base_cg_x / max(mac, 1e-9)
    cg_x = (
        stage1_total_mass * base_cg_x
        + h_volume_mass * htail_ac_x
        + v_volume_mass * vtail_ac_x
    ) / max(stage3_total_mass, 1e-9)
    cg_fraction = cg_x / max(mac, 1e-9)
    cg_error_m = cg_x - target_cg_x
    required_base_cg_fraction = required_base_cg_x / max(mac, 1e-9)

    main_wing_incidence_deg = geometry["main_wing_incidence_deg"]
    cl_wing_cruise = weight_n / max(q_cruise * s_wing, 1e-9)
    alpha_wing_section = _alpha_for_cl(main_polar, cl_wing_cruise)
    cruise_body_alpha_proxy_deg = alpha_wing_section - main_wing_incidence_deg
    cm_wing = _interp_polar(main_polar, "CM", alpha_wing_section)
    cd_profile = max(_interp_polar(main_polar, "CD", alpha_wing_section), 0.006)

    tail_cl_cruise = (
        cm_wing + cl_wing_cruise * (cg_fraction - config.wing_aerodynamic_center_fraction_mac)
    ) / max(config.tail_dynamic_pressure_ratio * h_volume, 1e-9)
    tail_lift_slope = _finite_wing_lift_slope_per_rad(geometry["htail_aspect_ratio"])
    downwash_deg = config.downwash_gradient * alpha_wing_section
    htail_incidence_deg = (
        math.degrees(tail_cl_cruise / max(tail_lift_slope, 1e-9))
        - cruise_body_alpha_proxy_deg
        + downwash_deg
    )

    wing_lift_slope = _finite_wing_lift_slope_per_rad(aspect_ratio)
    x_np = wing_ac_x + (
        config.tail_dynamic_pressure_ratio
        * (tail_lift_slope / max(wing_lift_slope, 1e-9))
        * (1.0 - config.downwash_gradient)
        * (geometry["htail_area_m2"] / max(s_wing, 1e-9))
        * geometry["tail_arm_m"]
    )
    static_margin = (x_np - cg_x) / max(mac, 1e-9)

    induced_wing = cl_wing_cruise**2 / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
    cd_wing = cd_profile + induced_wing
    cd_h_tail = config.tail_zero_lift_cd + tail_cl_cruise**2 / max(
        math.pi * 0.82 * geometry["htail_aspect_ratio"],
        1e-9,
    )
    cd_v_tail = config.tail_zero_lift_cd
    cruise_wing_profile_drag = q_cruise * s_wing * cd_profile
    cruise_wing_induced_drag = q_cruise * s_wing * induced_wing
    cruise_htail_drag = q_cruise * geometry["htail_area_m2"] * cd_h_tail
    cruise_vtail_drag = q_cruise * geometry["vtail_area_m2"] * cd_v_tail
    cruise_fuselage_drag = q_cruise * config.fuselage_cd_area_m2
    cruise_drag = (
        cruise_wing_profile_drag
        + cruise_wing_induced_drag
        + cruise_htail_drag
        + cruise_vtail_drag
        + cruise_fuselage_drag
    )
    cruise_op = _solve_prop_power_for_thrust(
        mission,
        candidate,
        mission.cruise_speed_mps,
        mission.thrust_margin_cruise * cruise_drag,
        mission.cruise_rpm_bounds,
        config,
    )
    cruise_max_op = _stage3_prop_operating_point(
        mission,
        candidate,
        _effective_rpm_bounds_for_prop_model(mission, candidate, mission.cruise_rpm_bounds, config)[1],
        mission.cruise_speed_mps,
        config,
    )

    effective_weight_n = max(weight_n, stage3_total_mass * mission.gravity_mps2)
    eta_blown = _safe_float(stage1_row.get("low_speed_blown_span_fraction"), 0.35)
    eta_blown = _clamp(eta_blown, 0.05, 0.95)
    s_blown = eta_blown * s_wing
    s_unblown = (1.0 - eta_blown) * s_wing
    cruise_actual_veff = slipstream_velocity_after_prop(
        mission,
        candidate,
        cruise_op["thrust_total_n"],
        mission.cruise_speed_mps,
    )
    q_cruise_blown = 0.5 * mission.air_density_kgpm3 * cruise_actual_veff**2
    q_cruise_effective = (q_cruise * s_unblown + q_cruise_blown * s_blown) / max(s_wing, 1e-9)
    cruise_uniform_local_cl_required = effective_weight_n / max(
        q_cruise * s_unblown + q_cruise_blown * s_blown,
        1e-9,
    )
    flap_down_clmax = _flap_down_clmax_values(wing_context, config)
    lift_remaining = max(
        effective_weight_n - q_low * s_unblown * flap_down_clmax["design_unblown_clmax"],
        0.0,
    )
    q_blown_required = lift_remaining / max(s_blown * flap_down_clmax["design_blown_clmax"], 1e-9)
    required_veff = max(
        slow_flight_speed,
        math.sqrt(max(2.0 * q_blown_required / mission.air_density_kgpm3, 0.0)),
    )
    thrust_req_veff = thrust_required_for_veff(
        mission,
        candidate,
        required_veff,
        slow_flight_speed,
    )

    slow_freestream_cl_required = effective_weight_n / max(q_low * s_wing, 1e-9)
    baseline_low_drag = _safe_float(stage1_row.get("low_speed_drag_n"), 8.0)
    low_fuselage_drag = q_low * config.fuselage_cd_area_m2
    low_htail_drag = q_low * geometry["htail_area_m2"] * 0.035
    low_vtail_drag = q_low * geometry["vtail_area_m2"] * 0.030

    def low_drag_model_for_thrust(thrust_n: float) -> dict[str, float]:
        actual_veff_local = slipstream_velocity_after_prop(
            mission,
            candidate,
            thrust_n,
            slow_flight_speed,
        )
        q_blown_local = 0.5 * mission.air_density_kgpm3 * actual_veff_local**2
        unblown_local_cl = min(
            flap_down_clmax["design_unblown_clmax"],
            effective_weight_n / max(q_low * s_unblown, 1e-9),
        )
        lift_from_unblown = q_low * s_unblown * unblown_local_cl
        lift_remaining_local = max(effective_weight_n - lift_from_unblown, 0.0)
        blown_local_cl = lift_remaining_local / max(q_blown_local * s_blown, 1e-9)
        unblown_profile_drag = q_low * s_unblown * mission.cd0_flapped
        blown_profile_drag = (
            q_blown_local
            * s_blown
            * mission.cd0_flapped
            * mission.blown_profile_drag_factor
        )
        unblown_induced_drag = q_low * s_unblown * (
            unblown_local_cl**2 / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
        )
        blown_induced_drag = q_blown_local * s_blown * (
            blown_local_cl**2 / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
        )
        local_induced_drag = unblown_induced_drag + blown_induced_drag
        freestream_equiv_induced_drag = q_low * s_wing * (
            slow_freestream_cl_required**2
            / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
        )
        wing_profile_drag = unblown_profile_drag + blown_profile_drag
        wing_induced_drag = local_induced_drag
        natural_drag = (
            wing_profile_drag
            + wing_induced_drag
            + low_htail_drag
            + low_vtail_drag
            + low_fuselage_drag
        )
        return {
            "actual_veff": actual_veff_local,
            "q_actual_blown": q_blown_local,
            "slow_unblown_local_cl": unblown_local_cl,
            "slow_blown_local_cl": blown_local_cl,
            "low_wing_profile_drag": wing_profile_drag,
            "low_wing_induced_drag": wing_induced_drag,
            "low_wing_induced_local_drag": local_induced_drag,
            "low_wing_induced_freestream_equiv_drag": freestream_equiv_induced_drag,
            "low_unblown_induced_drag": unblown_induced_drag,
            "low_blown_induced_drag": blown_induced_drag,
            "low_natural_drag": natural_drag,
        }

    initial_low_natural_drag = baseline_low_drag + low_htail_drag + low_vtail_drag + low_fuselage_drag
    low_required_thrust = max(mission.thrust_margin_low_speed * initial_low_natural_drag, thrust_req_veff)
    low_op = _solve_prop_power_for_thrust(
        mission,
        candidate,
        slow_flight_speed,
        low_required_thrust,
        mission.low_speed_rpm_bounds,
        config,
    )
    low_drag_model = low_drag_model_for_thrust(low_op["thrust_total_n"])
    explicit_drag_thrust_required = max(
        mission.thrust_margin_low_speed * low_drag_model["low_natural_drag"],
        thrust_req_veff,
    )
    if explicit_drag_thrust_required > low_required_thrust + 1e-6:
        low_required_thrust = explicit_drag_thrust_required
        low_op = _solve_prop_power_for_thrust(
            mission,
            candidate,
            slow_flight_speed,
            low_required_thrust,
            mission.low_speed_rpm_bounds,
            config,
        )
        low_drag_model = low_drag_model_for_thrust(low_op["thrust_total_n"])
    low_max_op = _stage3_prop_operating_point(
        mission,
        candidate,
        _effective_rpm_bounds_for_prop_model(mission, candidate, mission.low_speed_rpm_bounds, config)[1],
        slow_flight_speed,
        config,
    )

    actual_veff = low_drag_model["actual_veff"]
    q_actual_blown = low_drag_model["q_actual_blown"]
    q_low_effective = (q_low * s_unblown + q_actual_blown * s_blown) / max(s_wing, 1e-9)
    slow_flight_uniform_local_cl_required = effective_weight_n / max(
        q_low * s_unblown + q_actual_blown * s_blown,
        1e-9,
    )
    slow_unblown_local_cl = low_drag_model["slow_unblown_local_cl"]
    slow_blown_local_cl = low_drag_model["slow_blown_local_cl"]
    slow_local_cl_margin = flap_down_clmax["design_blown_clmax"] - slow_blown_local_cl
    low_wing_profile_drag = low_drag_model["low_wing_profile_drag"]
    low_wing_induced_drag = low_drag_model["low_wing_induced_drag"]
    low_wing_induced_local_drag = low_drag_model["low_wing_induced_local_drag"]
    low_wing_induced_freestream_equiv_drag = low_drag_model["low_wing_induced_freestream_equiv_drag"]
    low_unblown_induced_drag = low_drag_model["low_unblown_induced_drag"]
    low_blown_induced_drag = low_drag_model["low_blown_induced_drag"]
    low_natural_drag = low_drag_model["low_natural_drag"]
    low_steady_drag = max(low_natural_drag, low_op["thrust_total_n"])
    low_added_drag_required = max(0.0, low_steady_drag - low_natural_drag)
    low_drag_delta_vs_cruise = low_steady_drag - cruise_drag
    slow_lift_available = (
        q_low * s_unblown * flap_down_clmax["unblown_clmax"]
        + q_actual_blown * s_blown * flap_down_clmax["blown_clmax"]
    )
    slow_flight_unblown_equivalent_clmax = flap_down_clmax["unblown_clmax"]
    slow_flight_blown_equivalent_clmax = slow_lift_available / max(q_low * s_wing, 1e-9)
    slow_flight_design_blown_equivalent_clmax = slow_flight_blown_equivalent_clmax / max(
        config.slow_flight_stall_margin_factor,
        1e-9,
    )
    slow_lift_target = config.slow_flight_stall_margin_factor * effective_weight_n
    slow_lift_margin_n = slow_lift_available - slow_lift_target
    slow_lift_margin_fraction = slow_lift_margin_n / max(slow_lift_target, 1e-9)
    equivalent_flap_down_stall_speed = math.sqrt(
        2.0
        * effective_weight_n
        * config.slow_flight_stall_margin_factor
        / max(mission.air_density_kgpm3 * s_wing * flap_down_clmax["unblown_clmax"], 1e-9)
    )
    slow_flight_feasible = (
        required_veff <= actual_veff + 1e-9
        and slow_blown_local_cl <= flap_down_clmax["design_blown_clmax"] + 1e-9
        and low_op["rpm_feasible"] >= 1.0
    )

    mass_budget_margin = mission.max_mass_kg - stage3_total_mass
    penalties = 0.0

    def add_lower_penalty(value: float, lower: float, scale: float, weight: float) -> None:
        nonlocal penalties
        penalties += weight * max(0.0, (lower - value) / max(scale, 1e-9)) ** 2

    def add_upper_penalty(value: float, upper: float, scale: float, weight: float) -> None:
        nonlocal penalties
        penalties += weight * max(0.0, (value - upper) / max(scale, 1e-9)) ** 2

    h_volume_min, h_volume_max = _horizontal_tail_volume_range(config)
    v_volume_min, v_volume_max = _vertical_tail_volume_range(config)
    add_lower_penalty(h_volume, h_volume_min, 0.05, 18.0)
    add_upper_penalty(h_volume, h_volume_max, 0.08, 8.0)
    add_lower_penalty(v_volume, v_volume_min, 0.008, 16.0)
    add_upper_penalty(v_volume, v_volume_max, 0.010, 8.0)
    add_lower_penalty(static_margin, config.min_static_margin_mac, 0.02, 24.0)
    add_upper_penalty(static_margin, config.max_static_margin_mac, 0.04, 8.0)
    add_lower_penalty(mass_budget_margin, 0.0, 0.10, 60.0)
    if config.enforce_target_cg:
        add_upper_penalty(abs(cg_error_m), 1e-4, 0.002, 80.0)
        add_lower_penalty(required_base_cg_fraction, config.baseline_cg_fraction_bounds_mac[0], 0.02, 18.0)
        add_upper_penalty(required_base_cg_fraction, config.baseline_cg_fraction_bounds_mac[1], 0.02, 18.0)
    add_lower_penalty(slow_lift_margin_fraction, config.min_slow_flight_lift_margin, 0.05, 18.0)
    add_lower_penalty(actual_veff - required_veff, 0.0, 0.50, 80.0)
    add_lower_penalty(slow_local_cl_margin, 0.0, 0.10, 80.0)
    add_upper_penalty(abs(tail_cl_cruise), 0.65, 0.10, 10.0)
    add_upper_penalty(abs(htail_incidence_deg), config.h_tail_incidence_bounds_deg[1], 1.0, 12.0)
    add_upper_penalty(
        abs(cruise_body_alpha_proxy_deg - config.target_cruise_body_alpha_deg),
        0.50,
        1.0,
        2.0,
    )
    add_lower_penalty(low_op["rpm_feasible"], 1.0, 1.0, 12.0)
    add_lower_penalty(cruise_op["rpm_feasible"], 1.0, 1.0, 12.0)

    add_lower_penalty(
        geometry["htail_root_chord_m"],
        config.h_tail_root_chord_bounds_m[0],
        0.02,
        8.0,
    )
    add_upper_penalty(
        geometry["htail_root_chord_m"],
        config.h_tail_root_chord_bounds_m[1],
        0.03,
        8.0,
    )
    add_lower_penalty(
        geometry["vtail_root_chord_m"],
        config.v_tail_root_chord_bounds_m[0],
        0.02,
        8.0,
    )
    add_upper_penalty(
        geometry["vtail_root_chord_m"],
        config.v_tail_root_chord_bounds_m[1],
        0.03,
        8.0,
    )

    normalized_static_error = ((static_margin - config.target_static_margin_mac) / 0.08) ** 2
    score = (
        config.cruise_weight * cruise_op["power_electric_total_w"] / config.reference_cruise_power_w
        + config.slow_flight_weight * low_op["power_electric_total_w"] / config.reference_slow_flight_power_w
        + config.material_mass_weight * tail_foam_mass / max(0.25, mission.max_mass_kg)
        + config.stability_weight * normalized_static_error
        + penalties
    )

    return {
        **geometry,
        "geometry_score": score,
        "penalty_score": penalties,
        "horizontal_tail_volume": h_volume,
        "vertical_tail_volume": v_volume,
        "htail_incidence_proxy_deg": htail_incidence_deg,
        "tail_cl_cruise_proxy": tail_cl_cruise,
        "cruise_alpha_proxy_deg": cruise_body_alpha_proxy_deg,
        "cruise_section_alpha_proxy_deg": alpha_wing_section,
        "cruise_drag_n": cruise_drag,
        "cruise_wing_profile_drag_n": cruise_wing_profile_drag,
        "cruise_wing_induced_drag_n": cruise_wing_induced_drag,
        "cruise_htail_drag_n": cruise_htail_drag,
        "cruise_vtail_drag_n": cruise_vtail_drag,
        "cruise_fuselage_drag_n": cruise_fuselage_drag,
        "cruise_power_w": cruise_op["power_electric_total_w"],
        "cruise_propulsion_electric_power_without_aircraft_losses_w": cruise_op[
            "propulsion_electric_power_without_aircraft_losses_w"
        ],
        "cruise_propulsion_wiring_loss_w": cruise_op["propulsion_wiring_loss_w"],
        "cruise_avionics_power_loss_w": cruise_op["avionics_power_loss_w"],
        "cruise_aircraft_power_losses_w": cruise_op["aircraft_power_losses_w"],
        "cruise_rpm": cruise_op["rpm"],
        "cruise_thrust_required_n": cruise_op["thrust_required_n"],
        "cruise_throttle_percent": _clamp(
            100.0 * cruise_op["thrust_required_n"] / max(cruise_max_op["thrust_total_n"], 1e-9),
            0.0,
            100.0,
        ),
        "cruise_ct": cruise_op["ct"],
        "cruise_cp": cruise_op["cp"],
        "cruise_blown_effective_velocity_mps": cruise_actual_veff,
        "cruise_unblown_dynamic_pressure_pa": q_cruise,
        "cruise_blown_dynamic_pressure_pa": q_cruise_blown,
        "cruise_effective_dynamic_pressure_pa": q_cruise_effective,
        "cruise_blown_to_unblown_q_ratio": q_cruise_blown / max(q_cruise, 1e-9),
        "cruise_uniform_local_cl_required": cruise_uniform_local_cl_required,
        "low_speed_drag_n": low_steady_drag,
        "low_speed_natural_drag_n": low_natural_drag,
        "low_speed_added_drag_required_n": low_added_drag_required,
        "low_speed_drag_delta_vs_cruise_n": low_drag_delta_vs_cruise,
        "low_speed_blown_lift_thrust_n": low_op["thrust_total_n"],
        "low_speed_stage1_baseline_drag_n": baseline_low_drag,
        "low_speed_wing_profile_drag_n": low_wing_profile_drag,
        "low_speed_wing_induced_drag_n": low_wing_induced_drag,
        "low_speed_wing_induced_local_drag_n": low_wing_induced_local_drag,
        "low_speed_wing_induced_freestream_equiv_drag_n": low_wing_induced_freestream_equiv_drag,
        "low_speed_unblown_induced_drag_n": low_unblown_induced_drag,
        "low_speed_blown_induced_drag_n": low_blown_induced_drag,
        "low_speed_htail_drag_n": low_htail_drag,
        "low_speed_vtail_drag_n": low_vtail_drag,
        "low_speed_fuselage_drag_n": low_fuselage_drag,
        "slow_flight_freestream_cl_required": slow_freestream_cl_required,
        "slow_flight_uniform_local_cl_required": slow_flight_uniform_local_cl_required,
        "slow_flight_unblown_dynamic_pressure_pa": q_low,
        "slow_flight_blown_dynamic_pressure_pa": q_actual_blown,
        "slow_flight_effective_dynamic_pressure_pa": q_low_effective,
        "slow_flight_blown_to_unblown_q_ratio": q_actual_blown / max(q_low, 1e-9),
        "slow_flight_blown_area_m2": s_blown,
        "slow_flight_unblown_area_m2": s_unblown,
        "slow_flight_unblown_local_cl": slow_unblown_local_cl,
        "slow_flight_blown_local_cl": slow_blown_local_cl,
        "slow_flight_local_cl_margin": slow_local_cl_margin,
        "slow_flight_feasible": 1.0 if slow_flight_feasible else 0.0,
        "slow_flight_required_veff_margin_mps": actual_veff - required_veff,
        "low_speed_required_veff_mps": required_veff,
        "low_speed_actual_veff_mps": actual_veff,
        "slow_flight_unblown_clmax": flap_down_clmax["unblown_clmax"],
        "slow_flight_blown_clmax": slow_flight_blown_equivalent_clmax,
        "slow_flight_blown_local_clmax": flap_down_clmax["blown_clmax"],
        "slow_flight_unblown_equivalent_clmax": slow_flight_unblown_equivalent_clmax,
        "slow_flight_blown_equivalent_clmax": slow_flight_blown_equivalent_clmax,
        "slow_flight_design_unblown_clmax": flap_down_clmax["design_unblown_clmax"],
        "slow_flight_design_blown_clmax": slow_flight_design_blown_equivalent_clmax,
        "slow_flight_design_blown_local_clmax": flap_down_clmax["design_blown_clmax"],
        "slow_flight_lift_available_n": slow_lift_available,
        "slow_flight_lift_target_n": slow_lift_target,
        "slow_flight_lift_margin_n": slow_lift_margin_n,
        "slow_flight_lift_margin_percent": 100.0 * slow_lift_margin_fraction,
        "slow_flight_equiv_stall_speed_mps": equivalent_flap_down_stall_speed,
        "low_speed_power_w": low_op["power_electric_total_w"],
        "low_speed_propulsion_electric_power_without_aircraft_losses_w": low_op[
            "propulsion_electric_power_without_aircraft_losses_w"
        ],
        "low_speed_propulsion_wiring_loss_w": low_op["propulsion_wiring_loss_w"],
        "low_speed_avionics_power_loss_w": low_op["avionics_power_loss_w"],
        "low_speed_aircraft_power_losses_w": low_op["aircraft_power_losses_w"],
        "low_speed_rpm": low_op["rpm"],
        "low_speed_throttle_percent": _clamp(
            100.0 * low_op["thrust_required_n"] / max(low_max_op["thrust_total_n"], 1e-9),
            0.0,
            100.0,
        ),
        "low_speed_ct": low_op["ct"],
        "low_speed_cp": low_op["cp"],
        "low_speed_thrust_required_n": low_required_thrust,
        "propulsion_model_source": low_op.get(
            "propulsion_model_source",
            cruise_op.get("propulsion_model_source", "Stage 1 generic prop surrogate"),
        ),
        "cg_x_m": cg_x,
        "cg_percent_mac": 100.0 * cg_fraction,
        "cg_target_x_m": target_cg_x,
        "cg_target_percent_mac": 100.0 * config.target_cg_fraction_mac,
        "cg_error_m": cg_error_m,
        "cg_error_percent_mac": 100.0 * (cg_fraction - config.target_cg_fraction_mac),
        "stage1_baseline_cg_used_percent_mac": 100.0 * base_cg_fraction,
        "stage1_baseline_cg_required_percent_mac": 100.0 * required_base_cg_fraction,
        "neutral_point_x_m": x_np,
        "static_margin_mac": static_margin,
        "stage1_total_built_mass_kg": stage1_total_mass,
        "stage3_total_built_mass_kg": stage3_total_mass,
        "gross_flight_mass_kg": mission.gross_mass_kg,
        "remaining_mass_to_gross_kg": mission.gross_mass_kg - stage3_total_mass,
        "mass_budget_margin_kg": mass_budget_margin,
        "htail_foam_mass_kg": h_volume_mass,
        "vtail_foam_mass_kg": v_volume_mass,
        "total_tail_foam_mass_kg": tail_foam_mass,
        "blown_span_fraction": eta_blown,
        "clmax_source": flap_down_clmax["source"],
        "clmax_curve_source": flap_down_clmax["curve_source"],
        "physical_clmax_source": flap_down_clmax["physical_clmax_source"],
        "stage12_clean_section_clmax_unblown": flap_down_clmax["stage12_clean_section_clmax_unblown"],
        "stage12_clean_section_clmax_blown": flap_down_clmax["stage12_clean_section_clmax_blown"],
        "stage12_flapped_section_clmax_unblown": flap_down_clmax["stage12_flapped_section_clmax_unblown"],
        "stage12_flapped_section_clmax_blown": flap_down_clmax["stage12_flapped_section_clmax_blown"],
        "stage12_weighted_blown_physical_clmax": flap_down_clmax["stage12_weighted_blown_physical_clmax"],
        "no_flap_clmax": flap_down_clmax["no_flap_clmax"],
        "flap_only_clmax": flap_down_clmax["flap_only_clmax"],
        "clean_blowing_clmax": flap_down_clmax["clean_blowing_clmax"],
        "flap_down_blown_clmax": flap_down_clmax["flap_down_blown_clmax"],
        "raw_no_flap_clmax": flap_down_clmax["raw_no_flap_clmax"],
        "raw_flap_only_clmax": flap_down_clmax["raw_flap_only_clmax"],
        "raw_clean_blowing_clmax": flap_down_clmax["raw_clean_blowing_clmax"],
        "raw_flap_down_blown_clmax": flap_down_clmax["raw_flap_down_blown_clmax"],
        "clmax_was_capped": flap_down_clmax["clmax_was_capped"],
        "no_flap_clmax_alpha_deg": flap_down_clmax["no_flap_clmax_alpha_deg"],
        "flap_only_clmax_alpha_deg": flap_down_clmax["flap_only_clmax_alpha_deg"],
        "clean_blowing_clmax_alpha_deg": flap_down_clmax["clean_blowing_clmax_alpha_deg"],
        "flap_down_blown_clmax_alpha_deg": flap_down_clmax["flap_down_blown_clmax_alpha_deg"],
        "cruise_rpm_feasible": cruise_op["rpm_feasible"],
        "low_speed_rpm_feasible": low_op["rpm_feasible"],
    }


def optimize_stage3_tail(
    mission: Stage1MissionConfig,
    stage2_row: dict[str, str],
    stage1_row: dict[str, str],
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
) -> dict[str, float]:
    from scipy.optimize import minimize

    candidate = _stage1_candidate_from_row(stage2_row)
    reynolds_cruise = (
        mission.air_density_kgpm3
        * mission.cruise_speed_mps
        * mission.chord_m
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    main_polar = _main_airfoil_polar(wing_context["selected_airfoil"], reynolds_cruise)
    tail_area_coeff = _airfoil_area_coefficient(config.tail_airfoil)
    bounds = _optimization_bounds(config)

    def objective(values: Any) -> float:
        info = _evaluate_tail_proxy(
            list(values),
            mission,
            config,
            candidate,
            stage1_row,
            wing_context,
            main_polar,
            tail_area_coeff,
        )
        return float(info["geometry_score"])

    best_values: list[float] | None = None
    best_score = float("inf")
    best_success = False
    for seed in _seed_vectors(mission, config):
        result = minimize(
            objective,
            seed,
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": 220, "ftol": 1e-8, "disp": False},
        )
        values = list(result.x if result.success else seed)
        score = objective(values)
        if score < best_score:
            best_score = score
            best_values = values
            best_success = bool(result.success)

    if best_values is None:
        best_values = _seed_vectors(mission, config)[0]

    best = _evaluate_tail_proxy(
        best_values,
        mission,
        config,
        candidate,
        stage1_row,
        wing_context,
        main_polar,
        tail_area_coeff,
    )
    best["optimizer_success"] = 1.0 if best_success else 0.0
    return best


def _control_surface_tau(
    chord_fraction: float,
    reference_fraction: float,
    exponent: float,
) -> float:
    if reference_fraction <= 0.0:
        return 1.0
    return _clamp((chord_fraction / reference_fraction) ** exponent, 0.35, 1.80)


def size_slow_flight_control_surfaces(
    geometry: dict[str, Any],
    config: Stage3SizingConfig,
) -> dict[str, float]:
    """Size elevator and rudder chord fractions for slow-flight control authority."""

    h_volume = float(geometry["horizontal_tail_volume"])
    v_volume = float(geometry["vertical_tail_volume"])
    h_lift_slope = _finite_wing_lift_slope_per_rad(float(geometry["htail_aspect_ratio"]))
    v_lift_slope = _finite_wing_lift_slope_per_rad(float(geometry["vtail_aspect_ratio"]))
    elevator_deflection_rad = math.radians(max(config.elevator_max_deflection_deg, 1e-6))
    rudder_deflection_rad = math.radians(max(config.rudder_max_deflection_deg, 1e-6))
    exponent = max(config.control_surface_effectiveness_exponent, 1e-6)
    eta = max(config.slow_tail_dynamic_pressure_ratio, 1e-6)

    required_elevator_tau = config.target_slow_pitch_control_cm / max(
        eta * h_lift_slope * h_volume * elevator_deflection_rad,
        1e-9,
    )
    elevator_fraction = config.elevator_chord_fraction * max(required_elevator_tau, 0.0) ** (1.0 / exponent)
    elevator_fraction = _clamp(
        elevator_fraction,
        config.elevator_chord_fraction_bounds[0],
        config.elevator_chord_fraction_bounds[1],
    )
    elevator_tau = _control_surface_tau(
        elevator_fraction,
        config.elevator_chord_fraction,
        exponent,
    )
    elevator_cm_authority = eta * h_lift_slope * h_volume * elevator_tau * elevator_deflection_rad

    required_rudder_tau = config.target_slow_yaw_control_cn / max(
        eta * v_lift_slope * v_volume * rudder_deflection_rad,
        1e-9,
    )
    rudder_fraction = config.rudder_chord_fraction * max(required_rudder_tau, 0.0) ** (1.0 / exponent)
    rudder_fraction = _clamp(
        rudder_fraction,
        config.rudder_chord_fraction_bounds[0],
        config.rudder_chord_fraction_bounds[1],
    )
    rudder_tau = _control_surface_tau(
        rudder_fraction,
        config.rudder_chord_fraction,
        exponent,
    )
    rudder_cn_authority = eta * v_lift_slope * v_volume * rudder_tau * rudder_deflection_rad

    return {
        "elevator_chord_fraction_sized": elevator_fraction,
        "rudder_chord_fraction_sized": rudder_fraction,
        "elevator_max_deflection_deg": config.elevator_max_deflection_deg,
        "rudder_max_deflection_deg": config.rudder_max_deflection_deg,
        "target_slow_pitch_control_cm": config.target_slow_pitch_control_cm,
        "target_slow_yaw_control_cn": config.target_slow_yaw_control_cn,
        "slow_pitch_control_cm_authority": elevator_cm_authority,
        "slow_yaw_control_cn_authority": rudder_cn_authority,
        "slow_pitch_control_margin_percent": 100.0
        * (elevator_cm_authority / max(config.target_slow_pitch_control_cm, 1e-9) - 1.0),
        "slow_yaw_control_margin_percent": 100.0
        * (rudder_cn_authority / max(config.target_slow_yaw_control_cn, 1e-9) - 1.0),
    }


def build_airplane_geometry(
    mission: Stage1MissionConfig,
    geometry: dict[str, float],
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
    *,
    cg_x_m: float,
    htail_incidence_deg: float,
    flap_deflection_deg: float = 0.0,
    aileron_deflection_deg: float = 0.0,
    elevator_deflection_deg: float = 0.0,
    rudder_deflection_deg: float = 0.0,
) -> tuple[Any, dict[str, float]]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb

    main_airfoil = asb.Airfoil(wing_context["selected_airfoil"])
    tail_airfoil = asb.Airfoil(config.tail_airfoil)
    main_wing_incidence_deg = _safe_float(
        geometry.get("main_wing_incidence_deg"),
        config.main_wing_incidence_deg,
    )
    semispan = mission.semispan_m
    chord = mission.chord_m
    flap_end_y = semispan * wing_context["flap_span_fraction"]
    aileron_start_y = semispan * (1.0 - wing_context["aileron_span_fraction"])

    wing = asb.Wing(
        name="Main Wing",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[0.0, 0.0, 0.0],
                chord=chord,
                twist=main_wing_incidence_deg,
                airfoil=main_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="flap",
                        symmetric=True,
                        hinge_point=1.0 - wing_context["flap_chord_fraction"],
                        deflection=flap_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[0.0, flap_end_y, 0.0],
                chord=chord,
                twist=main_wing_incidence_deg,
                airfoil=main_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="flap",
                        symmetric=True,
                        hinge_point=1.0 - wing_context["flap_chord_fraction"],
                        deflection=flap_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[0.0, aileron_start_y, 0.0],
                chord=chord,
                twist=main_wing_incidence_deg,
                airfoil=main_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="aileron",
                        symmetric=False,
                        hinge_point=1.0 - wing_context["aileron_chord_fraction"],
                        deflection=aileron_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[0.0, semispan, 0.0],
                chord=chord,
                twist=main_wing_incidence_deg + config.main_wing_washout_deg,
                airfoil=main_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="aileron",
                        symmetric=False,
                        hinge_point=1.0 - wing_context["aileron_chord_fraction"],
                        deflection=aileron_deflection_deg,
                    )
                ],
            ),
        ],
    )

    wing_ac_x = config.wing_aerodynamic_center_fraction_mac * chord
    htail_ac_x = wing_ac_x + geometry["tail_arm_m"]
    htail_root_x = htail_ac_x - 0.25 * geometry["htail_root_chord_m"]
    htail_tip_x = htail_ac_x - 0.25 * geometry["htail_tip_chord_m"] + 0.04 * geometry["htail_span_m"] / 2.0
    htail_z = 0.035
    elevator_chord_fraction = _safe_float(
        geometry.get("elevator_chord_fraction_sized"),
        config.elevator_chord_fraction,
    )
    rudder_chord_fraction = _safe_float(
        geometry.get("rudder_chord_fraction_sized"),
        config.rudder_chord_fraction,
    )

    horizontal_tail = asb.Wing(
        name="Horizontal Tail",
        symmetric=True,
        xsecs=[
            asb.WingXSec(
                xyz_le=[htail_root_x, 0.0, htail_z],
                chord=geometry["htail_root_chord_m"],
                twist=htail_incidence_deg,
                airfoil=tail_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="elevator",
                        symmetric=True,
                        hinge_point=1.0 - elevator_chord_fraction,
                        deflection=elevator_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[htail_tip_x, geometry["htail_span_m"] / 2.0, htail_z],
                chord=geometry["htail_tip_chord_m"],
                twist=htail_incidence_deg,
                airfoil=tail_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="elevator",
                        symmetric=True,
                        hinge_point=1.0 - elevator_chord_fraction,
                        deflection=elevator_deflection_deg,
                    )
                ],
            ),
        ],
    )

    vtail_root_x = htail_root_x - 0.015
    vtail_tip_x = htail_ac_x - 0.25 * geometry["vtail_tip_chord_m"] + 0.05 * geometry["vtail_span_m"]
    vertical_tail = asb.Wing(
        name="Vertical Tail",
        symmetric=False,
        xsecs=[
            asb.WingXSec(
                xyz_le=[vtail_root_x, 0.0, 0.0],
                chord=geometry["vtail_root_chord_m"],
                twist=config.vertical_tail_incidence_deg,
                airfoil=tail_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="rudder",
                        symmetric=False,
                        hinge_point=1.0 - rudder_chord_fraction,
                        deflection=rudder_deflection_deg,
                    )
                ],
            ),
            asb.WingXSec(
                xyz_le=[vtail_tip_x, 0.0, geometry["vtail_span_m"]],
                chord=geometry["vtail_tip_chord_m"],
                twist=config.vertical_tail_incidence_deg,
                airfoil=tail_airfoil,
                control_surfaces=[
                    asb.ControlSurface(
                        name="rudder",
                        symmetric=False,
                        hinge_point=1.0 - rudder_chord_fraction,
                        deflection=rudder_deflection_deg,
                    )
                ],
            ),
        ],
    )

    fuselage_end_x = max(htail_ac_x + 0.65 * geometry["htail_root_chord_m"], 1.25)
    fuselage = asb.Fuselage(
        name="Fuselage",
        xsecs=[
            asb.FuselageXSec(xyz_c=[config.fuselage_nose_x_m, 0.0, 0.0], radius=0.001),
            asb.FuselageXSec(xyz_c=[0.24, 0.0, 0.0], width=config.fuselage_width_m, height=config.fuselage_height_m),
            asb.FuselageXSec(xyz_c=[0.90, 0.0, 0.0], width=config.fuselage_tail_width_m, height=config.fuselage_tail_height_m),
            asb.FuselageXSec(xyz_c=[fuselage_end_x, 0.0, 0.01], radius=0.016),
        ],
    )

    airplane = asb.Airplane(
        name="AA146 Stage 3 Fixed Wing Tail Refinement",
        xyz_ref=[cg_x_m, 0.0, 0.0],
        wings=[wing, horizontal_tail, vertical_tail],
        fuselages=[fuselage],
        s_ref=mission.wing_area_m2,
        c_ref=mission.chord_m,
        b_ref=mission.span_m,
    )
    meta = {
        "flap_end_y_m": flap_end_y,
        "aileron_start_y_m": aileron_start_y,
        "wing_ac_x_m": wing_ac_x,
        "htail_ac_x_m": htail_ac_x,
        "fuselage_nose_x_m": config.fuselage_nose_x_m,
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
    results = runtime.asb.VortexLatticeMethod(
        airplane=airplane,
        op_point=runtime.asb.OperatingPoint(
            velocity=float(velocity_mps),
            alpha=float(alpha_deg),
        ),
        spanwise_resolution=10,
        chordwise_resolution=7,
        run_symmetric_if_possible=False,
        verbose=False,
    ).run()
    out: dict[str, float] = {}
    for key, value in results.items():
        try:
            out[key] = float(value)
        except (TypeError, ValueError):
            pass
    return out


def solve_vlm_cruise_trim(
    mission: Stage1MissionConfig,
    geometry: dict[str, float],
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
) -> tuple[float, float, dict[str, float]]:
    from scipy.optimize import least_squares

    cg_x = geometry["cg_x_m"]

    def residual(values: Any) -> list[float]:
        alpha_deg = float(values[0])
        htail_incidence_deg = float(values[1])
        airplane, _ = build_airplane_geometry(
            mission,
            geometry,
            wing_context,
            config,
            cg_x_m=cg_x,
            htail_incidence_deg=htail_incidence_deg,
        )
        result = _run_vlm(airplane, velocity_mps=mission.cruise_speed_mps, alpha_deg=alpha_deg)
        return [
            (result.get("L", 0.0) - mission.gross_weight_n) / max(mission.gross_weight_n, 1e-9),
            result.get("Cm", 0.0) / 0.08,
        ]

    guess = [
        _clamp(geometry["cruise_alpha_proxy_deg"], -4.0, 14.0),
        _clamp(
            geometry["htail_incidence_proxy_deg"],
            config.h_tail_incidence_bounds_deg[0],
            config.h_tail_incidence_bounds_deg[1],
        ),
    ]
    fit = least_squares(
        residual,
        guess,
        bounds=([-5.0, config.h_tail_incidence_bounds_deg[0]], [16.0, config.h_tail_incidence_bounds_deg[1]]),
        max_nfev=72,
        xtol=1e-5,
        ftol=1e-5,
        gtol=1e-5,
    )
    alpha = float(fit.x[0])
    incidence = float(fit.x[1])
    airplane, _ = build_airplane_geometry(
        mission,
        geometry,
        wing_context,
        config,
        cg_x_m=cg_x,
        htail_incidence_deg=incidence,
    )
    result = _run_vlm(airplane, velocity_mps=mission.cruise_speed_mps, alpha_deg=alpha)
    result["trim_residual_lift_n"] = result.get("L", 0.0) - mission.gross_weight_n
    result["trim_residual_cm"] = result.get("Cm", 0.0)
    result["trim_solver_success"] = 1.0 if fit.success else 0.0
    return alpha, incidence, result


def evaluate_section_polars(
    mission: Stage1MissionConfig,
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
) -> dict[str, Any]:
    runtime = ensure_stage3_runtime()
    asb = runtime.asb
    onp = runtime.onp
    airfoil = asb.Airfoil(wing_context["selected_airfoil"])
    alphas = onp.linspace(-8.0, 22.0, 95)
    reynolds = (
        mission.air_density_kgpm3
        * mission.cruise_speed_mps
        * mission.chord_m
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    low_reynolds = (
        mission.air_density_kgpm3
        * _stage3_slow_flight_speed_mps(config)
        * mission.chord_m
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    clean = airfoil.get_aero_from_neuralfoil(alpha=alphas, Re=reynolds)
    flapped = airfoil.get_aero_from_neuralfoil(
        alpha=alphas,
        Re=low_reynolds,
        control_surfaces=[
            asb.ControlSurface(
                name="flap",
                symmetric=True,
                deflection=_slow_flap_deflection_deg(wing_context, config),
                hinge_point=1.0 - wing_context["flap_chord_fraction"],
            )
        ],
    )
    stage12_clean = _workflow_curve_polar(wing_context, "clean")
    stage12_flaps_down = _workflow_curve_polar(wing_context, "flaps_down")
    return {
        "alphas_deg": alphas,
        "clean": clean,
        "flapped": flapped,
        "stage12_clean": stage12_clean,
        "stage12_flaps_down": stage12_flaps_down,
        "stage12_high_lift_polar_source": wing_context.get("clmax_curve_source", ""),
        "reynolds_cruise": float(reynolds),
        "reynolds_low": float(low_reynolds),
        "clean_section_clmax": float(onp.asarray(clean["CL"], dtype=float).max()),
        "flapped_section_clmax": float(onp.asarray(flapped["CL"], dtype=float).max()),
    }


def render_top_view(
    mission: Stage1MissionConfig,
    geometry: dict[str, float],
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
    stage2_row: dict[str, str],
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    patches = runtime.patches
    semispan = mission.semispan_m
    chord = mission.chord_m
    flap_end = semispan * wing_context["flap_span_fraction"]
    aileron_start = semispan * (1.0 - wing_context["aileron_span_fraction"])

    fig, ax = plt.subplots(figsize=(10, 5.6))

    wing_right = [(0.0, 0.0), (0.0, semispan), (chord, semispan), (chord, 0.0)]
    wing_left = [(x, -y) for x, y in reversed(wing_right)]
    for poly in (wing_right, wing_left):
        ax.add_patch(patches.Polygon(poly, closed=True, facecolor="#dbe4ee", edgecolor="#111827", linewidth=1.1))

    flap_x0 = chord * (1.0 - wing_context["flap_chord_fraction"])
    flap_right = [(flap_x0, 0.0), (flap_x0, flap_end), (chord, flap_end), (chord, 0.0)]
    flap_left = [(x, -y) for x, y in reversed(flap_right)]
    for poly in (flap_right, flap_left):
        ax.add_patch(patches.Polygon(poly, closed=True, facecolor="#f4a261", edgecolor="#111827", linewidth=0.8, alpha=0.85))

    aileron_x0 = chord * (1.0 - wing_context["aileron_chord_fraction"])
    aileron_right = [(aileron_x0, aileron_start), (aileron_x0, semispan), (chord, semispan), (chord, aileron_start)]
    aileron_left = [(x, -y) for x, y in reversed(aileron_right)]
    for poly in (aileron_right, aileron_left):
        ax.add_patch(patches.Polygon(poly, closed=True, facecolor="#457b9d", edgecolor="#111827", linewidth=0.8, alpha=0.85))

    wing_ac_x = config.wing_aerodynamic_center_fraction_mac * chord
    htail_ac_x = wing_ac_x + geometry["tail_arm_m"]
    htail_root_x = htail_ac_x - 0.25 * geometry["htail_root_chord_m"]
    htail_tip_x = htail_ac_x - 0.25 * geometry["htail_tip_chord_m"] + 0.04 * geometry["htail_span_m"] / 2.0
    htail_right = [
        (htail_root_x, 0.0),
        (htail_tip_x, geometry["htail_span_m"] / 2.0),
        (htail_tip_x + geometry["htail_tip_chord_m"], geometry["htail_span_m"] / 2.0),
        (htail_root_x + geometry["htail_root_chord_m"], 0.0),
    ]
    htail_left = [(x, -y) for x, y in reversed(htail_right)]
    for poly in (htail_right, htail_left):
        ax.add_patch(patches.Polygon(poly, closed=True, facecolor="#b8e0d2", edgecolor="#111827", linewidth=1.0, alpha=0.85))

    fuselage_half_width = config.fuselage_width_m / 2.0
    fuselage_end_x = max(htail_ac_x + 0.65 * geometry["htail_root_chord_m"], 1.25)
    fuselage_nose_x = config.fuselage_nose_x_m
    ax.add_patch(
        patches.FancyBboxPatch(
            (fuselage_nose_x, -fuselage_half_width),
            fuselage_end_x - fuselage_nose_x,
            2.0 * fuselage_half_width,
            boxstyle="round,pad=0.02,rounding_size=0.025",
            linewidth=1.0,
            edgecolor="#374151",
            facecolor="#f8fafc",
            alpha=0.92,
        )
    )

    prop_radius = _safe_float(stage2_row["prop_diameter_in"]) * 0.0254 / 2.0
    prop_x = _prop_axial_x_m(mission, config)
    for y_center in parse_prop_centers(stage2_row["prop_centers_m"]):
        ax.add_patch(
            patches.Circle(
                (prop_x, y_center),
                prop_radius,
                facecolor="none",
                edgecolor="#7c3aed",
                linestyle="--",
                linewidth=1.0,
                alpha=0.80,
            )
        )

    ax.scatter([geometry["cg_x_m"]], [0.0], marker="x", color="#dc2626", s=60, label="CG")
    ax.text(
        0.40,
        0.98,
        (
            f"wing b={mission.span_m:.2f} m, c={mission.chord_m:.2f} m\n"
            f"nose x={fuselage_nose_x:.3f} m\n"
            f"props x={prop_x:.3f} m, N={int(float(stage2_row['n_props']))}\n"
            f"H-tail b={geometry['htail_span_m']:.3f} m, S={geometry['htail_area_m2']:.3f} m^2\n"
            f"V-tail h={geometry['vtail_span_m']:.3f} m, S={geometry['vtail_area_m2']:.3f} m^2\n"
            f"SM={geometry['static_margin_mac']:.3f} MAC"
        ),
        transform=ax.transAxes,
        ha="left",
        va="top",
        bbox=dict(facecolor="white", edgecolor="#cbd5e1", alpha=0.92),
        fontsize=9,
    )
    ax.set_aspect("equal")
    ax.set_xlim(min(fuselage_nose_x - 0.04, prop_x - prop_radius - 0.04), fuselage_end_x + 0.18)
    ax.set_ylim(-1.12, 1.12)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Stage 3 Fixed-Wing Tail Layout")
    ax.grid(True, alpha=0.18)
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


def render_3d_wireframe(
    airplane: Any,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    onp = runtime.onp
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    points, faces = airplane.mesh_body(method="quad", thin_wings=True, stack_meshes=True)
    points = onp.asarray(points, dtype=float)
    faces = onp.asarray(faces, dtype=int)
    polygons = [points[face] for face in faces if len(face) >= 3]

    fig = plt.figure(figsize=(8.5, 6.2))
    ax = fig.add_subplot(111, projection="3d")
    mesh = Poly3DCollection(
        polygons,
        facecolor="#c7d2fe",
        edgecolor="#111827",
        linewidths=0.22,
        alpha=0.74,
    )
    ax.add_collection3d(mesh)
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = 0.5 * (mins + maxs)
    radius = 0.55 * max(maxs - mins)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - 0.35 * radius, centers[2] + 0.70 * radius)
    ax.view_init(elev=24, azim=-135)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Stage 3 3D Rendering with Wireframe Overlay")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def render_polar_plot(
    polars: dict[str, Any],
    cruise_alpha_deg: float,
    output_path: Path,
) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt
    onp = runtime.onp

    fig, axs = plt.subplots(1, 2, figsize=(12, 4.4))
    axs[0].plot(polars["alphas_deg"], onp.asarray(polars["clean"]["CL"], dtype=float), label="Clean cruise Re")
    axs[0].plot(polars["alphas_deg"], onp.asarray(polars["flapped"]["CL"], dtype=float), label="Flapped low Re")
    if polars.get("stage12_flaps_down") is not None:
        stage12 = polars["stage12_flaps_down"]
        axs[0].plot(stage12["alpha_deg"], stage12["CL"], linestyle="--", linewidth=2.0, label="Stage 1/2 all high-lift")
    axs[0].axvline(cruise_alpha_deg, color="#6b7280", linestyle="--", linewidth=1.0)
    axs[0].set_xlabel("Alpha [deg]")
    axs[0].set_ylabel("Section CL")
    axs[0].set_title("Main Airfoil Lift")
    axs[0].grid(True, alpha=0.3)
    axs[0].legend()

    axs[1].plot(polars["alphas_deg"], onp.asarray(polars["clean"]["CD"], dtype=float), label="Clean cruise Re")
    axs[1].plot(polars["alphas_deg"], onp.asarray(polars["flapped"]["CD"], dtype=float), label="Flapped low Re")
    if polars.get("stage12_flaps_down") is not None:
        stage12 = polars["stage12_flaps_down"]
        axs[1].plot(stage12["alpha_deg"], stage12["CD"], linestyle="--", linewidth=2.0, label="Stage 1/2 all high-lift")
    axs[1].set_xlabel("Alpha [deg]")
    axs[1].set_ylabel("Section CD")
    axs[1].set_title("Main Airfoil Drag")
    axs[1].grid(True, alpha=0.3)
    axs[1].legend()

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def _section_polar_from_stage3_polars(polars: dict[str, Any], mode: str) -> dict[str, Any]:
    if mode == "flaps_down" and polars.get("stage12_flaps_down") is not None:
        return polars["stage12_flaps_down"]
    data = polars["flapped"] if mode == "flaps_down" else polars["clean"]
    return {
        "alpha_deg": polars["alphas_deg"],
        "CL": data["CL"],
        "CD": data["CD"],
        "CM": data["CM"],
    }


def _drag_breakdown_for_condition(
    mission: Stage1MissionConfig,
    geometry: dict[str, Any],
    config: Stage3SizingConfig,
    polar: dict[str, Any],
    *,
    velocity_mps: float,
    alpha_deg: float,
    mode: str,
) -> dict[str, float]:
    q = 0.5 * mission.air_density_kgpm3 * velocity_mps**2
    s_wing = mission.wing_area_m2
    cl = _interp_polar(polar, "CL", alpha_deg)
    cm = _interp_polar(polar, "CM", alpha_deg)
    cd_profile_floor = 0.012 if mode == "flaps_down" else 0.006
    cd_profile = max(_interp_polar(polar, "CD", alpha_deg), cd_profile_floor)
    wing_induced_cd = cl**2 / max(math.pi * mission.oswald_e * mission.aspect_ratio, 1e-9)

    h_volume = geometry["horizontal_tail_volume"]
    cg_fraction = geometry["cg_x_m"] / max(mission.chord_m, 1e-9)
    tail_eta = config.slow_tail_dynamic_pressure_ratio if mode == "flaps_down" else config.tail_dynamic_pressure_ratio
    tail_cl = (
        cm + cl * (cg_fraction - config.wing_aerodynamic_center_fraction_mac)
    ) / max(tail_eta * h_volume, 1e-9)
    tail_cl = _clamp(tail_cl, -0.85, 0.85)

    wing_profile_drag = q * s_wing * cd_profile
    wing_induced_drag = q * s_wing * wing_induced_cd
    htail_drag = q * geometry["htail_area_m2"] * (
        config.tail_zero_lift_cd
        + tail_cl**2 / max(math.pi * 0.82 * geometry["htail_aspect_ratio"], 1e-9)
    )
    vtail_drag = q * geometry["vtail_area_m2"] * config.tail_zero_lift_cd
    fuselage_drag = q * config.fuselage_cd_area_m2
    total_drag = wing_profile_drag + wing_induced_drag + htail_drag + vtail_drag + fuselage_drag

    return {
        "velocity_mps": velocity_mps,
        "alpha_deg": alpha_deg,
        "cl": cl,
        "cm": cm,
        "tail_cl": tail_cl,
        "wing_profile_drag_n": wing_profile_drag,
        "wing_induced_drag_n": wing_induced_drag,
        "htail_drag_n": htail_drag,
        "vtail_drag_n": vtail_drag,
        "fuselage_drag_n": fuselage_drag,
        "total_drag_n": total_drag,
    }


def _power_for_drag(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    velocity_mps: float,
    drag_n: float,
    mode: str,
    config: Stage3SizingConfig,
) -> dict[str, float]:
    rpm_bounds = mission.low_speed_rpm_bounds if mode == "flaps_down" else mission.cruise_rpm_bounds
    op = _solve_prop_power_for_thrust(
        mission,
        candidate,
        velocity_mps,
        max(drag_n, 0.0),
        rpm_bounds,
        config,
    )
    return {
        "power_w": op["power_electric_total_w"],
        "rpm": op["rpm"],
        "thrust_required_n": op["thrust_required_n"],
        "thrust_available_n": op["thrust_total_n"],
        "rpm_feasible": op["rpm_feasible"],
    }


def _propwash_dynamic_pressure_summary(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    geometry: dict[str, Any],
    thrust_total_n: float,
    freestream_mps: float,
) -> dict[str, float]:
    """Pressure-weight the propwash and freestream sections for a flight condition."""

    q_unblown = 0.5 * mission.air_density_kgpm3 * freestream_mps**2
    blown_velocity = slipstream_velocity_after_prop(
        mission,
        candidate,
        max(thrust_total_n, 0.0),
        freestream_mps,
    )
    q_blown = 0.5 * mission.air_density_kgpm3 * blown_velocity**2
    blown_fraction = _clamp(_safe_float(geometry.get("blown_span_fraction"), 0.0), 0.0, 1.0)
    q_effective = (1.0 - blown_fraction) * q_unblown + blown_fraction * q_blown
    return {
        "blown_effective_velocity_mps": blown_velocity,
        "unblown_dynamic_pressure_pa": q_unblown,
        "blown_dynamic_pressure_pa": q_blown,
        "effective_dynamic_pressure_pa": q_effective,
    }


def _append_power_to_breakdown(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    breakdown: dict[str, float],
    mode: str,
    config: Stage3SizingConfig,
) -> dict[str, float]:
    power = _power_for_drag(
        mission,
        candidate,
        breakdown["velocity_mps"],
        breakdown["total_drag_n"],
        mode,
        config,
    )
    return {**breakdown, **power}


def _power_plot_value(op: dict[str, float]) -> float:
    """Avoid plotting saturated max-RPM power as if an infeasible point were solved."""

    if _safe_float(op.get("rpm_feasible"), 1.0) < 0.5:
        return float("nan")
    return _safe_float(op.get("power_w"), float("nan"))


def _flaps_down_split_sweep_condition(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    geometry: dict[str, Any],
    config: Stage3SizingConfig,
    flapped_polar: dict[str, Any],
    *,
    velocity_mps: float,
    mass_kg: float,
    alpha_deg: float | None = None,
) -> dict[str, float]:
    """Stage 3 flaps-down sweep using split blown/unblown wing drag.

    Stage 1/2's all-high-lift CL/CD curve is an equivalent whole-wing curve
    referenced to freestream dynamic pressure. It is excellent traceability for
    lift capacity, but it is not a physical profile-drag polar. For the Stage 3
    drag/power plots, compute drag from the actual unblown panel, blown panel,
    tail, and fuselage components instead.
    """

    speed = max(float(velocity_mps), 1e-6)
    weight_n = mass_kg * mission.gravity_mps2
    q_unblown = 0.5 * mission.air_density_kgpm3 * speed**2
    s_wing = mission.wing_area_m2
    aspect_ratio = mission.aspect_ratio
    blown_fraction = _clamp(_safe_float(geometry.get("blown_span_fraction"), 0.35), 0.05, 0.95)
    s_blown = blown_fraction * s_wing
    s_unblown = (1.0 - blown_fraction) * s_wing
    design_unblown_clmax = _safe_float(geometry.get("slow_flight_design_unblown_clmax"), 1.0)
    design_blown_clmax = _safe_float(geometry.get("slow_flight_design_blown_local_clmax"), 1.0)

    if alpha_deg is None:
        unblown_local_cl = min(
            design_unblown_clmax,
            weight_n / max(q_unblown * s_unblown, 1e-9),
        )
        blown_local_cl_for_lift = design_blown_clmax
        alpha_for_report = _alpha_for_cl(flapped_polar, blown_local_cl_for_lift)
        cd_profile = mission.cd0_flapped
    else:
        alpha_for_report = float(alpha_deg)
        local_cl = _interp_polar(flapped_polar, "CL", alpha_for_report)
        if local_cl <= 1e-9:
            nan = float("nan")
            return {
                "velocity_mps": speed,
                "alpha_deg": alpha_for_report,
                "cl": local_cl,
                "cl_required": weight_n / max(q_unblown * s_wing, 1e-9),
                "cm": _interp_polar(flapped_polar, "CM", alpha_for_report),
                "tail_cl": nan,
                "wing_profile_drag_n": nan,
                "wing_induced_drag_n": nan,
                "htail_drag_n": nan,
                "vtail_drag_n": nan,
                "fuselage_drag_n": nan,
                "total_drag_n": nan,
                "power_w": nan,
                "rpm": nan,
                "thrust_required_n": nan,
                "thrust_available_n": nan,
                "rpm_feasible": 0.0,
                "trim_feasible": 0.0,
                "added_drag_required_n": nan,
                "blown_effective_velocity_mps": nan,
            }
        unblown_local_cl = min(local_cl, design_unblown_clmax)
        blown_local_cl_for_lift = min(local_cl, design_blown_clmax)
        cd_profile = mission.cd0_flapped

    lift_from_unblown = q_unblown * s_unblown * unblown_local_cl
    lift_remaining = max(weight_n - lift_from_unblown, 0.0)
    q_blown_required = lift_remaining / max(s_blown * blown_local_cl_for_lift, 1e-9)
    required_veff = max(speed, math.sqrt(max(2.0 * q_blown_required / mission.air_density_kgpm3, 0.0)))
    thrust_req_veff = thrust_required_for_veff(mission, candidate, required_veff, speed)

    def drag_for_thrust(thrust_n: float) -> dict[str, float]:
        blown_velocity = slipstream_velocity_after_prop(mission, candidate, max(thrust_n, 0.0), speed)
        q_blown = 0.5 * mission.air_density_kgpm3 * blown_velocity**2
        blown_local_cl = lift_remaining / max(q_blown * s_blown, 1e-9)
        wing_profile_drag = (
            q_unblown * s_unblown * cd_profile
            + q_blown * s_blown * cd_profile * mission.blown_profile_drag_factor
        )
        unblown_induced_drag = q_unblown * s_unblown * (
            unblown_local_cl**2 / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
        )
        blown_induced_drag = q_blown * s_blown * (
            blown_local_cl**2 / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
        )
        htail_drag = q_unblown * geometry["htail_area_m2"] * 0.035
        vtail_drag = q_unblown * geometry["vtail_area_m2"] * 0.030
        fuselage_drag = q_unblown * config.fuselage_cd_area_m2
        natural_drag = (
            wing_profile_drag
            + unblown_induced_drag
            + blown_induced_drag
            + htail_drag
            + vtail_drag
            + fuselage_drag
        )
        return {
            "blown_velocity": blown_velocity,
            "q_blown": q_blown,
            "blown_local_cl": blown_local_cl,
            "wing_profile_drag_n": wing_profile_drag,
            "wing_induced_drag_n": unblown_induced_drag + blown_induced_drag,
            "htail_drag_n": htail_drag,
            "vtail_drag_n": vtail_drag,
            "fuselage_drag_n": fuselage_drag,
            "natural_drag_n": natural_drag,
        }

    preliminary_drag = drag_for_thrust(thrust_req_veff)
    thrust_required = max(thrust_req_veff, mission.thrust_margin_low_speed * preliminary_drag["natural_drag_n"])
    if alpha_deg is None and abs(speed - _stage3_slow_flight_speed_mps(config)) <= 1e-9:
        thrust_required = max(
            thrust_required,
            _safe_float(geometry.get("low_speed_thrust_required_n"), thrust_required),
        )
    op = _solve_prop_power_for_thrust(mission, candidate, speed, thrust_required, mission.low_speed_rpm_bounds, config)
    drag = drag_for_thrust(op["thrust_total_n"])
    explicit_required = max(thrust_req_veff, mission.thrust_margin_low_speed * drag["natural_drag_n"])
    if explicit_required > thrust_required + 1e-6:
        thrust_required = explicit_required
        op = _solve_prop_power_for_thrust(mission, candidate, speed, thrust_required, mission.low_speed_rpm_bounds, config)
        drag = drag_for_thrust(op["thrust_total_n"])

    q_effective = (q_unblown * s_unblown + drag["q_blown"] * s_blown) / max(s_wing, 1e-9)
    uniform_local_cl = weight_n / max(q_effective * s_wing, 1e-9)
    cm = _interp_polar(flapped_polar, "CM", alpha_for_report)
    power = {
        "power_w": op["power_electric_total_w"],
        "rpm": op["rpm"],
        "thrust_required_n": op["thrust_required_n"],
        "thrust_available_n": op["thrust_total_n"],
        "rpm_feasible": op["rpm_feasible"],
    }
    return {
        "velocity_mps": speed,
        "alpha_deg": alpha_for_report,
        "cl": uniform_local_cl,
        "cl_required": weight_n / max(q_unblown * s_wing, 1e-9),
        "cm": cm,
        "tail_cl": float("nan"),
        "wing_profile_drag_n": drag["wing_profile_drag_n"],
        "wing_induced_drag_n": drag["wing_induced_drag_n"],
        "htail_drag_n": drag["htail_drag_n"],
        "vtail_drag_n": drag["vtail_drag_n"],
        "fuselage_drag_n": drag["fuselage_drag_n"],
        "total_drag_n": drag["natural_drag_n"],
        **power,
        "power_w": _power_plot_value(power),
        "trim_feasible": 1.0 if op["rpm_feasible"] >= 0.5 else 0.0,
        "added_drag_required_n": max(0.0, op["thrust_total_n"] - drag["natural_drag_n"]),
        "blown_effective_velocity_mps": drag["blown_velocity"],
    }


def calculate_approach_trim(
    mission: Stage1MissionConfig,
    geometry: dict[str, Any],
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
    candidate: Stage1Candidate,
    flapped_polar: dict[str, Any],
) -> dict[str, float]:
    """Estimate steady flaps-down approach trim at the user-specified descent rate."""

    approach_speed = max(config.approach_speed_mps, 1e-6)
    max_descent_rate_mps = max(config.max_descent_rate_fps, 0.0) * 0.3048
    target_sink_rate_mps = max(config.approach_target_sink_rate_fps, 0.0) * 0.3048
    if target_sink_rate_mps <= 1e-9:
        requested_gamma_deg = max(config.approach_target_angle_deg, 0.0)
        requested_gamma_rad = math.radians(requested_gamma_deg)
        target_sink_rate_mps = approach_speed * math.sin(requested_gamma_rad)
    requested_sink_rate_mps = min(target_sink_rate_mps, max_descent_rate_mps)
    sink_rate_mps = min(requested_sink_rate_mps, max_descent_rate_mps, 0.999 * approach_speed)
    gamma_rad = math.asin(_clamp(sink_rate_mps / approach_speed, 0.0, 0.999))
    gamma_deg = math.degrees(gamma_rad)
    q = 0.5 * mission.air_density_kgpm3 * approach_speed**2
    lift_required = mission.gross_weight_n * math.cos(gamma_rad)
    cl_required = lift_required / max(q * mission.wing_area_m2, 1e-9)
    alpha_deg = _alpha_for_cl(flapped_polar, cl_required)
    breakdown = _drag_breakdown_for_condition(
        mission,
        geometry,
        config,
        flapped_polar,
        velocity_mps=approach_speed,
        alpha_deg=alpha_deg,
        mode="flaps_down",
    )
    drag_n = breakdown["total_drag_n"]
    thrust_required = max(drag_n - mission.gross_weight_n * math.sin(gamma_rad), 0.0)
    op = _solve_prop_power_for_thrust(
        mission,
        candidate,
        approach_speed,
        thrust_required,
        mission.low_speed_rpm_bounds,
        config,
    )
    max_op = _stage3_prop_operating_point(
        mission,
        candidate,
        _effective_rpm_bounds_for_prop_model(mission, candidate, mission.low_speed_rpm_bounds, config)[1],
        approach_speed,
        config,
    )
    throttle_percent = 100.0 * thrust_required / max(max_op["thrust_total_n"], 1e-9)
    propwash = _propwash_dynamic_pressure_summary(
        mission,
        candidate,
        geometry,
        op["thrust_total_n"],
        approach_speed,
    )
    local_cl_required = lift_required / max(propwash["effective_dynamic_pressure_pa"] * mission.wing_area_m2, 1e-9)

    h_lift_slope = _finite_wing_lift_slope_per_rad(geometry["htail_aspect_ratio"])
    elevator_tau = _control_surface_tau(
        _safe_float(geometry.get("elevator_chord_fraction_sized"), config.elevator_chord_fraction),
        config.elevator_chord_fraction,
        config.control_surface_effectiveness_exponent,
    )
    cm_to_trim = -(
        breakdown["cm"]
        + breakdown["cl"]
        * (
            geometry["cg_x_m"] / max(mission.chord_m, 1e-9)
            - config.wing_aerodynamic_center_fraction_mac
        )
    )
    cm_per_deg = (
        config.slow_tail_dynamic_pressure_ratio
        * h_lift_slope
        * geometry["horizontal_tail_volume"]
        * elevator_tau
        * max(config.approach_tail_trim_effectiveness, 1e-9)
        * math.pi
        / 180.0
    )
    elevator_trim_deg = _clamp(
        cm_to_trim / max(cm_per_deg, 1e-9),
        -config.approach_max_elevator_trim_deg,
        config.approach_max_elevator_trim_deg,
    )

    return {
        "approach_target_angle_deg": gamma_deg,
        "approach_target_sink_rate_fps": config.approach_target_sink_rate_fps,
        "approach_target_sink_rate_mps": target_sink_rate_mps,
        "approach_speed_mps": approach_speed,
        "approach_alpha_deg": alpha_deg,
        "approach_cl_required": cl_required,
        "approach_uniform_local_cl_required": local_cl_required,
        "approach_lift_required_n": lift_required,
        "approach_drag_n": drag_n,
        "approach_thrust_required_n": thrust_required,
        "approach_power_w": op["power_electric_total_w"],
        "approach_rpm": op["rpm"],
        "approach_throttle_percent": _clamp(throttle_percent, 0.0, 100.0),
        "approach_max_descent_rate_fps": config.max_descent_rate_fps,
        "approach_descent_rate_limited": 1.0 if sink_rate_mps < requested_sink_rate_mps - 1e-9 else 0.0,
        "approach_sink_rate_target_met": 1.0 if abs(sink_rate_mps - requested_sink_rate_mps) <= 1e-6 else 0.0,
        "approach_sink_rate_mps": sink_rate_mps,
        "approach_sink_rate_fps": sink_rate_mps / 0.3048,
        "approach_glide_ratio": 1.0 / max(math.tan(gamma_rad), 1e-9),
        "approach_elevator_trim_deg": elevator_trim_deg,
        "approach_rudder_trim_deg": 0.0,
        "approach_flap_deflection_deg": config.approach_flap_deflection_deg,
        "approach_wing_profile_drag_n": breakdown["wing_profile_drag_n"],
        "approach_wing_induced_drag_n": breakdown["wing_induced_drag_n"],
        "approach_htail_drag_n": breakdown["htail_drag_n"],
        "approach_vtail_drag_n": breakdown["vtail_drag_n"],
        "approach_fuselage_drag_n": breakdown["fuselage_drag_n"],
        "approach_rpm_feasible": op["rpm_feasible"],
        "approach_blown_effective_velocity_mps": propwash["blown_effective_velocity_mps"],
        "approach_unblown_dynamic_pressure_pa": propwash["unblown_dynamic_pressure_pa"],
        "approach_blown_dynamic_pressure_pa": propwash["blown_dynamic_pressure_pa"],
        "approach_effective_dynamic_pressure_pa": propwash["effective_dynamic_pressure_pa"],
    }


def calculate_climb_trim(
    mission: Stage1MissionConfig,
    geometry: dict[str, Any],
    config: Stage3SizingConfig,
    candidate: Stage1Candidate,
    clean_polar: dict[str, Any],
) -> dict[str, float]:
    """Estimate max-rate climb trim using the configured vertical-rate cap."""

    climb_speed = max(config.climb_trim_speed_mps, 1e-6)
    target_rate_mps = max(config.max_climb_rate_fps, 0.0) * 0.3048
    climb_rate_mps = min(target_rate_mps, 0.999 * climb_speed)
    gamma_rad = math.asin(_clamp(climb_rate_mps / climb_speed, 0.0, 0.999))
    q = 0.5 * mission.air_density_kgpm3 * climb_speed**2
    lift_required = mission.gross_weight_n * math.cos(gamma_rad)
    cl_required = lift_required / max(q * mission.wing_area_m2, 1e-9)
    alpha_deg = _alpha_for_cl(clean_polar, cl_required)
    breakdown = _drag_breakdown_for_condition(
        mission,
        geometry,
        config,
        clean_polar,
        velocity_mps=climb_speed,
        alpha_deg=alpha_deg,
        mode="clean",
    )
    thrust_required = breakdown["total_drag_n"] + mission.gross_weight_n * math.sin(gamma_rad)
    op = _solve_prop_power_for_thrust(
        mission,
        candidate,
        climb_speed,
        thrust_required,
        mission.low_speed_rpm_bounds,
        config,
    )
    max_op = _stage3_prop_operating_point(
        mission,
        candidate,
        _effective_rpm_bounds_for_prop_model(mission, candidate, mission.low_speed_rpm_bounds, config)[1],
        climb_speed,
        config,
    )
    throttle_percent = 100.0 * thrust_required / max(max_op["thrust_total_n"], 1e-9)
    propwash = _propwash_dynamic_pressure_summary(
        mission,
        candidate,
        geometry,
        op["thrust_total_n"],
        climb_speed,
    )
    local_cl_required = lift_required / max(propwash["effective_dynamic_pressure_pa"] * mission.wing_area_m2, 1e-9)

    h_lift_slope = _finite_wing_lift_slope_per_rad(geometry["htail_aspect_ratio"])
    elevator_tau = _control_surface_tau(
        _safe_float(geometry.get("elevator_chord_fraction_sized"), config.elevator_chord_fraction),
        config.elevator_chord_fraction,
        config.control_surface_effectiveness_exponent,
    )
    cm_to_trim = -(
        breakdown["cm"]
        + breakdown["cl"]
        * (
            geometry["cg_x_m"] / max(mission.chord_m, 1e-9)
            - config.wing_aerodynamic_center_fraction_mac
        )
    )
    cm_per_deg = (
        config.tail_dynamic_pressure_ratio
        * h_lift_slope
        * geometry["horizontal_tail_volume"]
        * elevator_tau
        * max(config.approach_tail_trim_effectiveness, 1e-9)
        * math.pi
        / 180.0
    )
    elevator_trim_deg = _clamp(
        cm_to_trim / max(cm_per_deg, 1e-9),
        -config.approach_max_elevator_trim_deg,
        config.approach_max_elevator_trim_deg,
    )

    return {
        "climb_target_rate_fps": config.max_climb_rate_fps,
        "climb_target_rate_mps": climb_rate_mps,
        "climb_angle_deg": math.degrees(gamma_rad),
        "climb_speed_mps": climb_speed,
        "climb_cl_required": cl_required,
        "climb_uniform_local_cl_required": local_cl_required,
        "climb_drag_n": breakdown["total_drag_n"],
        "climb_thrust_required_n": thrust_required,
        "climb_power_w": op["power_electric_total_w"],
        "climb_rpm": op["rpm"],
        "climb_throttle_percent": _clamp(throttle_percent, 0.0, 100.0),
        "climb_elevator_trim_deg": elevator_trim_deg,
        "climb_rpm_feasible": op["rpm_feasible"],
        "climb_blown_effective_velocity_mps": propwash["blown_effective_velocity_mps"],
        "climb_unblown_dynamic_pressure_pa": propwash["unblown_dynamic_pressure_pa"],
        "climb_blown_dynamic_pressure_pa": propwash["blown_dynamic_pressure_pa"],
        "climb_effective_dynamic_pressure_pa": propwash["effective_dynamic_pressure_pa"],
    }


def _max_operating_thrust(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    velocity_mps: float,
    rpm_bounds: tuple[int, int],
    config: Stage3SizingConfig,
) -> float:
    return _stage3_prop_operating_point(
        mission,
        candidate,
        _effective_rpm_bounds_for_prop_model(mission, candidate, rpm_bounds, config)[1],
        velocity_mps,
        config,
    )["thrust_total_n"]


def _mass_case_cruise_performance(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    geometry: dict[str, Any],
    config: Stage3SizingConfig,
    clean_polar: dict[str, Any],
    mass_kg: float,
) -> dict[str, float]:
    case_mission = replace(mission, gross_mass_kg=mass_kg)
    q = 0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2
    weight_n = case_mission.gross_weight_n
    cl_required = weight_n / max(q * mission.wing_area_m2, 1e-9)
    alpha_deg = _alpha_for_cl(clean_polar, cl_required)
    breakdown = _drag_breakdown_for_condition(
        case_mission,
        geometry,
        config,
        clean_polar,
        velocity_mps=mission.cruise_speed_mps,
        alpha_deg=alpha_deg,
        mode="clean",
    )
    thrust_required = mission.thrust_margin_cruise * breakdown["total_drag_n"]
    op = _solve_prop_power_for_thrust(
        mission,
        candidate,
        mission.cruise_speed_mps,
        thrust_required,
        mission.cruise_rpm_bounds,
        config,
    )
    propwash = _propwash_dynamic_pressure_summary(
        mission,
        candidate,
        geometry,
        op["thrust_total_n"],
        mission.cruise_speed_mps,
    )
    local_cl_required = weight_n / max(propwash["effective_dynamic_pressure_pa"] * mission.wing_area_m2, 1e-9)
    max_thrust = _max_operating_thrust(mission, candidate, mission.cruise_speed_mps, mission.cruise_rpm_bounds, config)
    return {
        "mass_kg": mass_kg,
        "speed_mps": mission.cruise_speed_mps,
        "cl_required": cl_required,
        "uniform_local_cl_required": local_cl_required,
        "drag_n": breakdown["total_drag_n"],
        "thrust_required_n": thrust_required,
        "throttle_percent": _clamp(100.0 * thrust_required / max(max_thrust, 1e-9), 0.0, 100.0),
        "rpm": op["rpm"],
        "power_w": op["power_electric_total_w"],
        "blown_effective_velocity_mps": propwash["blown_effective_velocity_mps"],
        "effective_dynamic_pressure_pa": propwash["effective_dynamic_pressure_pa"],
        "rpm_feasible": op["rpm_feasible"],
    }


def _mass_case_slow_performance(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    geometry: dict[str, Any],
    config: Stage3SizingConfig,
    mass_kg: float,
) -> dict[str, float]:
    speed = _stage3_slow_flight_speed_mps(config)
    weight_n = mass_kg * mission.gravity_mps2
    q_low = 0.5 * mission.air_density_kgpm3 * speed**2
    s_wing = mission.wing_area_m2
    aspect_ratio = mission.aspect_ratio
    eta_blown = _clamp(_safe_float(geometry.get("blown_span_fraction"), 0.35), 0.05, 0.95)
    s_blown = eta_blown * s_wing
    s_unblown = (1.0 - eta_blown) * s_wing
    design_unblown_clmax = _safe_float(geometry.get("slow_flight_design_unblown_clmax"), 1.0)
    design_blown_local_clmax = _safe_float(geometry.get("slow_flight_design_blown_local_clmax"), 1.0)
    lift_remaining = max(weight_n - q_low * s_unblown * design_unblown_clmax, 0.0)
    q_blown_required = lift_remaining / max(s_blown * design_blown_local_clmax, 1e-9)
    required_veff = max(speed, math.sqrt(max(2.0 * q_blown_required / mission.air_density_kgpm3, 0.0)))
    thrust_req_veff = thrust_required_for_veff(mission, candidate, required_veff, speed)

    low_htail_drag = q_low * geometry["htail_area_m2"] * 0.035
    low_vtail_drag = q_low * geometry["vtail_area_m2"] * 0.030
    low_fuselage_drag = q_low * config.fuselage_cd_area_m2
    freestream_cl_required = weight_n / max(q_low * s_wing, 1e-9)

    def drag_for_thrust(thrust_n: float) -> dict[str, float]:
        actual_veff = slipstream_velocity_after_prop(mission, candidate, thrust_n, speed)
        q_blown = 0.5 * mission.air_density_kgpm3 * actual_veff**2
        unblown_local_cl = min(design_unblown_clmax, weight_n / max(q_low * s_unblown, 1e-9))
        lift_from_unblown = q_low * s_unblown * unblown_local_cl
        blown_local_cl = max(weight_n - lift_from_unblown, 0.0) / max(q_blown * s_blown, 1e-9)
        wing_profile_drag = (
            q_low * s_unblown * mission.cd0_flapped
            + q_blown * s_blown * mission.cd0_flapped * mission.blown_profile_drag_factor
        )
        unblown_induced_drag = q_low * s_unblown * (
            unblown_local_cl**2 / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
        )
        blown_induced_drag = q_blown * s_blown * (
            blown_local_cl**2 / max(math.pi * mission.oswald_e * aspect_ratio, 1e-9)
        )
        natural_drag = (
            wing_profile_drag
            + unblown_induced_drag
            + blown_induced_drag
            + low_htail_drag
            + low_vtail_drag
            + low_fuselage_drag
        )
        return {
            "actual_veff": actual_veff,
            "q_blown": q_blown,
            "unblown_local_cl": unblown_local_cl,
            "blown_local_cl": blown_local_cl,
            "wing_profile_drag_n": wing_profile_drag,
            "wing_induced_drag_n": unblown_induced_drag + blown_induced_drag,
            "natural_drag_n": natural_drag,
        }

    thrust_required = max(
        thrust_req_veff,
        mission.thrust_margin_low_speed * _safe_float(geometry.get("low_speed_natural_drag_n"), 0.0) * mass_kg / max(mission.gross_mass_kg, 1e-9),
    )
    op = _solve_prop_power_for_thrust(mission, candidate, speed, thrust_required, mission.low_speed_rpm_bounds, config)
    drag_model = drag_for_thrust(op["thrust_total_n"])
    explicit_required = max(thrust_req_veff, mission.thrust_margin_low_speed * drag_model["natural_drag_n"])
    if explicit_required > thrust_required + 1e-6:
        thrust_required = explicit_required
        op = _solve_prop_power_for_thrust(mission, candidate, speed, thrust_required, mission.low_speed_rpm_bounds, config)
        drag_model = drag_for_thrust(op["thrust_total_n"])

    q_effective = (q_low * s_unblown + drag_model["q_blown"] * s_blown) / max(s_wing, 1e-9)
    local_cl_required = weight_n / max(q_effective * s_wing, 1e-9)
    steady_drag = max(drag_model["natural_drag_n"], op["thrust_total_n"])
    max_thrust = _max_operating_thrust(mission, candidate, speed, mission.low_speed_rpm_bounds, config)
    return {
        "mass_kg": mass_kg,
        "speed_mps": speed,
        "cl_required": freestream_cl_required,
        "uniform_local_cl_required": local_cl_required,
        "drag_n": steady_drag,
        "natural_drag_n": drag_model["natural_drag_n"],
        "thrust_required_n": op["thrust_required_n"],
        "throttle_percent": _clamp(100.0 * op["thrust_required_n"] / max(max_thrust, 1e-9), 0.0, 100.0),
        "rpm": op["rpm"],
        "power_w": op["power_electric_total_w"],
        "blown_effective_velocity_mps": drag_model["actual_veff"],
        "effective_dynamic_pressure_pa": q_effective,
        "unblown_local_cl": drag_model["unblown_local_cl"],
        "blown_local_cl": drag_model["blown_local_cl"],
        "local_cl_margin": design_blown_local_clmax - drag_model["blown_local_cl"],
        "added_drag_required_n": max(0.0, steady_drag - drag_model["natural_drag_n"]),
        "rpm_feasible": op["rpm_feasible"],
    }


def _mass_case_condition_summary(
    mission: Stage1MissionConfig,
    candidate: Stage1Candidate,
    geometry: dict[str, Any],
    config: Stage3SizingConfig,
    clean_polar: dict[str, Any],
    flapped_polar: dict[str, Any],
    mass_kg: float,
) -> dict[str, float]:
    case_mission = replace(mission, gross_mass_kg=mass_kg)
    cruise = _mass_case_cruise_performance(mission, candidate, geometry, config, clean_polar, mass_kg)
    slow = _mass_case_slow_performance(mission, candidate, geometry, config, mass_kg)
    approach = calculate_approach_trim(case_mission, geometry, {}, config, candidate, flapped_polar)
    climb = calculate_climb_trim(case_mission, geometry, config, candidate, clean_polar)

    return {
        "mass_kg": mass_kg,
        "cruise_cl_required": cruise["cl_required"],
        "cruise_uniform_local_cl_required": cruise["uniform_local_cl_required"],
        "cruise_drag_n": cruise["drag_n"],
        "cruise_thrust_required_n": cruise["thrust_required_n"],
        "cruise_throttle_percent": cruise["throttle_percent"],
        "cruise_rpm": cruise["rpm"],
        "cruise_power_w": cruise["power_w"],
        "cruise_blown_effective_velocity_mps": cruise["blown_effective_velocity_mps"],
        "cruise_effective_dynamic_pressure_pa": cruise["effective_dynamic_pressure_pa"],
        "slow_cl_required": slow["cl_required"],
        "slow_uniform_local_cl_required": slow["uniform_local_cl_required"],
        "slow_drag_n": slow["drag_n"],
        "slow_natural_drag_n": slow["natural_drag_n"],
        "slow_added_drag_required_n": slow["added_drag_required_n"],
        "slow_thrust_required_n": slow["thrust_required_n"],
        "slow_throttle_percent": slow["throttle_percent"],
        "slow_rpm": slow["rpm"],
        "slow_power_w": slow["power_w"],
        "slow_blown_effective_velocity_mps": slow["blown_effective_velocity_mps"],
        "slow_effective_dynamic_pressure_pa": slow["effective_dynamic_pressure_pa"],
        "slow_unblown_local_cl": slow["unblown_local_cl"],
        "slow_blown_local_cl": slow["blown_local_cl"],
        "slow_local_cl_margin": slow["local_cl_margin"],
        "approach_cl_required": approach["approach_cl_required"],
        "approach_uniform_local_cl_required": approach["approach_uniform_local_cl_required"],
        "approach_drag_n": approach["approach_drag_n"],
        "approach_thrust_required_n": approach["approach_thrust_required_n"],
        "approach_throttle_percent": approach["approach_throttle_percent"],
        "approach_rpm": approach["approach_rpm"],
        "approach_power_w": approach["approach_power_w"],
        "approach_blown_effective_velocity_mps": approach["approach_blown_effective_velocity_mps"],
        "approach_effective_dynamic_pressure_pa": approach["approach_effective_dynamic_pressure_pa"],
        "approach_elevator_trim_deg": approach["approach_elevator_trim_deg"],
        "approach_rpm_feasible": approach["approach_rpm_feasible"],
        "climb_cl_required": climb["climb_cl_required"],
        "climb_uniform_local_cl_required": climb["climb_uniform_local_cl_required"],
        "climb_drag_n": climb["climb_drag_n"],
        "climb_thrust_required_n": climb["climb_thrust_required_n"],
        "climb_throttle_percent": climb["climb_throttle_percent"],
        "climb_rpm": climb["climb_rpm"],
        "climb_power_w": climb["climb_power_w"],
        "climb_blown_effective_velocity_mps": climb["climb_blown_effective_velocity_mps"],
        "climb_effective_dynamic_pressure_pa": climb["climb_effective_dynamic_pressure_pa"],
        "climb_elevator_trim_deg": climb["climb_elevator_trim_deg"],
        "climb_rpm_feasible": climb["climb_rpm_feasible"],
    }


def generate_performance_sweep_outputs(
    mission: Stage1MissionConfig,
    geometry: dict[str, Any],
    wing_context: dict[str, Any],
    config: Stage3SizingConfig,
    candidate: Stage1Candidate,
    polars: dict[str, Any],
    *,
    csv_path: Path,
    plot_path: Path,
    components_path: Path,
) -> dict[str, Any]:
    runtime = ensure_stage3_runtime()
    onp = runtime.onp
    plt = runtime.plt
    clean_polar = _section_polar_from_stage3_polars(polars, "clean")
    flapped_polar = {
        "alpha_deg": polars["alphas_deg"],
        "CL": polars["flapped"]["CL"],
        "CD": polars["flapped"]["CD"],
        "CM": polars["flapped"]["CM"],
    }

    velocities = onp.linspace(
        config.performance_sweep_velocity_min_mps,
        config.performance_sweep_velocity_max_mps,
        max(2, int(config.performance_sweep_velocity_points)),
    )
    alphas = onp.linspace(
        config.performance_sweep_alpha_min_deg,
        config.performance_sweep_alpha_max_deg,
        max(2, int(config.performance_sweep_alpha_points)),
    )
    rows: list[dict[str, Any]] = []
    curves: dict[tuple[str, str], list[dict[str, float]]] = {}

    for mode, polar in (("clean", clean_polar), ("flaps_down", flapped_polar)):
        velocity_curve: list[dict[str, float]] = []
        for velocity in velocities:
            q = 0.5 * mission.air_density_kgpm3 * float(velocity) ** 2
            cl_required = mission.gross_weight_n / max(q * mission.wing_area_m2, 1e-9)
            if mode == "flaps_down":
                breakdown = _flaps_down_split_sweep_condition(
                    mission,
                    candidate,
                    geometry,
                    config,
                    flapped_polar,
                    velocity_mps=float(velocity),
                    mass_kg=mission.gross_mass_kg,
                )
            else:
                polar_cl = onp.asarray(polar["CL"], dtype=float)
                max_cl = float(onp.nanmax(polar_cl))
                min_cl = float(onp.nanmin(polar_cl))
                if cl_required > max_cl or cl_required < min_cl:
                    breakdown = {
                        "velocity_mps": float(velocity),
                        "alpha_deg": float("nan"),
                        "cl": float("nan"),
                        "cl_required": cl_required,
                        "cm": float("nan"),
                        "tail_cl": float("nan"),
                        "wing_profile_drag_n": float("nan"),
                        "wing_induced_drag_n": float("nan"),
                        "htail_drag_n": float("nan"),
                        "vtail_drag_n": float("nan"),
                        "fuselage_drag_n": float("nan"),
                        "total_drag_n": float("nan"),
                        "power_w": float("nan"),
                        "rpm": float("nan"),
                        "thrust_required_n": float("nan"),
                        "thrust_available_n": float("nan"),
                        "rpm_feasible": 0.0,
                        "trim_feasible": 0.0,
                        "added_drag_required_n": float("nan"),
                        "blown_effective_velocity_mps": float("nan"),
                    }
                else:
                    alpha = _alpha_for_cl(polar, cl_required)
                    breakdown = _append_power_to_breakdown(
                        mission,
                        candidate,
                        _drag_breakdown_for_condition(
                            mission,
                            geometry,
                            config,
                            polar,
                            velocity_mps=float(velocity),
                            alpha_deg=alpha,
                            mode=mode,
                        ),
                        mode,
                        config,
                    )
                    breakdown["trim_feasible"] = breakdown["rpm_feasible"]
                    breakdown["added_drag_required_n"] = 0.0
                    breakdown["blown_effective_velocity_mps"] = float("nan")
                    breakdown["power_w"] = _power_plot_value(breakdown)
                breakdown["cl_required"] = cl_required
            velocity_curve.append(breakdown)
            rows.append({"mode": mode, "sweep_type": "velocity", **breakdown})
        curves[(mode, "velocity")] = velocity_curve

        alpha_curve: list[dict[str, float]] = []
        fixed_speed = config.flaps_down_sweep_speed_mps if mode == "flaps_down" else config.clean_sweep_speed_mps
        for alpha in alphas:
            if mode == "flaps_down":
                breakdown = _flaps_down_split_sweep_condition(
                    mission,
                    candidate,
                    geometry,
                    config,
                    flapped_polar,
                    velocity_mps=fixed_speed,
                    alpha_deg=float(alpha),
                    mass_kg=mission.gross_mass_kg,
                )
            else:
                breakdown = _append_power_to_breakdown(
                    mission,
                    candidate,
                    _drag_breakdown_for_condition(
                        mission,
                        geometry,
                        config,
                        polar,
                        velocity_mps=fixed_speed,
                        alpha_deg=float(alpha),
                        mode=mode,
                    ),
                    mode,
                    config,
                )
                breakdown["trim_feasible"] = breakdown["rpm_feasible"]
                breakdown["added_drag_required_n"] = 0.0
                breakdown["blown_effective_velocity_mps"] = float("nan")
                breakdown["power_w"] = _power_plot_value(breakdown)
                breakdown["cl_required"] = float("nan")
            alpha_curve.append(breakdown)
            rows.append({"mode": mode, "sweep_type": "alpha", **breakdown})
        curves[(mode, "alpha")] = alpha_curve

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "mode",
        "sweep_type",
        "velocity_mps",
        "alpha_deg",
        "cl",
        "cl_required",
        "cm",
        "tail_cl",
        "wing_profile_drag_n",
        "wing_induced_drag_n",
        "htail_drag_n",
        "vtail_drag_n",
        "fuselage_drag_n",
        "total_drag_n",
        "power_w",
        "rpm",
        "thrust_required_n",
        "thrust_available_n",
        "rpm_feasible",
        "trim_feasible",
        "added_drag_required_n",
        "blown_effective_velocity_mps",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    colors = {"clean": "#2563eb", "flaps_down": "#dc2626"}
    labels = {"clean": "Clean cruise", "flaps_down": "Flaps down slow"}
    fig, axs = plt.subplots(2, 2, figsize=(12.4, 8.4))
    for mode in ("clean", "flaps_down"):
        v_curve = curves[(mode, "velocity")]
        a_curve = curves[(mode, "alpha")]
        axs[0, 0].plot([p["velocity_mps"] for p in v_curve], [p["total_drag_n"] for p in v_curve], label=labels[mode], color=colors[mode])
        axs[1, 0].plot([p["velocity_mps"] for p in v_curve], [p["power_w"] for p in v_curve], label=labels[mode], color=colors[mode])
        axs[0, 1].plot([p["alpha_deg"] for p in a_curve], [p["total_drag_n"] for p in a_curve], label=labels[mode], color=colors[mode])
        axs[1, 1].plot([p["alpha_deg"] for p in a_curve], [p["power_w"] for p in a_curve], label=labels[mode], color=colors[mode])

    def set_axis_limits_from_data(ax: Any, x_values: list[float], y_values: list[float]) -> None:
        finite_x = [float(value) for value in x_values if math.isfinite(float(value))]
        finite_y = [float(value) for value in y_values if math.isfinite(float(value))]
        if finite_x:
            x_min = min(finite_x)
            x_max = max(finite_x)
            x_pad = 0.03 * max(x_max - x_min, 1.0)
            ax.set_xlim(x_min - x_pad, x_max + x_pad)
        if finite_y:
            y_min = min(finite_y)
            y_max = max(finite_y)
            if y_max <= y_min:
                y_pad = max(abs(y_max) * 0.08, 1.0)
            else:
                y_pad = 0.08 * (y_max - y_min)
            ax.set_ylim(min(0.0, y_min - y_pad), y_max + y_pad)

    velocity_x = [
        point["velocity_mps"]
        for mode in ("clean", "flaps_down")
        for point in curves[(mode, "velocity")]
    ]
    alpha_x = [
        point["alpha_deg"]
        for mode in ("clean", "flaps_down")
        for point in curves[(mode, "alpha")]
    ]
    velocity_drag_y = [
        point["total_drag_n"]
        for mode in ("clean", "flaps_down")
        for point in curves[(mode, "velocity")]
    ]
    velocity_power_y = [
        point["power_w"]
        for mode in ("clean", "flaps_down")
        for point in curves[(mode, "velocity")]
    ]
    alpha_drag_y = [
        point["total_drag_n"]
        for mode in ("clean", "flaps_down")
        for point in curves[(mode, "alpha")]
    ]
    alpha_power_y = [
        point["power_w"]
        for mode in ("clean", "flaps_down")
        for point in curves[(mode, "alpha")]
    ]
    set_axis_limits_from_data(axs[0, 0], velocity_x, velocity_drag_y)
    set_axis_limits_from_data(axs[1, 0], velocity_x, velocity_power_y)
    set_axis_limits_from_data(axs[0, 1], alpha_x, alpha_drag_y)
    set_axis_limits_from_data(axs[1, 1], alpha_x, alpha_power_y)

    axs[0, 0].set_title("Total Drag vs Velocity")
    axs[0, 1].set_title("Total Drag vs Angle of Attack")
    axs[1, 0].set_title("Electrical Power Required vs Velocity")
    axs[1, 1].set_title("Electrical Power Required vs Angle of Attack")
    axs[0, 0].set_xlabel("Velocity [m/s]")
    axs[1, 0].set_xlabel("Velocity [m/s]")
    axs[0, 1].set_xlabel("Angle of attack [deg]")
    axs[1, 1].set_xlabel("Angle of attack [deg]")
    axs[0, 0].set_ylabel("Drag [N]")
    axs[0, 1].set_ylabel("Drag [N]")
    axs[1, 0].set_ylabel("Power [W]")
    axs[1, 1].set_ylabel("Power [W]")
    for ax in axs.flat:
        ax.grid(True, alpha=0.3)
        ax.legend()
    note = (
        "Clean power balances trimmed drag; flaps-down power uses the split blown/unblown wing model. "
        "Infeasible max-RPM points are omitted instead of plotted as saturated power."
    )
    fig.text(0.5, 0.01, note, ha="center", va="bottom", fontsize=8, color="#374151")
    fig.tight_layout()
    fig.subplots_adjust(bottom=0.08)
    fig.savefig(plot_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    clean_components = {
        "Wing profile": geometry["cruise_wing_profile_drag_n"],
        "Wing induced": geometry["cruise_wing_induced_drag_n"],
        "H-tail": geometry["cruise_htail_drag_n"],
        "V-tail": geometry["cruise_vtail_drag_n"],
        "Fuselage": geometry["cruise_fuselage_drag_n"],
    }
    slow_components = {
        "Wing profile": geometry["low_speed_wing_profile_drag_n"],
        "Wing induced (used)": geometry["low_speed_wing_induced_drag_n"],
        "H-tail": geometry["low_speed_htail_drag_n"],
        "V-tail": geometry["low_speed_vtail_drag_n"],
        "Fuselage": geometry["low_speed_fuselage_drag_n"],
        "Added drag / airbrakes": geometry["low_speed_added_drag_required_n"],
    }
    fig, axs = plt.subplots(1, 2, figsize=(12.4, 4.8), sharey=False)
    for ax, title, components in (
        (axs[0], "Clean Cruise Drag Components", clean_components),
        (axs[1], "Flaps-Down Slow-Flight Drag Components", slow_components),
    ):
        names = list(components)
        values = [components[name] for name in names]
        palette = ["#2563eb", "#60a5fa", "#14b8a6", "#a78bfa", "#f97316", "#ef4444"]
        bars = ax.barh(names, values, color=[palette[i % len(palette)] for i in range(len(names))])
        ax.set_title(title)
        ax.set_xlabel("Drag [N]")
        ax.grid(True, axis="x", alpha=0.25)
        for bar, value in zip(bars, values):
            ax.text(value, bar.get_y() + bar.get_height() / 2.0, f" {value:.2f}", va="center", fontsize=8)
    fig.tight_layout()
    fig.savefig(components_path, dpi=180, bbox_inches="tight")
    plt.close(fig)

    approach = calculate_approach_trim(
        mission,
        geometry,
        wing_context,
        config,
        candidate,
        flapped_polar,
    )
    climb = calculate_climb_trim(
        mission,
        geometry,
        config,
        candidate,
        clean_polar,
    )
    return {
        "performance_sweep_csv": str(csv_path),
        "drag_power_sweeps_png": str(plot_path),
        "drag_components_png": str(components_path),
        **approach,
        **climb,
    }


def write_trade_space_plot(results: list[dict[str, Any]], output_path: Path) -> None:
    runtime = ensure_stage3_runtime()
    plt = runtime.plt

    successful = [row for row in results if row["status"] == "SUCCESS"]
    if not successful:
        return

    fig, ax = plt.subplots(figsize=(7.6, 5.4))
    x = [_safe_float(row["cruise_power_w"]) for row in successful]
    y = [_safe_float(row["low_speed_power_w"]) for row in successful]
    s = [80.0 + 900.0 * _safe_float(row["total_tail_foam_mass_kg"]) for row in successful]
    c = [_safe_float(row["static_margin_mac"]) for row in successful]
    scatter = ax.scatter(x, y, s=s, c=c, cmap="viridis", alpha=0.86, edgecolor="#111827")
    for row in successful[:8]:
        ax.annotate(
            f"#{row['rank']}",
            (_safe_float(row["cruise_power_w"]), _safe_float(row["low_speed_power_w"])),
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
        )
    colorbar = fig.colorbar(scatter, ax=ax)
    colorbar.set_label("Static margin [MAC]")
    ax.set_xlabel("Cruise electrical power [W]")
    ax.set_ylabel("Slow-flight electrical power [W]")
    ax.set_title("Stage 3 Tail/Material Trade Space")
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
        "Generated from the fixed-wing AeroSandbox tail/material refinement pass.",
        "",
    ]
    for row in successful[:5]:
        lines.extend(
            [
                f"## Rank {row['rank']}: {row['n_props']} props, {row['prop_diameter_in']} in, {row['prop_family']}",
                "",
                f"- Airfoil: `{row['selected_airfoil']}`",
                f"- Wing: `{row['wing_span_m']} m` span x `{row['wing_chord_m']} m` chord",
                f"- H-tail: `{row['htail_span_m']} m` span, `{row['htail_root_chord_m']} m` root chord, `{row['htail_tip_chord_m']} m` tip chord",
                f"- V-tail: `{row['vtail_span_m']} m` height, `{row['vtail_root_chord_m']} m` root chord, `{row['vtail_tip_chord_m']} m` tip chord",
                f"- Cruise power: `{row['cruise_power_w']} W`",
                f"- Slow-flight power: `{row['low_speed_power_w']} W`",
                "",
                f"![Top view]({Path(str(row['top_view_png'])).name})",
                "",
                f"![3D wireframe render]({Path(str(row['render_3d_png'])).name})",
                "",
                f"![Three view]({Path(str(row['three_view_png'])).name})",
                "",
                f"![Drag and power sweeps]({Path(str(row['drag_power_sweeps_png'])).name})",
                "",
                f"![Drag components]({Path(str(row['drag_components_png'])).name})",
                "",
            ]
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")


def write_stage3_report(
    results: list[dict[str, Any]],
    output_path: Path,
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> None:
    successful = [row for row in results if row["status"] == "SUCCESS"]
    lines = [
        "# Stage 3 AeroSandbox Fixed-Wing Tail Refinement",
        "",
        "## Scope",
        "",
        "Stage 3 freezes the selected Stage 1/2 main wing and propulsion layout, then sizes the horizontal and vertical tail using AeroSandbox geometry, VLM trim/stability checks, NeuralFoil section data, prop RPM re-solves, and an explicit NGX250 foam material model.",
        "",
        "## Fixed Inputs",
        "",
        f"- Gross flight mass for lift balance: `{mission.gross_mass_kg:.3f} kg`",
        f"- Maximum as-built mass: `{mission.max_mass_kg:.3f} kg`",
        f"- Main wing span: `{mission.span_m:.3f} m`",
        f"- Main wing chord: `{mission.chord_m:.3f} m`",
        f"- Main wing area: `{mission.wing_area_m2:.3f} m^2`",
        f"- Fuselage nose station: `{config.fuselage_nose_x_m:.3f} m` from wing leading edge",
        f"- Fuselage cross-section: `{config.fuselage_width_m * 1000.0:.0f} mm` wide x `{config.fuselage_height_m * 1000.0:.0f} mm` tall",
        f"- Stage 3 slow-flight mode: `{_stage3_slow_flight_speed_mps(config):.3f} m/s`",
        f"- Upstream Stage 1/2 low-speed mode remains: `{mission.low_speed_mps:.3f} m/s`",
        f"- Cruise mode: `{mission.cruise_speed_mps:.3f} m/s`",
        f"- Foam density: `{config.foam_density_kgpm3:.1f} kg/m^3`",
        f"- eCalc propulsion calibration enabled: `{config.ecalc_propulsion_enabled}`",
        "",
        "## Optimizer Notes",
        "",
        f"- Editable Stage 3 constraints are loaded from `{STAGE3_CONSTRAINTS_YAML}` when that YAML file is present.",
        "- Main wing dimensions and selected airfoil are not optimized in Stage 3.",
        "- Horizontal and vertical stabilizer planform dimensions are optimized primarily for cruise trim, cruise drag, material mass, static margin, and the user-specified tail-volume ranges.",
        "- Elevator and rudder chord fractions are sized separately after planform optimization to meet slow-flight pitch and yaw authority targets.",
        "- Slow-flight sizing is evaluated with the frozen slotted flaps deployed.",
        "- Raw Stage 1/2 equivalent CLmax values are read for traceability. Physical local CLmax limits come from the Stage 1/2 NeuralFoil flap-sweep section polars when available; the YAML values are only fallbacks. Blown lift is modeled through higher local dynamic pressure, not through unphysical airfoil CLmax.",
        f"- Fuselage drag is included as an equivalent parasite drag area, `CdA = {config.fuselage_cd_area_m2:.3f} m^2`, so `D_fuse = q * CdA`.",
        "- Tail foam mass is added to the Stage 1/2 built-mass estimate before checking the mass margin.",
        "- Tail sizing is constrained by horizontal/vertical tail volume, static margin, trim incidence, material mass, and prop RPM feasibility.",
        "- Cruise power and slow-flight power are both re-solved. If the frozen propulsion layout matches the eCalc calibration files, Stage 3 uses the eCalc thrust/power table and dynamic CT correction; otherwise it falls back to the Stage 1 generic prop surrogate.",
        "",
    ]

    if successful:
        best = successful[0]
        lines.extend(
            [
                "## Recommended Aircraft Dimensions",
                "",
                "| Quantity | Value |",
                "| --- | ---: |",
                f"| Propulsion layout | {best['n_props']} x {best['prop_diameter_in']} in, P/D {best['prop_pitch_ratio']}, {best['prop_family']} |",
                f"| Propulsion model | {best['propulsion_model_source']} |",
                f"| eCalc static CSV | {best['ecalc_static_csv']} |",
                f"| Main airfoil | {best['selected_airfoil']} |",
                f"| Main wing span | {float(best['wing_span_m']):.3f} m |",
                f"| Main wing chord | {float(best['wing_chord_m']):.3f} m |",
                f"| Main wing incidence | {float(best['main_wing_incidence_deg']):.3f} deg |",
                f"| Cruise body alpha proxy | {float(best['cruise_body_alpha_proxy_deg']):.3f} deg |",
                f"| Cruise wing section alpha proxy | {float(best['cruise_section_alpha_proxy_deg']):.3f} deg |",
                f"| Cruise VLM trim alpha diagnostic | {float(best['cruise_alpha_deg']):.3f} deg |",
                f"| Propeller axial x position | {float(best['prop_axial_x_m']):.3f} m |",
                f"| Fuselage nose station | {float(best['fuselage_nose_x_m']):.3f} m from wing LE |",
                f"| Fuselage width x height | {float(best['fuselage_width_m']) * 1000.0:.0f} mm x {float(best['fuselage_height_m']) * 1000.0:.0f} mm |",
                f"| Horizontal tail span | {float(best['htail_span_m']):.3f} m |",
                f"| Horizontal tail root chord | {float(best['htail_root_chord_m']):.3f} m |",
                f"| Horizontal tail tip chord | {float(best['htail_tip_chord_m']):.3f} m |",
                f"| Horizontal tail incidence | {float(best['htail_incidence_deg']):.3f} deg |",
                f"| Elevator chord fraction | {float(best['elevator_chord_fraction']):.3f} chord |",
                f"| Vertical tail height/span | {float(best['vtail_span_m']):.3f} m |",
                f"| Vertical tail root chord | {float(best['vtail_root_chord_m']):.3f} m |",
                f"| Vertical tail tip chord | {float(best['vtail_tip_chord_m']):.3f} m |",
                f"| Vertical tail incidence | {float(best['vertical_tail_incidence_deg']):.3f} deg |",
                f"| Rudder chord fraction | {float(best['rudder_chord_fraction']):.3f} chord |",
                f"| Tail arm | {float(best['tail_arm_m']):.3f} m |",
                f"| CG target / actual | {float(best['cg_target_percent_mac']):.2f}% / {float(best['cg_percent_mac']):.2f}% MAC |",
                f"| Required pre-tail baseline CG | {float(best['stage1_baseline_cg_required_percent_mac']):.2f}% MAC |",
                f"| Static margin | {float(best['static_margin_mac']):.3f} MAC |",
                f"| H-tail volume range / actual | {float(best['horizontal_tail_volume_min']):.3f}-{float(best['horizontal_tail_volume_max']):.3f} / {float(best['horizontal_tail_volume']):.3f} |",
                f"| V-tail volume range / actual | {float(best['vertical_tail_volume_min']):.3f}-{float(best['vertical_tail_volume_max']):.3f} / {float(best['vertical_tail_volume']):.3f} |",
                f"| Slow pitch control authority / target | {float(best['slow_pitch_control_cm_authority']):.3f} / {float(best['target_slow_pitch_control_cm']):.3f} Cm |",
                f"| Slow yaw control authority / target | {float(best['slow_yaw_control_cn_authority']):.3f} / {float(best['target_slow_yaw_control_cn']):.3f} Cn |",
                f"| Gross flight mass used in all lift/trim calculations | {float(best['gross_flight_mass_kg']):.3f} kg |",
                f"| Component-estimated built mass before ballast/payload | {float(best['stage3_total_built_mass_kg']):.3f} kg |",
                f"| Remaining mass to 5 kg gross target | {float(best['remaining_mass_to_gross_kg']):.3f} kg |",
                f"| Tail foam mass | {float(best['total_tail_foam_mass_kg']):.3f} kg |",
                f"| Physical CLmax source | {best['physical_clmax_source']} |",
                f"| Stage 1/2 clean section CLmax, unblown / blown | {float(best['stage12_clean_section_clmax_unblown']):.3f} / {float(best['stage12_clean_section_clmax_blown']):.3f} |",
                f"| Stage 1/2 flapped section CLmax, unblown / blown | {float(best['stage12_flapped_section_clmax_unblown']):.3f} / {float(best['stage12_flapped_section_clmax_blown']):.3f} |",
                f"| Physical no-flap CLmax used | {float(best['no_flap_clmax']):.3f} |",
                f"| Physical flap-only CLmax used | {float(best['flap_only_clmax']):.3f} |",
                f"| Physical weighted blown-panel CLmax used | {float(best['stage12_weighted_blown_physical_clmax']):.3f} |",
                f"| Raw Stage 1/2 flap-down blown equivalent CLmax | {float(best['raw_flap_down_blown_clmax']):.3f} |",
                f"| Stage 1/2 equivalent CLmax capped? | {bool(float(best['clmax_was_capped']))} |",
                f"| Slow-flight flap-down CLmax, unblown equivalent | {float(best['slow_flight_unblown_equivalent_clmax']):.3f} |",
                f"| Slow-flight flap-down CLmax, blown equivalent | {float(best['slow_flight_blown_equivalent_clmax']):.3f} |",
                f"| Slow-flight blown-panel local physical CLmax | {float(best['slow_flight_blown_local_clmax']):.3f} |",
                f"| Slow-flight lift margin | {float(best['slow_flight_lift_margin_percent']):.1f}% |",
                f"| Cruise fuselage drag | {float(best['cruise_fuselage_drag_n']):.3f} N |",
                f"| Cruise power | {float(best['cruise_power_w']):.2f} W |",
                f"| Cruise freestream CL / local blown-wing CL / RPM | {float(best['cruise_cl_required']):.3f} / {float(best['cruise_uniform_local_cl_required']):.3f} / {float(best['cruise_rpm']):.0f} rpm |",
                f"| Cruise q unblown / blown / effective | {float(best['cruise_unblown_dynamic_pressure_pa']):.1f} / {float(best['cruise_blown_dynamic_pressure_pa']):.1f} / {float(best['cruise_effective_dynamic_pressure_pa']):.1f} Pa |",
                f"| Cruise blown effective velocity | {float(best['cruise_blown_effective_velocity_mps']):.3f} m/s |",
                f"| Cruise CT / CP | {float(best['cruise_ct']):.4f} / {float(best['cruise_cp']):.4f} |",
                f"| Power loss factors, wiring / avionics | {float(best['wiring_power_loss_factor']):.3f} / {float(best['avionics_power_loss_factor']):.3f} |",
                f"| Cruise aircraft electrical losses | {float(best['cruise_aircraft_power_losses_w']):.2f} W |",
                f"| Slow-flight power | {float(best['low_speed_power_w']):.2f} W |",
                f"| Slow-flight freestream CL / uniform local CL / RPM | {float(best['slow_flight_freestream_cl_required']):.3f} / {float(best['slow_flight_uniform_local_cl_required']):.3f} / {float(best['low_speed_rpm']):.0f} rpm |",
                f"| Slow-flight local CL, unblown / blown panels | {float(best['slow_flight_unblown_local_cl']):.3f} / {float(best['slow_flight_blown_local_cl']):.3f} |",
                f"| Slow-flight q unblown / blown / effective | {float(best['slow_flight_unblown_dynamic_pressure_pa']):.1f} / {float(best['slow_flight_blown_dynamic_pressure_pa']):.1f} / {float(best['slow_flight_effective_dynamic_pressure_pa']):.1f} Pa |",
                f"| Slow-flight CT / CP | {float(best['low_speed_ct']):.4f} / {float(best['low_speed_cp']):.4f} |",
                f"| Slow-flight aircraft electrical losses | {float(best['low_speed_aircraft_power_losses_w']):.2f} W |",
                f"| Slow-flight natural drag before added drag | {float(best['low_speed_natural_drag_n']):.3f} N |",
                f"| Slow-flight wing profile drag | {float(best['low_speed_wing_profile_drag_n']):.3f} N |",
                f"| Slow-flight wing induced drag used | {float(best['low_speed_wing_induced_drag_n']):.3f} N |",
                f"| Slow-flight induced drag, local blown-panel estimate | {float(best['low_speed_wing_induced_local_drag_n']):.3f} N |",
                f"| Slow-flight induced drag, freestream-equivalent check | {float(best['low_speed_wing_induced_freestream_equiv_drag_n']):.3f} N |",
                f"| Slow-flight added drag required | {float(best['low_speed_added_drag_required_n']):.3f} N |",
                f"| Slow-flight steady total drag | {float(best['low_speed_drag_n']):.3f} N |",
                f"| Slow-flight steady drag minus cruise drag | {float(best['low_speed_drag_delta_vs_cruise_n']):.3f} N |",
                f"| Main wing Reynolds number, cruise / slow | {float(best['main_wing_re_cruise']):.0f} / {float(best['main_wing_re_low_speed']):.0f} |",
                f"| H-tail Reynolds number, cruise / slow | {float(best['htail_re_cruise']):.0f} / {float(best['htail_re_low_speed']):.0f} |",
                f"| V-tail Reynolds number, cruise / slow | {float(best['vtail_re_cruise']):.0f} / {float(best['vtail_re_low_speed']):.0f} |",
                f"| Approach target sink rate | {float(best['approach_target_sink_rate_fps']):.2f} ft/s |",
                f"| Actual approach sink rate | {float(best['approach_sink_rate_fps']):.2f} ft/s |",
                f"| Approach sink-rate target met? | {bool(float(best['approach_sink_rate_target_met']))} |",
                f"| Approach descent angle | {float(best['approach_target_angle_deg']):.2f} deg below horizontal |",
                f"| Approach speed | {float(best['approach_speed_mps']):.2f} m/s |",
                f"| Approach/descent CL required / RPM | {float(best['approach_cl_required']):.3f} / {float(best['approach_rpm']):.0f} rpm |",
                f"| Approach elevator trim | {float(best['approach_elevator_trim_deg']):.2f} deg |",
                f"| Approach throttle estimate | {float(best['approach_throttle_percent']):.1f}% |",
                "",
                "## Flight-Condition Operating Values",
                "",
                "| Condition | Speed [m/s] | RPM | Thrust [N] | Throttle [%] | Blown velocity [m/s] | q unblown/blown/eff [Pa] | CL free/local | Drag [N] | Power [W] |",
                "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
                f"| Cruise, clean | {mission.cruise_speed_mps:.2f} | {float(best['cruise_rpm']):.0f} | {float(best['cruise_thrust_required_n']):.3f} | {float(best['cruise_throttle_percent']):.1f} | {float(best['cruise_blown_effective_velocity_mps']):.2f} | {float(best['cruise_unblown_dynamic_pressure_pa']):.1f}/{float(best['cruise_blown_dynamic_pressure_pa']):.1f}/{float(best['cruise_effective_dynamic_pressure_pa']):.1f} | {float(best['cruise_cl_required']):.3f}/{float(best['cruise_uniform_local_cl_required']):.3f} | {float(best['cruise_drag_n']):.3f} | {float(best['cruise_power_w']):.2f} |",
                f"| Slow flight, flaps down | {_stage3_slow_flight_speed_mps(config):.2f} | {float(best['low_speed_rpm']):.0f} | {float(best['low_speed_thrust_required_n']):.3f} | {float(best['low_speed_throttle_percent']):.1f} | {float(best['low_speed_actual_veff_mps']):.2f} | {float(best['slow_flight_unblown_dynamic_pressure_pa']):.1f}/{float(best['slow_flight_blown_dynamic_pressure_pa']):.1f}/{float(best['slow_flight_effective_dynamic_pressure_pa']):.1f} | {float(best['slow_flight_freestream_cl_required']):.3f}/{float(best['slow_flight_uniform_local_cl_required']):.3f}; split {float(best['slow_flight_unblown_local_cl']):.3f}/{float(best['slow_flight_blown_local_cl']):.3f} | {float(best['low_speed_drag_n']):.3f} | {float(best['low_speed_power_w']):.2f} |",
                f"| Approach/descent, flaps down | {float(best['approach_speed_mps']):.2f} | {float(best['approach_rpm']):.0f} | {float(best['approach_thrust_required_n']):.3f} | {float(best['approach_throttle_percent']):.1f} | {float(best['approach_blown_effective_velocity_mps']):.2f} | {float(best['approach_unblown_dynamic_pressure_pa']):.1f}/{float(best['approach_blown_dynamic_pressure_pa']):.1f}/{float(best['approach_effective_dynamic_pressure_pa']):.1f} | {float(best['approach_cl_required']):.3f}/{float(best['approach_uniform_local_cl_required']):.3f} | {float(best['approach_drag_n']):.3f} | {float(best['approach_power_w']):.2f} |",
                f"| Climb, clean | {float(best['climb_speed_mps']):.2f} | {float(best['climb_rpm']):.0f} | {float(best['climb_thrust_required_n']):.3f} | {float(best['climb_throttle_percent']):.1f} | {float(best['climb_blown_effective_velocity_mps']):.2f} | {float(best['climb_unblown_dynamic_pressure_pa']):.1f}/{float(best['climb_blown_dynamic_pressure_pa']):.1f}/{float(best['climb_effective_dynamic_pressure_pa']):.1f} | {float(best['climb_cl_required']):.3f}/{float(best['climb_uniform_local_cl_required']):.3f} | {float(best['climb_drag_n']):.3f} | {float(best['climb_power_w']):.2f} |",
                "",
                "## Battery Capacity Estimate",
                "",
                "| Segment | Duration [min] | Power [W] | Energy [Wh] |",
                "| --- | ---: | ---: | ---: |",
                f"| Cruise | {float(best['battery_cruise_segment_min']):.1f} | {float(best['cruise_power_w']):.2f} | {float(best['battery_cruise_energy_wh']):.2f} |",
                f"| Slow flight | {float(best['battery_slow_segment_min']):.1f} | {float(best['low_speed_power_w']):.2f} | {float(best['battery_slow_energy_wh']):.2f} |",
                f"| Usable energy subtotal | {float(best['battery_cruise_segment_min']) + float(best['battery_slow_segment_min']):.1f} | -- | {float(best['battery_usable_energy_wh']):.2f} |",
                f"| Pack capacity with {float(best['battery_reserve_fraction']) * 100.0:.0f}% reserve | -- | -- | {float(best['battery_capacity_required_wh']):.2f} |",
                "",
                f"Equivalent 4S LiPo capacity for this conservative sizing case: `{float(best['battery_capacity_required_mah']):.0f} mAh` at `{float(best['battery_pack_voltage_v']):.1f} V` nominal.",
                "",
                f"Estimated battery mass at {mission.battery_specific_energy_wh_per_kg:.0f} Wh/kg: `{float(best['battery_mass_required_kg']):.3f} kg`.",
                "",
                "## Mission Regime Energy Table",
                "",
                *_mission_regime_markdown_lines(best, mission, config),
                "",
                f"Required battery for this regime table: `{float(best['mission_profile_capacity_required_mah']):.0f} mAh` at `{float(best['mission_profile_pack_voltage_v']):.1f} V` nominal (`{int(float(best['mission_profile_battery_cell_count']))}S LiPo`), including `{float(best['mission_profile_reserve_fraction']) * 100.0:.0f}%` reserve. Usable energy before reserve is `{float(best['mission_profile_usable_energy_wh']):.2f} Wh`; reserve-adjusted capacity is `{float(best['mission_profile_capacity_required_wh']):.2f} Wh`.",
                "",
                "Takeoff is estimated from the payload-on climb condition. Landing is estimated from the post-delivery flaps-down slow/flare condition, which avoids using the saturated steep-approach thrust case as the final landing power.",
                "",
                "## Payload-On vs Post-Delivery Performance",
                "",
                f"Payload mass is `{float(best['payload_mass_lb']):.2f} lb` (`{float(best['payload_mass_kg']):.3f} kg`). Battery sizing uses the payload-on case for the whole mission, so a canceled delivery can return home without re-sizing the pack.",
                "",
                "| Configuration | Mass [kg] | Cruise P/RPM/throttle | Slow P/RPM/throttle | Approach P/RPM/throttle | Climb P/RPM/throttle |",
                "| --- | ---: | ---: | ---: | ---: | ---: |",
                f"| Payload on | {float(best['payload_on_mass_kg']):.3f} | {float(best['payload_on_cruise_power_w']):.1f} W / {float(best['payload_on_cruise_rpm']):.0f} rpm / {float(best['payload_on_cruise_throttle_percent']):.1f}% | {float(best['payload_on_slow_power_w']):.1f} W / {float(best['payload_on_slow_rpm']):.0f} rpm / {float(best['payload_on_slow_throttle_percent']):.1f}% | {float(best['payload_on_approach_power_w']):.1f} W / {float(best['payload_on_approach_rpm']):.0f} rpm / {float(best['payload_on_approach_throttle_percent']):.1f}% | {float(best['payload_on_climb_power_w']):.1f} W / {float(best['payload_on_climb_rpm']):.0f} rpm / {float(best['payload_on_climb_throttle_percent']):.1f}% |",
                f"| Post delivery | {float(best['post_delivery_mass_kg']):.3f} | {float(best['post_delivery_cruise_power_w']):.1f} W / {float(best['post_delivery_cruise_rpm']):.0f} rpm / {float(best['post_delivery_cruise_throttle_percent']):.1f}% | {float(best['post_delivery_slow_power_w']):.1f} W / {float(best['post_delivery_slow_rpm']):.0f} rpm / {float(best['post_delivery_slow_throttle_percent']):.1f}% | {float(best['post_delivery_approach_power_w']):.1f} W / {float(best['post_delivery_approach_rpm']):.0f} rpm / {float(best['post_delivery_approach_throttle_percent']):.1f}% | {float(best['post_delivery_climb_power_w']):.1f} W / {float(best['post_delivery_climb_rpm']):.0f} rpm / {float(best['post_delivery_climb_throttle_percent']):.1f}% |",
                "",
                "## Main-Wing Stall / Pitch-Up Margin",
                "",
                f"Stage 3 assumes a `{float(best['main_wing_pitch_up_stall_margin_factor']):.2f}x` local-CL pitch-up margin for executive review. This means the aircraft should operate at no more than `{100.0 / float(best['main_wing_pitch_up_stall_margin_factor']):.0f}%` of local CLmax in clean pitch-up review conditions.",
                "",
                "| Case | Cruise pitch-up margin | Climb pitch-up margin |",
                "| --- | ---: | ---: |",
                f"| Payload on | {float(best['payload_on_cruise_pitch_up_stall_margin_percent']):.1f}% | {float(best['payload_on_climb_pitch_up_stall_margin_percent']):.1f}% |",
                f"| Post delivery | {float(best['post_delivery_cruise_pitch_up_stall_margin_percent']):.1f}% | {float(best['post_delivery_climb_pitch_up_stall_margin_percent']):.1f}% |",
                "",
                "## Drag Components",
                "",
                "| Component | Clean cruise drag [N] | Flaps-down slow-flight drag [N] |",
                "| --- | ---: | ---: |",
                f"| Main wing profile | {float(best['cruise_wing_profile_drag_n']):.3f} | {float(best['low_speed_wing_profile_drag_n']):.3f} |",
                f"| Main wing induced, used | {float(best['cruise_wing_induced_drag_n']):.3f} | {float(best['low_speed_wing_induced_drag_n']):.3f} |",
                f"| Horizontal tail | {float(best['cruise_htail_drag_n']):.3f} | {float(best['low_speed_htail_drag_n']):.3f} |",
                f"| Vertical tail | {float(best['cruise_vtail_drag_n']):.3f} | {float(best['low_speed_vtail_drag_n']):.3f} |",
                f"| Fuselage | {float(best['cruise_fuselage_drag_n']):.3f} | {float(best['low_speed_fuselage_drag_n']):.3f} |",
                f"| Added drag required to cancel blown-lift thrust | 0.000 | {float(best['low_speed_added_drag_required_n']):.3f} |",
                f"| Total | {float(best['cruise_drag_n']):.3f} | {float(best['low_speed_drag_n']):.3f} |",
                "",
                "Slow-flight induced drag is the physical split-panel sum: unblown-panel induced drag plus blown-panel induced drag. The freestream-equivalent finite-wing value is reported only as a diagnostic check and is not used in the force balance.",
                "",
                "## Approach Estimate",
                "",
                "| Quantity | Value |",
                "| --- | ---: |",
                f"| Target sink rate | {float(best['approach_target_sink_rate_fps']):.2f} ft/s |",
                f"| Actual sink rate | {float(best['approach_sink_rate_fps']):.2f} ft/s |",
                f"| Sink-rate target met? | {bool(float(best['approach_sink_rate_target_met']))} |",
                f"| Descent angle | {float(best['approach_target_angle_deg']):.2f} deg |",
                f"| Flap deflection | {float(best['approach_flap_deflection_deg']):.1f} deg |",
                f"| Approach alpha | {float(best['approach_alpha_deg']):.2f} deg |",
                f"| Required thrust | {float(best['approach_thrust_required_n']):.3f} N |",
                f"| Electrical power | {float(best['approach_power_w']):.2f} W |",
                f"| RPM | {float(best['approach_rpm']):.0f} rpm |",
            f"| Throttle estimate | {float(best['approach_throttle_percent']):.1f}% |",
            f"| Propulsion feasible at approach | {bool(float(best['approach_rpm_feasible']))} |",
            f"| Elevator trim | {float(best['approach_elevator_trim_deg']):.2f} deg |",
            f"| Rudder trim | {float(best['approach_rudder_trim_deg']):.2f} deg |",
            f"| Max descent rate limit | {float(best['approach_max_descent_rate_fps']):.1f} ft/s |",
                f"| Sink rate | {float(best['approach_sink_rate_mps']):.3f} m/s ({float(best['approach_sink_rate_fps']):.2f} ft/s) |",
            f"| Descent rate limited? | {bool(float(best['approach_descent_rate_limited']))} |",
            f"| Climb target rate | {float(best['climb_target_rate_fps']):.1f} ft/s |",
            f"| Climb speed | {float(best['climb_speed_mps']):.2f} m/s |",
            f"| Climb angle | {float(best['climb_angle_deg']):.2f} deg |",
            f"| Climb CL required | {float(best['climb_cl_required']):.3f} |",
            f"| Climb thrust required | {float(best['climb_thrust_required_n']):.3f} N |",
            f"| Climb throttle estimate | {float(best['climb_throttle_percent']):.1f}% |",
            f"| Climb feasible | {bool(float(best['climb_rpm_feasible']))} |",
            "",
                "## Best-Design Artifacts",
                "",
                f"- Top view: `{best['top_view_png']}`",
                f"- 3D wireframe rendering: `{best['render_3d_png']}`",
                f"- Three view: `{best['three_view_png']}`",
                f"- Wireframe: `{best['wireframe_png']}`",
                f"- Drag and power sweep chart: `{best['drag_power_sweeps_png']}`",
                f"- Drag components chart: `{best['drag_components_png']}`",
                f"- Performance sweep CSV: `{best['performance_sweep_csv']}`",
                f"- Mesh: `{best['mesh_npz']}`",
                "",
                "## Top Designs",
                "",
                "| Rank | Props | Cruise W | Slow W | H-tail span | V-tail height | Mass margin | Static margin |",
                "| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |",
            ]
        )
        for row in successful[:8]:
            lines.append(
                f"| {row['rank']} | {row['n_props']} x {row['prop_diameter_in']} in {row['prop_family']} | "
                f"{float(row['cruise_power_w']):.2f} | {float(row['low_speed_power_w']):.2f} | "
                f"{float(row['htail_span_m']):.3f} | {float(row['vtail_span_m']):.3f} | "
                f"{float(row['mass_budget_margin_kg']):.3f} | {float(row['static_margin_mac']):.3f} |"
            )
        lines.append("")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")


def write_stage3_readable_results(
    results: list[dict[str, Any]],
    output_path: Path,
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> None:
    """Write a concise human-readable Stage 3 result sheet beside the CSVs."""

    successful = [row for row in results if row["status"] == "SUCCESS"]
    lines = [
        "# Stage 3 Results Summary",
        "",
        "This is the readable results sheet for the fixed-propulsion Stage 3 AeroSandbox tail-sizing run. The CSV files remain the source for machine-readable data.",
        "",
    ]

    if not successful:
        lines.extend(
            [
                "## Result",
                "",
                "No successful Stage 3 design was produced. Check `outputs/stage3_aerosandbox_results.csv` for the failure status and warning fields.",
                "",
            ]
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("\n".join(lines), encoding="utf-8")
        return

    row = successful[0]
    warnings = str(row.get("design_warnings", "")).strip()
    warning_text = warnings if warnings else "None"

    lines.extend(
        [
            "## Final Configuration",
            "",
            "| Item | Result |",
            "| --- | ---: |",
            f"| Status | {row['status']} |",
            f"| Warnings | {warning_text} |",
            f"| Propellers / motors | {int(row['n_props'])} |",
            f"| Propeller diameter | {float(row['prop_diameter_in']):.2f} in |",
            f"| Propeller pitch ratio | {float(row['prop_pitch_ratio']):.2f} |",
            f"| Propeller family | {row['prop_family']} |",
            f"| Propulsion model | {row['propulsion_model_source']} |",
            f"| eCalc static CSV | `{row['ecalc_static_csv']}` |",
            f"| Main airfoil | {row['selected_airfoil']} |",
            f"| Tail airfoil | {row['tail_airfoil']} |",
            "",
            "## Main Wing",
            "",
            "| Dimension | Result |",
            "| --- | ---: |",
            f"| Span | {float(row['wing_span_m']):.3f} m |",
            f"| Chord | {float(row['wing_chord_m']):.3f} m |",
            f"| Area | {float(row['wing_area_m2']):.3f} m^2 |",
            f"| Aspect ratio | {float(row['wing_aspect_ratio']):.3f} |",
            f"| Incidence | {float(row['main_wing_incidence_deg']):.3f} deg |",
            f"| Washout | {float(row['main_wing_washout_deg']):.3f} deg |",
            f"| Propeller axial location | {float(row['prop_axial_location_fraction_of_chord']):.3f} chord from wing LE |",
            f"| Propeller x position | {float(row['prop_axial_x_m']):.3f} m |",
            f"| Fuselage nose station | {float(row['fuselage_nose_x_m']):.3f} m from wing LE |",
            f"| Fuselage width | {float(row['fuselage_width_m']) * 1000.0:.0f} mm |",
            f"| Fuselage height | {float(row['fuselage_height_m']) * 1000.0:.0f} mm |",
            f"| Flap span fraction | {float(row['flap_span_fraction']):.3f} semispan |",
            f"| Flap chord fraction | {float(row['flap_chord_fraction']):.3f} chord |",
            f"| Slow-flight flap deflection | {float(row['flap_deflection_slow_deg']):.1f} deg |",
            f"| Aileron span fraction | {float(row['aileron_span_fraction']):.3f} semispan |",
            f"| Aileron chord fraction | {float(row['aileron_chord_fraction']):.3f} chord |",
            "",
            "## Horizontal Stabilizer",
            "",
            "| Dimension | Result |",
            "| --- | ---: |",
            f"| Span | {float(row['htail_span_m']):.3f} m |",
            f"| Root chord | {float(row['htail_root_chord_m']):.3f} m |",
            f"| Tip chord | {float(row['htail_tip_chord_m']):.3f} m |",
            f"| Taper ratio | {float(row['htail_taper']):.3f} |",
            f"| Area | {float(row['htail_area_m2']):.4f} m^2 |",
            f"| Aspect ratio | {float(row['htail_aspect_ratio']):.3f} |",
            f"| Incidence | {float(row['htail_incidence_deg']):.3f} deg |",
            f"| Elevator chord fraction | {float(row['elevator_chord_fraction']):.3f} chord |",
            f"| Elevator max deflection for sizing | {float(row['elevator_max_deflection_deg']):.1f} deg |",
            f"| Slow pitch control target | {float(row['target_slow_pitch_control_cm']):.3f} Cm |",
            f"| Slow pitch control authority | {float(row['slow_pitch_control_cm_authority']):.3f} Cm |",
            f"| Slow pitch control margin | {float(row['slow_pitch_control_margin_percent']):.1f}% |",
            "",
            "## Vertical Stabilizer",
            "",
            "| Dimension | Result |",
            "| --- | ---: |",
            f"| Height/span | {float(row['vtail_span_m']):.3f} m |",
            f"| Root chord | {float(row['vtail_root_chord_m']):.3f} m |",
            f"| Tip chord | {float(row['vtail_tip_chord_m']):.3f} m |",
            f"| Taper ratio | {float(row['vtail_taper']):.3f} |",
            f"| Area | {float(row['vtail_area_m2']):.4f} m^2 |",
            f"| Aspect ratio | {float(row['vtail_aspect_ratio']):.3f} |",
            f"| Incidence | {float(row['vertical_tail_incidence_deg']):.3f} deg |",
            f"| Rudder chord fraction | {float(row['rudder_chord_fraction']):.3f} chord |",
            f"| Rudder max deflection for sizing | {float(row['rudder_max_deflection_deg']):.1f} deg |",
            f"| Slow yaw control target | {float(row['target_slow_yaw_control_cn']):.3f} Cn |",
            f"| Slow yaw control authority | {float(row['slow_yaw_control_cn_authority']):.3f} Cn |",
            f"| Slow yaw control margin | {float(row['slow_yaw_control_margin_percent']):.1f}% |",
            "",
            "## Stability And Mass",
            "",
            "| Metric | Result |",
            "| --- | ---: |",
            f"| Tail arm | {float(row['tail_arm_m']):.3f} m |",
            f"| Horizontal tail volume | {float(row['horizontal_tail_volume']):.3f} |",
            f"| Horizontal tail volume range | {float(row['horizontal_tail_volume_min']):.3f} to {float(row['horizontal_tail_volume_max']):.3f} |",
            f"| Vertical tail volume | {float(row['vertical_tail_volume']):.3f} |",
            f"| Vertical tail volume range | {float(row['vertical_tail_volume_min']):.3f} to {float(row['vertical_tail_volume_max']):.3f} |",
            f"| CG location | {float(row['cg_x_m']):.3f} m |",
            f"| CG as percent MAC | {float(row['cg_percent_mac']):.2f}% |",
            f"| CG target location | {float(row['cg_target_x_m']):.3f} m |",
            f"| CG target as percent MAC | {float(row['cg_target_percent_mac']):.2f}% |",
            f"| CG error | {float(row['cg_error_m']):.6f} m |",
            f"| CG error as percent MAC | {float(row['cg_error_percent_mac']):.4f}% |",
            f"| Required pre-tail baseline CG | {float(row['stage1_baseline_cg_required_percent_mac']):.2f}% MAC |",
            f"| Used pre-tail baseline CG | {float(row['stage1_baseline_cg_used_percent_mac']):.2f}% MAC |",
            f"| Neutral point | {float(row['neutral_point_x_m']):.3f} m |",
            f"| Static margin | {float(row['static_margin_mac']):.3f} MAC |",
            f"| VLM Cm-alpha | {float(row['vlm_cm_alpha_per_deg']):.5f} per deg |",
            f"| NGX250 foam density | {config.foam_density_kgpm3:.1f} kg/m^3 |",
            f"| Main wing foam mass estimate | {float(row['main_wing_foam_mass_kg']):.3f} kg |",
            f"| Horizontal tail foam mass | {float(row['htail_foam_mass_kg']):.3f} kg |",
            f"| Vertical tail foam mass | {float(row['vtail_foam_mass_kg']):.3f} kg |",
            f"| Total tail foam mass | {float(row['total_tail_foam_mass_kg']):.3f} kg |",
            f"| Gross flight mass used for lift/trim | {float(row['gross_flight_mass_kg']):.3f} kg |",
            f"| Component-estimated built mass before ballast/payload | {float(row['stage3_total_built_mass_kg']):.3f} kg |",
            f"| Remaining mass to gross target | {float(row['remaining_mass_to_gross_kg']):.3f} kg |",
            f"| Mass budget margin | {float(row['mass_budget_margin_kg']):.3f} kg |",
            "",
            "The component-estimated built mass is not the flight mass used in aerodynamics. Stage 3 keeps the aircraft gross flight mass locked at the Stage 1/2 value for lift, trim, climb, descent, and power calculations; the component estimate shows how much mass is currently accounted for by the Stage 1 propulsion/battery/system model plus the Stage 3 foam-tail model. The remaining mass is available for payload, structure not yet itemized, fasteners, wiring, covering, ballast, and contingency.",
            "",
            "## Aerodynamic Margins",
            "",
            "| Margin / Source | Result |",
            "| --- | ---: |",
            f"| Slow-flight flap state | Slotted flaps down |",
            f"| CLmax source | `{row['clmax_source']}` |",
            f"| CL curve source | `{row['clmax_curve_source']}` |",
            f"| Stage 1/2 high-lift polar source | `{row['stage12_high_lift_polar_source']}` |",
            f"| Physical CLmax source | `{row['physical_clmax_source']}` |",
            f"| Stage 1/2 clean section CLmax, unblown / blown | {float(row['stage12_clean_section_clmax_unblown']):.3f} / {float(row['stage12_clean_section_clmax_blown']):.3f} |",
            f"| Stage 1/2 flapped section CLmax, unblown / blown | {float(row['stage12_flapped_section_clmax_unblown']):.3f} / {float(row['stage12_flapped_section_clmax_blown']):.3f} |",
            f"| Stage 1/2 weighted blown-panel physical CLmax | {float(row['stage12_weighted_blown_physical_clmax']):.3f} |",
            f"| Raw Stage 1/2 no-flap CLmax | {float(row['raw_no_flap_clmax']):.3f} at alpha {float(row['no_flap_clmax_alpha_deg']):.2f} deg |",
            f"| Raw Stage 1/2 flap-only CLmax | {float(row['raw_flap_only_clmax']):.3f} at alpha {float(row['flap_only_clmax_alpha_deg']):.2f} deg |",
            f"| Raw Stage 1/2 clean blown equivalent CLmax | {float(row['raw_clean_blowing_clmax']):.3f} at alpha {float(row['clean_blowing_clmax_alpha_deg']):.2f} deg |",
            f"| Raw Stage 1/2 flap-down blown equivalent CLmax | {float(row['raw_flap_down_blown_clmax']):.3f} at alpha {float(row['flap_down_blown_clmax_alpha_deg']):.2f} deg |",
            f"| Physical clean CLmax used | {float(row['no_flap_clmax']):.3f} |",
            f"| Physical flaps-down blown-panel CLmax used | {float(row['flap_down_blown_clmax']):.3f} |",
            f"| Stage 1/2 equivalent CLmax capped? | {bool(float(row['clmax_was_capped']))} |",
            f"| Stall margin factor | {float(row['slow_flight_stall_margin_factor']):.2f} |",
            f"| Unblown flap-down CLmax, freestream equivalent | {float(row['slow_flight_unblown_equivalent_clmax']):.3f} |",
            f"| Blown flap-down CLmax, freestream equivalent at solved RPM | {float(row['slow_flight_blown_equivalent_clmax']):.3f} |",
            f"| Blown-panel local physical CLmax | {float(row['slow_flight_blown_local_clmax']):.3f} |",
            f"| Design unblown CLmax after stall margin | {float(row['slow_flight_design_unblown_clmax']):.3f} |",
            f"| Design blown equivalent CLmax after stall margin | {float(row['slow_flight_design_blown_clmax']):.3f} |",
            f"| Design blown-panel local CLmax after stall margin | {float(row['slow_flight_design_blown_local_clmax']):.3f} |",
            f"| Available slow-flight lift | {float(row['slow_flight_lift_available_n']):.3f} N |",
            f"| Required lift target with margin | {float(row['slow_flight_lift_target_n']):.3f} N |",
            f"| Freestream-reference CL required at Stage 3 slow-flight speed | {float(row['slow_flight_freestream_cl_required']):.3f} |",
            f"| Pressure-weighted uniform local CL required at solved slow RPM | {float(row['slow_flight_uniform_local_cl_required']):.3f} |",
            f"| Unblown-panel local CL used | {float(row['slow_flight_unblown_local_cl']):.3f} |",
            f"| Blown-panel local CL used | {float(row['slow_flight_blown_local_cl']):.3f} |",
            f"| Slow-flight dynamic pressure, unblown / blown / effective | {float(row['slow_flight_unblown_dynamic_pressure_pa']):.1f} / {float(row['slow_flight_blown_dynamic_pressure_pa']):.1f} / {float(row['slow_flight_effective_dynamic_pressure_pa']):.1f} Pa |",
            f"| Slow-flight blown-to-unblown dynamic-pressure ratio | {float(row['slow_flight_blown_to_unblown_q_ratio']):.2f} |",
            f"| Slow-flight wing area, unblown / blown | {float(row['slow_flight_unblown_area_m2']):.3f} / {float(row['slow_flight_blown_area_m2']):.3f} m^2 |",
            f"| Local CL margin to design cap | {float(row['slow_flight_local_cl_margin']):.3f} |",
            f"| Required effective velocity margin | {float(row['slow_flight_required_veff_margin_mps']):.3f} m/s |",
            f"| Slow-flight feasible with physical CL caps | {bool(float(row['slow_flight_feasible']))} |",
            f"| Slow-flight lift margin | {float(row['slow_flight_lift_margin_n']):.3f} N |",
            f"| Slow-flight lift margin percent | {float(row['slow_flight_lift_margin_percent']):.1f}% |",
            f"| Equivalent unblown flap-down stall speed with margin | {float(row['slow_flight_equiv_stall_speed_mps']):.3f} m/s |",
            "",
            f"Stage 3 no longer treats the large Stage 1/2 blown-lift equivalent CL values as physical airfoil CLmax. It reads the Stage 1/2 NeuralFoil-derived flap-sweep section polars first, then uses the YAML clean/flapped caps only as missing-file fallbacks. Propwash enters through the blown-panel dynamic pressure, so the local blown-panel CLmax remains physical while the freestream-referenced blown equivalent CLmax increases. The `{config.slow_flight_stall_margin_factor:.2f}` stall factor then requires {(config.slow_flight_stall_margin_factor - 1.0) * 100.0:.0f}% extra local-CL headroom.",
            "",
            "## Performance",
            "",
            "| Metric | Result |",
            "| --- | ---: |",
            f"| Stage 3 slow-flight speed | {_stage3_slow_flight_speed_mps(config):.2f} m/s |",
            f"| Upstream Stage 1/2 low-speed setting | {mission.low_speed_mps:.2f} m/s |",
            f"| Slow-flight freestream CL / local CL / RPM | {float(row['slow_flight_freestream_cl_required']):.3f} / {float(row['slow_flight_uniform_local_cl_required']):.3f} / {float(row['low_speed_rpm']):.0f} rpm |",
            f"| Required blown velocity | {float(row['low_speed_required_veff_mps']):.3f} m/s |",
            f"| Actual blown velocity from solved RPM | {float(row['low_speed_actual_veff_mps']):.3f} m/s |",
            f"| Slow-flight natural drag before drag devices | {float(row['low_speed_natural_drag_n']):.3f} N |",
            f"| Slow-flight wing profile drag | {float(row['low_speed_wing_profile_drag_n']):.3f} N |",
            f"| Slow-flight wing induced drag used | {float(row['low_speed_wing_induced_drag_n']):.3f} N |",
            f"| Slow-flight induced drag, local blown-panel estimate | {float(row['low_speed_wing_induced_local_drag_n']):.3f} N |",
            f"| Slow-flight induced drag, freestream-equivalent check | {float(row['low_speed_wing_induced_freestream_equiv_drag_n']):.3f} N |",
            f"| Slow-flight blown-lift thrust | {float(row['low_speed_blown_lift_thrust_n']):.3f} N |",
            f"| Added drag required for no acceleration | {float(row['low_speed_added_drag_required_n']):.3f} N |",
            f"| Slow-flight steady total drag | {float(row['low_speed_drag_n']):.3f} N |",
            f"| Slow steady drag minus cruise drag | {float(row['low_speed_drag_delta_vs_cruise_n']):.3f} N |",
            f"| Slow-flight fuselage drag increment | {float(row['low_speed_fuselage_drag_n']):.3f} N |",
            f"| Slow-flight electrical power | {float(row['low_speed_power_w']):.2f} W |",
            f"| Slow-flight aircraft electrical losses | {float(row['low_speed_aircraft_power_losses_w']):.2f} W |",
            f"| Slow-flight energy for configured segment | {float(row['low_speed_energy_wh']):.2f} Wh |",
            f"| Slow-flight RPM | {float(row['low_speed_rpm']):.0f} rpm |",
            f"| Slow-flight CT / CP | {float(row['low_speed_ct']):.4f} / {float(row['low_speed_cp']):.4f} |",
            f"| Cruise speed | {mission.cruise_speed_mps:.2f} m/s |",
            f"| Cruise freestream CL / local CL / RPM | {float(row['cruise_cl_required']):.3f} / {float(row['cruise_uniform_local_cl_required']):.3f} / {float(row['cruise_rpm']):.0f} rpm |",
            f"| Cruise blown effective velocity | {float(row['cruise_blown_effective_velocity_mps']):.3f} m/s |",
            f"| Cruise dynamic pressure, unblown / blown / effective | {float(row['cruise_unblown_dynamic_pressure_pa']):.1f} / {float(row['cruise_blown_dynamic_pressure_pa']):.1f} / {float(row['cruise_effective_dynamic_pressure_pa']):.1f} Pa |",
            f"| Cruise blown-to-unblown dynamic-pressure ratio | {float(row['cruise_blown_to_unblown_q_ratio']):.2f} |",
            f"| Cruise drag | {float(row['cruise_drag_n']):.3f} N |",
            f"| Cruise fuselage drag | {float(row['cruise_fuselage_drag_n']):.3f} N |",
            f"| Cruise electrical power | {float(row['cruise_power_w']):.2f} W |",
            f"| Cruise aircraft electrical losses | {float(row['cruise_aircraft_power_losses_w']):.2f} W |",
            f"| Power loss factors, wiring / avionics | {float(row['wiring_power_loss_factor']):.3f} / {float(row['avionics_power_loss_factor']):.3f} |",
            f"| Cruise RPM | {float(row['cruise_rpm']):.0f} rpm |",
            f"| Cruise CT / CP | {float(row['cruise_ct']):.4f} / {float(row['cruise_cp']):.4f} |",
            f"| Cruise body alpha proxy | {float(row['cruise_body_alpha_proxy_deg']):.3f} deg |",
            f"| Cruise wing section alpha proxy | {float(row['cruise_section_alpha_proxy_deg']):.3f} deg |",
            f"| Cruise VLM trim alpha diagnostic | {float(row['cruise_alpha_deg']):.3f} deg |",
            f"| Cruise CL | {float(row['cruise_cl']):.3f} |",
            f"| Cruise CD | {float(row['cruise_cd']):.4f} |",
            f"| Cruise L/D | {float(row['cruise_l_over_d']):.2f} |",
            f"| Trim lift residual | {float(row['trim_residual_lift_n']):.6f} N |",
            f"| Trim Cm residual | {float(row['trim_residual_cm']):.6f} |",
            "",
            "## Flight-Condition Operating Values",
            "",
            "| Condition | Speed [m/s] | RPM | Thrust [N] | Throttle [%] | Blown velocity [m/s] | q unblown/blown/eff [Pa] | CL free/local | Drag [N] | Power [W] |",
            "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
            f"| Cruise, clean | {mission.cruise_speed_mps:.2f} | {float(row['cruise_rpm']):.0f} | {float(row['cruise_thrust_required_n']):.3f} | {float(row['cruise_throttle_percent']):.1f} | {float(row['cruise_blown_effective_velocity_mps']):.2f} | {float(row['cruise_unblown_dynamic_pressure_pa']):.1f}/{float(row['cruise_blown_dynamic_pressure_pa']):.1f}/{float(row['cruise_effective_dynamic_pressure_pa']):.1f} | {float(row['cruise_cl_required']):.3f}/{float(row['cruise_uniform_local_cl_required']):.3f} | {float(row['cruise_drag_n']):.3f} | {float(row['cruise_power_w']):.2f} |",
            f"| Slow flight, flaps down | {_stage3_slow_flight_speed_mps(config):.2f} | {float(row['low_speed_rpm']):.0f} | {float(row['low_speed_thrust_required_n']):.3f} | {float(row['low_speed_throttle_percent']):.1f} | {float(row['low_speed_actual_veff_mps']):.2f} | {float(row['slow_flight_unblown_dynamic_pressure_pa']):.1f}/{float(row['slow_flight_blown_dynamic_pressure_pa']):.1f}/{float(row['slow_flight_effective_dynamic_pressure_pa']):.1f} | {float(row['slow_flight_freestream_cl_required']):.3f}/{float(row['slow_flight_uniform_local_cl_required']):.3f}; split {float(row['slow_flight_unblown_local_cl']):.3f}/{float(row['slow_flight_blown_local_cl']):.3f} | {float(row['low_speed_drag_n']):.3f} | {float(row['low_speed_power_w']):.2f} |",
            f"| Approach/descent, flaps down | {float(row['approach_speed_mps']):.2f} | {float(row['approach_rpm']):.0f} | {float(row['approach_thrust_required_n']):.3f} | {float(row['approach_throttle_percent']):.1f} | {float(row['approach_blown_effective_velocity_mps']):.2f} | {float(row['approach_unblown_dynamic_pressure_pa']):.1f}/{float(row['approach_blown_dynamic_pressure_pa']):.1f}/{float(row['approach_effective_dynamic_pressure_pa']):.1f} | {float(row['approach_cl_required']):.3f}/{float(row['approach_uniform_local_cl_required']):.3f} | {float(row['approach_drag_n']):.3f} | {float(row['approach_power_w']):.2f} |",
            f"| Climb, clean | {float(row['climb_speed_mps']):.2f} | {float(row['climb_rpm']):.0f} | {float(row['climb_thrust_required_n']):.3f} | {float(row['climb_throttle_percent']):.1f} | {float(row['climb_blown_effective_velocity_mps']):.2f} | {float(row['climb_unblown_dynamic_pressure_pa']):.1f}/{float(row['climb_blown_dynamic_pressure_pa']):.1f}/{float(row['climb_effective_dynamic_pressure_pa']):.1f} | {float(row['climb_cl_required']):.3f}/{float(row['climb_uniform_local_cl_required']):.3f} | {float(row['climb_drag_n']):.3f} | {float(row['climb_power_w']):.2f} |",
            "",
            "## Battery Capacity Estimate",
            "",
            "| Segment | Duration [min] | Power [W] | Energy [Wh] |",
            "| --- | ---: | ---: | ---: |",
            f"| Cruise | {float(row['battery_cruise_segment_min']):.1f} | {float(row['cruise_power_w']):.2f} | {float(row['battery_cruise_energy_wh']):.2f} |",
            f"| Slow flight | {float(row['battery_slow_segment_min']):.1f} | {float(row['low_speed_power_w']):.2f} | {float(row['battery_slow_energy_wh']):.2f} |",
            f"| Usable energy subtotal | {float(row['battery_cruise_segment_min']) + float(row['battery_slow_segment_min']):.1f} | -- | {float(row['battery_usable_energy_wh']):.2f} |",
            f"| Pack capacity with {float(row['battery_reserve_fraction']) * 100.0:.0f}% reserve | -- | -- | {float(row['battery_capacity_required_wh']):.2f} |",
            "",
            f"Equivalent 4S LiPo capacity for this conservative sizing case: `{float(row['battery_capacity_required_mah']):.0f} mAh` at `{float(row['battery_pack_voltage_v']):.1f} V` nominal.",
            "",
            f"Estimated battery mass at {mission.battery_specific_energy_wh_per_kg:.0f} Wh/kg: `{float(row['battery_mass_required_kg']):.3f} kg`.",
            "",
            "## Mission Regime Energy Table",
            "",
            *_mission_regime_markdown_lines(row, mission, config),
            "",
            f"Required battery for this regime table: `{float(row['mission_profile_capacity_required_mah']):.0f} mAh` at `{float(row['mission_profile_pack_voltage_v']):.1f} V` nominal (`{int(float(row['mission_profile_battery_cell_count']))}S LiPo`), including `{float(row['mission_profile_reserve_fraction']) * 100.0:.0f}%` reserve. Usable energy before reserve is `{float(row['mission_profile_usable_energy_wh']):.2f} Wh`; reserve-adjusted capacity is `{float(row['mission_profile_capacity_required_wh']):.2f} Wh`.",
            "",
            "Takeoff is estimated from the payload-on climb condition. Landing is estimated from the post-delivery flaps-down slow/flare condition, which avoids using the saturated steep-approach thrust case as the final landing power.",
            "",
            "## Payload-On vs Post-Delivery Performance",
            "",
            f"Payload mass is `{float(row['payload_mass_lb']):.2f} lb` (`{float(row['payload_mass_kg']):.3f} kg`). Battery sizing uses the payload-on case for the whole mission, so a canceled delivery can return home without re-sizing the pack.",
            "",
            "| Configuration | Mass [kg] | Cruise P/RPM/throttle | Slow P/RPM/throttle | Approach P/RPM/throttle | Climb P/RPM/throttle |",
            "| --- | ---: | ---: | ---: | ---: | ---: |",
            f"| Payload on | {float(row['payload_on_mass_kg']):.3f} | {float(row['payload_on_cruise_power_w']):.1f} W / {float(row['payload_on_cruise_rpm']):.0f} rpm / {float(row['payload_on_cruise_throttle_percent']):.1f}% | {float(row['payload_on_slow_power_w']):.1f} W / {float(row['payload_on_slow_rpm']):.0f} rpm / {float(row['payload_on_slow_throttle_percent']):.1f}% | {float(row['payload_on_approach_power_w']):.1f} W / {float(row['payload_on_approach_rpm']):.0f} rpm / {float(row['payload_on_approach_throttle_percent']):.1f}% | {float(row['payload_on_climb_power_w']):.1f} W / {float(row['payload_on_climb_rpm']):.0f} rpm / {float(row['payload_on_climb_throttle_percent']):.1f}% |",
            f"| Post delivery | {float(row['post_delivery_mass_kg']):.3f} | {float(row['post_delivery_cruise_power_w']):.1f} W / {float(row['post_delivery_cruise_rpm']):.0f} rpm / {float(row['post_delivery_cruise_throttle_percent']):.1f}% | {float(row['post_delivery_slow_power_w']):.1f} W / {float(row['post_delivery_slow_rpm']):.0f} rpm / {float(row['post_delivery_slow_throttle_percent']):.1f}% | {float(row['post_delivery_approach_power_w']):.1f} W / {float(row['post_delivery_approach_rpm']):.0f} rpm / {float(row['post_delivery_approach_throttle_percent']):.1f}% | {float(row['post_delivery_climb_power_w']):.1f} W / {float(row['post_delivery_climb_rpm']):.0f} rpm / {float(row['post_delivery_climb_throttle_percent']):.1f}% |",
            "",
            "## Main-Wing Stall / Pitch-Up Margin",
            "",
            f"Stage 3 assumes a `{float(row['main_wing_pitch_up_stall_margin_factor']):.2f}x` local-CL pitch-up margin for executive review. This means the aircraft should operate at no more than `{100.0 / float(row['main_wing_pitch_up_stall_margin_factor']):.0f}%` of local CLmax in clean pitch-up review conditions.",
            "",
            "| Case | Cruise pitch-up margin | Climb pitch-up margin |",
            "| --- | ---: | ---: |",
            f"| Payload on | {float(row['payload_on_cruise_pitch_up_stall_margin_percent']):.1f}% | {float(row['payload_on_climb_pitch_up_stall_margin_percent']):.1f}% |",
            f"| Post delivery | {float(row['post_delivery_cruise_pitch_up_stall_margin_percent']):.1f}% | {float(row['post_delivery_climb_pitch_up_stall_margin_percent']):.1f}% |",
            "",
            "## Reynolds Numbers",
            "",
            "Reynolds numbers are computed as `Re = rho * V * characteristic_chord / mu`. The main wing uses the fixed wing chord; the H-tail and V-tail use their trapezoidal mean aerodynamic chords.",
            "",
            "| Aero surface | Cruise Re | Slow-flight Re | Characteristic chord |",
            "| --- | ---: | ---: | --- |",
            f"| Main wing | {float(row['main_wing_re_cruise']):.0f} | {float(row['main_wing_re_low_speed']):.0f} | main wing chord |",
            f"| Horizontal stabilizer | {float(row['htail_re_cruise']):.0f} | {float(row['htail_re_low_speed']):.0f} | H-tail MAC |",
            f"| Vertical stabilizer | {float(row['vtail_re_cruise']):.0f} | {float(row['vtail_re_low_speed']):.0f} | V-tail MAC |",
            "",
            f"Fuselage drag is included as an equivalent parasite drag area: `D_fuselage = q * CdA`, with `CdA = {config.fuselage_cd_area_m2:.3f} m^2`. This reflects the updated {config.fuselage_width_m * 1000.0:.0f} mm x {config.fuselage_height_m * 1000.0:.0f} mm fuselage cross-section with a modest bluff-body drag allowance. At cruise this is added directly to the drag buildup; at slow flight the same equivalent area is added as an explicit increment on top of the Stage 1 flap-down baseline and Stage 3 tail drag.",
            "",
            "Slow-flight drag is now force-balanced against the blown-lift propeller thrust. The natural airframe drag is still reported, but the headline slow-flight total drag includes the added drag required so the propeller can generate the needed slipstream without accelerating the aircraft. This added drag is the first-pass sizing target for future airbrakes or other drag devices.",
            "",
            "For slow-flight induced drag, Stage 3 now uses the physical split-panel buildup: unblown-panel induced drag plus blown-panel induced drag. The freestream-equivalent finite-wing value is still reported as a diagnostic check, but it is not used in the force balance.",
            "",
            "## Drag Components",
            "",
            "| Component | Clean cruise [N] | Flaps-down slow flight [N] |",
            "| --- | ---: | ---: |",
            f"| Main wing profile | {float(row['cruise_wing_profile_drag_n']):.3f} | {float(row['low_speed_wing_profile_drag_n']):.3f} |",
            f"| Main wing induced, used | {float(row['cruise_wing_induced_drag_n']):.3f} | {float(row['low_speed_wing_induced_drag_n']):.3f} |",
            f"| Horizontal tail | {float(row['cruise_htail_drag_n']):.3f} | {float(row['low_speed_htail_drag_n']):.3f} |",
            f"| Vertical tail | {float(row['cruise_vtail_drag_n']):.3f} | {float(row['low_speed_vtail_drag_n']):.3f} |",
            f"| Fuselage | {float(row['cruise_fuselage_drag_n']):.3f} | {float(row['low_speed_fuselage_drag_n']):.3f} |",
            f"| Added drag / airbrakes | 0.000 | {float(row['low_speed_added_drag_required_n']):.3f} |",
            f"| Total | {float(row['cruise_drag_n']):.3f} | {float(row['low_speed_drag_n']):.3f} |",
            "",
            "| Slow-flight induced-drag diagnostic | Drag [N] |",
            "| --- | ---: |",
            f"| Local blown-panel estimate | {float(row['low_speed_wing_induced_local_drag_n']):.3f} |",
            f"| Freestream-equivalent finite-wing check | {float(row['low_speed_wing_induced_freestream_equiv_drag_n']):.3f} |",
            f"| Value used in force balance | {float(row['low_speed_wing_induced_drag_n']):.3f} |",
            "",
            f"Drag-components chart: `{row['drag_components_png']}`",
            "",
            "## Approach Estimate",
            "",
            "The approach model assumes steady flaps-down descent at the target sink rate in the YAML file, clipped by the configured maximum descent rate and by the selected approach airspeed. The climb model checks the configured maximum climb rate in clean configuration.",
            "",
            "| Quantity | Result |",
            "| --- | ---: |",
            f"| Target approach sink rate | {float(row['approach_target_sink_rate_fps']):.2f} ft/s |",
            f"| Actual approach sink rate | {float(row['approach_sink_rate_fps']):.2f} ft/s |",
            f"| Sink-rate target met? | {bool(float(row['approach_sink_rate_target_met']))} |",
            f"| Resulting approach angle | {float(row['approach_target_angle_deg']):.2f} deg below horizontal |",
            f"| Approach speed | {float(row['approach_speed_mps']):.2f} m/s |",
            f"| Flap deflection | {float(row['approach_flap_deflection_deg']):.1f} deg |",
            f"| Approach alpha | {float(row['approach_alpha_deg']):.2f} deg |",
            f"| Required CL | {float(row['approach_cl_required']):.3f} |",
            f"| Required CL / RPM | {float(row['approach_cl_required']):.3f} / {float(row['approach_rpm']):.0f} rpm |",
            f"| Approach drag | {float(row['approach_drag_n']):.3f} N |",
            f"| Required thrust | {float(row['approach_thrust_required_n']):.3f} N |",
            f"| Electrical power | {float(row['approach_power_w']):.2f} W |",
            f"| RPM | {float(row['approach_rpm']):.0f} rpm |",
            f"| Throttle estimate | {float(row['approach_throttle_percent']):.1f}% |",
            f"| Propulsion feasible at this approach angle | {bool(float(row['approach_rpm_feasible']))} |",
            f"| Elevator trim | {float(row['approach_elevator_trim_deg']):.2f} deg |",
            f"| Rudder trim | {float(row['approach_rudder_trim_deg']):.2f} deg |",
            f"| Max descent rate limit | {float(row['approach_max_descent_rate_fps']):.1f} ft/s |",
            f"| Sink rate | {float(row['approach_sink_rate_mps']):.3f} m/s ({float(row['approach_sink_rate_fps']):.2f} ft/s) |",
            f"| Descent rate limited? | {bool(float(row['approach_descent_rate_limited']))} |",
            f"| Glide-ratio equivalent | {float(row['approach_glide_ratio']):.2f}:1 |",
            "",
            "## Climb Trim Estimate",
            "",
            "| Quantity | Result |",
            "| --- | ---: |",
            f"| Target climb rate | {float(row['climb_target_rate_fps']):.1f} ft/s |",
            f"| Climb speed | {float(row['climb_speed_mps']):.2f} m/s |",
            f"| Climb angle | {float(row['climb_angle_deg']):.2f} deg |",
            f"| Required CL | {float(row['climb_cl_required']):.3f} |",
            f"| Required CL / RPM | {float(row['climb_cl_required']):.3f} / {float(row['climb_rpm']):.0f} rpm |",
            f"| Climb drag | {float(row['climb_drag_n']):.3f} N |",
            f"| Required thrust | {float(row['climb_thrust_required_n']):.3f} N |",
            f"| Electrical power | {float(row['climb_power_w']):.2f} W |",
            f"| RPM | {float(row['climb_rpm']):.0f} rpm |",
            f"| Throttle estimate | {float(row['climb_throttle_percent']):.1f}% |",
            f"| Elevator trim | {float(row['climb_elevator_trim_deg']):.2f} deg |",
            f"| Propulsion feasible at target climb rate | {bool(float(row['climb_rpm_feasible']))} |",
            "",
            "## Sweep Charts",
            "",
            f"- Drag and power chart: `{row['drag_power_sweeps_png']}`",
            f"- Sweep data CSV: `{row['performance_sweep_csv']}`",
            "",
            "## Output Files",
            "",
            f"- Editable Stage 3 constraints: `{STAGE3_CONSTRAINTS_YAML}`",
            f"- Full CSV results: `outputs/stage3_aerosandbox_results.csv`",
            f"- Top-design CSV: `outputs/stage3_aerosandbox_top_designs.csv`",
            f"- Technical report: `{row['report_md']}`",
            f"- Top view: `{row['top_view_png']}`",
            f"- Three view: `{row['three_view_png']}`",
            f"- Wireframe: `{row['wireframe_png']}`",
            f"- 3D wireframe render: `{row['render_3d_png']}`",
            f"- Section polar plot: `{row['polar_png']}`",
            f"- Drag and power sweep chart: `{row['drag_power_sweeps_png']}`",
            f"- Drag components chart: `{row['drag_components_png']}`",
            f"- Performance sweep CSV: `{row['performance_sweep_csv']}`",
            f"- Mesh file: `{row['mesh_npz']}`",
            "",
        ]
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")


def write_stage3_engineering_tex(
    results: list[dict[str, Any]],
    output_path: Path,
    mission: Stage1MissionConfig,
    config: Stage3SizingConfig,
) -> None:
    """Write a LaTeX engineering report for the fixed-propulsion Stage 3 design."""

    successful = [row for row in results if row["status"] == "SUCCESS"]
    if not successful:
        lines = [
            r"\documentclass[11pt]{article}",
            r"\usepackage[margin=1in]{geometry}",
            r"\usepackage{booktabs}",
            r"\usepackage{siunitx}",
            r"\usepackage[hidelinks]{hyperref}",
            r"\title{Stage 3 Fixed-Wing Tail Refinement}",
            r"\author{AA146 Capstone Optimizer Notes}",
            r"\date{\today}",
            r"\begin{document}",
            r"\maketitle",
            r"No successful Stage 3 result was available when this file was generated.",
            r"\end{document}",
            "",
        ]
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("\n".join(lines), encoding="utf-8")
        return

    row = successful[0]
    warnings = str(row.get("design_warnings", "")).strip() or "None"
    top_view = _tex_path(row["top_view_png"])
    three_view = _tex_path(row["three_view_png"])
    render_3d = _tex_path(row["render_3d_png"])
    polar_plot = _tex_path(row["polar_png"])
    drag_power_sweeps = _tex_path(row["drag_power_sweeps_png"])
    drag_components_plot = _tex_path(row["drag_components_png"])
    stage3_mass = _tex_float(row, "stage3_total_built_mass_kg")
    tail_mass = _tex_float(row, "total_tail_foam_mass_kg")
    cruise_power = _tex_float(row, "cruise_power_w")
    slow_power = _tex_float(row, "low_speed_power_w")
    cruise_drag = _tex_float(row, "cruise_drag_n")
    slow_drag = _tex_float(row, "low_speed_drag_n")
    q_cruise = 0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2
    q_low = 0.5 * mission.air_density_kgpm3 * _stage3_slow_flight_speed_mps(config)**2

    lines = [
        r"\documentclass[11pt]{article}",
        "",
        r"\usepackage[margin=1in]{geometry}",
        r"\usepackage{amsmath, amssymb, mathtools}",
        r"\usepackage{booktabs}",
        r"\usepackage{array}",
        r"\usepackage{longtable}",
        r"\usepackage{tabularx}",
        r"\usepackage{graphicx}",
        r"\usepackage{float}",
        r"\usepackage{siunitx}",
        r"\usepackage{enumitem}",
        r"\usepackage[hidelinks]{hyperref}",
        "",
        r"\sisetup{per-mode=symbol, group-separator={,}}",
        r"\setlength\LTleft{0pt}",
        "",
        r"\title{Stage 3 Fixed-Wing Tail Refinement and AeroSandbox Verification}",
        r"\author{AA146 Capstone Optimizer Notes}",
        r"\date{\today}",
        "",
        r"\begin{document}",
        r"\maketitle",
        "",
        r"\begin{abstract}",
        (
            "This document records the current Stage~3 fixed-wing tail refinement for the "
            "AA146 distributed-propulsion aircraft. Stage~3 freezes the Stage~1/2 propulsion, "
            "main-wing, airfoil, flap, and aileron selections. It sizes the stabilizer planform "
            "and tail arm for cruise trim, cruise drag, mass, static margin, and tail-volume "
            "constraints, then sizes the elevator and rudder chord fractions for slow-flight "
            "control authority. The present recommended configuration uses "
            f"{int(_tex_float(row, 'n_props'))} propellers of diameter "
            f"\\SI{{{_tex_float(row, 'prop_diameter_in'):.2f}}}{{in}}, the "
            f"{_tex_escape(row['selected_airfoil'])} main airfoil, a horizontal tail span of "
            f"\\SI{{{_tex_float(row, 'htail_span_m'):.3f}}}{{m}}, and a vertical tail height of "
            f"\\SI{{{_tex_float(row, 'vtail_span_m'):.3f}}}{{m}}. The aerodynamics remain tied "
            f"to the locked gross flight mass of \\SI{{{_tex_float(row, 'gross_flight_mass_kg'):.3f}}}{{kg}}; "
            f"the current component-estimated mass before ballast or payload is "
            f"\\SI{{{stage3_mass:.3f}}}{{kg}}, with \\SI{{{tail_mass:.3f}}}{{kg}} of added "
            f"tail foam mass, cruise electrical power of \\SI{{{cruise_power:.1f}}}{{W}}, and "
            f"low-speed electrical power of \\SI{{{slow_power:.1f}}}{{W}}."
        ),
        r"\end{abstract}",
        "",
        r"\section{Executive Summary}",
        (
            "Tables~\\ref{tab:exec-configuration}, \\ref{tab:exec-performance}, and "
            "\\ref{tab:exec-margins} summarize the aircraft as it should be reviewed at the "
            "configuration level. Stage~1/2 selections are listed explicitly because Stage~3 "
            "treats them as frozen inputs; Stage~3 only refines the fixed-wing tail, trim, "
            "drag buildup, and reporting around that selected aircraft."
        ),
        "",
        r"\begin{longtable}{@{}p{0.29\linewidth}p{0.30\linewidth}p{0.34\linewidth}@{}}",
        r"\caption{Executive aircraft configuration summary.}\label{tab:exec-configuration}\\",
        r"\toprule",
        r"Item & Current value & Source or design note \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"Item & Current value & Source or design note \\",
        r"\midrule",
        r"\endhead",
        rf"Gross flight mass & \SI{{{_tex_float(row, 'gross_flight_mass_kg'):.3f}}}{{kg}} & Locked aircraft mass used for lift, trim, climb, descent, and power calculations. \\",
        rf"Maximum mass requirement & \SI{{{mission.max_mass_kg:.3f}}}{{kg}} & Stage~1 mission-level requirement. \\",
        rf"Payload mass & \SI{{{_tex_float(row, 'payload_mass_lb'):.2f}}}{{lb}} / \SI{{{_tex_float(row, 'payload_mass_kg'):.3f}}}{{kg}} & Delivery payload carried before drop-off. \\",
        rf"Post-delivery flight mass & \SI{{{_tex_float(row, 'post_delivery_mass_kg'):.3f}}}{{kg}} & Used for after-delivery performance reporting only. \\",
        rf"Stage~1/2 component-estimated mass & \SI{{{_tex_float(row, 'stage1_total_built_mass_kg'):.3f}}}{{kg}} & Frozen upstream propulsion, battery, and baseline system mass estimate. \\",
        rf"Stage~3 component-estimated mass & \SI{{{_tex_float(row, 'stage3_total_built_mass_kg'):.3f}}}{{kg}} & Stage~1/2 estimate plus the Stage~3 NGX250 foam tail estimate. \\",
        rf"Remaining mass to gross target & \SI{{{_tex_float(row, 'remaining_mass_to_gross_kg'):.3f}}}{{kg}} & Payload, unmodeled structure, wiring, fasteners, covering, ballast, and contingency. \\",
        rf"Main airfoil & {_tex_escape(row['selected_airfoil'])} & Frozen Stage~1/2 wing workflow selection. \\",
        rf"Tail airfoil & {_tex_escape(row['tail_airfoil']).upper()} & Fixed NACA~0008 for both stabilizers. \\",
        rf"Main wing span, chord, area & \SI{{{_tex_float(row, 'wing_span_m'):.3f}}}{{m}}, \SI{{{_tex_float(row, 'wing_chord_m'):.3f}}}{{m}}, \SI{{{_tex_float(row, 'wing_area_m2'):.3f}}}{{m^2}} & Frozen Stage~1/2 rectangular main wing. \\",
        rf"Main wing aspect ratio & {_tex_float(row, 'wing_aspect_ratio'):.3f} & Derived from Stage~1/2 wing dimensions. \\",
        rf"Main wing incidence & \SI{{{_tex_float(row, 'main_wing_incidence_deg'):.3f}}}{{deg}} & Optimized by Stage~3 for cruise body-alpha compatibility. \\",
        rf"Main wing washout & \SI{{{_tex_float(row, 'main_wing_washout_deg'):.3f}}}{{deg}} & Editable Stage~3 constraint; currently held fixed. \\",
        rf"Propulsion layout & {int(_tex_float(row, 'n_props'))} \(\times\) \SI{{{_tex_float(row, 'prop_diameter_in'):.2f}}}{{in}}, \(P/D={_tex_float(row, 'prop_pitch_ratio'):.2f}\), {_tex_escape(row['prop_family'])} & Frozen Stage~1/2 motor/propeller selection. \\",
        rf"Stage~1/2 seed RPM, slow/cruise & \num{{{_tex_float(row, 'seed_low_speed_rpm'):.0f}}} / \num{{{_tex_float(row, 'seed_cruise_rpm'):.0f}}} & Upstream RPMs retained for traceability; Stage~3 recomputes RPM using the active eCalc data. \\",
        rf"Stage~3 RPM, slow/cruise & \num{{{_tex_float(row, 'low_speed_rpm'):.0f}}} / \num{{{_tex_float(row, 'cruise_rpm'):.0f}}} & Final propeller operating points from the active Stage~3 propulsion model. \\",
        rf"Power loss factors, wiring/avionics & {_tex_float(row, 'wiring_power_loss_factor'):.3f} / {_tex_float(row, 'avionics_power_loss_factor'):.3f} & Applied after eCalc/generic motor power to include battery harness, connector, distribution, BEC/regulator, and avionics wiring losses. \\",
        rf"Propeller axial station & \SI{{{_tex_float(row, 'prop_axial_x_m'):.4f}}}{{m}} & Relative to wing leading edge; negative is forward of the leading edge. \\",
        rf"Fuselage nose and cross-section & \(x_n=\SI{{{_tex_float(row, 'fuselage_nose_x_m'):.3f}}}{{m}}\), \SI{{{_tex_float(row, 'fuselage_width_m') * 1000.0:.0f}}}{{mm}} \(\times\) \SI{{{_tex_float(row, 'fuselage_height_m') * 1000.0:.0f}}}{{mm}} & Nose is 300~mm ahead of the wing leading edge; cross-section from the current packaging assumption. \\",
        rf"Flap span/chord/deflection & {_tex_float(row, 'flap_span_fraction'):.3f} semispan, {_tex_float(row, 'flap_chord_fraction'):.3f} chord, \SI{{{_tex_float(row, 'flap_deflection_slow_deg'):.1f}}}{{deg}} & Frozen Stage~1/2 slotted-flap sizing. \\",
        rf"Aileron span/chord & {_tex_float(row, 'aileron_span_fraction'):.3f} semispan, {_tex_float(row, 'aileron_chord_fraction'):.3f} chord & Frozen Stage~1/2 aileron sizing. \\",
        rf"Horizontal tail span and area & \SI{{{_tex_float(row, 'htail_span_m'):.3f}}}{{m}}, \SI{{{_tex_float(row, 'htail_area_m2'):.3f}}}{{m^2}} & Stage~3 tail-volume and cruise-trim sizing. \\",
        rf"Horizontal tail root/tip chord & \SI{{{_tex_float(row, 'htail_root_chord_m'):.3f}}}{{m}} / \SI{{{_tex_float(row, 'htail_tip_chord_m'):.3f}}}{{m}} & Includes taper as an optimized/exported dimension. \\",
        rf"Horizontal tail incidence & \SI{{{_tex_float(row, 'htail_incidence_deg'):.3f}}}{{deg}} & Cruise trim setting. \\",
        rf"Elevator chord fraction & {_tex_float(row, 'elevator_chord_fraction'):.3f} & Sized for slow-flight pitch authority after stabilizer sizing. \\",
        rf"Vertical tail height and area & \SI{{{_tex_float(row, 'vtail_span_m'):.3f}}}{{m}}, \SI{{{_tex_float(row, 'vtail_area_m2'):.3f}}}{{m^2}} & Stage~3 directional-stability and control sizing. \\",
        rf"Vertical tail root/tip chord & \SI{{{_tex_float(row, 'vtail_root_chord_m'):.3f}}}{{m}} / \SI{{{_tex_float(row, 'vtail_tip_chord_m'):.3f}}}{{m}} & Includes taper as an optimized/exported dimension. \\",
        rf"Vertical tail incidence & \SI{{{_tex_float(row, 'vertical_tail_incidence_deg'):.3f}}}{{deg}} & Fixed yaw/incidence setting. \\",
        rf"Rudder chord fraction & {_tex_float(row, 'rudder_chord_fraction'):.3f} & Sized for slow-flight yaw authority after stabilizer sizing. \\",
        rf"Tail arm & \SI{{{_tex_float(row, 'tail_arm_m'):.3f}}}{{m}} & Wing aerodynamic center to tail aerodynamic center. \\",
        rf"CG location & \SI{{{_tex_float(row, 'cg_x_m'):.4f}}}{{m}} / {_tex_float(row, 'cg_percent_mac'):.2f}\% MAC & Enforced quarter-chord final CG target. \\",
        r"\bottomrule",
        r"\end{longtable}",
        "",
        r"\begin{longtable}{@{}p{0.25\linewidth}p{0.17\linewidth}p{0.18\linewidth}p{0.18\linewidth}p{0.14\linewidth}@{}}",
        r"\caption{Executive system-performance summary by flight condition.}\label{tab:exec-performance}\\",
        r"\toprule",
        r"Metric & Cruise & Slow flight & Approach/descent & Climb \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"Metric & Cruise & Slow flight & Approach/descent & Climb \\",
        r"\midrule",
        r"\endhead",
        rf"Configuration & Clean & Flaps down & Flaps down & Clean \\",
        rf"Freestream speed & \SI{{{mission.cruise_speed_mps:.2f}}}{{m/s}} & \SI{{{_stage3_slow_flight_speed_mps(config):.2f}}}{{m/s}} & \SI{{{_tex_float(row, 'approach_speed_mps'):.2f}}}{{m/s}} & \SI{{{_tex_float(row, 'climb_speed_mps'):.2f}}}{{m/s}} \\",
        rf"Vertical rate target & -- & -- & \SI{{{_tex_float(row, 'approach_target_sink_rate_fps'):.2f}}}{{ft/s}} sink & \SI{{{_tex_float(row, 'climb_target_rate_fps'):.2f}}}{{ft/s}} climb \\",
        rf"Actual vertical rate & -- & -- & \SI{{{_tex_float(row, 'approach_sink_rate_fps'):.2f}}}{{ft/s}} sink & \SI{{{_tex_float(row, 'climb_target_rate_mps') / 0.3048:.2f}}}{{ft/s}} climb \\",
        rf"Flight-path angle & Level & Level & \SI{{{_tex_float(row, 'approach_target_angle_deg'):.2f}}}{{deg}} down & \SI{{{_tex_float(row, 'climb_angle_deg'):.2f}}}{{deg}} up \\",
        rf"Free-stream / local \(C_L\) & {_tex_float(row, 'cruise_cl_required'):.3f} / {_tex_float(row, 'cruise_uniform_local_cl_required'):.3f} & {_tex_float(row, 'slow_flight_freestream_cl_required'):.3f} / {_tex_float(row, 'slow_flight_uniform_local_cl_required'):.3f} & {_tex_float(row, 'approach_cl_required'):.3f} / {_tex_float(row, 'approach_uniform_local_cl_required'):.3f} & {_tex_float(row, 'climb_cl_required'):.3f} / {_tex_float(row, 'climb_uniform_local_cl_required'):.3f} \\",
        rf"Blown wing velocity & \SI{{{_tex_float(row, 'cruise_blown_effective_velocity_mps'):.2f}}}{{m/s}} & \SI{{{_tex_float(row, 'low_speed_actual_veff_mps'):.2f}}}{{m/s}} & \SI{{{_tex_float(row, 'approach_blown_effective_velocity_mps'):.2f}}}{{m/s}} & \SI{{{_tex_float(row, 'climb_blown_effective_velocity_mps'):.2f}}}{{m/s}} \\",
        rf"Effective dynamic pressure & \SI{{{_tex_float(row, 'cruise_effective_dynamic_pressure_pa'):.1f}}}{{Pa}} & \SI{{{_tex_float(row, 'slow_flight_effective_dynamic_pressure_pa'):.1f}}}{{Pa}} & \SI{{{_tex_float(row, 'approach_effective_dynamic_pressure_pa'):.1f}}}{{Pa}} & \SI{{{_tex_float(row, 'climb_effective_dynamic_pressure_pa'):.1f}}}{{Pa}} \\",
        rf"Total drag & \SI{{{_tex_float(row, 'cruise_drag_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'low_speed_drag_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'approach_drag_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'climb_drag_n'):.3f}}}{{N}} \\",
        rf"Required thrust & \SI{{{_tex_float(row, 'cruise_thrust_required_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'low_speed_thrust_required_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'approach_thrust_required_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'climb_thrust_required_n'):.3f}}}{{N}} \\",
        rf"Throttle estimate & {_tex_float(row, 'cruise_throttle_percent'):.1f}\% & {_tex_float(row, 'low_speed_throttle_percent'):.1f}\% & {_tex_float(row, 'approach_throttle_percent'):.1f}\% & {_tex_float(row, 'climb_throttle_percent'):.1f}\% \\",
        rf"Motor/prop RPM & \num{{{_tex_float(row, 'cruise_rpm'):.0f}}} & \num{{{_tex_float(row, 'low_speed_rpm'):.0f}}} & \num{{{_tex_float(row, 'approach_rpm'):.0f}}} & \num{{{_tex_float(row, 'climb_rpm'):.0f}}} \\",
        rf"Electrical power & \SI{{{_tex_float(row, 'cruise_power_w'):.2f}}}{{W}} & \SI{{{_tex_float(row, 'low_speed_power_w'):.2f}}}{{W}} & \SI{{{_tex_float(row, 'approach_power_w'):.2f}}}{{W}} & \SI{{{_tex_float(row, 'climb_power_w'):.2f}}}{{W}} \\",
        rf"Aircraft electrical losses & \SI{{{_tex_float(row, 'cruise_aircraft_power_losses_w'):.2f}}}{{W}} & \SI{{{_tex_float(row, 'low_speed_aircraft_power_losses_w'):.2f}}}{{W}} & Included in power & Included in power \\",
        rf"Energy budgeted & \SI{{{_tex_float(row, 'battery_cruise_energy_wh'):.2f}}}{{Wh}} & \SI{{{_tex_float(row, 'battery_slow_energy_wh'):.2f}}}{{Wh}} & Not in pack sizing & Not in pack sizing \\",
        rf"Trim state & Body \(\alpha\) \SI{{{_tex_float(row, 'cruise_body_alpha_proxy_deg'):.2f}}}{{deg}}; \(i_h\) \SI{{{_tex_float(row, 'htail_incidence_deg'):.2f}}}{{deg}} & \(i_h\) \SI{{{_tex_float(row, 'htail_incidence_deg'):.2f}}}{{deg}}; flaps \SI{{{_tex_float(row, 'flap_deflection_slow_deg'):.0f}}}{{deg}} & Elevator \SI{{{_tex_float(row, 'approach_elevator_trim_deg'):.2f}}}{{deg}}; rudder \SI{{{_tex_float(row, 'approach_rudder_trim_deg'):.2f}}}{{deg}} & Elevator \SI{{{_tex_float(row, 'climb_elevator_trim_deg'):.2f}}}{{deg}} \\",
        r"\bottomrule",
        r"\end{longtable}",
        "",
        r"\begin{longtable}{@{}p{0.18\linewidth}p{0.12\linewidth}p{0.18\linewidth}p{0.18\linewidth}p{0.18\linewidth}p{0.12\linewidth}@{}}",
        r"\caption{Payload-on and post-delivery performance comparison. Battery capacity is sized from the payload-on case.}\label{tab:exec-payload-performance}\\",
        r"\toprule",
        r"Case & Mass & Cruise power/RPM/throttle & Slow power/RPM/throttle & Climb power/RPM/throttle & Cruise/Climb pitch margin \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"Case & Mass & Cruise power/RPM/throttle & Slow power/RPM/throttle & Climb power/RPM/throttle & Cruise/Climb pitch margin \\",
        r"\midrule",
        r"\endhead",
        rf"Payload on & \SI{{{_tex_float(row, 'payload_on_mass_kg'):.3f}}}{{kg}} & \SI{{{_tex_float(row, 'payload_on_cruise_power_w'):.1f}}}{{W}} / \num{{{_tex_float(row, 'payload_on_cruise_rpm'):.0f}}} / {_tex_float(row, 'payload_on_cruise_throttle_percent'):.1f}\% & \SI{{{_tex_float(row, 'payload_on_slow_power_w'):.1f}}}{{W}} / \num{{{_tex_float(row, 'payload_on_slow_rpm'):.0f}}} / {_tex_float(row, 'payload_on_slow_throttle_percent'):.1f}\% & \SI{{{_tex_float(row, 'payload_on_climb_power_w'):.1f}}}{{W}} / \num{{{_tex_float(row, 'payload_on_climb_rpm'):.0f}}} / {_tex_float(row, 'payload_on_climb_throttle_percent'):.1f}\% & {_tex_float(row, 'payload_on_cruise_pitch_up_stall_margin_percent'):.0f}\% / {_tex_float(row, 'payload_on_climb_pitch_up_stall_margin_percent'):.0f}\% \\",
        rf"Post delivery & \SI{{{_tex_float(row, 'post_delivery_mass_kg'):.3f}}}{{kg}} & \SI{{{_tex_float(row, 'post_delivery_cruise_power_w'):.1f}}}{{W}} / \num{{{_tex_float(row, 'post_delivery_cruise_rpm'):.0f}}} / {_tex_float(row, 'post_delivery_cruise_throttle_percent'):.1f}\% & \SI{{{_tex_float(row, 'post_delivery_slow_power_w'):.1f}}}{{W}} / \num{{{_tex_float(row, 'post_delivery_slow_rpm'):.0f}}} / {_tex_float(row, 'post_delivery_slow_throttle_percent'):.1f}\% & \SI{{{_tex_float(row, 'post_delivery_climb_power_w'):.1f}}}{{W}} / \num{{{_tex_float(row, 'post_delivery_climb_rpm'):.0f}}} / {_tex_float(row, 'post_delivery_climb_throttle_percent'):.1f}\% & {_tex_float(row, 'post_delivery_cruise_pitch_up_stall_margin_percent'):.0f}\% / {_tex_float(row, 'post_delivery_climb_pitch_up_stall_margin_percent'):.0f}\% \\",
        r"\bottomrule",
        r"\end{longtable}",
        "",
        r"\begin{longtable}{@{}p{0.31\linewidth}p{0.26\linewidth}p{0.35\linewidth}@{}}",
        r"\caption{Executive margins and PDR review notes.}\label{tab:exec-margins}\\",
        r"\toprule",
        r"Review item & Current value & Note \\",
        r"\midrule",
        r"\endfirsthead",
        r"\toprule",
        r"Review item & Current value & Note \\",
        r"\midrule",
        r"\endhead",
        rf"Static margin & {_tex_float(row, 'static_margin_mac'):.3f} MAC & Required range {config.min_static_margin_mac:.3f}--{config.max_static_margin_mac:.3f} MAC; target {config.target_static_margin_mac:.3f} MAC. \\",
        rf"CG error from quarter chord & \SI{{{_tex_float(row, 'cg_error_m'):.6f}}}{{m}} & Positive means aft of target; current target is {_tex_float(row, 'cg_target_percent_mac'):.2f}\% MAC. \\",
        rf"Horizontal tail volume & {_tex_float(row, 'horizontal_tail_volume'):.3f} & Constraint range {_tex_float(row, 'horizontal_tail_volume_min'):.3f}--{_tex_float(row, 'horizontal_tail_volume_max'):.3f}. \\",
        rf"Vertical tail volume & {_tex_float(row, 'vertical_tail_volume'):.3f} & Constraint range {_tex_float(row, 'vertical_tail_volume_min'):.3f}--{_tex_float(row, 'vertical_tail_volume_max'):.3f}. \\",
        rf"Slow-flight lift margin & {_tex_float(row, 'slow_flight_lift_margin_percent'):.1f}\% & Margin after the {_tex_float(row, 'slow_flight_stall_margin_factor'):.2f} stall-margin factor is applied. \\",
        rf"Slow blown-panel local \(C_L\) margin & {_tex_float(row, 'slow_flight_local_cl_margin'):.3f} & Positive means the blown panel is below its design local \(C_{{L,\max}}\). \\",
        rf"Slow required blown-velocity margin & \SI{{{_tex_float(row, 'slow_flight_required_veff_margin_mps'):.3f}}}{{m/s}} & Actual minus required slipstream velocity at the slow-flight sizing point. \\",
        rf"Main-wing pitch-up stall margin rule & {_tex_float(row, 'main_wing_pitch_up_stall_margin_factor'):.2f}x & Review assumption: operate at no more than \(1/{_tex_float(row, 'main_wing_pitch_up_stall_margin_factor'):.2f}\) of local \(C_{{L,\max}}\) for clean emergency pitch-up margin. \\",
        rf"Payload-on clean pitch margins & {_tex_float(row, 'payload_on_cruise_pitch_up_stall_margin_percent'):.1f}\% cruise / {_tex_float(row, 'payload_on_climb_pitch_up_stall_margin_percent'):.1f}\% climb & Positive margin indicates available local \(C_L\) headroom beyond the review rule. \\",
        rf"Post-delivery clean pitch margins & {_tex_float(row, 'post_delivery_cruise_pitch_up_stall_margin_percent'):.1f}\% cruise / {_tex_float(row, 'post_delivery_climb_pitch_up_stall_margin_percent'):.1f}\% climb & Lower weight increases clean-wing pitch-up headroom. \\",
        rf"Added drag target for slow flight & \SI{{{_tex_float(row, 'low_speed_added_drag_required_n'):.3f}}}{{N}} & First-pass airbrake/spoiler drag requirement so blown-lift thrust does not accelerate the aircraft. \\",
        rf"Pitch control margin & {_tex_float(row, 'slow_pitch_control_margin_percent'):.1f}\% & Elevator authority margin at slow flight. \\",
        rf"Yaw control margin & {_tex_float(row, 'slow_yaw_control_margin_percent'):.1f}\% & Rudder authority margin at slow flight. \\",
        rf"Approach RPM feasible flag & {int(_tex_float(row, 'approach_rpm_feasible'))} & 1 means the requested thrust is inside the active RPM bracket. \\",
        rf"Climb RPM feasible flag & {int(_tex_float(row, 'climb_rpm_feasible'))} & 1 means the requested thrust is inside the active RPM bracket. \\",
        rf"Battery capacity requirement & \SI{{{_tex_float(row, 'battery_capacity_required_wh'):.2f}}}{{Wh}} / \num{{{_tex_float(row, 'battery_capacity_required_mah'):.0f}}}~mAh & 4S LiPo equivalent including {_tex_float(row, 'battery_reserve_fraction') * 100.0:.0f}\% reserve for \SI{{{_tex_float(row, 'battery_cruise_segment_min'):.1f}}}{{min}} cruise plus \SI{{{_tex_float(row, 'battery_slow_segment_min'):.1f}}}{{min}} slow flight. \\",
        rf"Battery mass estimate & \SI{{{_tex_float(row, 'battery_mass_required_kg'):.3f}}}{{kg}} & Uses Stage~1 pack specific energy of \SI{{{mission.battery_specific_energy_wh_per_kg:.0f}}}{{Wh/kg}}. \\",
        rf"Design warnings & \texttt{{{_tex_escape(warnings)}}} & Generated warning flags from the Stage~3 result row. \\",
        r"\bottomrule",
        r"\end{longtable}",
        "",
        r"\section*{Nomenclature}",
        r"\begin{longtable}{@{}ll@{}}",
        r"\toprule",
        r"Symbol & Meaning \\",
        r"\midrule",
        r"\(b,c,S\) & main-wing span, chord, and reference area \\",
        r"\(S_h,S_v\) & horizontal and vertical tail planform areas \\",
        r"\(b_h,b_v\) & horizontal tail span and vertical tail height \\",
        r"\(l_t\) & tail arm from wing aerodynamic center to tail aerodynamic center \\",
        r"\(V_h,V_v\) & horizontal and vertical tail volume coefficients \\",
        r"\(x_{cg},x_{np}\) & longitudinal center of gravity and neutral point \\",
        r"\(SM\) & static margin, \((x_{np}-x_{cg})/c\) \\",
        r"\(C_{L,\max}\) & freestream-reference maximum lift coefficient \\",
        r"\(C_{dA,f}\) & equivalent fuselage drag area \\",
        r"\(q=\frac{1}{2}\rho V^2\) & dynamic pressure \\",
        r"\bottomrule",
        r"\end{longtable}",
        "",
        r"\section{Scope and Frozen Inputs}",
        (
            "Stage~3 is not a re-optimization of the whole aircraft. The main wing and propulsion "
            "architecture are inherited from the Stage~1/2 workflow, and the propeller count is "
            "held fixed. The active stabilizer-planform design variables are the horizontal tail area, horizontal tail "
            "aspect ratio, horizontal tail taper ratio, vertical tail area, vertical tail aspect "
            "ratio, vertical tail taper ratio, and tail arm. Elevator and rudder chord fractions are not cruise-optimization variables; they are sized after planform selection from slow-speed pitch and yaw authority targets. The run is generated by "
            r"\texttt{optimizer/stages/stage3\_aerosandbox.py}; editable Stage~3 constraints are "
            f"read from \\texttt{{{_tex_escape(STAGE3_CONSTRAINTS_YAML)}}}."
        ),
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{llc}",
        r"\toprule",
        r"Quantity & Source & Value \\",
        r"\midrule",
        rf"Gross flight mass & mission config & \SI{{{mission.gross_mass_kg:.3f}}}{{kg}} \\",
        rf"Maximum built mass & mission config & \SI{{{mission.max_mass_kg:.3f}}}{{kg}} \\",
        rf"Main wing span \(b\) & Stage 1/2 fixed & \SI{{{mission.span_m:.3f}}}{{m}} \\",
        rf"Main wing chord \(c\) & Stage 1/2 fixed & \SI{{{mission.chord_m:.3f}}}{{m}} \\",
        rf"Main wing area \(S\) & derived & \SI{{{mission.wing_area_m2:.3f}}}{{m^2}} \\",
        rf"Main airfoil & wing workflow & {_tex_escape(row['selected_airfoil'])} \\",
        rf"Tail airfoil & Stage 3 config & {_tex_escape(row['tail_airfoil'])} \\",
        rf"Propeller layout & Stage 1/2 fixed & {int(_tex_float(row, 'n_props'))} \(\times\) \SI{{{_tex_float(row, 'prop_diameter_in'):.2f}}}{{in}}, \(P/D={_tex_float(row, 'prop_pitch_ratio'):.2f}\) \\",
        rf"Propulsion model & Stage 3/eCalc & {_tex_escape(row['propulsion_model_source'])} \\",
        rf"Power loss factors & Stage 3 config & wiring {_tex_float(row, 'wiring_power_loss_factor'):.3f}; avionics {_tex_float(row, 'avionics_power_loss_factor'):.3f} \\",
        rf"Fuselage nose station & Stage 3 config & \SI{{{_tex_float(row, 'fuselage_nose_x_m'):.3f}}}{{m}} from wing leading edge \\",
        rf"Stage 3 slow-flight condition & Stage 3 config & \SI{{{_stage3_slow_flight_speed_mps(config):.2f}}}{{m/s}} \\",
        rf"Upstream Stage 1/2 low-speed condition & mission config & \SI{{{mission.low_speed_mps:.2f}}}{{m/s}} \\",
        rf"Cruise condition & mission config & \SI{{{mission.cruise_speed_mps:.2f}}}{{m/s}} \\",
        rf"Fuselage cross-section & Stage 3 config & \SI{{{_tex_float(row, 'fuselage_width_m') * 1000.0:.0f}}}{{mm}} \(\times\) \SI{{{_tex_float(row, 'fuselage_height_m') * 1000.0:.0f}}}{{mm}} \\",
        rf"NGX250 foam density & material input & \SI{{{config.foam_density_kgpm3:.1f}}}{{kg/m^3}} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Frozen aircraft-level inputs and material assumptions used by Stage~3.}",
        r"\label{tab:stage3-fixed-inputs}",
        r"\end{table}",
        "",
        r"\section{How to Run and Reproduce This Stage}",
        (
            "Stage~3 is intended to be run from the repository root after Stage~1 and Stage~2 "
            "have produced their CSV outputs. The command is"
        ),
        r"\begin{verbatim}",
        r"python3 -m optimizer.stages.stage3_aerosandbox",
        r"\end{verbatim}",
        (
            "A successful run writes the queue CSV, the full results CSV, the top-design CSV, "
            "the readable Markdown summary, this LaTeX engineering report, and all geometry and "
            "performance figures. The recommended post-run validation command is"
        ),
        r"\begin{verbatim}",
        r"python3 -m optimizer.stages.validate_outputs",
        r"\end{verbatim}",
        (
            "The editable user-facing inputs are stored in "
            f"\\texttt{{{_tex_escape(STAGE3_CONSTRAINTS_YAML)}}}. Values in that YAML file are "
            "loaded into the Stage3SizingConfig dataclass. Unknown keys are ignored, so comments "
            "and temporary notes can be added without breaking the run."
        ),
        "",
        r"\subsection{Data flow}",
        r"\begin{enumerate}[leftmargin=*]",
        (
            r"\item Stage~3 reads \texttt{outputs/stage2\_prop\_span\_report.csv} and filters it "
            r"to the frozen propulsion layout selected by the Stage~1/2 wing workflow."
        ),
        (
            r"\item It reads \texttt{outputs/stage1\_pareto\_front.csv} to recover the matching "
            r"mass, drag, propeller, battery, and RPM context for that frozen layout."
        ),
        (
            r"\item It reads the wing workflow summary to recover the selected airfoil, flap "
            r"geometry, aileron geometry, propeller spanwise positions, and the Stage~1/2 "
            r"\(C_{L,\max}\) curves."
        ),
        (
            r"\item If enabled in the YAML constraints and the frozen propulsion layout matches, "
            r"it reads the eCalc static thrust/power table and dynamic design-point \(C_T/C_P\) "
            r"CSV files to replace the generic propeller surrogate used in early screening."
        ),
        (
            r"\item After the propulsion model returns motor/ESC electric power, Stage~3 applies "
            r"the editable wiring and avionics loss factors from the YAML file. The reported "
            r"condition powers and battery-capacity estimate therefore include harness, connector, "
            r"distribution, BEC/regulator, and avionics wiring allowances."
        ),
        (
            r"\item It optimizes only the tail planform and tail arm. The main wing, propeller "
            r"count, propeller size, propeller pitch ratio, flap sizing, aileron sizing, and "
            r"mass target stay fixed."
        ),
        (
            r"\item It sizes the elevator and rudder chord fractions after the stabilizer "
            r"planform is selected, because those moving surfaces are judged mainly by "
            r"slow-flight control authority rather than cruise drag."
        ),
        (
            r"\item It renders geometry figures, writes sweep plots, writes drag-component "
            r"plots, and exports the final design rows to the Stage~3 CSV outputs."
        ),
        r"\end{enumerate}",
        "",
        r"\subsection{Primary output files}",
        r"\begin{itemize}[leftmargin=*]",
        r"\item \texttt{outputs/stage3\_readable\_results.md}: concise engineering summary for humans.",
        r"\item \texttt{outputs/stage3\_aerosandbox\_results.csv}: full machine-readable results.",
        r"\item \texttt{outputs/stage3\_aerosandbox\_top\_designs.csv}: first ranked designs.",
        r"\item \texttt{outputs/stage3\_visuals/*drag\_power\_sweeps.png}: drag and power curves.",
        r"\item \texttt{outputs/stage3\_visuals/*drag\_components.png}: drag-buildup plot.",
        r"\item \texttt{outputs/stage3\_visuals/*performance\_sweep.csv}: underlying sweep data.",
        r"\item \texttt{outputs/stage3\_visuals/*mesh.npz}: AeroSandbox mesh points and faces.",
        r"\end{itemize}",
        "",
        r"\section{Modeling Method}",
        r"\subsection{Tail geometry parameterization}",
        (
            "Both stabilizers are modeled as tapered trapezoids. The optimizer chooses area, "
            "aspect ratio, taper ratio, and tail arm inside the user-specified tail-volume ranges. The span and root/tip chords are then "
            "computed from"
        ),
        r"\begin{align}",
        r"b_h &= \sqrt{S_h AR_h}, & c_{r,h} &= \frac{2S_h}{b_h(1+\lambda_h)}, & c_{t,h} &= \lambda_h c_{r,h}, \\",
        r"b_v &= \sqrt{S_v AR_v}, & c_{r,v} &= \frac{2S_v}{b_v(1+\lambda_v)}, & c_{t,v} &= \lambda_v c_{r,v}.",
        r"\end{align}",
        (
            "The horizontal and vertical tail volume coefficients are evaluated as"
        ),
        r"\begin{equation}",
        r"V_h = \frac{S_h l_t}{S c}, \qquad V_v = \frac{S_v l_t}{S b}.",
        r"\end{equation}",
        (
            f"For this run, the requested horizontal tail-volume range is "
            f"{_tex_float(row, 'horizontal_tail_volume_min'):.3f} to {_tex_float(row, 'horizontal_tail_volume_max'):.3f}, "
            f"and the requested vertical tail-volume range is "
            f"{_tex_float(row, 'vertical_tail_volume_min'):.3f} to {_tex_float(row, 'vertical_tail_volume_max'):.3f}."
        ),
        "",
        r"\subsection{Optimization objective and penalties}",
        (
            "The optimizer uses a scalar score rather than a hard single-objective aerodynamic "
            "minimum. This is deliberate: a tail with very low drag but poor static margin, "
            "unreasonable incidence, excessive mass, or inadequate slow-flight lift margin is not "
            "a useful aircraft design. The score has normalized cruise-power, slow-flight-power, "
            "tail-mass, and static-margin terms:"
        ),
        r"\begin{equation}",
        r"J = w_c \frac{P_c}{P_{c,ref}} + w_s \frac{P_s}{P_{s,ref}} + w_m \frac{m_t}{m_{max}} + w_{SM}\left(\frac{SM-SM_{target}}{0.08}\right)^2 + J_{penalty}.",
        r"\end{equation}",
        (
            f"The current weights are \\(w_c={config.cruise_weight:.2f}\\), "
            f"\\(w_s={config.slow_flight_weight:.2f}\\), \\(w_m={config.material_mass_weight:.2f}\\), "
            f"and \\(w_{{SM}}={config.stability_weight:.2f}\\). Cruise power is weighted most heavily "
            "because the stabilizer planform is supposed to be cruise-oriented. Slow-flight "
            "control authority is not ignored; it is handled by the post-optimization elevator "
            "and rudder sizing step."
        ),
        (
            "Penalty terms are applied when a candidate falls outside the configured tail-volume "
            "ranges, static-margin range, mass budget, lift-margin requirement, trim-incidence "
            "limit, root-chord bounds, or propeller RPM feasibility. This keeps the optimizer from "
            "using an attractive but non-buildable corner of the design space."
        ),
        "",
        r"\subsection{Slow-flight elevator and rudder sizing}",
        (
            "After the cruise-oriented stabilizer planform is selected, Stage~3 sizes the elevator "
            "and rudder chord fractions against slow-flight control-authority targets. The authority "
            "model uses finite-wing lift slope, tail volume, maximum deflection, slow-flight tail "
            "dynamic-pressure ratio, and a bounded control-surface effectiveness model:"
        ),
        r"\begin{align}",
        r"C_{m,\delta_e}^{\max} &\approx \eta_{t,\mathrm{slow}} a_h V_h \tau_e \delta_{e,\max}, \\",
        r"C_{n,\delta_r}^{\max} &\approx \eta_{t,\mathrm{slow}} a_v V_v \tau_r \delta_{r,\max}.",
        r"\end{align}",
        (
            "This keeps the stabilizer dimensions focused on cruise performance while still ensuring "
            "that the moving surfaces retain low-speed control authority."
        ),
        "",
        r"\subsection{Foam mass accounting}",
        (
            "The NGX250 foam mass is included during the tail sizing loop. For a trapezoidal foam "
            r"surface with airfoil-area coefficient \(k_A\), span \(b_s\), root chord \(c_r\), and "
            r"tip chord \(c_t\), Stage~3 uses"
        ),
        r"\begin{equation}",
        r"m_{\mathrm{foam}} = \rho_{\mathrm{foam}} k_A b_s \frac{c_r^2 + c_r c_t + c_t^2}{3}.",
        r"\end{equation}",
        (
            "The tail foam mass is added to the selected Stage~1/2 built-mass estimate before "
            "checking the mass margin and recomputing the longitudinal center of gravity."
        ),
        "",
        r"\subsection{Trim, static margin, and drag buildup}",
        (
            "Cruise trim is solved with AeroSandbox VortexLatticeMethod. A proxy neutral point is "
            "also evaluated during optimization using finite-wing lift slopes, tail dynamic-pressure "
            "ratio, downwash gradient, and tail arm. The static margin constraint is"
        ),
        r"\begin{equation}",
        r"SM = \frac{x_{np}-x_{cg}}{c}.",
        r"\end{equation}",
        (
            "Cruise drag is assembled from main-wing profile and induced drag, horizontal-tail trim "
            "drag, vertical-tail profile drag, and an equivalent fuselage parasite increment:"
        ),
        r"\begin{equation}",
        r"D_{\mathrm{cruise}} = qS(C_{D,p}+C_L^2/\pi e AR) + qS_h C_{D,h} + qS_v C_{D,v} + q C_{dA,f}.",
        r"\end{equation}",
        rf"For this run, \(C_{{dA,f}}=\SI{{{config.fuselage_cd_area_m2:.3f}}}{{m^2}}\), \(q_{{\mathrm{{low}}}}=\SI{{{q_low:.3f}}}{{Pa}}\), and \(q_{{\mathrm{{cruise}}}}=\SI{{{q_cruise:.3f}}}{{Pa}}\).",
        "",
        r"\subsection{Slow-flight high-lift treatment}",
        (
            "Slow flight is always evaluated with slotted flaps deployed. Stage~3 no longer invents "
            r"a large physical \(C_{L,\max}\) for this mode. It reads the selected Stage~1/2 "
            r"equivalent high-lift outputs for traceability, and it reads the Stage~1/2 "
            r"NeuralFoil flap-sweep section polars for the physical local clean and flapped "
            r"\(C_{L,\max}\) limits. The YAML CLmax values are only missing-file fallbacks. "
            "Propwash increases local dynamic pressure rather than airfoil CLmax. The loaded sources are"
        ),
        r"\begin{itemize}[leftmargin=*]",
        rf"\item \texttt{{{_tex_escape(row['clmax_source'])}}}",
        rf"\item \texttt{{{_tex_escape(row['clmax_curve_source'])}}}",
        rf"\item \texttt{{{_tex_escape(row['physical_clmax_source'])}}}",
        r"\end{itemize}",
        (
            rf"The Stage~1/2 section-polar values are \(C_{{L,\max,clean}}\)="
            rf"{_tex_float(row, 'stage12_clean_section_clmax_unblown'):.3f}/"
            rf"{_tex_float(row, 'stage12_clean_section_clmax_blown'):.3f} "
            rf"(unblown/blown) and \(C_{{L,\max,flap}}\)="
            rf"{_tex_float(row, 'stage12_flapped_section_clmax_unblown'):.3f}/"
            rf"{_tex_float(row, 'stage12_flapped_section_clmax_blown'):.3f}. "
            rf"The area-weighted physical blown-panel limit used by Stage~3 is "
            rf"{_tex_float(row, 'stage12_weighted_blown_physical_clmax'):.3f}."
        ),
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lcc}",
        r"\toprule",
        r"Condition & Raw equivalent \(C_L\) & Physical cap used \\",
        r"\midrule",
        rf"No flap & {_tex_float(row, 'raw_no_flap_clmax'):.3f} & {_tex_float(row, 'no_flap_clmax'):.3f} \\",
        rf"Flap only & {_tex_float(row, 'raw_flap_only_clmax'):.3f} & {_tex_float(row, 'flap_only_clmax'):.3f} \\",
        rf"Clean blown & {_tex_float(row, 'raw_clean_blowing_clmax'):.3f} & {_tex_float(row, 'clean_blowing_clmax'):.3f} \\",
        rf"Flap-down blown & {_tex_float(row, 'raw_flap_down_blown_clmax'):.3f} & {_tex_float(row, 'flap_down_blown_clmax'):.3f} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Raw Stage~1/2 equivalent high-lift coefficients and the physical local CLmax caps used by Stage~3.}",
        r"\label{tab:stage3-clmax}",
        r"\end{table}",
        (
            f"The slow-flight sizing coefficients are divided by the stall margin factor "
            f"{config.slow_flight_stall_margin_factor:.2f}. Thus the design flap-only coefficient is "
            f"{_tex_float(row, 'slow_flight_design_unblown_clmax'):.3f}. The local physical "
            f"blown-panel design coefficient is {_tex_float(row, 'slow_flight_design_blown_local_clmax'):.3f}, "
            f"while the freestream-referenced blown equivalent design coefficient is "
            f"{_tex_float(row, 'slow_flight_design_blown_clmax'):.3f}. The current slow-flight "
            f"blown-panel local CL is {_tex_float(row, 'slow_flight_blown_local_cl'):.3f}."
        ),
        "",
        r"\subsection{Slow-flight force balance and added drag}",
        (
            "The blown-lift calculation can require more propeller thrust than the natural aircraft "
            "drag at the low-speed condition. If that excess thrust is not cancelled, the aircraft "
            "would accelerate and would no longer remain at the intended slow-flight speed. Stage~3 "
            "therefore reports the drag that must be added by future drag devices."
        ),
        r"\begin{align}",
        r"D_{\mathrm{natural,slow}} &= D_{\mathrm{Stage1/2,flap}} + D_h + D_v + D_f, \\",
        r"T_{\mathrm{blown}} &= \max(T_{D,\mathrm{slow}}, T_{V_{\mathrm{eff}}}), \\",
        r"\Delta D_{\mathrm{add}} &= \max(0, T_{\mathrm{blown}} - D_{\mathrm{natural,slow}}), \\",
        r"D_{\mathrm{steady,slow}} &= D_{\mathrm{natural,slow}} + \Delta D_{\mathrm{add}}.",
        r"\end{align}",
        (
            f"For this design, the natural slow-flight drag is "
            f"\\SI{{{_tex_float(row, 'low_speed_natural_drag_n'):.3f}}}{{N}}, the propeller thrust "
            f"needed by the blown-lift condition is \\SI{{{_tex_float(row, 'low_speed_blown_lift_thrust_n'):.3f}}}{{N}}, "
            f"and the added-drag target is \\SI{{{_tex_float(row, 'low_speed_added_drag_required_n'):.3f}}}{{N}}. "
            "This number should be treated as the first sizing target for airbrakes, spoilerons, "
            "deployable plates, or other high-drag devices."
        ),
        "",
        r"\subsection{Performance sweeps}",
        (
            "The drag and power sweep figure is not part of the optimizer objective; it is a "
            "diagnostic output for the final ranked design. The clean velocity sweep solves the "
            "angle of attack needed to support aircraft weight and omits points outside the clean "
            "polar lift range. The flaps-down sweeps use the same split blown/unblown component "
            "model as the Stage~3 slow-flight sizing calculation, with profile drag, induced drag, "
            "tail drag, fuselage drag, and propeller power evaluated from that model."
        ),
        (
            "The power curves no longer plot saturated max-RPM points as valid requirements. "
            "If the requested trim, drag balance, or blown-lift slipstream condition cannot be "
            "met within the configured RPM range, the point is left blank in the plot and marked "
            "with zero feasibility in the sweep CSV."
        ),
        "",
        r"\section{Assumption Justification}",
        (
            "The assumptions below are intentionally exposed in the YAML file when they are likely "
            "to change during design iteration. The current values are first-pass engineering "
            "choices, not immutable truth."
        ),
        r"\begin{longtable}{@{}p{0.24\linewidth}p{0.18\linewidth}p{0.49\linewidth}@{}}",
        r"\toprule",
        r"Assumption & Current value & Justification \\",
        r"\midrule",
        rf"Tail airfoil & {_tex_escape(row['tail_airfoil'])} & Fixed for both stabilizers. A symmetric 8\% section gives near-zero airfoil pitching moment, lower profile thickness than NACA~0012, and enough foam thickness for a small tail. \\",
        rf"Foam density & \SI{{{config.foam_density_kgpm3:.1f}}}{{kg/m^3}} & NGX250 pink foam is the selected construction material, so tail volume is converted directly into added mass. \\",
        rf"Horizontal tail-volume range & {_tex_float(row, 'horizontal_tail_volume_min'):.3f}--{_tex_float(row, 'horizontal_tail_volume_max'):.3f} & Conventional preliminary sizing range for a small stable aircraft, widened enough for the high-lift/slow-flight requirement. \\",
        rf"Vertical tail-volume range & {_tex_float(row, 'vertical_tail_volume_min'):.3f}--{_tex_float(row, 'vertical_tail_volume_max'):.3f} & Keeps the fin large enough for yaw control without allowing unnecessary cruise drag and foam mass. \\",
        rf"Static-margin target & {config.target_static_margin_mac:.3f} MAC & A moderate stability target. The lower bound avoids neutral/unstable pitch behavior; the upper bound avoids an overly nose-heavy aircraft with excess trim drag. \\",
        rf"Cruise tail dynamic-pressure ratio & {config.tail_dynamic_pressure_ratio:.2f} & The tail is assumed to see slightly less than freestream dynamic pressure due to wing wake, fuselage interference, and downwash. \\",
        rf"Slow tail dynamic-pressure ratio & {config.slow_tail_dynamic_pressure_ratio:.2f} & Slow-flight control sizing is made more conservative because flap wake, high angle of attack, and low Reynolds number can reduce tail effectiveness. \\",
        rf"Downwash gradient & {config.downwash_gradient:.2f} & A first-pass low-aspect-ratio wing estimate for how much wing angle of attack becomes downwash at the tail. \\",
        rf"Fuselage drag area & \SI{{{config.fuselage_cd_area_m2:.3f}}}{{m^2}} & Equivalent drag area for the \SI{{{config.fuselage_width_m * 1000.0:.0f}}}{{mm}} by \SI{{{config.fuselage_height_m * 1000.0:.0f}}}{{mm}} fuselage cross-section with a modest bluff-body/protuberance allowance. \\",
        rf"Tail zero-lift drag coefficient & {config.tail_zero_lift_cd:.3f} & Conservative small-tail, low-Reynolds-number profile drag estimate including practical surface roughness and hinge gaps. \\",
        rf"Stall margin factor & {config.slow_flight_stall_margin_factor:.2f} & Divides inherited Stage~1/2 \(C_{{L,\max}}\) by 1.20 so slow-flight sizing keeps 20\% lift headroom for gusts, manufacturing errors, surface waviness, and blown-lift uncertainty. \\",
        rf"Minimum slow-lift warning margin & {config.min_slow_flight_lift_margin:.2f} & Adds a warning if less than 10\% lift margin remains after the stall margin has already been applied. \\",
        rf"Elevator/rudder max deflection & \SI{{{config.elevator_max_deflection_deg:.0f}}}{{deg}} / \SI{{{config.rudder_max_deflection_deg:.0f}}}{{deg}} & Avoids assuming extreme deflections where hinge moments, separation, and servo loads become questionable. \\",
        rf"Control effectiveness exponent & {config.control_surface_effectiveness_exponent:.2f} & Represents diminishing returns as control-surface chord fraction grows; authority is not assumed to scale linearly forever. \\",
        rf"Wiring power loss factor & {config.wiring_power_loss_factor:.3f} & Multiplies propulsion electric power after the eCalc/generic motor model to cover battery harness, connector, and distribution losses that are not part of the aerodynamic thrust requirement. \\",
        rf"Avionics power loss factor & {config.avionics_power_loss_factor:.3f} & Multiplies the fixed avionics load to include BEC/regulator inefficiency and wiring loss in the avionics bus. \\",
        rf"Approach target & \SI{{{config.approach_target_sink_rate_fps:.1f}}}{{ft/s}}, \SI{{{config.approach_speed_mps:.1f}}}{{m/s}} & Editable first-pass descent condition. If the target sink rate exceeds what the selected approach speed can physically provide, Stage~3 clips it and reports that the target was not met. \\",
        rf"Battery sizing segments & \SI{{{config.battery_cruise_segment_min:.1f}}}{{min}} cruise, \SI{{{config.battery_slow_segment_min:.1f}}}{{min}} slow & Converts the final Stage~3 operating powers into a pack-capacity requirement. The reserve fraction and specific energy are inherited from the Stage~1 mission assumptions so the battery estimate remains tied to the same aircraft-level mass model. \\",
        r"\bottomrule",
        r"\end{longtable}",
        "",
        r"\section{Recommended Stage 3 Geometry}",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lcc}",
        r"\toprule",
        r"Quantity & Symbol & Value \\",
        r"\midrule",
        rf"Main wing span & \(b\) & \SI{{{_tex_float(row, 'wing_span_m'):.3f}}}{{m}} \\",
        rf"Main wing chord & \(c\) & \SI{{{_tex_float(row, 'wing_chord_m'):.3f}}}{{m}} \\",
        rf"Main wing incidence & \(i_w\) & \SI{{{_tex_float(row, 'main_wing_incidence_deg'):.3f}}}{{deg}} \\",
        rf"Cruise body alpha proxy & \(\alpha_b\) & \SI{{{_tex_float(row, 'cruise_body_alpha_proxy_deg'):.3f}}}{{deg}} \\",
        rf"Cruise section alpha proxy & \(\alpha_\mathrm{{sec}}\) & \SI{{{_tex_float(row, 'cruise_section_alpha_proxy_deg'):.3f}}}{{deg}} \\",
        rf"Propeller axial position & \(x_p\) & \SI{{{_tex_float(row, 'prop_axial_x_m'):.4f}}}{{m}} \\",
        rf"Fuselage nose station & \(x_n\) & \SI{{{_tex_float(row, 'fuselage_nose_x_m'):.4f}}}{{m}} \\",
        rf"Horizontal tail span & \(b_h\) & \SI{{{_tex_float(row, 'htail_span_m'):.3f}}}{{m}} \\",
        rf"Horizontal tail root chord & \(c_{{r,h}}\) & \SI{{{_tex_float(row, 'htail_root_chord_m'):.3f}}}{{m}} \\",
        rf"Horizontal tail tip chord & \(c_{{t,h}}\) & \SI{{{_tex_float(row, 'htail_tip_chord_m'):.3f}}}{{m}} \\",
        rf"Horizontal tail taper & \(\lambda_h\) & {_tex_float(row, 'htail_taper'):.3f} \\",
        rf"Horizontal tail incidence & \(i_h\) & \SI{{{_tex_float(row, 'htail_incidence_deg'):.3f}}}{{deg}} \\",
        rf"Elevator chord fraction & \(c_e/c_h\) & {_tex_float(row, 'elevator_chord_fraction'):.3f} \\",
        rf"Vertical tail height & \(b_v\) & \SI{{{_tex_float(row, 'vtail_span_m'):.3f}}}{{m}} \\",
        rf"Vertical tail root chord & \(c_{{r,v}}\) & \SI{{{_tex_float(row, 'vtail_root_chord_m'):.3f}}}{{m}} \\",
        rf"Vertical tail tip chord & \(c_{{t,v}}\) & \SI{{{_tex_float(row, 'vtail_tip_chord_m'):.3f}}}{{m}} \\",
        rf"Vertical tail taper & \(\lambda_v\) & {_tex_float(row, 'vtail_taper'):.3f} \\",
        rf"Vertical tail incidence & \(i_v\) & \SI{{{_tex_float(row, 'vertical_tail_incidence_deg'):.3f}}}{{deg}} \\",
        rf"Rudder chord fraction & \(c_r/c_v\) & {_tex_float(row, 'rudder_chord_fraction'):.3f} \\",
        rf"Tail arm & \(l_t\) & \SI{{{_tex_float(row, 'tail_arm_m'):.3f}}}{{m}} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Final Stage~3 aircraft dimensions.}",
        r"\label{tab:stage3-dimensions}",
        r"\end{table}",
        "",
        r"\begin{figure}[H]",
        r"\centering",
        rf"\includegraphics[width=0.72\linewidth]{{{top_view}}}",
        r"\caption{Stage~3 top-view fixed-wing tail layout. The dashed circles show the actual frozen Stage~2 propeller count and spanwise positions; the axial position is read from the Stage~3 YAML constraint file rather than hard-coded in the renderer.}",
        r"\label{fig:stage3-top-view}",
        r"\end{figure}",
        "",
        r"\begin{figure}[H]",
        r"\centering",
        rf"\includegraphics[width=0.92\linewidth]{{{three_view}}}",
        r"\caption{AeroSandbox-generated three-view rendering of the final Stage~3 geometry.}",
        r"\label{fig:stage3-three-view}",
        r"\end{figure}",
        "",
        r"\begin{figure}[H]",
        r"\centering",
        rf"\includegraphics[width=0.82\linewidth]{{{render_3d}}}",
        r"\caption{Three-dimensional wireframe rendering used as a geometry sanity check for the final Stage~3 aircraft.}",
        r"\label{fig:stage3-3d}",
        r"\end{figure}",
        "",
        r"\section{Mass, Stability, and Performance Results}",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lcc}",
        r"\toprule",
        r"Metric & Value & Comment \\",
        r"\midrule",
        rf"Stage~1/2 built mass & \SI{{{_tex_float(row, 'stage1_total_built_mass_kg'):.3f}}}{{kg}} & inherited baseline \\",
        rf"Horizontal tail foam mass & \SI{{{_tex_float(row, 'htail_foam_mass_kg'):.3f}}}{{kg}} & NGX250, geometry-derived \\",
        rf"Vertical tail foam mass & \SI{{{_tex_float(row, 'vtail_foam_mass_kg'):.3f}}}{{kg}} & NGX250, geometry-derived \\",
        rf"Total tail foam mass & \SI{{{tail_mass:.3f}}}{{kg}} & added during sizing \\",
        rf"Gross flight mass used in aerodynamics & \SI{{{_tex_float(row, 'gross_flight_mass_kg'):.3f}}}{{kg}} & locked Stage~1/2 flight mass \\",
        rf"Component-estimated mass before ballast/payload & \SI{{{stage3_mass:.3f}}}{{kg}} & baseline plus tail foam \\",
        rf"Remaining mass to gross target & \SI{{{_tex_float(row, 'remaining_mass_to_gross_kg'):.3f}}}{{kg}} & payload, unmodeled structure, wiring, fasteners, ballast, contingency \\",
        rf"Mass margin & \SI{{{_tex_float(row, 'mass_budget_margin_kg'):.3f}}}{{kg}} & relative to \SI{{{mission.max_mass_kg:.1f}}}{{kg}} limit \\",
        rf"CG location & \SI{{{_tex_float(row, 'cg_x_m'):.3f}}}{{m}} & from wing leading edge \\",
        rf"CG fraction MAC & {_tex_float(row, 'cg_percent_mac'):.2f}\% & based on rectangular MAC \\",
        rf"CG target & {_tex_float(row, 'cg_target_percent_mac'):.2f}\% MAC & quarter-chord requirement \\",
        rf"CG error & \SI{{{_tex_float(row, 'cg_error_m'):.6f}}}{{m}} & actual minus target \\",
        rf"Required pre-tail baseline CG & {_tex_float(row, 'stage1_baseline_cg_required_percent_mac'):.2f}\% MAC & mass placement before tail foam \\",
        rf"Neutral point & \SI{{{_tex_float(row, 'neutral_point_x_m'):.3f}}}{{m}} & proxy stability model \\",
        rf"Static margin & {_tex_float(row, 'static_margin_mac'):.3f} MAC & target {_tex_float(row, 'static_margin_mac'):.3f} MAC achieved \\",
        rf"Horizontal tail volume & {_tex_float(row, 'horizontal_tail_volume'):.3f} & range {_tex_float(row, 'horizontal_tail_volume_min'):.3f}--{_tex_float(row, 'horizontal_tail_volume_max'):.3f} \\",
        rf"Vertical tail volume & {_tex_float(row, 'vertical_tail_volume'):.3f} & range {_tex_float(row, 'vertical_tail_volume_min'):.3f}--{_tex_float(row, 'vertical_tail_volume_max'):.3f} \\",
        rf"VLM \(C_{{m,\alpha}}\) & {_tex_float(row, 'vlm_cm_alpha_per_deg'):.5f}/deg & negative is statically stable \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Mass and static-stability closure for the selected Stage~3 geometry.}",
        r"\label{tab:stage3-mass-stability}",
        r"\end{table}",
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lccc}",
        r"\toprule",
        r"Control axis & Target & Available & Margin \\",
        r"\midrule",
        rf"Pitch/elevator & \(C_m={_tex_float(row, 'target_slow_pitch_control_cm'):.3f}\) & \(C_m={_tex_float(row, 'slow_pitch_control_cm_authority'):.3f}\) & {_tex_float(row, 'slow_pitch_control_margin_percent'):.1f}\% \\",
        rf"Yaw/rudder & \(C_n={_tex_float(row, 'target_slow_yaw_control_cn'):.3f}\) & \(C_n={_tex_float(row, 'slow_yaw_control_cn_authority'):.3f}\) & {_tex_float(row, 'slow_yaw_control_margin_percent'):.1f}\% \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Slow-flight control-surface sizing results after cruise-oriented stabilizer planform selection.}",
        r"\label{tab:stage3-control-authority}",
        r"\end{table}",
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lcc}",
        r"\toprule",
        r"Metric & Low speed & Cruise \\",
        r"\midrule",
        rf"Freestream speed & \SI{{{_stage3_slow_flight_speed_mps(config):.2f}}}{{m/s}} & \SI{{{mission.cruise_speed_mps:.2f}}}{{m/s}} \\",
        rf"Total drag & \SI{{{slow_drag:.3f}}}{{N}} & \SI{{{cruise_drag:.3f}}}{{N}} \\",
        rf"Natural airframe drag & \SI{{{_tex_float(row, 'low_speed_natural_drag_n'):.3f}}}{{N}} & -- \\",
        rf"Slow wing induced, used & \SI{{{_tex_float(row, 'low_speed_wing_induced_drag_n'):.3f}}}{{N}} & -- \\",
        rf"Slow wing induced, local blown-panel & \SI{{{_tex_float(row, 'low_speed_wing_induced_local_drag_n'):.3f}}}{{N}} & -- \\",
        rf"Slow wing induced, freestream-equivalent & \SI{{{_tex_float(row, 'low_speed_wing_induced_freestream_equiv_drag_n'):.3f}}}{{N}} & -- \\",
        rf"Added drag required & \SI{{{_tex_float(row, 'low_speed_added_drag_required_n'):.3f}}}{{N}} & -- \\",
        rf"Slow drag minus cruise drag & \SI{{{_tex_float(row, 'low_speed_drag_delta_vs_cruise_n'):.3f}}}{{N}} & -- \\",
        rf"Fuselage drag increment & \SI{{{_tex_float(row, 'low_speed_fuselage_drag_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'cruise_fuselage_drag_n'):.3f}}}{{N}} \\",
        rf"Electrical power & \SI{{{slow_power:.2f}}}{{W}} & \SI{{{cruise_power:.2f}}}{{W}} \\",
        rf"Freestream-reference \(C_L\) & {_tex_float(row, 'slow_flight_freestream_cl_required'):.3f} & {_tex_float(row, 'cruise_cl_required'):.3f} \\",
        rf"Pressure-weighted local \(C_L\) & {_tex_float(row, 'slow_flight_uniform_local_cl_required'):.3f} & {_tex_float(row, 'cruise_uniform_local_cl_required'):.3f} \\",
        rf"Unblown/blown dynamic pressure & \SI{{{_tex_float(row, 'slow_flight_unblown_dynamic_pressure_pa'):.1f}}}{{Pa}} / \SI{{{_tex_float(row, 'slow_flight_blown_dynamic_pressure_pa'):.1f}}}{{Pa}} & \SI{{{_tex_float(row, 'cruise_unblown_dynamic_pressure_pa'):.1f}}}{{Pa}} / \SI{{{_tex_float(row, 'cruise_blown_dynamic_pressure_pa'):.1f}}}{{Pa}} \\",
        rf"Effective dynamic pressure & \SI{{{_tex_float(row, 'slow_flight_effective_dynamic_pressure_pa'):.1f}}}{{Pa}} & \SI{{{_tex_float(row, 'cruise_effective_dynamic_pressure_pa'):.1f}}}{{Pa}} \\",
        rf"Required thrust & \SI{{{_tex_float(row, 'low_speed_thrust_required_n'):.3f}}}{{N}} & \SI{{{_tex_float(row, 'cruise_thrust_required_n'):.3f}}}{{N}} \\",
        rf"Throttle estimate & {_tex_float(row, 'low_speed_throttle_percent'):.1f}\% & {_tex_float(row, 'cruise_throttle_percent'):.1f}\% \\",
        rf"RPM & \num{{{_tex_float(row, 'low_speed_rpm'):.0f}}} & \num{{{_tex_float(row, 'cruise_rpm'):.0f}}} \\",
        rf"\(C_T/C_P\) & {_tex_float(row, 'low_speed_ct'):.4f} / {_tex_float(row, 'low_speed_cp'):.4f} & {_tex_float(row, 'cruise_ct'):.4f} / {_tex_float(row, 'cruise_cp'):.4f} \\",
        rf"Required/actual blown velocity & \SI{{{_tex_float(row, 'low_speed_required_veff_mps'):.3f}}}{{m/s}} / \SI{{{_tex_float(row, 'low_speed_actual_veff_mps'):.3f}}}{{m/s}} & -- \\",
        rf"Cruise \(C_L/C_D/L/D\) & -- & {_tex_float(row, 'cruise_cl'):.3f} / {_tex_float(row, 'cruise_cd'):.4f} / {_tex_float(row, 'cruise_l_over_d'):.2f} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Low-speed and cruise performance after Stage~3 tail sizing.}",
        r"\label{tab:stage3-performance}",
        r"\end{table}",
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\small",
        r"\begin{tabular}{lrrrrrrr}",
        r"\toprule",
        r"Condition & Speed [m/s] & RPM & Thrust [N] & Throttle [\%] & \(C_L\) free/local & Drag [N] & Power [W] \\",
        r"\midrule",
        rf"Clean cruise & {mission.cruise_speed_mps:.2f} & \num{{{_tex_float(row, 'cruise_rpm'):.0f}}} & {_tex_float(row, 'cruise_thrust_required_n'):.3f} & {_tex_float(row, 'cruise_throttle_percent'):.1f} & {_tex_float(row, 'cruise_cl_required'):.3f} / {_tex_float(row, 'cruise_uniform_local_cl_required'):.3f} & {_tex_float(row, 'cruise_drag_n'):.3f} & {_tex_float(row, 'cruise_power_w'):.2f} \\",
        rf"Flaps-down slow flight & {_stage3_slow_flight_speed_mps(config):.2f} & \num{{{_tex_float(row, 'low_speed_rpm'):.0f}}} & {_tex_float(row, 'low_speed_thrust_required_n'):.3f} & {_tex_float(row, 'low_speed_throttle_percent'):.1f} & {_tex_float(row, 'slow_flight_freestream_cl_required'):.3f} / {_tex_float(row, 'slow_flight_uniform_local_cl_required'):.3f} & {_tex_float(row, 'low_speed_drag_n'):.3f} & {_tex_float(row, 'low_speed_power_w'):.2f} \\",
        rf"Flaps-down approach/descent & {_tex_float(row, 'approach_speed_mps'):.2f} & \num{{{_tex_float(row, 'approach_rpm'):.0f}}} & {_tex_float(row, 'approach_thrust_required_n'):.3f} & {_tex_float(row, 'approach_throttle_percent'):.1f} & {_tex_float(row, 'approach_cl_required'):.3f} / {_tex_float(row, 'approach_uniform_local_cl_required'):.3f} & {_tex_float(row, 'approach_drag_n'):.3f} & {_tex_float(row, 'approach_power_w'):.2f} \\",
        rf"Clean climb & {_tex_float(row, 'climb_speed_mps'):.2f} & \num{{{_tex_float(row, 'climb_rpm'):.0f}}} & {_tex_float(row, 'climb_thrust_required_n'):.3f} & {_tex_float(row, 'climb_throttle_percent'):.1f} & {_tex_float(row, 'climb_cl_required'):.3f} / {_tex_float(row, 'climb_uniform_local_cl_required'):.3f} & {_tex_float(row, 'climb_drag_n'):.3f} & {_tex_float(row, 'climb_power_w'):.2f} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Flight-condition operating values. The free-stream \(C_L\) uses only \(q_\infty\); the local/effective \(C_L\) includes the pressure-weighted blown and unblown wing sections where that model is available.}",
        r"\label{tab:stage3-cl-rpm}",
        r"\end{table}",
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lrrrr}",
        r"\toprule",
        r"Condition & Blown velocity [m/s] & \(q_\infty\) [Pa] & \(q_\mathrm{blown}\) [Pa] & \(q_\mathrm{eff}\) [Pa] \\",
        r"\midrule",
        rf"Clean cruise & {_tex_float(row, 'cruise_blown_effective_velocity_mps'):.3f} & {_tex_float(row, 'cruise_unblown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'cruise_blown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'cruise_effective_dynamic_pressure_pa'):.1f} \\",
        rf"Flaps-down slow flight & {_tex_float(row, 'low_speed_actual_veff_mps'):.3f} & {_tex_float(row, 'slow_flight_unblown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'slow_flight_blown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'slow_flight_effective_dynamic_pressure_pa'):.1f} \\",
        rf"Flaps-down approach/descent & {_tex_float(row, 'approach_blown_effective_velocity_mps'):.3f} & {_tex_float(row, 'approach_unblown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'approach_blown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'approach_effective_dynamic_pressure_pa'):.1f} \\",
        rf"Clean climb & {_tex_float(row, 'climb_blown_effective_velocity_mps'):.3f} & {_tex_float(row, 'climb_unblown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'climb_blown_dynamic_pressure_pa'):.1f} & {_tex_float(row, 'climb_effective_dynamic_pressure_pa'):.1f} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Propwash dynamic-pressure diagnostics used to translate free-stream lift demand into local wing-section lift demand.}",
        r"\label{tab:stage3-propwash-q}",
        r"\end{table}",
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lrrr}",
        r"\toprule",
        r"Segment & Duration [min] & Power [W] & Energy [Wh] \\",
        r"\midrule",
        rf"Cruise & {_tex_float(row, 'battery_cruise_segment_min'):.1f} & {_tex_float(row, 'cruise_power_w'):.2f} & {_tex_float(row, 'battery_cruise_energy_wh'):.2f} \\",
        rf"Slow flight & {_tex_float(row, 'battery_slow_segment_min'):.1f} & {_tex_float(row, 'low_speed_power_w'):.2f} & {_tex_float(row, 'battery_slow_energy_wh'):.2f} \\",
        rf"Usable subtotal & {_tex_float(row, 'battery_cruise_segment_min') + _tex_float(row, 'battery_slow_segment_min'):.1f} & -- & {_tex_float(row, 'battery_usable_energy_wh'):.2f} \\",
        rf"Pack capacity with {_tex_float(row, 'battery_reserve_fraction') * 100.0:.0f}\% reserve & -- & -- & {_tex_float(row, 'battery_capacity_required_wh'):.2f} \\",
        rf"Equivalent 4S LiPo capacity & -- & -- & {_tex_float(row, 'battery_capacity_required_mah'):.0f} mAh at \SI{{{_tex_float(row, 'battery_pack_voltage_v'):.1f}}}{{V}} \\",
        r"\bottomrule",
        r"\end{tabular}",
        rf"\caption{{Battery capacity estimate for \SI{{{_tex_float(row, 'battery_cruise_segment_min'):.1f}}}{{min}} of cruise and \SI{{{_tex_float(row, 'battery_slow_segment_min'):.1f}}}{{min}} of slow flight. At \SI{{{mission.battery_specific_energy_wh_per_kg:.0f}}}{{Wh/kg}}, the reserve-adjusted pack mass estimate is \SI{{{_tex_float(row, 'battery_mass_required_kg'):.3f}}}{{kg}}.}}",
        r"\label{tab:stage3-battery-capacity}",
        r"\end{table}",
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lrrrrrrr}",
        r"\toprule",
        r"Regime & RPM & Time [min] & Total P [W] & Total I [A] & Energy [Wh] & Used [mAh] & \% of sized 4S pack \\",
        r"\midrule",
        *_mission_regime_tex_lines(row, mission, config),
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Mission-regime energy estimate in the review-table format. Takeoff uses the payload-on climb condition; landing uses the post-delivery flaps-down slow/flare condition. Current is computed at nominal pack voltage.}",
        r"\label{tab:stage3-mission-regime-energy}",
        r"\end{table}",
        (
            rf"The mission-regime table sizes to a \textbf{{\num{{{_tex_float(row, 'mission_profile_capacity_required_mah'):.0f}}}~mAh}} "
            rf"{int(_tex_float(row, 'mission_profile_battery_cell_count'))}S LiPo at "
            rf"\SI{{{_tex_float(row, 'mission_profile_pack_voltage_v'):.1f}}}{{V}} nominal, including "
            rf"{_tex_float(row, 'mission_profile_reserve_fraction') * 100.0:.0f}\% reserve. "
            rf"The usable energy before reserve is \SI{{{_tex_float(row, 'mission_profile_usable_energy_wh'):.2f}}}{{Wh}}, "
            rf"and the reserve-adjusted capacity is \SI{{{_tex_float(row, 'mission_profile_capacity_required_wh'):.2f}}}{{Wh}}."
        ),
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lccc}",
        r"\toprule",
        r"Aero surface & Cruise Reynolds number & Slow-flight Reynolds number & Characteristic chord \\",
        r"\midrule",
        rf"Main wing & \num{{{_tex_float(row, 'main_wing_re_cruise'):.0f}}} & \num{{{_tex_float(row, 'main_wing_re_low_speed'):.0f}}} & main wing chord \\",
        rf"Horizontal stabilizer & \num{{{_tex_float(row, 'htail_re_cruise'):.0f}}} & \num{{{_tex_float(row, 'htail_re_low_speed'):.0f}}} & H-tail MAC \\",
        rf"Vertical stabilizer & \num{{{_tex_float(row, 'vtail_re_cruise'):.0f}}} & \num{{{_tex_float(row, 'vtail_re_low_speed'):.0f}}} & V-tail MAC \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Reynolds numbers for the main lifting surfaces. Stage~3 uses \(Re=\rho V c/\mu\), with the tail characteristic chord equal to each trapezoidal mean aerodynamic chord.}",
        r"\label{tab:stage3-reynolds}",
        r"\end{table}",
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lcc}",
        r"\toprule",
        r"Drag component & Clean cruise [N] & Flaps-down slow [N] \\",
        r"\midrule",
        rf"Main wing profile & {_tex_float(row, 'cruise_wing_profile_drag_n'):.3f} & {_tex_float(row, 'low_speed_wing_profile_drag_n'):.3f} \\",
        rf"Main wing induced, used & {_tex_float(row, 'cruise_wing_induced_drag_n'):.3f} & {_tex_float(row, 'low_speed_wing_induced_drag_n'):.3f} \\",
        rf"Horizontal tail & {_tex_float(row, 'cruise_htail_drag_n'):.3f} & {_tex_float(row, 'low_speed_htail_drag_n'):.3f} \\",
        rf"Vertical tail & {_tex_float(row, 'cruise_vtail_drag_n'):.3f} & {_tex_float(row, 'low_speed_vtail_drag_n'):.3f} \\",
        rf"Fuselage & {_tex_float(row, 'cruise_fuselage_drag_n'):.3f} & {_tex_float(row, 'low_speed_fuselage_drag_n'):.3f} \\",
        rf"Added drag / airbrakes & 0.000 & {_tex_float(row, 'low_speed_added_drag_required_n'):.3f} \\",
        rf"Total & {_tex_float(row, 'cruise_drag_n'):.3f} & {_tex_float(row, 'low_speed_drag_n'):.3f} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Drag buildup for the clean cruise and flaps-down slow-flight sizing points. The slow-flight induced value is the physical split-panel sum; the freestream-equivalent value is retained only as a diagnostic check.}",
        r"\label{tab:stage3-drag-components}",
        r"\end{table}",
        "",
        r"\begin{figure}[H]",
        r"\centering",
        rf"\includegraphics[width=0.92\linewidth]{{{drag_power_sweeps}}}",
        r"\caption{Total drag and electrical power required versus velocity and angle of attack for clean cruise and flaps-down slow-flight configurations.}",
        r"\label{fig:stage3-drag-power-sweeps}",
        r"\end{figure}",
        "",
        r"\begin{figure}[H]",
        r"\centering",
        rf"\includegraphics[width=0.88\linewidth]{{{drag_components_plot}}}",
        r"\caption{Drag component comparison for the final Stage~3 configuration.}",
        r"\label{fig:stage3-drag-components}",
        r"\end{figure}",
        "",
        r"\section{Approach Trim Estimate}",
        (
            r"The approach calculation assumes a steady flaps-down descent at the user-specified "
            r"target flight-path angle. The weight component along the flight path reduces the "
            r"thrust required according to \(T = D - W\sin\gamma\), clipped at zero thrust. "
            r"Elevator trim is estimated from the flapped pitching moment, the CG/aerodynamic-center "
            r"offset, horizontal tail volume, slow-flight tail dynamic-pressure ratio, and the "
            r"editable trim-effectiveness factor in the YAML constraints."
        ),
        "",
        r"\begin{table}[H]",
        r"\centering",
        r"\begin{tabular}{lcc}",
        r"\toprule",
        r"Quantity & Symbol & Value \\",
        r"\midrule",
        rf"Target sink rate & -- & \SI{{{_tex_float(row, 'approach_target_sink_rate_fps'):.2f}}}{{ft/s}} \\",
        rf"Actual sink rate & -- & \SI{{{_tex_float(row, 'approach_sink_rate_mps'):.3f}}}{{m/s}} / \SI{{{_tex_float(row, 'approach_sink_rate_fps'):.2f}}}{{ft/s}} \\",
        rf"Sink-rate target met & -- & {int(_tex_float(row, 'approach_sink_rate_target_met'))} \\",
        rf"Approach angle below horizontal & \(\gamma\) & \SI{{{_tex_float(row, 'approach_target_angle_deg'):.2f}}}{{deg}} \\",
        rf"Approach speed & \(V_A\) & \SI{{{_tex_float(row, 'approach_speed_mps'):.2f}}}{{m/s}} \\",
        rf"Flap deflection & \(\delta_f\) & \SI{{{_tex_float(row, 'approach_flap_deflection_deg'):.1f}}}{{deg}} \\",
        rf"Angle of attack & \(\alpha_A\) & \SI{{{_tex_float(row, 'approach_alpha_deg'):.2f}}}{{deg}} \\",
        rf"Required lift coefficient & \(C_L\) & {_tex_float(row, 'approach_cl_required'):.3f} \\",
        rf"Approach drag & \(D_A\) & \SI{{{_tex_float(row, 'approach_drag_n'):.3f}}}{{N}} \\",
        rf"Required thrust & \(T_A\) & \SI{{{_tex_float(row, 'approach_thrust_required_n'):.3f}}}{{N}} \\",
        rf"Electrical power & \(P_A\) & \SI{{{_tex_float(row, 'approach_power_w'):.2f}}}{{W}} \\",
        rf"Throttle estimate & -- & {_tex_float(row, 'approach_throttle_percent'):.1f}\% \\",
        rf"Propulsion feasible & -- & {int(_tex_float(row, 'approach_rpm_feasible'))} \\",
        rf"RPM & -- & \num{{{_tex_float(row, 'approach_rpm'):.0f}}} \\",
        rf"Elevator trim & \(\delta_e\) & \SI{{{_tex_float(row, 'approach_elevator_trim_deg'):.2f}}}{{deg}} \\",
        rf"Rudder trim & \(\delta_r\) & \SI{{{_tex_float(row, 'approach_rudder_trim_deg'):.2f}}}{{deg}} \\",
        rf"Max descent rate limit & -- & \SI{{{_tex_float(row, 'approach_max_descent_rate_fps'):.2f}}}{{ft/s}} \\",
        r"\bottomrule",
        r"\end{tabular}",
        r"\caption{Estimated trims and throttle setting for the configured approach angle.}",
        r"\label{tab:stage3-approach}",
        r"\end{table}",
        "",
        r"\section{Trim and Aerodynamic Verification}",
        (
            "The final geometry is checked with AeroSandbox VLM at cruise. The trim solver adjusts "
            "aircraft angle of attack and horizontal-tail incidence to close lift and pitching "
            "moment. The final residuals are"
        ),
        r"\begin{equation}",
        rf"\Delta L = \SI{{{_tex_float(row, 'trim_residual_lift_n'):.6f}}}{{N}}, \qquad \Delta C_M = {_tex_float(row, 'trim_residual_cm'):.6f}.",
        r"\end{equation}",
        (
            f"The airfoil-polar cruise body-angle proxy is \\SI{{{_tex_float(row, 'cruise_body_alpha_proxy_deg'):.3f}}}{{deg}}, "
            f"with a wing-section alpha proxy of \\SI{{{_tex_float(row, 'cruise_section_alpha_proxy_deg'):.3f}}}{{deg}}. "
            f"The VLM trim-alpha diagnostic is \\SI{{{_tex_float(row, 'cruise_alpha_deg'):.3f}}}{{deg}}, "
            f"and the horizontal-tail incidence is \\SI{{{_tex_float(row, 'htail_incidence_deg'):.3f}}}{{deg}}. "
            "The selected result has warning status "
            f"\\texttt{{{_tex_escape(warnings)}}}."
        ),
        "",
        r"\begin{figure}[H]",
        r"\centering",
        rf"\includegraphics[width=0.82\linewidth]{{{polar_plot}}}",
        r"\caption{Section polar diagnostic used by Stage~3 for cruise and low-speed airfoil checks. The whole-wing slow-flight \(C_{L,\max}\) values used for sizing are inherited from the Stage~1/2 high-lift workflow rather than from this diagnostic plot alone.}",
        r"\label{fig:stage3-polars}",
        r"\end{figure}",
        "",
        r"\section{Engineering Interpretation and Open Items}",
        r"\begin{enumerate}[leftmargin=*]",
        (
            r"\item The tail geometry meets the active tail-volume and static-margin targets while "
            r"adding only \SI{" + f"{tail_mass:.3f}" + r"}{kg} of NGX250 foam mass."
        ),
        (
            r"\item The propeller drawing now uses the Stage~2 propeller centers and the Stage~3 "
            r"YAML axial-location parameter. This makes the layout plot a geometry check rather "
            r"than a schematic with a hard-coded propeller station."
        ),
        (
            r"\item The fuselage model has been updated to a \SI{170}{mm} by \SI{130}{mm} cross-section. "
            r"The equivalent drag area remains a concept-level parasite-drag surrogate and should "
            r"eventually be replaced by a measured or CFD-derived fuselage drag polar."
        ),
        (
            r"\item The slow-flight lift margin is large because Stage~3 consumes the Stage~1/2 "
            r"flap-down blown \(C_{L,\max}\) directly. This is consistent with the current user "
            r"requirement, but it also means the high-lift model remains one of the dominant "
            r"validation risks."
        ),
        (
            r"\item The next fidelity step should be a coupled tail-plus-flap trim case at the "
            r"low-speed condition, because the current high-lift sizing focuses on lift availability "
            r"and propeller power rather than full low-speed moment trim with the flap deployed."
        ),
        r"\end{enumerate}",
        "",
        r"\section{Generated Artifacts}",
        r"\begin{itemize}[leftmargin=*]",
        rf"\item Machine-readable results: \texttt{{../outputs/stage3\_aerosandbox\_results.csv}}",
        rf"\item Human-readable Markdown summary: \texttt{{../outputs/stage3\_readable\_results.md}}",
        rf"\item Technical Markdown report: \texttt{{../outputs/stage3\_aerosandbox\_report.md}}",
        rf"\item Top-view geometry plot: \texttt{{{_tex_escape(top_view)}}}",
        rf"\item Three-view geometry plot: \texttt{{{_tex_escape(three_view)}}}",
        rf"\item 3D wireframe rendering: \texttt{{{_tex_escape(render_3d)}}}",
        rf"\item Drag and power sweep plot: \texttt{{{_tex_escape(drag_power_sweeps)}}}",
        rf"\item Drag component plot: \texttt{{{_tex_escape(drag_components_plot)}}}",
        rf"\item Performance sweep CSV: \texttt{{{_tex_escape(_tex_path(row['performance_sweep_csv']))}}}",
        rf"\item Mesh file: \texttt{{{_tex_escape(_tex_path(row['mesh_npz']))}}}",
        r"\end{itemize}",
        "",
        r"\end{document}",
        "",
    ]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines), encoding="utf-8")


def _artifact_slug(rank_seed: int, stage2_row: dict[str, str]) -> str:
    return (
        f"seed{rank_seed:02d}_n{int(float(stage2_row['n_props']))}"
        f"_d{float(stage2_row['prop_diameter_in']):.1f}"
        f"_pd{float(stage2_row['prop_pitch_ratio']):.2f}"
        f"_{stage2_row['prop_family']}"
    ).replace(".", "p")


def refine_stage3_candidate(
    mission: Stage1MissionConfig,
    stage2_row: dict[str, str],
    stage1_row: dict[str, str],
    *,
    rank_seed: int,
    wing_context: dict[str, Any] | None = None,
    config: Stage3SizingConfig | None = None,
) -> dict[str, Any]:
    config = config or Stage3SizingConfig()
    wing_context = wing_context or load_selected_wing_context(config)

    candidate = _stage1_candidate_from_row(stage2_row)
    geometry = optimize_stage3_tail(mission, stage2_row, stage1_row, wing_context, config)
    geometry.update(size_slow_flight_control_surfaces(geometry, config))
    alpha_trim, htail_incidence, vlm_trim = solve_vlm_cruise_trim(
        mission,
        geometry,
        wing_context,
        config,
    )
    geometry["htail_incidence_deg"] = htail_incidence
    neutral_airplane, meta = build_airplane_geometry(
        mission,
        geometry,
        wing_context,
        config,
        cg_x_m=geometry["cg_x_m"],
        htail_incidence_deg=htail_incidence,
    )

    alpha_minus = _run_vlm(neutral_airplane, velocity_mps=mission.cruise_speed_mps, alpha_deg=alpha_trim - 1.0)
    alpha_plus = _run_vlm(neutral_airplane, velocity_mps=mission.cruise_speed_mps, alpha_deg=alpha_trim + 1.0)
    cm_alpha_per_deg = 0.5 * (alpha_plus.get("Cm", 0.0) - alpha_minus.get("Cm", 0.0))

    polars = evaluate_section_polars(mission, wing_context, config)

    main_area_coeff = _airfoil_area_coefficient(wing_context["selected_airfoil"])
    main_wing_foam_mass = (
        _foam_volume_trapezoid(mission.span_m, mission.chord_m, mission.chord_m, main_area_coeff)
        * config.foam_density_kgpm3
    )

    slug = _artifact_slug(rank_seed, stage2_row)
    top_view_path = STAGE3_VISUAL_DIR / f"{slug}_top_view.png"
    three_view_path = STAGE3_VISUAL_DIR / f"{slug}_three_view.png"
    wireframe_path = STAGE3_VISUAL_DIR / f"{slug}_wireframe.png"
    render_3d_path = STAGE3_VISUAL_DIR / f"{slug}_3d_wireframe.png"
    polar_path = STAGE3_VISUAL_DIR / f"{slug}_polars.png"
    performance_sweep_csv_path = STAGE3_VISUAL_DIR / f"{slug}_performance_sweep.csv"
    drag_power_sweeps_path = STAGE3_VISUAL_DIR / f"{slug}_drag_power_sweeps.png"
    drag_components_path = STAGE3_VISUAL_DIR / f"{slug}_drag_components.png"
    mesh_path = STAGE3_VISUAL_DIR / f"{slug}_mesh.npz"

    render_top_view(mission, geometry, wing_context, config, stage2_row, top_view_path)
    render_three_view_and_wireframe(neutral_airplane, three_view_path, wireframe_path)
    render_3d_wireframe(neutral_airplane, render_3d_path)
    render_polar_plot(polars, alpha_trim, polar_path)
    performance_outputs = generate_performance_sweep_outputs(
        mission,
        geometry,
        wing_context,
        config,
        candidate,
        polars,
        csv_path=performance_sweep_csv_path,
        plot_path=drag_power_sweeps_path,
        components_path=drag_components_path,
    )
    clean_condition_polar = _section_polar_from_stage3_polars(polars, "clean")
    flapped_condition_polar = _section_polar_from_stage3_polars(polars, "flaps_down")
    payload_mass_kg = max(config.payload_mass_lb, 0.0) * 0.45359237
    post_delivery_mass_kg = max(
        geometry["stage3_total_built_mass_kg"],
        mission.gross_mass_kg - payload_mass_kg,
    )
    payload_on_performance = {
        "mass_kg": mission.gross_mass_kg,
        "cruise_cl_required": mission.gross_weight_n
        / max(0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2 * mission.wing_area_m2, 1e-9),
        "cruise_uniform_local_cl_required": geometry["cruise_uniform_local_cl_required"],
        "cruise_drag_n": geometry["cruise_drag_n"],
        "cruise_thrust_required_n": geometry["cruise_thrust_required_n"],
        "cruise_throttle_percent": geometry["cruise_throttle_percent"],
        "cruise_rpm": geometry["cruise_rpm"],
        "cruise_power_w": geometry["cruise_power_w"],
        "cruise_blown_effective_velocity_mps": geometry["cruise_blown_effective_velocity_mps"],
        "cruise_effective_dynamic_pressure_pa": geometry["cruise_effective_dynamic_pressure_pa"],
        "slow_cl_required": geometry["slow_flight_freestream_cl_required"],
        "slow_uniform_local_cl_required": geometry["slow_flight_uniform_local_cl_required"],
        "slow_drag_n": geometry["low_speed_drag_n"],
        "slow_natural_drag_n": geometry["low_speed_natural_drag_n"],
        "slow_added_drag_required_n": geometry["low_speed_added_drag_required_n"],
        "slow_thrust_required_n": geometry["low_speed_thrust_required_n"],
        "slow_throttle_percent": geometry["low_speed_throttle_percent"],
        "slow_rpm": geometry["low_speed_rpm"],
        "slow_power_w": geometry["low_speed_power_w"],
        "slow_blown_effective_velocity_mps": geometry["low_speed_actual_veff_mps"],
        "slow_effective_dynamic_pressure_pa": geometry["slow_flight_effective_dynamic_pressure_pa"],
        "slow_unblown_local_cl": geometry["slow_flight_unblown_local_cl"],
        "slow_blown_local_cl": geometry["slow_flight_blown_local_cl"],
        "slow_local_cl_margin": geometry["slow_flight_local_cl_margin"],
        "approach_cl_required": performance_outputs["approach_cl_required"],
        "approach_uniform_local_cl_required": performance_outputs["approach_uniform_local_cl_required"],
        "approach_drag_n": performance_outputs["approach_drag_n"],
        "approach_thrust_required_n": performance_outputs["approach_thrust_required_n"],
        "approach_throttle_percent": performance_outputs["approach_throttle_percent"],
        "approach_rpm": performance_outputs["approach_rpm"],
        "approach_power_w": performance_outputs["approach_power_w"],
        "approach_blown_effective_velocity_mps": performance_outputs["approach_blown_effective_velocity_mps"],
        "approach_effective_dynamic_pressure_pa": performance_outputs["approach_effective_dynamic_pressure_pa"],
        "approach_elevator_trim_deg": performance_outputs["approach_elevator_trim_deg"],
        "approach_rpm_feasible": performance_outputs["approach_rpm_feasible"],
        "climb_cl_required": performance_outputs["climb_cl_required"],
        "climb_uniform_local_cl_required": performance_outputs["climb_uniform_local_cl_required"],
        "climb_drag_n": performance_outputs["climb_drag_n"],
        "climb_thrust_required_n": performance_outputs["climb_thrust_required_n"],
        "climb_throttle_percent": performance_outputs["climb_throttle_percent"],
        "climb_rpm": performance_outputs["climb_rpm"],
        "climb_power_w": performance_outputs["climb_power_w"],
        "climb_blown_effective_velocity_mps": performance_outputs["climb_blown_effective_velocity_mps"],
        "climb_effective_dynamic_pressure_pa": performance_outputs["climb_effective_dynamic_pressure_pa"],
        "climb_elevator_trim_deg": performance_outputs["climb_elevator_trim_deg"],
        "climb_rpm_feasible": performance_outputs["climb_rpm_feasible"],
    }
    post_delivery_performance = _mass_case_condition_summary(
        mission,
        candidate,
        geometry,
        config,
        clean_condition_polar,
        flapped_condition_polar,
        post_delivery_mass_kg,
    )

    main_wing_clean_design_clmax = geometry["no_flap_clmax"] / max(
        config.main_wing_pitch_up_stall_margin_factor,
        1e-9,
    )
    main_wing_flapped_design_local_clmax = geometry["slow_flight_design_blown_local_clmax"]
    payload_on_cruise_pitch_margin = (
        main_wing_clean_design_clmax / max(payload_on_performance["cruise_uniform_local_cl_required"], 1e-9)
        - 1.0
    )
    post_delivery_cruise_pitch_margin = (
        main_wing_clean_design_clmax / max(post_delivery_performance["cruise_uniform_local_cl_required"], 1e-9)
        - 1.0
    )
    payload_on_climb_pitch_margin = (
        main_wing_clean_design_clmax / max(payload_on_performance["climb_uniform_local_cl_required"], 1e-9)
        - 1.0
    )
    post_delivery_climb_pitch_margin = (
        main_wing_clean_design_clmax / max(post_delivery_performance["climb_uniform_local_cl_required"], 1e-9)
        - 1.0
    )

    runtime = ensure_stage3_runtime()
    points, faces = neutral_airplane.mesh_body(method="quad", thin_wings=True, stack_meshes=True)
    runtime.onp.savez(mesh_path, points=runtime.onp.asarray(points), faces=runtime.onp.asarray(faces))

    warnings: list[str] = []
    if geometry["mass_budget_margin_kg"] < -1e-6:
        warnings.append("mass_budget_exceeded")
    if geometry["static_margin_mac"] < config.min_static_margin_mac:
        warnings.append("low_static_margin")
    if geometry["slow_flight_lift_margin_percent"] < 100.0 * config.min_slow_flight_lift_margin:
        warnings.append("low_slow_flight_lift_margin")
    if geometry["slow_flight_feasible"] < 1.0:
        warnings.append("slow_flight_infeasible")
    if geometry["slow_flight_freestream_cl_required"] > geometry["flap_down_blown_clmax"]:
        warnings.append("slow_flight_requires_powered_lift")
    if geometry["clmax_was_capped"] > 0.5:
        warnings.append("stage12_equivalent_clmax_capped")
    if geometry["slow_pitch_control_margin_percent"] < -1e-6:
        warnings.append("low_slow_pitch_control_authority")
    if geometry["slow_yaw_control_margin_percent"] < -1e-6:
        warnings.append("low_slow_yaw_control_authority")
    if min(payload_on_cruise_pitch_margin, payload_on_climb_pitch_margin) < 0.0:
        warnings.append("low_main_wing_pitch_up_stall_margin")
    if _safe_float(performance_outputs.get("approach_rpm_feasible"), 1.0) < 1.0:
        warnings.append("approach_throttle_saturated")
    if _safe_float(performance_outputs.get("climb_rpm_feasible"), 1.0) < 1.0:
        warnings.append("climb_throttle_saturated")
    if cm_alpha_per_deg >= 0.0:
        warnings.append("vlm_static_instability")
    if abs(vlm_trim.get("trim_residual_lift_n", 0.0)) > 1.0:
        warnings.append("vlm_lift_trim_residual")
    if abs(vlm_trim.get("trim_residual_cm", 0.0)) > 0.03:
        warnings.append("vlm_moment_trim_residual")

    cruise_cl = mission.gross_weight_n / (
        0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2 * mission.wing_area_m2
    )
    cruise_cd = geometry["cruise_drag_n"] / (
        0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2 * mission.wing_area_m2
    )
    slow_energy_wh = geometry["low_speed_power_w"] * mission.climb_segment_min / 60.0
    battery_cruise_energy_wh = geometry["cruise_power_w"] * config.battery_cruise_segment_min / 60.0
    battery_slow_energy_wh = geometry["low_speed_power_w"] * config.battery_slow_segment_min / 60.0
    battery_usable_energy_wh = battery_cruise_energy_wh + battery_slow_energy_wh
    battery_capacity_required_wh = battery_usable_energy_wh / max(1.0 - mission.battery_reserve_fraction, 1e-9)
    battery_pack_voltage_v = _mission_profile_pack_voltage_v(mission, config)
    battery_capacity_required_mah = battery_capacity_required_wh / max(battery_pack_voltage_v, 1e-9) * 1000.0
    battery_mass_required_kg = battery_capacity_required_wh / max(mission.battery_specific_energy_wh_per_kg, 1e-9)
    main_wing_re_cruise = (
        mission.air_density_kgpm3
        * mission.cruise_speed_mps
        * mission.chord_m
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    main_wing_re_low_speed = (
        mission.air_density_kgpm3
        * _stage3_slow_flight_speed_mps(config)
        * mission.chord_m
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    htail_re_cruise = (
        mission.air_density_kgpm3
        * mission.cruise_speed_mps
        * geometry["htail_mac_m"]
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    htail_re_low_speed = (
        mission.air_density_kgpm3
        * _stage3_slow_flight_speed_mps(config)
        * geometry["htail_mac_m"]
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    vtail_re_cruise = (
        mission.air_density_kgpm3
        * mission.cruise_speed_mps
        * geometry["vtail_mac_m"]
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )
    vtail_re_low_speed = (
        mission.air_density_kgpm3
        * _stage3_slow_flight_speed_mps(config)
        * geometry["vtail_mac_m"]
        / max(mission.dynamic_viscosity_pas, 1e-12)
    )

    mass_case_outputs: dict[str, float] = {
        "payload_mass_lb": config.payload_mass_lb,
        "payload_mass_kg": payload_mass_kg,
        "payload_on_mass_kg": mission.gross_mass_kg,
        "post_delivery_mass_kg": post_delivery_mass_kg,
        "post_delivery_mass_delta_kg": mission.gross_mass_kg - post_delivery_mass_kg,
        "battery_sizing_assumes_payload_on": 1.0,
        "main_wing_pitch_up_stall_margin_factor": config.main_wing_pitch_up_stall_margin_factor,
        "main_wing_clean_design_clmax_for_pitch_up": main_wing_clean_design_clmax,
        "main_wing_flapped_design_local_clmax": main_wing_flapped_design_local_clmax,
        "payload_on_cruise_pitch_up_stall_margin_percent": 100.0 * payload_on_cruise_pitch_margin,
        "post_delivery_cruise_pitch_up_stall_margin_percent": 100.0 * post_delivery_cruise_pitch_margin,
        "payload_on_climb_pitch_up_stall_margin_percent": 100.0 * payload_on_climb_pitch_margin,
        "post_delivery_climb_pitch_up_stall_margin_percent": 100.0 * post_delivery_climb_pitch_margin,
    }
    for prefix, data in (
        ("payload_on", payload_on_performance),
        ("post_delivery", post_delivery_performance),
    ):
        for key, value in data.items():
            mass_case_outputs[f"{prefix}_{key}"] = value
    mission_regime_summary = _mission_regime_energy_summary(
        {**geometry, **mass_case_outputs},
        mission,
        config,
    )

    return {
        "rank": 0,
        "status": "SUCCESS",
        "design_warnings": ";".join(warnings),
        "n_props": candidate.n_props,
        "prop_diameter_in": candidate.prop_diameter_in,
        "prop_pitch_ratio": candidate.prop_pitch_ratio,
        "prop_family": candidate.prop_family,
        "seed_low_speed_rpm": _safe_float(stage2_row["solved_low_speed_rpm"]),
        "seed_cruise_rpm": _safe_float(stage2_row["solved_cruise_rpm"]),
        "selected_airfoil": wing_context["selected_airfoil"],
        "tail_airfoil": config.tail_airfoil,
        "wing_context_source": wing_context["source"],
        "wing_span_m": mission.span_m,
        "wing_chord_m": mission.chord_m,
        "wing_area_m2": mission.wing_area_m2,
        "wing_aspect_ratio": mission.aspect_ratio,
        "main_wing_incidence_deg": geometry["main_wing_incidence_deg"],
        "main_wing_washout_deg": config.main_wing_washout_deg,
        "prop_axial_location_fraction_of_chord": config.prop_axial_location_fraction_of_chord,
        "prop_axial_x_m": _prop_axial_x_m(mission, config),
        "propulsion_model_source": geometry["propulsion_model_source"],
        "ecalc_static_csv": config.ecalc_static_csv if config.ecalc_propulsion_enabled else "",
        "ecalc_dynamic_csv": config.ecalc_dynamic_csv if config.ecalc_propulsion_enabled else "",
        "wiring_power_loss_factor": config.wiring_power_loss_factor,
        "avionics_power_loss_factor": config.avionics_power_loss_factor,
        "avionics_power_base_w": mission.avionics_power_w,
        "avionics_power_with_losses_w": mission.avionics_power_w
        * max(config.avionics_power_loss_factor, 0.0),
        "fuselage_nose_x_m": config.fuselage_nose_x_m,
        "fuselage_width_m": config.fuselage_width_m,
        "fuselage_height_m": config.fuselage_height_m,
        "stage3_slow_flight_speed_mps": _stage3_slow_flight_speed_mps(config),
        "upstream_stage12_low_speed_mps": mission.low_speed_mps,
        "flap_span_fraction": wing_context["flap_span_fraction"],
        "flap_chord_fraction": wing_context["flap_chord_fraction"],
        "flap_deflection_slow_deg": _slow_flap_deflection_deg(wing_context, config),
        "aileron_span_fraction": wing_context["aileron_span_fraction"],
        "aileron_chord_fraction": wing_context["aileron_chord_fraction"],
        "htail_span_m": geometry["htail_span_m"],
        "htail_root_chord_m": geometry["htail_root_chord_m"],
        "htail_tip_chord_m": geometry["htail_tip_chord_m"],
        "htail_area_m2": geometry["htail_area_m2"],
        "htail_aspect_ratio": geometry["htail_aspect_ratio"],
        "htail_taper": geometry["htail_taper"],
        "htail_incidence_deg": htail_incidence,
        "elevator_chord_fraction": geometry["elevator_chord_fraction_sized"],
        "elevator_max_deflection_deg": geometry["elevator_max_deflection_deg"],
        "target_slow_pitch_control_cm": geometry["target_slow_pitch_control_cm"],
        "slow_pitch_control_cm_authority": geometry["slow_pitch_control_cm_authority"],
        "slow_pitch_control_margin_percent": geometry["slow_pitch_control_margin_percent"],
        "elevator_trim_cruise_deg": 0.0,
        "vtail_span_m": geometry["vtail_span_m"],
        "vtail_root_chord_m": geometry["vtail_root_chord_m"],
        "vtail_tip_chord_m": geometry["vtail_tip_chord_m"],
        "vtail_area_m2": geometry["vtail_area_m2"],
        "vtail_aspect_ratio": geometry["vtail_aspect_ratio"],
        "vtail_taper": geometry["vtail_taper"],
        "vertical_tail_incidence_deg": config.vertical_tail_incidence_deg,
        "rudder_chord_fraction": geometry["rudder_chord_fraction_sized"],
        "rudder_max_deflection_deg": geometry["rudder_max_deflection_deg"],
        "target_slow_yaw_control_cn": geometry["target_slow_yaw_control_cn"],
        "slow_yaw_control_cn_authority": geometry["slow_yaw_control_cn_authority"],
        "slow_yaw_control_margin_percent": geometry["slow_yaw_control_margin_percent"],
        "tail_arm_m": geometry["tail_arm_m"],
        "horizontal_tail_volume": geometry["horizontal_tail_volume"],
        "vertical_tail_volume": geometry["vertical_tail_volume"],
        "horizontal_tail_volume_min": _horizontal_tail_volume_range(config)[0],
        "horizontal_tail_volume_max": _horizontal_tail_volume_range(config)[1],
        "vertical_tail_volume_min": _vertical_tail_volume_range(config)[0],
        "vertical_tail_volume_max": _vertical_tail_volume_range(config)[1],
        "cg_x_m": geometry["cg_x_m"],
        "cg_percent_mac": geometry["cg_percent_mac"],
        "cg_target_x_m": geometry["cg_target_x_m"],
        "cg_target_percent_mac": geometry["cg_target_percent_mac"],
        "cg_error_m": geometry["cg_error_m"],
        "cg_error_percent_mac": geometry["cg_error_percent_mac"],
        "stage1_baseline_cg_used_percent_mac": geometry["stage1_baseline_cg_used_percent_mac"],
        "stage1_baseline_cg_required_percent_mac": geometry["stage1_baseline_cg_required_percent_mac"],
        "neutral_point_x_m": geometry["neutral_point_x_m"],
        "static_margin_mac": geometry["static_margin_mac"],
        "vlm_cm_alpha_per_deg": cm_alpha_per_deg,
        "main_wing_foam_mass_kg": main_wing_foam_mass,
        "htail_foam_mass_kg": geometry["htail_foam_mass_kg"],
        "vtail_foam_mass_kg": geometry["vtail_foam_mass_kg"],
        "total_tail_foam_mass_kg": geometry["total_tail_foam_mass_kg"],
        "stage1_total_built_mass_kg": geometry["stage1_total_built_mass_kg"],
        "stage3_total_built_mass_kg": geometry["stage3_total_built_mass_kg"],
        "gross_flight_mass_kg": geometry["gross_flight_mass_kg"],
        "remaining_mass_to_gross_kg": geometry["remaining_mass_to_gross_kg"],
        "mass_budget_margin_kg": geometry["mass_budget_margin_kg"],
        "blown_span_fraction": geometry["blown_span_fraction"],
        "clmax_source": geometry["clmax_source"],
        "clmax_curve_source": geometry["clmax_curve_source"],
        "stage12_high_lift_polar_source": polars.get("stage12_high_lift_polar_source", ""),
        "physical_clmax_source": geometry["physical_clmax_source"],
        "stage12_clean_section_clmax_unblown": geometry["stage12_clean_section_clmax_unblown"],
        "stage12_clean_section_clmax_blown": geometry["stage12_clean_section_clmax_blown"],
        "stage12_flapped_section_clmax_unblown": geometry["stage12_flapped_section_clmax_unblown"],
        "stage12_flapped_section_clmax_blown": geometry["stage12_flapped_section_clmax_blown"],
        "stage12_weighted_blown_physical_clmax": geometry["stage12_weighted_blown_physical_clmax"],
        "no_flap_clmax": geometry["no_flap_clmax"],
        "flap_only_clmax": geometry["flap_only_clmax"],
        "clean_blowing_clmax": geometry["clean_blowing_clmax"],
        "flap_down_blown_clmax": geometry["flap_down_blown_clmax"],
        "raw_no_flap_clmax": geometry["raw_no_flap_clmax"],
        "raw_flap_only_clmax": geometry["raw_flap_only_clmax"],
        "raw_clean_blowing_clmax": geometry["raw_clean_blowing_clmax"],
        "raw_flap_down_blown_clmax": geometry["raw_flap_down_blown_clmax"],
        "clmax_was_capped": geometry["clmax_was_capped"],
        "no_flap_clmax_alpha_deg": geometry["no_flap_clmax_alpha_deg"],
        "flap_only_clmax_alpha_deg": geometry["flap_only_clmax_alpha_deg"],
        "clean_blowing_clmax_alpha_deg": geometry["clean_blowing_clmax_alpha_deg"],
        "flap_down_blown_clmax_alpha_deg": geometry["flap_down_blown_clmax_alpha_deg"],
        "fallback_slotted_flap_section_clmax": config.slotted_flap_section_clmax,
        "slow_flight_stall_margin_factor": config.slow_flight_stall_margin_factor,
        "slow_flight_unblown_clmax": geometry["slow_flight_unblown_clmax"],
        "slow_flight_blown_clmax": geometry["slow_flight_blown_clmax"],
        "slow_flight_blown_local_clmax": geometry["slow_flight_blown_local_clmax"],
        "slow_flight_unblown_equivalent_clmax": geometry["slow_flight_unblown_equivalent_clmax"],
        "slow_flight_blown_equivalent_clmax": geometry["slow_flight_blown_equivalent_clmax"],
        "slow_flight_design_unblown_clmax": geometry["slow_flight_design_unblown_clmax"],
        "slow_flight_design_blown_clmax": geometry["slow_flight_design_blown_clmax"],
        "slow_flight_design_blown_local_clmax": geometry["slow_flight_design_blown_local_clmax"],
        "low_speed_required_veff_mps": geometry["low_speed_required_veff_mps"],
        "low_speed_actual_veff_mps": geometry["low_speed_actual_veff_mps"],
        "low_speed_drag_n": geometry["low_speed_drag_n"],
        "low_speed_natural_drag_n": geometry["low_speed_natural_drag_n"],
        "low_speed_added_drag_required_n": geometry["low_speed_added_drag_required_n"],
        "low_speed_drag_delta_vs_cruise_n": geometry["low_speed_drag_delta_vs_cruise_n"],
        "low_speed_blown_lift_thrust_n": geometry["low_speed_blown_lift_thrust_n"],
        "low_speed_stage1_baseline_drag_n": geometry["low_speed_stage1_baseline_drag_n"],
        "low_speed_wing_profile_drag_n": geometry["low_speed_wing_profile_drag_n"],
        "low_speed_wing_induced_drag_n": geometry["low_speed_wing_induced_drag_n"],
        "low_speed_wing_induced_local_drag_n": geometry["low_speed_wing_induced_local_drag_n"],
        "low_speed_wing_induced_freestream_equiv_drag_n": geometry[
            "low_speed_wing_induced_freestream_equiv_drag_n"
        ],
        "low_speed_unblown_induced_drag_n": geometry["low_speed_unblown_induced_drag_n"],
        "low_speed_blown_induced_drag_n": geometry["low_speed_blown_induced_drag_n"],
        "low_speed_htail_drag_n": geometry["low_speed_htail_drag_n"],
        "low_speed_vtail_drag_n": geometry["low_speed_vtail_drag_n"],
        "low_speed_fuselage_drag_n": geometry["low_speed_fuselage_drag_n"],
        "slow_flight_lift_available_n": geometry["slow_flight_lift_available_n"],
        "slow_flight_lift_target_n": geometry["slow_flight_lift_target_n"],
        "slow_flight_lift_margin_n": geometry["slow_flight_lift_margin_n"],
        "slow_flight_lift_margin_percent": geometry["slow_flight_lift_margin_percent"],
        "slow_flight_equiv_stall_speed_mps": geometry["slow_flight_equiv_stall_speed_mps"],
        "slow_flight_freestream_cl_required": geometry["slow_flight_freestream_cl_required"],
        "slow_flight_uniform_local_cl_required": geometry["slow_flight_uniform_local_cl_required"],
        "slow_flight_unblown_dynamic_pressure_pa": geometry["slow_flight_unblown_dynamic_pressure_pa"],
        "slow_flight_blown_dynamic_pressure_pa": geometry["slow_flight_blown_dynamic_pressure_pa"],
        "slow_flight_effective_dynamic_pressure_pa": geometry["slow_flight_effective_dynamic_pressure_pa"],
        "slow_flight_blown_to_unblown_q_ratio": geometry["slow_flight_blown_to_unblown_q_ratio"],
        "slow_flight_blown_area_m2": geometry["slow_flight_blown_area_m2"],
        "slow_flight_unblown_area_m2": geometry["slow_flight_unblown_area_m2"],
        "slow_flight_unblown_local_cl": geometry["slow_flight_unblown_local_cl"],
        "slow_flight_blown_local_cl": geometry["slow_flight_blown_local_cl"],
        "slow_flight_local_cl_margin": geometry["slow_flight_local_cl_margin"],
        "slow_flight_required_veff_margin_mps": geometry["slow_flight_required_veff_margin_mps"],
        "slow_flight_feasible": geometry["slow_flight_feasible"],
        "low_speed_power_w": geometry["low_speed_power_w"],
        "low_speed_propulsion_electric_power_without_aircraft_losses_w": geometry[
            "low_speed_propulsion_electric_power_without_aircraft_losses_w"
        ],
        "low_speed_propulsion_wiring_loss_w": geometry["low_speed_propulsion_wiring_loss_w"],
        "low_speed_avionics_power_loss_w": geometry["low_speed_avionics_power_loss_w"],
        "low_speed_aircraft_power_losses_w": geometry["low_speed_aircraft_power_losses_w"],
        "low_speed_energy_wh": slow_energy_wh,
        "battery_cruise_segment_min": config.battery_cruise_segment_min,
        "battery_slow_segment_min": config.battery_slow_segment_min,
        "battery_cruise_energy_wh": battery_cruise_energy_wh,
        "battery_slow_energy_wh": battery_slow_energy_wh,
        "battery_usable_energy_wh": battery_usable_energy_wh,
        "battery_reserve_fraction": mission.battery_reserve_fraction,
        "battery_capacity_required_wh": battery_capacity_required_wh,
        "battery_pack_voltage_v": battery_pack_voltage_v,
        "battery_capacity_required_mah": battery_capacity_required_mah,
        "battery_mass_required_kg": battery_mass_required_kg,
        **mission_regime_summary,
        "low_speed_rpm": geometry["low_speed_rpm"],
        "low_speed_rpm_feasible": geometry["low_speed_rpm_feasible"],
        "low_speed_thrust_required_n": geometry["low_speed_thrust_required_n"],
        "low_speed_throttle_percent": geometry["low_speed_throttle_percent"],
        "low_speed_ct": geometry["low_speed_ct"],
        "low_speed_cp": geometry["low_speed_cp"],
        "cruise_drag_n": geometry["cruise_drag_n"],
        "cruise_wing_profile_drag_n": geometry["cruise_wing_profile_drag_n"],
        "cruise_wing_induced_drag_n": geometry["cruise_wing_induced_drag_n"],
        "cruise_htail_drag_n": geometry["cruise_htail_drag_n"],
        "cruise_vtail_drag_n": geometry["cruise_vtail_drag_n"],
        "cruise_fuselage_drag_n": geometry["cruise_fuselage_drag_n"],
        "cruise_power_w": geometry["cruise_power_w"],
        "cruise_propulsion_electric_power_without_aircraft_losses_w": geometry[
            "cruise_propulsion_electric_power_without_aircraft_losses_w"
        ],
        "cruise_propulsion_wiring_loss_w": geometry["cruise_propulsion_wiring_loss_w"],
        "cruise_avionics_power_loss_w": geometry["cruise_avionics_power_loss_w"],
        "cruise_aircraft_power_losses_w": geometry["cruise_aircraft_power_losses_w"],
        "cruise_rpm": geometry["cruise_rpm"],
        "cruise_rpm_feasible": geometry["cruise_rpm_feasible"],
        "cruise_thrust_required_n": geometry["cruise_thrust_required_n"],
        "cruise_throttle_percent": geometry["cruise_throttle_percent"],
        "cruise_ct": geometry["cruise_ct"],
        "cruise_cp": geometry["cruise_cp"],
        "cruise_blown_effective_velocity_mps": geometry["cruise_blown_effective_velocity_mps"],
        "cruise_unblown_dynamic_pressure_pa": geometry["cruise_unblown_dynamic_pressure_pa"],
        "cruise_blown_dynamic_pressure_pa": geometry["cruise_blown_dynamic_pressure_pa"],
        "cruise_effective_dynamic_pressure_pa": geometry["cruise_effective_dynamic_pressure_pa"],
        "cruise_blown_to_unblown_q_ratio": geometry["cruise_blown_to_unblown_q_ratio"],
        "cruise_uniform_local_cl_required": geometry["cruise_uniform_local_cl_required"],
        "cruise_alpha_deg": alpha_trim,
        "cruise_body_alpha_proxy_deg": geometry["cruise_alpha_proxy_deg"],
        "cruise_section_alpha_proxy_deg": geometry["cruise_section_alpha_proxy_deg"],
        "cruise_lift_n": vlm_trim.get("L", 0.0),
        "cruise_vlm_induced_drag_n": vlm_trim.get("D", 0.0),
        "cruise_cl_required": mission.gross_weight_n / max(
            0.5 * mission.air_density_kgpm3 * mission.cruise_speed_mps**2 * mission.wing_area_m2,
            1e-9,
        ),
        "cruise_cl": cruise_cl,
        "cruise_cd": cruise_cd,
        "cruise_l_over_d": cruise_cl / max(cruise_cd, 1e-9),
        "cruise_cm": vlm_trim.get("Cm", 0.0),
        "trim_residual_lift_n": vlm_trim.get("trim_residual_lift_n", 0.0),
        "trim_residual_cm": vlm_trim.get("trim_residual_cm", 0.0),
        "section_re_cruise": polars["reynolds_cruise"],
        "section_re_low_speed": polars["reynolds_low"],
        "main_wing_re_cruise": main_wing_re_cruise,
        "main_wing_re_low_speed": main_wing_re_low_speed,
        "htail_re_cruise": htail_re_cruise,
        "htail_re_low_speed": htail_re_low_speed,
        "vtail_re_cruise": vtail_re_cruise,
        "vtail_re_low_speed": vtail_re_low_speed,
        "clean_section_clmax": polars["clean_section_clmax"],
        "flapped_section_clmax": polars["flapped_section_clmax"],
        "geometry_score": geometry["geometry_score"],
        "penalty_score": geometry["penalty_score"],
        "top_view_png": str(top_view_path),
        "three_view_png": str(three_view_path),
        "wireframe_png": str(wireframe_path),
        "render_3d_png": str(render_3d_path),
        "polar_png": str(polar_path),
        "performance_sweep_csv": performance_outputs["performance_sweep_csv"],
        "drag_power_sweeps_png": performance_outputs["drag_power_sweeps_png"],
        "drag_components_png": performance_outputs["drag_components_png"],
        "mesh_npz": str(mesh_path),
        "report_md": str(STAGE3_REPORT_MD),
        **mass_case_outputs,
        **{
            key: value
            for key, value in performance_outputs.items()
            if key.startswith("approach_") or key.startswith("climb_")
        },
    }
