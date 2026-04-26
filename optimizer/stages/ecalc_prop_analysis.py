from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from optimizer.core.data_models import Stage1Candidate, Stage1MissionConfig
from optimizer.core.physics import evaluate_stage1_candidate
from optimizer.core.workflow_style import MIT_RED_SWEEP


G = 9.80665
RHO = 1.225
PROP_DIAMETER_IN = 5.5
PROP_DIAMETER_M = PROP_DIAMETER_IN * 0.0254
PROP_PITCH_IN = 3.5
PROP_BLADE_COUNT = 3
N_PROPS = 10
BATTERY_VOLTAGE_V = 11.1
FLIGHT_SPEED_KMH_DYNAMIC = 37.0
FLIGHT_SPEED_MPS_DYNAMIC = FLIGHT_SPEED_KMH_DYNAMIC / 3.6
PITCH_SPEED_MARGIN_FACTOR = 1.15

OUTPUT_DIR = Path("outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b")


@dataclass(frozen=True)
class EcalcStaticPoint:
    rpm: float
    pitch_speed_kmh: float
    throttle_percent: float
    current_a: float
    voltage_v: float
    electric_power_w: float
    efficiency_percent: float
    thrust_g: float

    @property
    def shaft_power_w(self) -> float:
        return self.electric_power_w * (self.efficiency_percent / 100.0)

    @property
    def thrust_n(self) -> float:
        return self.thrust_g / 1000.0 * G

    @property
    def n_rev_per_sec(self) -> float:
        return self.rpm / 60.0

    @property
    def ct(self) -> float:
        return self.thrust_n / (RHO * self.n_rev_per_sec**2 * PROP_DIAMETER_M**4)

    @property
    def cp(self) -> float:
        return self.shaft_power_w / (RHO * self.n_rev_per_sec**3 * PROP_DIAMETER_M**5)


STATIC_POINTS: tuple[EcalcStaticPoint, ...] = (
    EcalcStaticPoint(3200, 17, 22, 0.1, 11.1, 1.2, 59.0, 19),
    EcalcStaticPoint(4000, 21, 27, 0.2, 11.1, 2.0, 67.0, 30),
    EcalcStaticPoint(4800, 26, 33, 0.3, 11.1, 3.3, 71.9, 43),
    EcalcStaticPoint(5600, 30, 39, 0.5, 11.1, 5.0, 74.9, 58),
    EcalcStaticPoint(6400, 34, 45, 0.7, 11.0, 7.2, 76.8, 76),
    EcalcStaticPoint(7200, 38, 52, 0.9, 11.0, 10.2, 77.8, 96),
    EcalcStaticPoint(8000, 43, 58, 1.3, 11.0, 13.8, 78.3, 119),
    EcalcStaticPoint(8800, 47, 65, 1.7, 11.0, 18.4, 78.5, 144),
    EcalcStaticPoint(9600, 51, 72, 2.2, 10.9, 23.9, 78.4, 171),
    EcalcStaticPoint(10400, 55, 80, 2.8, 10.9, 30.5, 78.2, 201),
    EcalcStaticPoint(11200, 60, 87, 3.6, 10.8, 38.2, 77.8, 233),
    EcalcStaticPoint(12000, 64, 95, 4.4, 10.8, 47.3, 77.3, 267),
    EcalcStaticPoint(12440, 66, 100, 5.1, 10.7, 53.8, 76.2, 287),
)


def _dynamic_design_point() -> dict[str, float]:
    rpm = 12440.0
    n_rev_per_sec = rpm / 60.0
    thrust_g = 238.0
    thrust_n = thrust_g / 1000.0 * G
    shaft_power_w = 41.0
    advance_ratio = FLIGHT_SPEED_MPS_DYNAMIC / (n_rev_per_sec * PROP_DIAMETER_M)
    ct = thrust_n / (RHO * n_rev_per_sec**2 * PROP_DIAMETER_M**4)
    cp = shaft_power_w / (RHO * n_rev_per_sec**3 * PROP_DIAMETER_M**5)
    return {
        "flight_speed_kmh": FLIGHT_SPEED_KMH_DYNAMIC,
        "flight_speed_mps": FLIGHT_SPEED_MPS_DYNAMIC,
        "rpm": rpm,
        "thrust_g": thrust_g,
        "thrust_n": thrust_n,
        "shaft_power_w": shaft_power_w,
        "advance_ratio": advance_ratio,
        "ct": ct,
        "cp": cp,
    }


def _build_stage1_result(gross_mass_kg: float, cruise_speed_mps: float) -> dict[str, float]:
    candidate = Stage1Candidate(
        n_props=N_PROPS,
        prop_diameter_in=PROP_DIAMETER_IN,
        prop_pitch_ratio=PROP_PITCH_IN / PROP_DIAMETER_IN,
        prop_family="balanced",
    )
    mission = Stage1MissionConfig(
        gross_mass_kg=gross_mass_kg,
        max_mass_kg=5.0,
        battery_voltage_v=BATTERY_VOLTAGE_V,
        cruise_speed_mps=cruise_speed_mps,
        loiter_time_min=15.0,
    )
    # Preserve compatibility with the legacy typo in the frozen dataclass.
    object.__setattr__(mission, "k_span_expansion", mission.k_span_e8xpansion)
    result = evaluate_stage1_candidate(mission, candidate)

    q_low = 0.5 * mission.air_density_kgpm3 * mission.low_speed_mps**2
    q_cruise = 0.5 * mission.air_density_kgpm3 * cruise_speed_mps**2
    cl_req_low = mission.gross_weight_n / (q_low * mission.wing_area_m2)
    cl_req_cruise = mission.gross_weight_n / (q_cruise * mission.wing_area_m2)
    return {
        "gross_mass_kg": gross_mass_kg,
        "gross_weight_n": mission.gross_weight_n,
        "wing_area_m2": mission.wing_area_m2,
        "low_speed_mps": mission.low_speed_mps,
        "cruise_speed_mps": cruise_speed_mps,
        "cl_req_low": cl_req_low,
        "cl_req_cruise": cl_req_cruise,
        "low_speed_drag_n": result.low_speed_drag_n,
        "low_speed_thrust_n": result.low_speed_thrust_n,
        "low_speed_veff_mps": result.low_speed_veff_mps,
        "low_speed_required_veff_mps": result.low_speed_required_veff_mps,
        "solved_low_speed_rpm": result.solved_low_speed_rpm,
        "low_speed_power_w": result.low_speed_power_w,
        "cruise_drag_n": result.cruise_drag_n,
        "cruise_thrust_n": result.cruise_thrust_n,
        "solved_cruise_rpm": result.solved_cruise_rpm,
        "cruise_power_w": result.cruise_power_w,
        "low_speed_cmu": result.low_speed_momentum_coefficient,
    }


def _interp_from_static(x_values: list[float], y_values: list[float], target_y: float) -> float:
    if target_y <= y_values[0]:
        return x_values[0]
    if target_y >= y_values[-1]:
        return x_values[-1]
    for idx in range(len(y_values) - 1):
        y0 = y_values[idx]
        y1 = y_values[idx + 1]
        if y0 <= target_y <= y1:
            x0 = x_values[idx]
            x1 = x_values[idx + 1]
            frac = (target_y - y0) / max(y1 - y0, 1e-12)
            return x0 + frac * (x1 - x0)
    return x_values[-1]


def _rpm_from_static_thrust(thrust_per_prop_g: float) -> float:
    rpms = [p.rpm for p in STATIC_POINTS]
    thrusts = [p.thrust_g for p in STATIC_POINTS]
    return _interp_from_static(rpms, thrusts, thrust_per_prop_g)


def _rpm_from_pitch_speed(required_pitch_speed_kmh: float) -> float:
    rpms = [p.rpm for p in STATIC_POINTS]
    pitch_speeds = [p.pitch_speed_kmh for p in STATIC_POINTS]
    return _interp_from_static(rpms, pitch_speeds, required_pitch_speed_kmh)


def _power_from_rpm(rpm: float) -> float:
    rpms = [p.rpm for p in STATIC_POINTS]
    powers = [p.electric_power_w * N_PROPS for p in STATIC_POINTS]
    if rpm <= rpms[0]:
        return powers[0]
    if rpm >= rpms[-1]:
        return powers[-1]
    for idx in range(len(rpms) - 1):
        x0 = rpms[idx]
        x1 = rpms[idx + 1]
        if x0 <= rpm <= x1:
            y0 = powers[idx]
            y1 = powers[idx + 1]
            frac = (rpm - x0) / max(x1 - x0, 1e-12)
            return y0 + frac * (y1 - y0)
    return powers[-1]


def _write_static_csv() -> Path:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    csv_path = OUTPUT_DIR / "ecalc_static_partial_load.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "rpm",
                "pitch_speed_kmh",
                "throttle_percent",
                "current_a",
                "voltage_v",
                "electric_power_w",
                "efficiency_percent",
                "shaft_power_w",
                "thrust_g",
                "thrust_n",
                "ct_static",
                "cp_static",
            ]
        )
        for point in STATIC_POINTS:
            writer.writerow(
                [
                    f"{point.rpm:.0f}",
                    f"{point.pitch_speed_kmh:.1f}",
                    f"{point.throttle_percent:.1f}",
                    f"{point.current_a:.3f}",
                    f"{point.voltage_v:.3f}",
                    f"{point.electric_power_w:.3f}",
                    f"{point.efficiency_percent:.3f}",
                    f"{point.shaft_power_w:.3f}",
                    f"{point.thrust_g:.3f}",
                    f"{point.thrust_n:.6f}",
                    f"{point.ct:.6f}",
                    f"{point.cp:.6f}",
                ]
            )
    return csv_path


def _write_dynamic_csv(dynamic_point: dict[str, float]) -> Path:
    csv_path = OUTPUT_DIR / "ecalc_dynamic_design_point.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "flight_speed_kmh",
                "flight_speed_mps",
                "rpm",
                "thrust_g",
                "thrust_n",
                "shaft_power_w",
                "advance_ratio",
                "ct",
                "cp",
            ]
        )
        writer.writerow(
            [
                f"{dynamic_point['flight_speed_kmh']:.3f}",
                f"{dynamic_point['flight_speed_mps']:.6f}",
                f"{dynamic_point['rpm']:.3f}",
                f"{dynamic_point['thrust_g']:.3f}",
                f"{dynamic_point['thrust_n']:.6f}",
                f"{dynamic_point['shaft_power_w']:.6f}",
                f"{dynamic_point['advance_ratio']:.6f}",
                f"{dynamic_point['ct']:.6f}",
                f"{dynamic_point['cp']:.6f}",
            ]
        )
    return csv_path


def _plot_ct_cp(dynamic_point: dict[str, float]) -> tuple[Path, Path]:
    rpm = np.asarray([p.rpm for p in STATIC_POINTS], dtype=float)
    ct = np.asarray([p.ct for p in STATIC_POINTS], dtype=float)
    cp = np.asarray([p.cp for p in STATIC_POINTS], dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(9.0, 7.0), sharex=True)
    axes[0].plot(rpm, ct, color=MIT_RED_SWEEP[-2], linewidth=2.5)
    axes[0].scatter(
        [dynamic_point["rpm"]],
        [dynamic_point["ct"]],
        color="#7f1d1d",
        s=60,
        zorder=3,
        label=f"Dynamic @ {dynamic_point['flight_speed_kmh']:.0f} km/h",
    )
    axes[0].set_ylabel("C_T")
    axes[0].set_title("eCalc-Calibrated Static C_T vs RPM")
    axes[0].grid(True, alpha=0.25)
    axes[0].legend(loc="best")

    axes[1].plot(rpm, cp, color=MIT_RED_SWEEP[-1], linewidth=2.5)
    axes[1].scatter(
        [dynamic_point["rpm"]],
        [dynamic_point["cp"]],
        color="#7f1d1d",
        s=60,
        zorder=3,
        label=f"Dynamic @ {dynamic_point['flight_speed_kmh']:.0f} km/h",
    )
    axes[1].set_xlabel("RPM")
    axes[1].set_ylabel("C_P")
    axes[1].set_title("eCalc-Calibrated Static C_P vs RPM")
    axes[1].grid(True, alpha=0.25)
    axes[1].legend(loc="best")
    fig.tight_layout()
    ct_cp_path = OUTPUT_DIR / "ct_cp_vs_rpm.png"
    fig.savefig(ct_cp_path, dpi=220, bbox_inches="tight")
    plt.close(fig)

    fig2, axes2 = plt.subplots(2, 1, figsize=(9.0, 7.0), sharex=True)
    axes2[0].plot(rpm, [p.thrust_g for p in STATIC_POINTS], color=MIT_RED_SWEEP[-2], linewidth=2.5)
    axes2[0].set_ylabel("Thrust per Prop [g]")
    axes2[0].set_title("eCalc Static Thrust vs RPM")
    axes2[0].grid(True, alpha=0.25)

    axes2[1].plot(rpm, [p.electric_power_w for p in STATIC_POINTS], color=MIT_RED_SWEEP[-1], linewidth=2.5)
    axes2[1].set_xlabel("RPM")
    axes2[1].set_ylabel("Electric Power per Prop [W]")
    axes2[1].set_title("eCalc Static Electric Power vs RPM")
    axes2[1].grid(True, alpha=0.25)
    fig2.tight_layout()
    thrust_power_path = OUTPUT_DIR / "thrust_power_vs_rpm.png"
    fig2.savefig(thrust_power_path, dpi=220, bbox_inches="tight")
    plt.close(fig2)
    return ct_cp_path, thrust_power_path


def _write_mission_summary(
    dynamic_point: dict[str, float],
    pre_drop: dict[str, float],
    post_drop_low: dict[str, float],
    post_drop_loiter: dict[str, float],
) -> Path:
    mass_drop_kg = 2.5 * 0.45359237
    post_drop_mass_kg = 5.0 - mass_drop_kg

    dae51_clmax = 5.809968664868988
    q_low = 0.5 * RHO * pre_drop["low_speed_mps"] ** 2
    q_low_s = q_low * pre_drop["wing_area_m2"]
    cl_req_low_post = post_drop_mass_kg * G / q_low_s

    pre_low_thrust_per_prop_g = pre_drop["low_speed_thrust_n"] / N_PROPS / G * 1000.0
    post_low_thrust_per_prop_g = post_drop_low["low_speed_thrust_n"] / N_PROPS / G * 1000.0
    pre_low_rpm_static = _rpm_from_static_thrust(pre_low_thrust_per_prop_g)
    post_low_rpm_static = _rpm_from_static_thrust(post_low_thrust_per_prop_g)
    pre_cruise_thrust_per_prop_g = pre_drop["cruise_thrust_n"] / N_PROPS / G * 1000.0
    post_loiter_thrust_per_prop_g = post_drop_loiter["cruise_thrust_n"] / N_PROPS / G * 1000.0
    pre_cruise_rpm_thrust = _rpm_from_static_thrust(pre_cruise_thrust_per_prop_g)
    post_loiter_rpm_thrust = _rpm_from_static_thrust(post_loiter_thrust_per_prop_g)

    pre_cruise_required_pitch_kmh = pre_drop["cruise_speed_mps"] * 3.6 * PITCH_SPEED_MARGIN_FACTOR
    post_loiter_required_pitch_kmh = post_drop_loiter["cruise_speed_mps"] * 3.6 * PITCH_SPEED_MARGIN_FACTOR
    pre_cruise_rpm_pitch = _rpm_from_pitch_speed(pre_cruise_required_pitch_kmh)
    post_loiter_rpm_pitch = _rpm_from_pitch_speed(post_loiter_required_pitch_kmh)

    mission_schedules = [
        {
            "name": "endurance_lean",
            "segments": [
                ("Takeoff", 11200.0, 0.75),
                ("Loiter pre-drop", 8000.0, 10.0),
                ("Drop / low-speed", 9600.0, 1.0),
                ("Loiter post-drop", 7200.0, 5.0),
                ("Landing", 9600.0, 0.75),
            ],
        },
        {
            "name": "margin_rich",
            "segments": [
                ("Takeoff", 11200.0, 1.0),
                ("Loiter pre-drop", 8800.0, 10.0),
                ("Drop / low-speed", 10400.0, 1.5),
                ("Loiter post-drop", 8000.0, 5.0),
                ("Landing", 10400.0, 1.0),
            ],
        },
    ]
    usable_pack_energy_wh = 55.5 * 0.85
    for schedule in mission_schedules:
        total_wh = 0.0
        for _, rpm, minutes in schedule["segments"]:
            total_wh += _power_from_rpm(rpm) * (minutes / 60.0)
        schedule["total_wh"] = total_wh
        schedule["usable_margin_wh"] = usable_pack_energy_wh - total_wh

    summary_path = OUTPUT_DIR / "mission_summary.md"
    with summary_path.open("w", encoding="utf-8") as f:
        f.write("# eCalc / Blown-Wing Mission Analysis\n\n")
        f.write("## Configuration\n")
        f.write(f"- Motor: `SunnySky X2302-1500 V3`\n")
        f.write(f"- Battery: `3S 5000 mAh`\n")
        f.write(f"- Propeller: `{PROP_DIAMETER_IN:.1f} x {PROP_PITCH_IN:.1f} in`, `{PROP_BLADE_COUNT}-blade`\n")
        f.write(f"- Propulsors: `{N_PROPS}`\n")
        f.write(f"- Dynamic eCalc design point used: `{FLIGHT_SPEED_KMH_DYNAMIC:.0f} km/h`\n\n")

        f.write("## Static Coefficient Calibration\n")
        f.write(f"- Mean static `C_T` from the partial-load table: `{np.mean([p.ct for p in STATIC_POINTS]):.4f}`\n")
        f.write(f"- Mean static `C_P` from the partial-load table: `{np.mean([p.cp for p in STATIC_POINTS]):.4f}`\n")
        f.write(f"- Dynamic full-throttle point at `{dynamic_point['flight_speed_kmh']:.0f} km/h`: `J = {dynamic_point['advance_ratio']:.3f}`, `C_T = {dynamic_point['ct']:.4f}`, `C_P = {dynamic_point['cp']:.4f}`\n\n")

        f.write("## Low-Speed Drag Budget\n")
        f.write("### Before package drop (`5.0 kg`, `4 m/s`)\n")
        f.write(f"- Required whole-wing `C_L`: `{pre_drop['cl_req_low']:.3f}`\n")
        f.write(f"- Stage 1 steady low-speed drag: `{pre_drop['low_speed_drag_n']:.3f} N`\n")
        f.write(f"- Thrust required including the 5% margin / blown-velocity closure: `{pre_drop['low_speed_thrust_n']:.3f} N`\n")
        f.write(f"- Required effective blown velocity: `{pre_drop['low_speed_required_veff_mps']:.3f} m/s`\n")
        f.write(f"- Solver low-speed RPM for this prop family: `{pre_drop['solved_low_speed_rpm']:.0f} rpm`\n")
        f.write(f"- eCalc static-thrust RPM lower bound from the same thrust level: `{pre_low_rpm_static:.0f} rpm`\n")
        f.write("- Interpretation: this is the hard low-speed mission point; blowing is still doing the real work here.\n\n")

        f.write("### After package drop (`3.866 kg`, `4 m/s`)\n")
        f.write(f"- Required whole-wing `C_L`: `{cl_req_low_post:.3f}`\n")
        f.write(f"- Current DAE51 all-high-lift `C_L,max`: `{dae51_clmax:.3f}`\n")
        f.write(f"- Stage 1 steady low-speed drag: `{post_drop_low['low_speed_drag_n']:.3f} N`\n")
        f.write(f"- Thrust required including blown-velocity closure: `{post_drop_low['low_speed_thrust_n']:.3f} N`\n")
        f.write(f"- Required effective blown velocity: `{post_drop_low['low_speed_required_veff_mps']:.3f} m/s`\n")
        f.write(f"- eCalc static-thrust RPM lower bound from the same thrust level: `{post_low_rpm_static:.0f} rpm`\n")
        f.write("- Interpretation: after the drop, the `4 m/s` condition becomes much closer to the current DAE51 whole-wing capability, so landing is materially easier than the outbound low-speed segment.\n\n")

        f.write("## Cruise / Loiter Operating Points\n")
        f.write("### Outbound loiter (`5.0 kg`, `10 m/s`)\n")
        f.write(f"- Required steady drag: `{pre_drop['cruise_drag_n']:.3f} N`\n")
        f.write(f"- Thrust required with the Stage 1 cruise margin: `{pre_drop['cruise_thrust_n']:.3f} N`\n")
        f.write(f"- Static-thrust RPM lower bound from thrust alone: `{pre_cruise_rpm_thrust:.0f} rpm`\n")
        f.write(f"- Pitch-speed-margin RPM using `1.15 x V_cruise`: `{pre_cruise_rpm_pitch:.0f} rpm`\n")
        f.write("- Recommendation: use the pitch-speed-based number; it is the cleaner cruise input for the solver.\n\n")

        f.write("### Return loiter after drop (`3.866 kg`, `9.0 m/s`)\n")
        f.write(f"- Required steady drag: `{post_drop_loiter['cruise_drag_n']:.3f} N`\n")
        f.write(f"- Thrust required with the Stage 1 cruise margin: `{post_drop_loiter['cruise_thrust_n']:.3f} N`\n")
        f.write(f"- Static-thrust RPM lower bound from thrust alone: `{post_loiter_rpm_thrust:.0f} rpm`\n")
        f.write(f"- Pitch-speed-margin RPM using `1.15 x V_loiter`: `{post_loiter_rpm_pitch:.0f} rpm`\n")
        f.write("- Recommendation: a lighter post-drop endurance loiter around `8.5-9.0 m/s` is a better mission speed than holding `10 m/s` all the way home.\n\n")

        f.write("## eCalc Flight Speeds To Run\n")
        f.write("- `0 km/h`: cleanest static `C_T` / `C_P` calibration table.\n")
        f.write("- `14.4 km/h`: direct low-speed blown-flight calibration (`4.0 m/s`).\n")
        f.write("- `36.0 km/h`: direct nominal cruise calibration (`10.0 m/s`).\n")
        f.write("- `31-32 km/h`: post-drop endurance loiter calibration (`8.5-9.0 m/s`).\n")
        f.write("- If you only want one dynamic eCalc run beyond static, use `36 km/h`, because the solver already treats the low-speed point with a separate blown-velocity closure.\n\n")

        f.write("## Conservative Mission Energy Check\n")
        f.write("- Assumption: use the static eCalc partial-load powers as conservative stand-ins for segment power.\n")
        f.write(f"- Usable battery energy at `85%` of a `55.5 Wh` pack: `{usable_pack_energy_wh:.2f} Wh`\n")
        for schedule in mission_schedules:
            f.write(f"- `{schedule['name']}` schedule: `{schedule['total_wh']:.2f} Wh` total, margin `{schedule['usable_margin_wh']:.2f} Wh`\n")
        f.write("- Interpretation: the mission can close only if the loiter phases are run near the lower-RPM schedule; the higher-margin schedule is too energy-expensive for this pack.\n")
    return summary_path


def run() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    dynamic_point = _dynamic_design_point()
    static_csv = _write_static_csv()
    dynamic_csv = _write_dynamic_csv(dynamic_point)
    ct_cp_plot, thrust_power_plot = _plot_ct_cp(dynamic_point)
    pre_drop = _build_stage1_result(gross_mass_kg=5.0, cruise_speed_mps=10.0)
    post_drop_low = _build_stage1_result(gross_mass_kg=5.0 - 2.5 * 0.45359237, cruise_speed_mps=10.0)
    post_drop_loiter = _build_stage1_result(gross_mass_kg=5.0 - 2.5 * 0.45359237, cruise_speed_mps=9.0)
    summary_path = _write_mission_summary(dynamic_point, pre_drop, post_drop_low, post_drop_loiter)

    print("Wrote eCalc prop analysis:")
    print(f"  static table: {static_csv}")
    print(f"  dynamic point: {dynamic_csv}")
    print(f"  CT/CP plot: {ct_cp_plot}")
    print(f"  thrust/power plot: {thrust_power_plot}")
    print(f"  mission summary: {summary_path}")


if __name__ == "__main__":
    run()
