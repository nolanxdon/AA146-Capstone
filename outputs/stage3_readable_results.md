# Stage 3 Results Summary

This is the readable results sheet for the fixed-propulsion Stage 3 AeroSandbox tail-sizing run. The CSV files remain the source for machine-readable data.

## Final Configuration

| Item | Result |
| --- | ---: |
| Status | SUCCESS |
| Warnings | slow_flight_requires_powered_lift;stage12_equivalent_clmax_capped;approach_throttle_saturated |
| Propellers / motors | 10 |
| Propeller diameter | 5.50 in |
| Propeller pitch ratio | 0.40 |
| Propeller family | balanced |
| Propulsion model | eCalc static table: outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_static_partial_load.csv |
| eCalc static CSV | `outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_static_partial_load.csv` |
| Main airfoil | DAE51 |
| Tail airfoil | naca0008 |

## Main Wing

| Dimension | Result |
| --- | ---: |
| Span | 2.000 m |
| Chord | 0.350 m |
| Area | 0.700 m^2 |
| Aspect ratio | 5.714 |
| Incidence | 4.500 deg |
| Washout | 0.000 deg |
| Propeller axial location | -0.250 chord from wing LE |
| Propeller x position | -0.087 m |
| Fuselage nose station | -0.300 m from wing LE |
| Fuselage width | 170 mm |
| Fuselage height | 130 mm |
| Flap span fraction | 0.650 semispan |
| Flap chord fraction | 0.340 chord |
| Slow-flight flap deflection | 40.0 deg |
| Aileron span fraction | 0.280 semispan |
| Aileron chord fraction | 0.280 chord |

## Horizontal Stabilizer

| Dimension | Result |
| --- | ---: |
| Span | 0.606 m |
| Root chord | 0.202 m |
| Tip chord | 0.202 m |
| Taper ratio | 1.000 |
| Area | 0.1225 m^2 |
| Aspect ratio | 3.000 |
| Incidence | -4.747 deg |
| Elevator chord fraction | 0.220 chord |
| Elevator max deflection for sizing | 25.0 deg |
| Slow pitch control target | 0.180 Cm |
| Slow pitch control authority | 0.563 Cm |
| Slow pitch control margin | 212.5% |

## Vertical Stabilizer

| Dimension | Result |
| --- | ---: |
| Height/span | 0.377 m |
| Root chord | 0.152 m |
| Tip chord | 0.137 m |
| Taper ratio | 0.902 |
| Area | 0.0546 m^2 |
| Aspect ratio | 2.600 |
| Incidence | 0.000 deg |
| Rudder chord fraction | 0.353 chord |
| Rudder max deflection for sizing | 25.0 deg |
| Slow yaw control target | 0.055 Cn |
| Slow yaw control authority | 0.055 Cn |
| Slow yaw control margin | 0.0% |

## Stability And Mass

| Metric | Result |
| --- | ---: |
| Tail arm | 1.000 m |
| Horizontal tail volume | 0.500 |
| Horizontal tail volume range | 0.500 to 0.950 |
| Vertical tail volume | 0.039 |
| Vertical tail volume range | 0.039 to 0.041 |
| CG location | 0.088 m |
| CG as percent MAC | 25.00% |
| CG target location | 0.087 m |
| CG target as percent MAC | 25.00% |
| CG error | 0.000000 m |
| CG error as percent MAC | 0.0000% |
| Required pre-tail baseline CG | 19.77% MAC |
| Used pre-tail baseline CG | 19.77% MAC |
| Neutral point | 0.170 m |
| Static margin | 0.237 MAC |
| VLM Cm-alpha | -0.01750 per deg |
| NGX250 foam density | 25.0 kg/m^3 |
| Main wing foam mass estimate | 0.380 kg |
| Horizontal tail foam mass | 0.034 kg |
| Vertical tail foam mass | 0.011 kg |
| Total tail foam mass | 0.045 kg |
| Gross flight mass used for lift/trim | 5.000 kg |
| Component-estimated built mass before ballast/payload | 2.491 kg |
| Remaining mass to gross target | 2.509 kg |
| Mass budget margin | 2.509 kg |

The component-estimated built mass is not the flight mass used in aerodynamics. Stage 3 keeps the aircraft gross flight mass locked at the Stage 1/2 value for lift, trim, climb, descent, and power calculations; the component estimate shows how much mass is currently accounted for by the Stage 1 propulsion/battery/system model plus the Stage 3 foam-tail model. The remaining mass is available for payload, structure not yet itemized, fasteners, wiring, covering, ballast, and contingency.

## Aerodynamic Margins

| Margin / Source | Result |
| --- | ---: |
| Slow-flight flap state | Slotted flaps down |
| CLmax source | `outputs/wing_workflow/dae51/control_surface_sizing/rank01_n10_d5p5_p2p2_balanced_b3/dae51/control_surface_summary.csv` |
| CL curve source | `outputs/wing_workflow/dae51/control_surface_sizing/rank01_n10_d5p5_p2p2_balanced_b3/dae51/total_cl_curve.csv` |
| Stage 1/2 high-lift polar source | `outputs/wing_workflow/dae51/control_surface_sizing/rank01_n10_d5p5_p2p2_balanced_b3/dae51/total_cl_curve.csv` |
| Physical CLmax source | `outputs/wing_workflow/dae51/control_surface_sizing/rank01_n10_d5p5_p2p2_balanced_b3/dae51/flap_sweep.csv` |
| Stage 1/2 clean section CLmax, unblown / blown | 1.418 / 1.433 |
| Stage 1/2 flapped section CLmax, unblown / blown | 2.152 / 2.171 |
| Stage 1/2 weighted blown-panel physical CLmax | 1.876 |
| Raw Stage 1/2 no-flap CLmax | 1.418 at alpha 12.13 deg |
| Raw Stage 1/2 flap-only CLmax | 1.876 at alpha 12.42 deg |
| Raw Stage 1/2 clean blown equivalent CLmax | 5.545 at alpha 12.13 deg |
| Raw Stage 1/2 flap-down blown equivalent CLmax | 5.828 at alpha 12.70 deg |
| Physical clean CLmax used | 1.418 |
| Physical flaps-down blown-panel CLmax used | 1.876 |
| Stage 1/2 equivalent CLmax capped? | True |
| Stall margin factor | 1.20 |
| Unblown flap-down CLmax, freestream equivalent | 1.876 |
| Blown flap-down CLmax, freestream equivalent at solved RPM | 15.985 |
| Blown-panel local physical CLmax | 1.876 |
| Design unblown CLmax after stall margin | 1.564 |
| Design blown equivalent CLmax after stall margin | 13.321 |
| Design blown-panel local CLmax after stall margin | 1.563 |
| Available slow-flight lift | 109.659 N |
| Required lift target with margin | 58.840 N |
| Freestream-reference CL required at Stage 3 slow-flight speed | 7.148 |
| Pressure-weighted uniform local CL required at solved slow RPM | 0.839 |
| Unblown-panel local CL used | 1.564 |
| Blown-panel local CL used | 0.799 |
| Slow-flight dynamic pressure, unblown / blown / effective | 9.8 / 141.7 / 83.5 Pa |
| Slow-flight blown-to-unblown dynamic-pressure ratio | 14.45 |
| Slow-flight wing area, unblown / blown | 0.309 / 0.391 m^2 |
| Local CL margin to design cap | 0.764 |
| Required effective velocity margin | 4.334 m/s |
| Slow-flight feasible with physical CL caps | True |
| Slow-flight lift margin | 50.819 N |
| Slow-flight lift margin percent | 86.4% |
| Equivalent unblown flap-down stall speed with margin | 8.552 m/s |

Stage 3 no longer treats the large Stage 1/2 blown-lift equivalent CL values as physical airfoil CLmax. It reads the Stage 1/2 NeuralFoil-derived flap-sweep section polars first, then uses the YAML clean/flapped caps only as missing-file fallbacks. Propwash enters through the blown-panel dynamic pressure, so the local blown-panel CLmax remains physical while the freestream-referenced blown equivalent CLmax increases. The `1.20` stall factor then requires 20% extra local-CL headroom.

## Performance

| Metric | Result |
| --- | ---: |
| Stage 3 slow-flight speed | 4.00 m/s |
| Upstream Stage 1/2 low-speed setting | 4.00 m/s |
| Slow-flight freestream CL / local CL / RPM | 7.148 / 0.839 / 8457 rpm |
| Required blown velocity | 10.874 m/s |
| Actual blown velocity from solved RPM | 15.208 m/s |
| Slow-flight natural drag before drag devices | 10.519 N |
| Slow-flight wing profile drag | 6.735 N |
| Slow-flight wing induced drag used | 3.618 N |
| Slow-flight induced drag, local blown-panel estimate | 3.618 N |
| Slow-flight induced drag, freestream-equivalent check | 29.621 N |
| Slow-flight blown-lift thrust | 11.793 N |
| Added drag required for no acceleration | 1.274 N |
| Slow-flight steady total drag | 11.793 N |
| Slow steady drag minus cruise drag | 5.675 N |
| Slow-flight fuselage drag increment | 0.108 N |
| Slow-flight electrical power | 188.42 W |
| Slow-flight aircraft electrical losses | 14.14 W |
| Slow-flight energy for configured segment | 4.71 Wh |
| Slow-flight RPM | 8457 rpm |
| Slow-flight CT / CP | 0.1269 / 0.0706 |
| Cruise speed | 10.00 m/s |
| Cruise freestream CL / local CL / RPM | 1.144 / 0.501 / 6976 rpm |
| Cruise blown effective velocity | 18.152 m/s |
| Cruise dynamic pressure, unblown / blown / effective | 61.3 / 201.8 / 139.8 Pa |
| Cruise blown-to-unblown dynamic-pressure ratio | 3.30 |
| Cruise drag | 6.118 N |
| Cruise fuselage drag | 0.674 N |
| Cruise electrical power | 112.10 W |
| Cruise aircraft electrical losses | 8.49 W |
| Power loss factors, wiring / avionics | 1.080 / 1.100 |
| Cruise RPM | 6976 rpm |
| Cruise CT / CP | 0.0987 / 0.0706 |
| Cruise body alpha proxy | 1.790 deg |
| Cruise wing section alpha proxy | 6.290 deg |
| Cruise VLM trim alpha diagnostic | 7.691 deg |
| Cruise CL | 1.144 |
| Cruise CD | 0.1427 |
| Cruise L/D | 8.02 |
| Trim lift residual | -0.000940 N |
| Trim Cm residual | -0.000000 |

## Flight-Condition Operating Values

| Condition | Speed [m/s] | RPM | Thrust [N] | Throttle [%] | Blown velocity [m/s] | q unblown/blown/eff [Pa] | CL free/local | Drag [N] | Power [W] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Cruise, clean | 10.00 | 6976 | 6.240 | 34.8 | 18.15 | 61.3/201.8/139.8 | 1.144/0.501 | 6.118 | 112.10 |
| Slow flight, flaps down | 4.00 | 8457 | 11.789 | 44.9 | 15.21 | 9.8/141.7/83.5 | 7.148/0.839; split 1.564/0.799 | 11.793 | 188.42 |
| Approach/descent, flaps down | 5.00 | 12440 | 27.464 | 100.0 | 21.58 | 15.3/285.2/166.2 | 4.258/0.392 | 45.399 | 592.04 |
| Climb, clean | 10.00 | 10134 | 14.916 | 63.6 | 22.60 | 61.3/312.9/201.9 | 1.124/0.341 | 5.949 | 316.72 |

## Battery Capacity Estimate

| Segment | Duration [min] | Power [W] | Energy [Wh] |
| --- | ---: | ---: | ---: |
| Cruise | 20.0 | 112.10 | 37.37 |
| Slow flight | 1.5 | 188.42 | 4.71 |
| Usable energy subtotal | 21.5 | -- | 42.08 |
| Pack capacity with 20% reserve | -- | -- | 52.60 |

Equivalent 4S LiPo capacity for this conservative sizing case: `3554 mAh` at `14.8 V` nominal.

Estimated battery mass at 180 Wh/kg: `0.292 kg`.

## Mission Regime Energy Table

| Regime | RPM | Time [min] | Total P [W] | Total I [A] | Energy [Wh] | Used [mAh] | % of sized 4S pack |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Takeoff | 10134 | 0.75 | 317 | 21.4 | 3.96 | 268 | 9.41% |
| Loiter pre-drop | 6976 | 10.00 | 112 | 7.6 | 18.68 | 1262 | 44.40% |
| Drop / low-speed | 8457 | 1.00 | 188 | 12.7 | 3.14 | 212 | 7.46% |
| Loiter post-drop | 5948 | 5.00 | 75 | 5.1 | 6.28 | 424 | 14.92% |
| Landing | 7348 | 0.75 | 128 | 8.7 | 1.60 | 108 | 3.81% |
| Total | -- | 17.50 | -- | -- | 33.66 | 2275 | 80.00% |

Required battery for this regime table: `2843 mAh` at `14.8 V` nominal (`4S LiPo`), including `20%` reserve. Usable energy before reserve is `33.66 Wh`; reserve-adjusted capacity is `42.08 Wh`.

Takeoff is estimated from the payload-on climb condition. Landing is estimated from the post-delivery flaps-down slow/flare condition, which avoids using the saturated steep-approach thrust case as the final landing power.

## Payload-On vs Post-Delivery Performance

Payload mass is `2.50 lb` (`1.134 kg`). Battery sizing uses the payload-on case for the whole mission, so a canceled delivery can return home without re-sizing the pack.

| Configuration | Mass [kg] | Cruise P/RPM/throttle | Slow P/RPM/throttle | Approach P/RPM/throttle | Climb P/RPM/throttle |
| --- | ---: | ---: | ---: | ---: | ---: |
| Payload on | 5.000 | 112.1 W / 6976 rpm / 34.8% | 188.4 W / 8457 rpm / 44.9% | 592.0 W / 12440 rpm / 100.0% | 316.7 W / 10134 rpm / 63.6% |
| Post delivery | 3.866 | 75.3 W / 5948 rpm / 23.5% | 128.4 W / 7348 rpm / 33.2% | 592.0 W / 12440 rpm / 100.0% | 213.9 W / 8857 rpm / 46.7% |

## Main-Wing Stall / Pitch-Up Margin

Stage 3 assumes a `1.25x` local-CL pitch-up margin for executive review. This means the aircraft should operate at no more than `80%` of local CLmax in clean pitch-up review conditions.

| Case | Cruise pitch-up margin | Climb pitch-up margin |
| --- | ---: | ---: |
| Payload on | 126.5% | 232.7% |
| Post delivery | 156.6% | 273.5% |

## Reynolds Numbers

Reynolds numbers are computed as `Re = rho * V * characteristic_chord / mu`. The main wing uses the fixed wing chord; the H-tail and V-tail use their trapezoidal mean aerodynamic chords.

| Aero surface | Cruise Re | Slow-flight Re | Characteristic chord |
| --- | ---: | ---: | --- |
| Main wing | 236878 | 94751 | main wing chord |
| Horizontal stabilizer | 136747 | 54699 | H-tail MAC |
| Vertical stabilizer | 98166 | 39266 | V-tail MAC |

Fuselage drag is included as an equivalent parasite drag area: `D_fuselage = q * CdA`, with `CdA = 0.011 m^2`. This reflects the updated 170 mm x 130 mm fuselage cross-section with a modest bluff-body drag allowance. At cruise this is added directly to the drag buildup; at slow flight the same equivalent area is added as an explicit increment on top of the Stage 1 flap-down baseline and Stage 3 tail drag.

Slow-flight drag is now force-balanced against the blown-lift propeller thrust. The natural airframe drag is still reported, but the headline slow-flight total drag includes the added drag required so the propeller can generate the needed slipstream without accelerating the aircraft. This added drag is the first-pass sizing target for future airbrakes or other drag devices.

For slow-flight induced drag, Stage 3 now uses the physical split-panel buildup: unblown-panel induced drag plus blown-panel induced drag. The freestream-equivalent finite-wing value is still reported as a diagnostic check, but it is not used in the force balance.

## Drag Components

| Component | Clean cruise [N] | Flaps-down slow flight [N] |
| --- | ---: | ---: |
| Main wing profile | 0.528 | 6.735 |
| Main wing induced, used | 4.739 | 3.618 |
| Horizontal tail | 0.136 | 0.042 |
| Vertical tail | 0.040 | 0.016 |
| Fuselage | 0.674 | 0.108 |
| Added drag / airbrakes | 0.000 | 1.274 |
| Total | 6.118 | 11.793 |

| Slow-flight induced-drag diagnostic | Drag [N] |
| --- | ---: |
| Local blown-panel estimate | 3.618 |
| Freestream-equivalent finite-wing check | 29.621 |
| Value used in force balance | 3.618 |

Drag-components chart: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_drag_components.png`

## Approach Estimate

The approach model assumes steady flaps-down descent at the target sink rate in the YAML file, clipped by the configured maximum descent rate and by the selected approach airspeed. The climb model checks the configured maximum climb rate in clean configuration.

| Quantity | Result |
| --- | ---: |
| Target approach sink rate | 6.00 ft/s |
| Actual approach sink rate | 6.00 ft/s |
| Sink-rate target met? | True |
| Resulting approach angle | 21.45 deg below horizontal |
| Approach speed | 5.00 m/s |
| Flap deflection | 40.0 deg |
| Approach alpha | 13.06 deg |
| Required CL | 4.258 |
| Required CL / RPM | 4.258 / 12440 rpm |
| Approach drag | 45.399 N |
| Required thrust | 27.464 N |
| Electrical power | 592.04 W |
| RPM | 12440 rpm |
| Throttle estimate | 100.0% |
| Propulsion feasible at this approach angle | False |
| Elevator trim | 15.25 deg |
| Rudder trim | 0.00 deg |
| Max descent rate limit | 6.0 ft/s |
| Sink rate | 1.829 m/s (6.00 ft/s) |
| Descent rate limited? | False |
| Glide-ratio equivalent | 2.54:1 |

## Climb Trim Estimate

| Quantity | Result |
| --- | ---: |
| Target climb rate | 6.0 ft/s |
| Climb speed | 10.00 m/s |
| Climb angle | 10.54 deg |
| Required CL | 1.124 |
| Required CL / RPM | 1.124 / 10134 rpm |
| Climb drag | 5.949 N |
| Required thrust | 14.916 N |
| Electrical power | 316.72 W |
| RPM | 10134 rpm |
| Throttle estimate | 63.6% |
| Elevator trim | 4.86 deg |
| Propulsion feasible at target climb rate | True |

## Sweep Charts

- Drag and power chart: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_drag_power_sweeps.png`
- Sweep data CSV: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_performance_sweep.csv`

## Output Files

- Editable Stage 3 constraints: `optimizer/config/stage3_constraints.yaml`
- Full CSV results: `outputs/stage3_aerosandbox_results.csv`
- Top-design CSV: `outputs/stage3_aerosandbox_top_designs.csv`
- Technical report: `outputs/stage3_aerosandbox_report.md`
- Top view: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_top_view.png`
- Three view: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_three_view.png`
- Wireframe: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_wireframe.png`
- 3D wireframe render: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_3d_wireframe.png`
- Section polar plot: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_polars.png`
- Drag and power sweep chart: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_drag_power_sweeps.png`
- Drag components chart: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_drag_components.png`
- Performance sweep CSV: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_performance_sweep.csv`
- Mesh file: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_mesh.npz`
