# Stage 3 AeroSandbox Fixed-Wing Tail Refinement

## Scope

Stage 3 freezes the selected Stage 1/2 main wing and propulsion layout, then sizes the horizontal and vertical tail using AeroSandbox geometry, VLM trim/stability checks, NeuralFoil section data, prop RPM re-solves, and an explicit NGX250 foam material model.

## Fixed Inputs

- Gross flight mass for lift balance: `5.000 kg`
- Maximum as-built mass: `5.000 kg`
- Main wing span: `2.000 m`
- Main wing chord: `0.350 m`
- Main wing area: `0.700 m^2`
- Fuselage nose station: `-0.300 m` from wing leading edge
- Fuselage cross-section: `170 mm` wide x `130 mm` tall
- Stage 3 slow-flight mode: `4.000 m/s`
- Upstream Stage 1/2 low-speed mode remains: `4.000 m/s`
- Cruise mode: `10.000 m/s`
- Foam density: `25.0 kg/m^3`
- eCalc propulsion calibration enabled: `True`

## Optimizer Notes

- Editable Stage 3 constraints are loaded from `optimizer/config/stage3_constraints.yaml` when that YAML file is present.
- Main wing dimensions and selected airfoil are not optimized in Stage 3.
- Horizontal and vertical stabilizer planform dimensions are optimized primarily for cruise trim, cruise drag, material mass, static margin, and the user-specified tail-volume ranges.
- Elevator and rudder chord fractions are sized separately after planform optimization to meet slow-flight pitch and yaw authority targets.
- Slow-flight sizing is evaluated with the frozen slotted flaps deployed.
- Raw Stage 1/2 equivalent CLmax values are read for traceability. Physical local CLmax limits come from the Stage 1/2 NeuralFoil flap-sweep section polars when available; the YAML values are only fallbacks. Blown lift is modeled through higher local dynamic pressure, not through unphysical airfoil CLmax.
- Fuselage drag is included as an equivalent parasite drag area, `CdA = 0.011 m^2`, so `D_fuse = q * CdA`.
- Tail foam mass is added to the Stage 1/2 built-mass estimate before checking the mass margin.
- Tail sizing is constrained by horizontal/vertical tail volume, static margin, trim incidence, material mass, and prop RPM feasibility.
- Cruise power and slow-flight power are both re-solved. If the frozen propulsion layout matches the eCalc calibration files, Stage 3 uses the eCalc thrust/power table and dynamic CT correction; otherwise it falls back to the Stage 1 generic prop surrogate.

## Recommended Aircraft Dimensions

| Quantity | Value |
| --- | ---: |
| Propulsion layout | 10 x 5.5 in, P/D 0.4, balanced |
| Propulsion model | eCalc static table: outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_static_partial_load.csv |
| eCalc static CSV | outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_static_partial_load.csv |
| Main airfoil | DAE51 |
| Main wing span | 2.000 m |
| Main wing chord | 0.350 m |
| Main wing incidence | 4.500 deg |
| Cruise body alpha proxy | 1.790 deg |
| Cruise wing section alpha proxy | 6.290 deg |
| Cruise VLM trim alpha diagnostic | 7.691 deg |
| Propeller axial x position | -0.087 m |
| Fuselage nose station | -0.300 m from wing LE |
| Fuselage width x height | 170 mm x 130 mm |
| Horizontal tail span | 0.606 m |
| Horizontal tail root chord | 0.202 m |
| Horizontal tail tip chord | 0.202 m |
| Horizontal tail incidence | -4.747 deg |
| Elevator chord fraction | 0.220 chord |
| Vertical tail height/span | 0.377 m |
| Vertical tail root chord | 0.152 m |
| Vertical tail tip chord | 0.137 m |
| Vertical tail incidence | 0.000 deg |
| Rudder chord fraction | 0.353 chord |
| Tail arm | 1.000 m |
| CG target / actual | 25.00% / 25.00% MAC |
| Required pre-tail baseline CG | 19.77% MAC |
| Static margin | 0.237 MAC |
| H-tail volume range / actual | 0.500-0.950 / 0.500 |
| V-tail volume range / actual | 0.039-0.041 / 0.039 |
| Slow pitch control authority / target | 0.563 / 0.180 Cm |
| Slow yaw control authority / target | 0.055 / 0.055 Cn |
| Gross flight mass used in all lift/trim calculations | 5.000 kg |
| Component-estimated built mass before ballast/payload | 2.491 kg |
| Remaining mass to 5 kg gross target | 2.509 kg |
| Tail foam mass | 0.045 kg |
| Physical CLmax source | outputs/wing_workflow/dae51/control_surface_sizing/rank01_n10_d5p5_p2p2_balanced_b3/dae51/flap_sweep.csv |
| Stage 1/2 clean section CLmax, unblown / blown | 1.418 / 1.433 |
| Stage 1/2 flapped section CLmax, unblown / blown | 2.152 / 2.171 |
| Physical no-flap CLmax used | 1.418 |
| Physical flap-only CLmax used | 1.876 |
| Physical weighted blown-panel CLmax used | 1.876 |
| Raw Stage 1/2 flap-down blown equivalent CLmax | 5.828 |
| Stage 1/2 equivalent CLmax capped? | True |
| Slow-flight flap-down CLmax, unblown equivalent | 1.876 |
| Slow-flight flap-down CLmax, blown equivalent | 15.985 |
| Slow-flight blown-panel local physical CLmax | 1.876 |
| Slow-flight lift margin | 86.4% |
| Cruise fuselage drag | 0.674 N |
| Cruise power | 112.10 W |
| Cruise freestream CL / local blown-wing CL / RPM | 1.144 / 0.501 / 6976 rpm |
| Cruise q unblown / blown / effective | 61.3 / 201.8 / 139.8 Pa |
| Cruise blown effective velocity | 18.152 m/s |
| Cruise CT / CP | 0.0987 / 0.0706 |
| Power loss factors, wiring / avionics | 1.080 / 1.100 |
| Cruise aircraft electrical losses | 8.49 W |
| Slow-flight power | 188.42 W |
| Slow-flight freestream CL / uniform local CL / RPM | 7.148 / 0.839 / 8457 rpm |
| Slow-flight local CL, unblown / blown panels | 1.564 / 0.799 |
| Slow-flight q unblown / blown / effective | 9.8 / 141.7 / 83.5 Pa |
| Slow-flight CT / CP | 0.1269 / 0.0706 |
| Slow-flight aircraft electrical losses | 14.14 W |
| Slow-flight natural drag before added drag | 10.519 N |
| Slow-flight wing profile drag | 6.735 N |
| Slow-flight wing induced drag used | 3.618 N |
| Slow-flight induced drag, local blown-panel estimate | 3.618 N |
| Slow-flight induced drag, freestream-equivalent check | 29.621 N |
| Slow-flight added drag required | 1.274 N |
| Slow-flight steady total drag | 11.793 N |
| Slow-flight steady drag minus cruise drag | 5.675 N |
| Main wing Reynolds number, cruise / slow | 236878 / 94751 |
| H-tail Reynolds number, cruise / slow | 136747 / 54699 |
| V-tail Reynolds number, cruise / slow | 98166 / 39266 |
| Approach target sink rate | 6.00 ft/s |
| Actual approach sink rate | 6.00 ft/s |
| Approach sink-rate target met? | True |
| Approach descent angle | 21.45 deg below horizontal |
| Approach speed | 5.00 m/s |
| Approach/descent CL required / RPM | 4.258 / 12440 rpm |
| Approach elevator trim | 15.25 deg |
| Approach throttle estimate | 100.0% |

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

## Drag Components

| Component | Clean cruise drag [N] | Flaps-down slow-flight drag [N] |
| --- | ---: | ---: |
| Main wing profile | 0.528 | 6.735 |
| Main wing induced, used | 4.739 | 3.618 |
| Horizontal tail | 0.136 | 0.042 |
| Vertical tail | 0.040 | 0.016 |
| Fuselage | 0.674 | 0.108 |
| Added drag required to cancel blown-lift thrust | 0.000 | 1.274 |
| Total | 6.118 | 11.793 |

Slow-flight induced drag is the physical split-panel sum: unblown-panel induced drag plus blown-panel induced drag. The freestream-equivalent finite-wing value is reported only as a diagnostic check and is not used in the force balance.

## Approach Estimate

| Quantity | Value |
| --- | ---: |
| Target sink rate | 6.00 ft/s |
| Actual sink rate | 6.00 ft/s |
| Sink-rate target met? | True |
| Descent angle | 21.45 deg |
| Flap deflection | 40.0 deg |
| Approach alpha | 13.06 deg |
| Required thrust | 27.464 N |
| Electrical power | 592.04 W |
| RPM | 12440 rpm |
| Throttle estimate | 100.0% |
| Propulsion feasible at approach | False |
| Elevator trim | 15.25 deg |
| Rudder trim | 0.00 deg |
| Max descent rate limit | 6.0 ft/s |
| Sink rate | 1.829 m/s (6.00 ft/s) |
| Descent rate limited? | False |
| Climb target rate | 6.0 ft/s |
| Climb speed | 10.00 m/s |
| Climb angle | 10.54 deg |
| Climb CL required | 1.124 |
| Climb thrust required | 14.916 N |
| Climb throttle estimate | 63.6% |
| Climb feasible | True |

## Best-Design Artifacts

- Top view: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_top_view.png`
- 3D wireframe rendering: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_3d_wireframe.png`
- Three view: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_three_view.png`
- Wireframe: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_wireframe.png`
- Drag and power sweep chart: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_drag_power_sweeps.png`
- Drag components chart: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_drag_components.png`
- Performance sweep CSV: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_performance_sweep.csv`
- Mesh: `outputs/stage3_visuals/rank01_n10_d5p5_pd0p40_balanced_mesh.npz`

## Top Designs

| Rank | Props | Cruise W | Slow W | H-tail span | V-tail height | Mass margin | Static margin |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 10 x 5.5 in balanced | 112.10 | 188.42 | 0.606 | 0.377 | 2.509 | 0.237 |
