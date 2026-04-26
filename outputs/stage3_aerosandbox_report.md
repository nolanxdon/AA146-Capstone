# Stage 3 AeroSandbox Fixed-Wing Tail Refinement

## Scope

Stage 3 freezes the selected Stage 1/2 main wing and propulsion layout, then sizes the horizontal and vertical tail using AeroSandbox geometry, VLM trim/stability checks, NeuralFoil section data, prop RPM re-solves, and an explicit NGX250 foam material model.

## Fixed Inputs

- Gross flight mass for lift balance: `5.000 kg`
- Maximum as-built mass: `5.000 kg`
- Main wing span: `2.000 m`
- Main wing chord: `0.350 m`
- Main wing area: `0.700 m^2`
- Fuselage nose station: `-0.175 m` from wing leading edge
- Fuselage cross-section: `170 mm` wide x `130 mm` tall
- Low-speed mode: `4.000 m/s`
- Cruise mode: `10.000 m/s`
- Foam density: `25.0 kg/m^3`
- eCalc propulsion calibration enabled: `True`

## Optimizer Notes

- Editable Stage 3 constraints are loaded from `optimizer/config/stage3_constraints.yaml` when that YAML file is present.
- Main wing dimensions and selected airfoil are not optimized in Stage 3.
- Horizontal and vertical stabilizer planform dimensions are optimized primarily for cruise trim, cruise drag, material mass, static margin, and the user-specified tail-volume ranges.
- Elevator and rudder chord fractions are sized separately after planform optimization to meet slow-flight pitch and yaw authority targets.
- Slow-flight sizing is evaluated with the frozen slotted flaps deployed.
- Slow-flight CLmax values are read from the selected Stage 1/2 wing workflow when available; the sizing CLmax values are then divided by the stall margin factor `1.20`.
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
| Main wing incidence | 0.000 deg |
| Propeller axial x position | -0.087 m |
| Fuselage nose station | -0.175 m from wing LE |
| Fuselage width x height | 170 mm x 130 mm |
| Horizontal tail span | 0.868 m |
| Horizontal tail root chord | 0.145 m |
| Horizontal tail tip chord | 0.144 m |
| Horizontal tail incidence | -5.000 deg |
| Elevator chord fraction | 0.220 chord |
| Vertical tail height/span | 0.358 m |
| Vertical tail root chord | 0.145 m |
| Vertical tail tip chord | 0.130 m |
| Vertical tail incidence | 0.000 deg |
| Rudder chord fraction | 0.409 chord |
| Tail arm | 1.000 m |
| CG target / actual | 25.00% / 25.00% MAC |
| Required pre-tail baseline CG | 21.01% MAC |
| Static margin | 0.304 MAC |
| H-tail volume range / actual | 0.500-0.950 / 0.512 |
| V-tail volume range / actual | 0.035-0.085 / 0.035 |
| Slow pitch control authority / target | 0.721 / 0.180 Cm |
| Slow yaw control authority / target | 0.055 / 0.055 Cn |
| Stage 3 built mass | 2.480 kg |
| Tail foam mass | 0.034 kg |
| Stage 1/2 no-flap CLmax | 1.418 |
| Stage 1/2 flap-only CLmax | 1.876 |
| Stage 1/2 clean blown CLmax | 5.545 |
| Stage 1/2 flap-down blown CLmax | 5.828 |
| Slow-flight flap-down CLmax, unblown | 1.876 |
| Slow-flight flap-down CLmax, blown | 5.828 |
| Slow-flight lift margin | 458.6% |
| Cruise fuselage drag | 0.674 N |
| Cruise power | 103.11 W |
| Cruise CT / CP | 0.0986 / 0.0706 |
| Slow-flight power | 174.20 W |
| Slow-flight CT / CP | 0.1269 / 0.0706 |
| Slow-flight natural drag before added drag | 11.227 N |
| Slow-flight added drag required | 0.561 N |
| Slow-flight steady total drag | 11.789 N |
| Slow-flight steady drag minus cruise drag | 5.696 N |
| Main wing Reynolds number, cruise / slow | 236878 / 94751 |
| H-tail Reynolds number, cruise / slow | 97900 / 39160 |
| V-tail Reynolds number, cruise / slow | 93204 / 37282 |
| Approach target angle | 6.00 deg below horizontal |
| Approach speed | 5.00 m/s |
| Approach elevator trim | 11.91 deg |
| Approach throttle estimate | 100.0% |

## Drag Components

| Component | Clean cruise drag [N] | Flaps-down slow-flight drag [N] |
| --- | ---: | ---: |
| Main wing profile / Stage 1 flap baseline | 0.528 | 11.062 |
| Main wing induced | 4.739 | Included in Stage 1 baseline |
| Horizontal tail | 0.115 | 0.043 |
| Vertical tail | 0.036 | 0.014 |
| Fuselage | 0.674 | 0.108 |
| Added drag required to cancel blown-lift thrust | 0.000 | 0.561 |
| Total | 6.092 | 11.789 |

## Approach Estimate

| Quantity | Value |
| --- | ---: |
| Target descent angle | 6.00 deg |
| Flap deflection | 40.0 deg |
| Approach alpha | 13.06 deg |
| Required thrust | 40.214 N |
| Electrical power | 548.00 W |
| RPM | 14000 rpm |
| Throttle estimate | 100.0% |
| Propulsion feasible at approach | False |
| Elevator trim | 11.91 deg |
| Rudder trim | 0.00 deg |
| Sink rate | 0.523 m/s |

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
| 1 | 10 x 5.5 in balanced | 103.11 | 174.20 | 0.868 | 0.358 | 2.520 | 0.304 |
