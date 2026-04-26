# Stage 3 AeroSandbox Fixed-Wing Tail Refinement

## Scope

Stage 3 freezes the selected Stage 1/2 main wing and propulsion layout, then sizes the horizontal and vertical tail using AeroSandbox geometry, VLM trim/stability checks, NeuralFoil section data, prop RPM re-solves, and an explicit NGX250 foam material model.

## Fixed Inputs

- Gross flight mass for lift balance: `5.000 kg`
- Maximum as-built mass: `5.000 kg`
- Main wing span: `2.000 m`
- Main wing chord: `0.350 m`
- Main wing area: `0.700 m^2`
- Fuselage cross-section: `170 mm` wide x `130 mm` tall
- Low-speed mode: `4.000 m/s`
- Cruise mode: `10.000 m/s`
- Foam density: `25.0 kg/m^3`

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
- Cruise power and slow-flight power are both re-solved with the selected propeller surrogate instead of merely scaling the Stage 1 numbers.

## Recommended Aircraft Dimensions

| Quantity | Value |
| --- | ---: |
| Propulsion layout | 10 x 5.5 in, P/D 0.4, balanced |
| Main airfoil | DAE51 |
| Main wing span | 2.000 m |
| Main wing chord | 0.350 m |
| Main wing incidence | 0.000 deg |
| Propeller axial x position | -0.087 m |
| Fuselage width x height | 170 mm x 130 mm |
| Horizontal tail span | 0.498 m |
| Horizontal tail root chord | 0.214 m |
| Horizontal tail tip chord | 0.118 m |
| Horizontal tail incidence | -5.175 deg |
| Elevator chord fraction | 0.220 chord |
| Vertical tail height/span | 0.234 m |
| Vertical tail root chord | 0.208 m |
| Vertical tail tip chord | 0.200 m |
| Vertical tail incidence | 0.000 deg |
| Rudder chord fraction | 0.455 chord |
| Tail arm | 1.483 m |
| Static margin | 0.131 MAC |
| H-tail volume range / actual | 0.500-0.950 / 0.500 |
| V-tail volume range / actual | 0.035-0.085 / 0.051 |
| Slow pitch control authority / target | 0.563 / 0.180 Cm |
| Slow yaw control authority / target | 0.055 / 0.055 Cn |
| Stage 3 built mass | 2.434 kg |
| Tail foam mass | 0.033 kg |
| Stage 1/2 no-flap CLmax | 1.418 |
| Stage 1/2 flap-only CLmax | 1.876 |
| Stage 1/2 clean blown CLmax | 5.507 |
| Stage 1/2 flap-down blown CLmax | 5.810 |
| Slow-flight flap-down CLmax, unblown | 1.876 |
| Slow-flight flap-down CLmax, blown | 5.810 |
| Slow-flight lift margin | 420.7% |
| Cruise fuselage drag | 0.674 N |
| Cruise power | 113.38 W |
| Slow-flight power | 174.27 W |
| Slow-flight natural drag before added drag | 10.234 N |
| Slow-flight added drag required | 0.506 N |
| Slow-flight steady total drag | 10.740 N |
| Slow-flight steady drag minus cruise drag | 5.536 N |
| Approach target angle | 6.00 deg below horizontal |
| Approach speed | 5.00 m/s |
| Approach elevator trim | 6.59 deg |
| Approach throttle estimate | 100.0% |

## Drag Components

| Component | Clean cruise drag [N] | Flaps-down slow-flight drag [N] |
| --- | ---: | ---: |
| Main wing profile / Stage 1 flap baseline | 0.528 | 10.084 |
| Main wing induced | 3.905 | Included in Stage 1 baseline |
| Horizontal tail | 0.062 | 0.028 |
| Vertical tail | 0.035 | 0.014 |
| Fuselage | 0.674 | 0.108 |
| Added drag required to cancel blown-lift thrust | 0.000 | 0.506 |
| Total | 5.204 | 10.740 |

## Approach Estimate

| Quantity | Value |
| --- | ---: |
| Target descent angle | 6.00 deg |
| Flap deflection | 40.0 deg |
| Approach alpha | 13.06 deg |
| Required thrust | 39.775 N |
| Electrical power | 558.85 W |
| RPM | 14000 rpm |
| Throttle estimate | 100.0% |
| Propulsion feasible at approach | False |
| Elevator trim | 6.59 deg |
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
| 1 | 10 x 5.5 in balanced | 113.38 | 174.27 | 0.498 | 0.234 | 2.566 | 0.131 |
