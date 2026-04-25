# Rectangular-Wing Control Surface Sizing: Rank 6

## Selected propulsion concept

- Wing airfoil: `DAE51`
- Rank: `6`
- Prop layout: `10 x 5.5 x 2.2 in`
- Prop family: `balanced`
- Blade count metadata: `3`
- Low-speed RPM: `9695.8`
- Cruise RPM: `9564.5`
- Available low-speed blown velocity: `14.62 m/s`
- Low-speed induced velocity from actuator-disk relation: `5.31 m/s`
- Required low-speed blown velocity from Stage 1: `9.47 m/s`

## Rectangular-wing assumptions

- Span: `2.00 m`
- Chord: `0.35 m`
- Wing area: `0.700 m^2`
- Slotted flap modeling: `NeuralFoil flap polar + staged slotted-flap correction`
- Low-speed flap assumption: `max slotted flap deflection = 40.0 deg`
- Prop drop ahead of flap: `0.042 m` (`0.120 c`, `0.301 D_p`) for props that lie inside the flap span
- Cambridge-style jet model: `uniform 2D jet with immersion criterion` at `x_p/c = -0.25`

## Recommended flap

- Span fraction of semispan: `0.65`
- End station: `0.650 m`
- Chord fraction: `0.34`
- Deflection: `40.0 deg`
- Flap area: `0.455 m^2`
- Blown flap area: `0.235 m^2`
- Props with disk overlap over flap: `8/10`
- Positive-half prop overlap fraction: `80%`
- Positive-half disk overlap fraction: `60%`
- Common-alpha freestream-referenced wing CLmax: `5.810`
- Equivalent stall speed: `4.437 m/s`
- Lift margin at 4 m/s: `-18.7 %`
- Alpha at CLmax: `12.98 deg`
- Post-stall 90% point: `19.92 deg`
- Post-stall drop over +5 deg: `0.255`
- Whole-wing C_M at CLmax: `-0.684`
- Whole-wing C_M at CLmax (blowing only): `-0.213`
- Cambridge clean-strip immersion at CLmax: `1.000`
- Cambridge flap-strip immersion at CLmax: `1.000`
- Cambridge clean-strip jet margin at CLmax: `40.6 mm`
- Cambridge flap-strip jet margin at CLmax: `26.8 mm`

## Recommended aileron

- Span fraction of semispan: `0.28`
- Start station: `0.720 m`
- Chord fraction: `0.28`
- Aileron area: `0.055 m^2`
- Trim alpha at 10 m/s: `12.00 deg`
- Rolling moment derivative: `0.001717 /deg`
- Roll damping derivative: `-0.4330`
- Estimated roll rate at 14 deg: `31.8 deg/s`
- Estimated roll rate at 20 deg: `45.4 deg/s`
- Low-speed blown overlap over aileron span: `59%`
- Low-speed local dynamic-pressure ratio over aileron: `8.30`
- Low-speed effective local velocity over aileron: `11.53 m/s`
- Low-speed rolling-moment derivative proxy: `0.014260 /deg`
- Low-speed roll-damping proxy: `-3.4241`
- Low-speed roll-rate proxy at 14 deg: `13.4 deg/s`
- Low-speed roll-rate proxy at 20 deg: `19.1 deg/s`
- Internal target roll rate for sizing: `30.0 deg/s`

## Artifacts

- Layout plot: ![](layout.png)

- Flap heatmap: ![](flap_heatmap.png)

- Flap curves: ![](flap_curves.png)

- Section polars: ![](flap_section_polars.png)

- Whole-wing CL/CD curve: ![](total_cl_curve.png)

- Whole-wing C_M curve: ![](total_cm_curve.png)

- Aileron curves: ![](aileron_curves.png)

- Low-speed aileron proxy curves: ![](low_speed_aileron_curves.png)

## Notes

- The blade-count input is tracked as metadata only in this script; the current Stage 1/2 prop surrogate does not explicitly model blade count.
- The slotted-flap correction remains a concept-level surrogate layered on top of NeuralFoil flap polars; however, motor height is no longer represented by a purely empirical Gaussian bonus.
- The blown-strip contribution now follows a Cambridge-style uniform-jet immersion model: the blown benefit remains strong only while the wing stays submerged in the jet, and it collapses rapidly as the jet rides above the wing.
- The blown-section freestream-referenced lift contribution is smoothly limited to about `9.0` to stay consistent with the order of magnitude reported by 2D blown-flap wind-tunnel data.
- Flap candidates are now penalized if they fail to cover at least `80%` of positive-half prop disks and `55%` of positive-half disk span.
- The reported wing CLmax is now extracted from a common-alpha whole-wing lift curve, not from independently maximized section values.
- The aileron sizing is evaluated at cruise trim using AeroSandbox VLM on the rectangular wing. The report uses an estimated steady roll rate based on VLM aileron effectiveness and roll-damping derivative.
- The low-speed aileron metrics are explicitly labeled as proxies. They scale the cruise derivatives by the blown overlap of the aileron region and the low-speed dynamic-pressure ratio, rather than claiming a full blown-control CFD solution.