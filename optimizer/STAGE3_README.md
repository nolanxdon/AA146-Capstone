# Stage 3 AeroSandbox Tail Refinement

Stage 3 takes the aircraft decisions already made by Stage 1 and Stage 2, freezes them, and refines the fixed-wing tail layout. It does not re-pick the main wing, airfoil, propeller count, propeller diameter, propeller pitch ratio, flap sizing, aileron sizing, or total mass target. Its job is to size the horizontal stabilizer, vertical stabilizer, tail arm, elevator chord fraction, rudder chord fraction, main-wing incidence, trim settings, drag buildup, and Stage 3 report artifacts.

## Quick Start

Run Stage 3 from the repo root:

```bash
python3 -m optimizer.stages.stage3_aerosandbox
```

Then validate all generated outputs:

```bash
python3 -m optimizer.stages.validate_outputs
```

The main human-readable output is:

```text
outputs/stage3_readable_results.md
```

The main machine-readable outputs are:

```text
outputs/stage3_aerosandbox_results.csv
outputs/stage3_aerosandbox_top_designs.csv
outputs/stage3_aerosandbox_queue.csv
```

The editable Stage 3 assumptions are here:

```text
optimizer/config/stage3_constraints.yaml
```

## Required Inputs

Stage 3 expects Stage 1 and Stage 2 outputs to already exist. The key files are:

```text
outputs/stage1_pareto_front.csv
outputs/stage2_prop_span_report.csv
outputs/wing_workflow/dae51/wing_workflow_summary.csv
outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_static_partial_load.csv
outputs/ecalc_prop_analysis/x2302_1500kv_3s_5p5x3p5_3b/ecalc_dynamic_design_point.csv
```

The wing workflow summary points Stage 3 to the selected control-surface and high-lift files. In the normal workflow, Stage 3 reads the flap geometry, aileron geometry, `total_cl_curve.csv`, and the selected `flap_sweep.csv` row from Stage 1/2 outputs. The raw Stage 1/2 equivalent CLmax values remain traceable, while the physical local clean/flapped CLmax values come from the Stage 1/2 NeuralFoil flap-sweep section polars. The fallback values in `stage3_constraints.yaml` are only used if those files are missing.

The eCalc CSVs are optional but preferred. When `ecalc_propulsion_enabled` is true and the frozen propeller count/diameter matches the configured eCalc hardware, Stage 3 uses the eCalc RPM-thrust-power table plus the dynamic CT correction instead of the generic Stage 1 propeller surrogate.

## What Stage 3 Freezes

Stage 3 keeps these Stage 1/2 values fixed:

- Main wing span and chord.
- Main wing airfoil.
- Gross flight mass and maximum mass target.
- Number of propellers/motors.
- Propeller diameter, pitch ratio, and prop family.
- Propeller spanwise positions.
- Flap and aileron geometry.
- Stage 1/2 raw high-lift CLmax outputs as inputs. Stage 3 does not blindly accept powered-lift equivalent CLmax as an airfoil limit; it derives local DAE51 physical CLmax from the Stage 1/2 NeuralFoil/slotted-flap section polars and models propwash through dynamic pressure.

This is intentional. Stage 3 is the tail and trim refinement pass, not a full aircraft restart.

## What Stage 3 Optimizes

The cruise-oriented optimizer varies:

- Horizontal tail area.
- Horizontal tail aspect ratio.
- Horizontal tail taper ratio.
- Vertical tail area.
- Vertical tail aspect ratio.
- Vertical tail taper ratio.
- Tail arm.
- Main-wing incidence, within the configured incidence bounds.

From those variables, Stage 3 computes:

- Horizontal tail span, root chord, tip chord, and mean aerodynamic chord.
- Vertical tail height, root chord, tip chord, and mean aerodynamic chord.
- Tail volume coefficients.
- Tail foam mass.
- Updated CG after tail foam is added.
- Static margin.
- Cruise trim incidence.
- Cruise body angle of attack.
- Cruise drag and power.
- Slow-flight blown-lift power.

After the stabilizer planform is selected, Stage 3 sizes:

- Elevator chord fraction for slow-flight pitch authority.
- Rudder chord fraction for slow-flight yaw authority.

That split is important: stabilizer planforms are biased toward cruise performance, while moving-surface sizing is biased toward slow-flight control authority.

## Key Assumptions And Justifications

The values below live in `optimizer/config/stage3_constraints.yaml` so they can be changed without editing code.

| Assumption | Current Value | Why It Is Used |
| --- | ---: | --- |
| Tail airfoil | `naca0008` | Fixed by the current design direction. A symmetric 8% tail airfoil gives low pitching moment, lower thickness drag than NACA 0012, and enough foam thickness for a small stabilizer. |
| Foam density | `25 kg/m^3` | NGX250 pink foam density specified for construction mass accounting. |
| Horizontal tail volume range | `0.50 to 0.95` | Keeps the H-tail in a conventional small-aircraft range while allowing enough authority for a low-speed, high-lift configuration. |
| Vertical tail volume range | `0.035 to 0.085` | Typical first-pass range for small fixed-wing aircraft; high enough to avoid a tiny fin, low enough to avoid unnecessary drag and mass. |
| Main wing incidence bounds | `-2 to 8 deg` | Incidence is optimized so cruise can trim at a sensible body angle without changing the Stage 1/2 wing planform. |
| Physical clean CLmax fallback | `1.60` | Used only if the Stage 1/2 NeuralFoil section polar files are missing. Normal runs use the DAE51 clean CLmax from `flap_sweep.csv`. |
| Physical flapped CLmax fallback | `2.00` | Used only if the Stage 1/2 NeuralFoil section polar files are missing. Normal runs use the DAE51 slotted-flap CLmax from `flap_sweep.csv`. |
| Target static margin | `0.13 MAC` | A 13% static margin gives a stable but not excessively nose-heavy aircraft. The allowed range is `0.10 to 0.27 MAC`. |
| Final CG target | `0.25 MAC` | The final aircraft CG is forced to quarter chord after tail foam is added. Stage 3 back-calculates the required pre-tail baseline CG needed to hit this target. |
| Cruise tail dynamic pressure ratio | `0.90` | The tail is assumed to see slightly less dynamic pressure than freestream due to wake and downwash losses. |
| Slow tail dynamic pressure ratio | `0.85` | Slow-flight control sizing is made slightly more conservative because flap wake, high alpha, and low Reynolds number can reduce tail effectiveness. |
| Downwash gradient | `0.35` | First-pass low-aspect-ratio wing estimate for how much wing angle of attack appears as downwash at the tail. |
| Fuselage drag area | `0.011 m^2` | Approximate equivalent drag area for a 170 mm by 130 mm fuselage cross-section using a modest bluff-body drag allowance. |
| Tail zero-lift CD | `0.012` | Conservative low-Reynolds-number profile drag estimate for small symmetric tail sections with real construction roughness. |
| eCalc propulsion calibration | `enabled` | Uses the collaborator's updated eCalc static thrust/power table and dynamic CT/CP point for the frozen 10 x 5.5 in propulsion layout. |
| Stage 3 slow-flight speed | `4 m/s` | Stage 3-only slow-flight sizing condition. This does not modify the upstream Stage 1/2 low-speed condition or their generated outputs. |
| Slow-flight stall margin factor | `1.20` | Requires 20% CLmax headroom to cover gusts, build errors, surface waviness, hinge gaps, and uncertainty in the blown-lift model. |
| Minimum slow lift margin warning | `0.10` | Warns if less than 10% lift margin remains after the 20% stall factor has already been applied. |
| Elevator and rudder max deflection | `25 deg` | A practical first-pass control authority limit that avoids assuming extreme hinge moments or stalled control surfaces. |
| Control effectiveness exponent | `0.70` | Control authority does not scale linearly forever with chord fraction; this exponent gives diminishing returns for larger surfaces. |
| Approach sink-rate target | `6 ft/s` | Approach now targets the same descent rate as the configured max descent rate. If approach speed is too low to physically reach this vertical rate, Stage 3 clips it and reports that the target was not met. |
| Approach speed | `5 m/s` | First-pass flaps-down approach speed used with the sink-rate target. |
| Maximum climb rate | `6 ft/s` | Climb trim checks whether the frozen propulsion system can sustain this rate in clean configuration. Saturation is reported instead of hidden. |
| Maximum descent rate | `6 ft/s` | Used as the descent-rate cap and nominal approach descent-rate target. |
| Mission-regime battery | `4S LiPo` | Stage 3 sizes the report-table battery capacity itself from the regime energy, nominal `4 x 3.7 V`, and the configured reserve fraction. It no longer assumes a fixed 7200 mAh pack. |

## Slow-Flight Added Drag Accounting

The blown-lift slow-flight mode can require more propeller thrust than the natural airframe drag. If thrust is greater than drag, the aircraft would accelerate unless another drag device absorbs the excess.

Stage 3 now separates:

- Natural slow-flight drag: explicit flaps-down wing profile drag, conservative induced drag, Stage 3 tail drag, and fuselage drag.
- Blown-lift thrust: propeller thrust required to create the effective slipstream velocity needed for the CLmax calculation.
- Added drag required: `max(0, blown_lift_thrust - natural_slow_flight_drag)`.
- Steady slow-flight total drag: `natural_drag + added_drag_required`.

The added drag is the first-pass sizing target for future drag devices such as airbrakes, spoilerons, deployable plates, or other high-drag surfaces.

For induced drag, Stage 3 computes both:

- A local blown-panel induced-drag estimate using the local unblown and blown dynamic pressures.
- A conservative freestream-equivalent finite-wing induced-drag estimate using the very high freestream-reference CL required at the slow speed.

The solver uses the larger of those two induced-drag values. This prevents the slow-flight case from looking artificially low-drag when the aircraft is really asking the wing/propeller system for a very high equivalent CL.

The readable results report:

```text
Slow-flight natural drag before drag devices
Slow-flight wing induced drag used
Slow-flight induced drag, local blown-panel estimate
Slow-flight induced drag, freestream-equivalent check
Slow-flight blown-lift thrust
Added drag required for no acceleration
Slow-flight steady total drag
Slow steady drag minus cruise drag
```

## Approach Estimate

The approach calculation assumes a steady flaps-down descent at the target sink rate. It estimates:

- Required lift coefficient.
- Approach angle of attack.
- Approach drag.
- Required thrust.
- Electrical power.
- RPM.
- Throttle percentage.
- Elevator trim.
- Rudder trim.
- Sink rate.

The flight-path equation used is:

```text
T = D - W sin(gamma)
```

where `gamma` is the resulting descent angle below horizontal. If the requested sink rate is larger than the vertical component possible at the chosen approach speed, Stage 3 clips the sink rate and reports `approach_sink_rate_target_met = 0`. If the required thrust exceeds what the propellers can provide inside the low-speed RPM bound, Stage 3 reports `approach_throttle_saturated`.

The climb calculation is a companion trim check in clean configuration. It targets the configured maximum climb rate, currently `6 ft/s`, at `climb_trim_speed_mps`. If the frozen propulsion system cannot supply the required thrust inside the cruise RPM bound, Stage 3 reports `climb_throttle_saturated`.

## Generated Plots

Each successful ranked design creates:

```text
outputs/stage3_visuals/rank##_..._top_view.png
outputs/stage3_visuals/rank##_..._three_view.png
outputs/stage3_visuals/rank##_..._wireframe.png
outputs/stage3_visuals/rank##_..._3d_wireframe.png
outputs/stage3_visuals/rank##_..._polars.png
outputs/stage3_visuals/rank##_..._drag_power_sweeps.png
outputs/stage3_visuals/rank##_..._drag_components.png
outputs/stage3_visuals/rank##_..._performance_sweep.csv
outputs/stage3_visuals/rank##_..._mesh.npz
```

The drag/power sweep plot includes:

- Total drag versus velocity.
- Total drag versus angle of attack.
- Electrical power required versus velocity.
- Electrical power required versus angle of attack.

Each plot includes both:

- Clean cruise configuration.
- Flaps-down slow-flight configuration.

Important: the sweep power curves are diagnostics, not extra optimizer objectives. The clean velocity sweep solves the trimmed clean-wing drag balance and leaves speeds blank when the required CL is outside the clean polar. The flaps-down sweeps use the Stage 3 split blown/unblown drag model, so the plotted slow-flight power comes from the same slipstream, profile-drag, induced-drag, tail, and fuselage buildup used by the result table. Infeasible max-RPM points are left blank in the plot and marked with `rpm_feasible = 0` / `trim_feasible = 0` in the sweep CSV.

## Most Useful Result Fields

In `outputs/stage3_aerosandbox_results.csv`, the most important fields are:

| Field | Meaning |
| --- | --- |
| `htail_span_m`, `htail_root_chord_m`, `htail_tip_chord_m` | Final H-stab geometry. |
| `vtail_span_m`, `vtail_root_chord_m`, `vtail_tip_chord_m` | Final V-stab geometry. |
| `htail_incidence_deg` | Cruise-trim H-stab incidence. |
| `main_wing_incidence_deg` | Optimized main-wing incidence. |
| `cruise_body_alpha_proxy_deg` | Airfoil-polar estimate of cruise fuselage/body angle of attack after incidence is applied. |
| `cruise_section_alpha_proxy_deg` | Airfoil-polar estimate of the main-wing section angle of attack at cruise. |
| `cruise_alpha_deg` | AeroSandbox VLM trim-alpha diagnostic; use with care because VLM does not capture every airfoil-polar effect. |
| `vertical_tail_incidence_deg` | V-stab incidence. |
| `elevator_chord_fraction` | Elevator chord as fraction of local H-tail chord. |
| `rudder_chord_fraction` | Rudder chord as fraction of local V-tail chord. |
| `horizontal_tail_volume`, `vertical_tail_volume` | Tail volume coefficients. |
| `static_margin_mac` | Static margin in mean aerodynamic chord. |
| `cg_target_percent_mac`, `cg_percent_mac`, `cg_error_m` | Final CG target, final CG, and residual error. |
| `stage1_baseline_cg_required_percent_mac` | Pre-tail baseline CG required so the tail-included aircraft lands at quarter chord. |
| `cruise_drag_n`, `cruise_power_w` | Clean cruise performance. |
| `stage3_slow_flight_speed_mps` | Slow-flight speed used only by Stage 3 sizing and reports. |
| `upstream_stage12_low_speed_mps` | Original upstream low-speed condition retained from Stage 1/2. |
| `low_speed_natural_drag_n` | Slow-flight airframe drag before added drag devices. |
| `low_speed_wing_induced_drag_n` | Slow-flight induced drag used in the force balance. |
| `low_speed_wing_induced_local_drag_n` | Diagnostic local blown-panel induced estimate. |
| `low_speed_wing_induced_freestream_equiv_drag_n` | Diagnostic freestream-equivalent induced estimate. |
| `physical_clmax_source` | Source of the physical local CLmax limits, normally Stage 1/2 `flap_sweep.csv`. |
| `stage12_clean_section_clmax_unblown`, `stage12_flapped_section_clmax_unblown` | Clean/flapped local section CLmax values read from Stage 1/2. |
| `stage12_weighted_blown_physical_clmax` | Area-weighted physical local CLmax for the Stage 1/2 blown clean/flap panel mix. |
| `low_speed_added_drag_required_n` | Drag device target to prevent acceleration during blown-lift slow flight. |
| `low_speed_drag_n` | Steady slow-flight drag after added drag is included. |
| `low_speed_drag_delta_vs_cruise_n` | Slow steady drag minus cruise drag. |
| `approach_elevator_trim_deg` | Estimated elevator trim for the configured approach. |
| `approach_throttle_percent` | Estimated throttle for the configured approach. |
| `climb_target_rate_fps`, `climb_throttle_percent` | Clean climb trim check at the configured maximum climb rate. |
| `design_warnings` | Semicolon-separated warnings such as `slow_flight_infeasible`, `slow_flight_requires_powered_lift`, `stage12_equivalent_clmax_capped`, `approach_throttle_saturated`, or `climb_throttle_saturated`. |

## Recommended Workflow

1. Run Stage 1 and Stage 2.
2. Confirm Stage 1/2 selected the intended wing, controls, and propeller layout.
3. Edit `optimizer/config/stage3_constraints.yaml` if needed.
4. Run `python3 -m optimizer.stages.stage3_aerosandbox`.
5. Read `outputs/stage3_readable_results.md`.
6. Inspect the top-view and 3D wireframe plots.
7. Inspect the drag component and drag/power sweep plots.
8. Run `python3 -m optimizer.stages.validate_outputs`.
9. Use `low_speed_added_drag_required_n` as the first airbrake/drag-surface sizing target.

## Known Limitations

Stage 3 is still a conceptual and early preliminary design tool. The largest remaining risks are:

- The high-lift model depends on Stage 1/2 blown-lift assumptions, but Stage 3 now reads the Stage 1/2 CL-alpha curves and section polars directly to avoid accepting impossible equivalent CLmax values as physical airfoil limits.
- The tail is not yet trimmed with a full coupled flap-down VLM solve at slow speed.
- Fuselage drag is an equivalent drag-area approximation, not CFD or wind-tunnel data.
- Tail dynamic-pressure ratios are engineering assumptions, not measured wake data.
- Approach trim is an estimate, not a time-domain landing simulation.
- Airbrake drag is reported as a required delta, but no physical airbrake geometry is designed yet.

Those limitations are visible by design. Stage 3 is meant to show which assumptions dominate the aircraft before the design is frozen for fabrication or higher-fidelity CFD.
