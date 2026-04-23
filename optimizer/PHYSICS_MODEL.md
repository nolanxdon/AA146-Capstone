# Physics Model Notes

This file is the living reference for the optimizer physics, constraints, and output definitions.

It currently documents the implemented **Stage 1 V2** model in:

- [optimizer/core/data_models.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/data_models.py:1)
- [optimizer/core/physics.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/physics.py:1)
- [optimizer/core/mass_model.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/mass_model.py:1)
- [optimizer/stages/stage1_screen.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/stages/stage1_screen.py:1)

## Scope

The current optimizer is a **coarse concept screener** for distributed propulsion on a blown wing. It is intended to downselect:

- propeller count
- propeller diameter
- propeller pitch ratio
- coarse propeller family

The Stage 1 model is fast and intentionally approximate. It is useful for pruning the architecture space before Stage 2 spanwise prop sizing and Stage 3 AeroSandbox refinement.

## Current Mission And Geometry Assumptions

The default mission and aircraft assumptions are:

- Gross flight mass:
  $$
  m_{\mathrm{gross}} = 5.0 \ \mathrm{kg}
  $$
- Maximum built mass:
  $$
  m_{\max} = 5.0 \ \mathrm{kg}
  $$
- Fixed non-propulsion mass:
  $$
  m_{\mathrm{fixed}} = 2.0 \ \mathrm{kg}
  $$
- Wing span:
  $$
  b = 2.0 \ \mathrm{m}
  $$
- Wing chord:
  $$
  c = 0.35 \ \mathrm{m}
  $$
- Wing area:
  $$
  S = bc = 0.70 \ \mathrm{m^2}
  $$
- Aspect ratio:
  $$
  AR = \frac{b^2}{S}
  $$
- Battery voltage:
  $$
  V_{\mathrm{batt}} = 14.8 \ \mathrm{V}
  $$
- Low-speed flight point:
  $$
  V_{\infty,\mathrm{low}} = 4.0 \ \mathrm{m/s}
  $$
- Cruise / loiter flight point:
  $$
  V_{\infty,\mathrm{cruise}} = 10.0 \ \mathrm{m/s}
  $$
- Loiter time:
  $$
  t_{\mathrm{loiter}} = 18 \ \mathrm{min}
  $$
- Additional low-speed energy segment charged to the battery:
  $$
  t_{\mathrm{climb}} = 1.5 \ \mathrm{min}
  $$

Atmosphere and constants:

- Air density:
  $$
  \rho = 1.225 \ \mathrm{kg/m^3}
  $$
- Dynamic viscosity:
  $$
  \mu = 1.81 \times 10^{-5} \ \mathrm{Pa \cdot s}
  $$
- Gravity:
  $$
  g = 9.80665 \ \mathrm{m/s^2}
  $$
- Speed of sound:
  $$
  a = 343.0 \ \mathrm{m/s}
  $$

## Aerodynamic Assumptions

- Oswald efficiency:
  $$
  e = 0.8
  $$
- Flapped baseline section lift ceiling before blowing:
  $$
  C_{L,\max,\mathrm{flapped}} = 2.2
  $$
- Flapped section ceiling used by the Stage 1 blown-lift model:
  $$
  C_{L,\mathrm{ceil}} = 2.0
  $$
- Clean section lift ceiling:
  $$
  C_{L,\max,\mathrm{clean}} = 1.4
  $$
- Flapped zero-lift drag parameter:
  $$
  C_{D0,\mathrm{flapped}} = 0.11
  $$
- Clean zero-lift drag parameter:
  $$
  C_{D0,\mathrm{clean}} = 0.05
  $$
- Trim drag multiplier:
  $$
  f_{\mathrm{trim}} = 1.05
  $$
- Blown profile drag multiplier:
  $$
  f_{\mathrm{blown}} = 1.05
  $$
- Unblown profile drag multiplier:
  $$
  f_{\mathrm{unblown}} = 1.0
  $$
- Slipstream span-expansion factor:
  $$
  k_{\mathrm{exp}} = 0.8
  $$

The Stage 1 low-speed model treats the wing as two zones:

- unblown area
- blown area

with span coverage determined from the placed propeller disk intervals, not from a simple average-spacing formula.

## Blown-Lift Model

The Stage 1 blown-lift model is based on a wing-reference momentum coefficient:

$$
C_\mu = \frac{T}{q_\infty S}
$$

where

$$
q_\infty = \frac{1}{2} \rho V_\infty^2
$$

and \(T\) is the total propeller thrust at the low-speed operating point.

The local blown-section lift increment is modeled as:

$$
\Delta C_{L,\mu} =
\begin{cases}
0, & C_\mu < C_{\mu,\min} \\
k_{C\mu}\sqrt{C_\mu}, & C_\mu \ge C_{\mu,\min}
\end{cases}
$$

with:

- \(k_{C\mu} = 1.8\)
- \(C_{\mu,\min} = 0.02\)

The blown-section lift ceiling is then:

$$
C_{L,\max,b} = \min \left( C_{L,\max,\mathrm{flapped}} + \Delta C_{L,\mu}, \ C_{L,\mathrm{ceil}} \right)
$$

The unblown section uses:

$$
C_{L,\max,u} = \min \left( C_{L,\max,\mathrm{flapped}}, \ C_{L,\mathrm{ceil}} \right)
$$

## Stage 1 Design Variables

The current Stage 1 sweep spans:

- Number of props:
  $$
  N_{\mathrm{prop}} \in \{4,6,8,10,12,14,16\}
  $$
- Prop diameter:
  $$
  D \in \{4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 9.0, 10.0\} \ \mathrm{in}
  $$
- Prop pitch ratio:
  $$
  \frac{P}{D} \in \{0.40,0.50,0.60,0.70,0.80,0.90\}
  $$
- Prop family:
  $$
  \{\mathrm{high\_thrust}, \mathrm{balanced}, \mathrm{cruise}\}
  $$

Low-speed RPM and cruise RPM are **not** swept. They are solved internally with a bisection root solve.

The total candidate count is:

$$
7 \times 11 \times 6 \times 3 = 1386
$$

## Propeller Surrogate

Stage 1 still uses a coarse propeller surrogate. It is not yet a UIUC-backed prop database.

Each prop family defines:

- a baseline static thrust coefficient \(C_{T,\mathrm{base}}\)
- a baseline static power coefficient \(C_{P,\mathrm{base}}\)
- an advance-ratio scale \(J_{\mathrm{crit}}\)

The operating advance ratio is:

$$
J = \frac{V}{nD}
$$

where

$$
n = \frac{\mathrm{RPM}}{60}
$$

Pitch ratio adjusts the family static coefficients:

$$
C_{T,\mathrm{static}} =
C_{T,\mathrm{base}}
\left(\frac{P/D}{(P/D)_{\mathrm{ref}}}\right)^{0.35}
$$

$$
C_{P,\mathrm{static}} =
C_{P,\mathrm{base}}
\left(\frac{P/D}{(P/D)_{\mathrm{ref}}}\right)^{0.80}
$$

In code, these static terms are then multiplied by smooth decay factors in \(J\) and by a blade Reynolds penalty factor.

## Blade Reynolds Penalty

Stage 1 derates small/slow propellers using a blade Reynolds correction at the \(0.75R\) station.

Tangential blade speed:

$$
V_\theta = 2 \pi \left(0.75 \frac{D}{2}\right) n
$$

Resultant local speed:

$$
V_{\mathrm{res}} = \sqrt{V_\theta^2 + V_\infty^2}
$$

Blade chord is approximated by:

$$
c_{\mathrm{blade}} = \left(\frac{c}{D}\right)_{\mathrm{blade}} D
$$

with

$$
\left(\frac{c}{D}\right)_{\mathrm{blade}} = 0.10
$$

Blade Reynolds number:

$$
Re_{\mathrm{blade}} = \frac{\rho V_{\mathrm{res}} c_{\mathrm{blade}}}{\mu}
$$

The thrust and power coefficients are derated by:

$$
f_{Re} = \max \left( f_{\min}, \ \min \left(1,\left(\frac{Re_{\mathrm{blade}}}{Re_{\mathrm{ref}}}\right)^m \right) \right)
$$

with:

- \(Re_{\mathrm{ref}} = 80000\)
- \(m = 0.30\)
- \(f_{\min} = 0.60\)

## Propeller Thrust, Power, Torque, And Tip Mach

Per-prop thrust:

$$
T_{\mathrm{per}} = C_T \rho n^2 D^4
$$

Per-prop shaft power:

$$
P_{\mathrm{shaft,per}} = C_P \rho n^3 D^5
$$

Total thrust:

$$
T_{\mathrm{total}} = N_{\mathrm{prop}} T_{\mathrm{per}}
$$

Total shaft power:

$$
P_{\mathrm{shaft,total}} = N_{\mathrm{prop}} P_{\mathrm{shaft,per}}
$$

Total electrical power:

$$
P_{\mathrm{elec,total}} =
\frac{P_{\mathrm{shaft,total}}}{\eta_{\mathrm{chain}}}
+
P_{\mathrm{avionics}}
$$

with:

- \(\eta_{\mathrm{chain}} = 0.72\)
- \(P_{\mathrm{avionics}} = 10 \ \mathrm{W}\)

Per-prop torque:

$$
\tau_{\mathrm{per}} = \frac{P_{\mathrm{shaft,per}}}{2 \pi n}
$$

Tip speed:

$$
V_{\mathrm{tip}} = \sqrt{(\pi D n)^2 + V_\infty^2}
$$

Tip Mach:

$$
M_{\mathrm{tip}} = \frac{V_{\mathrm{tip}}}{a}
$$

Stage 1 constrains:

$$
M_{\mathrm{tip}} \le 0.55
$$

## Prop Packing Model

Stage 1 places props symmetrically about the fuselage. Only even prop counts are allowed.

The current clearances are:

- fuselage width:
  $$
  w_f = 0.20 \ \mathrm{m}
  $$
- fuselage-to-inboard-prop edge clearance:
  $$
  c_f = 1.0 \ \mathrm{in}
  $$
- inter-prop edge clearance:
  $$
  c_p = 1.0 \ \mathrm{in}
  $$
- wingtip margin:
  $$
  c_t = 1.5 \ \mathrm{in}
  $$

The placed prop centers define disk intervals and blown intervals. Feasibility requires nonnegative:

- packing margin
- fuselage margin
- inter-prop margin
- tip margin

## Slipstream Velocity Model

Total disk area:

$$
A_{\mathrm{disk}} = N_{\mathrm{prop}} \pi \left(\frac{D}{2}\right)^2
$$

Using an ideal actuator-disk estimate:

$$
v_i = \sqrt{\frac{T_{\mathrm{total}}}{2 \rho A_{\mathrm{disk}}}}
$$

The Stage 1 effective blown velocity is:

$$
V_{\mathrm{eff}} = V_\infty + 2 v_i
$$

The inverse form is also used to compute the thrust needed to achieve a required blown velocity:

$$
T_{\mathrm{req}}(V_{\mathrm{eff,req}})
=
2 \rho A_{\mathrm{disk}}
\left(\frac{V_{\mathrm{eff,req}} - V_\infty}{2}\right)^2
$$

## Low-Speed Two-Zone Lift And Drag

Blown span fraction:

$$
\eta_b = \frac{\text{merged blown span}}{b}
$$

Wing areas:

$$
S_u = (1-\eta_b)S
$$

$$
S_b = \eta_b S
$$

Dynamic pressures:

$$
q_\infty = \frac{1}{2}\rho V_{\infty,\mathrm{low}}^2
$$

$$
q_b = \frac{1}{2}\rho V_{\mathrm{eff}}^2
$$

Reference required lift coefficient:

$$
C_{L,\mathrm{req,ref}} = \frac{W}{q_\infty S}
$$

with

$$
W = m_{\mathrm{gross}} g
$$

Maximum unblown lift:

$$
L_{u,\max} = q_\infty S_u C_{L,\max,u}
$$

Remaining lift:

$$
L_{\mathrm{rem}} = W - L_{u,\max}
$$

If \(L_{\mathrm{rem}} \le 0\), the blown region is not needed and \(V_{\mathrm{eff,req}} = V_\infty\).

Otherwise:

$$
q_{b,\mathrm{req}} = \frac{L_{\mathrm{rem}}}{S_b C_{L,\max,b}}
$$

$$
V_{\mathrm{eff,req}} = \sqrt{\frac{2 q_{b,\mathrm{req}}}{\rho}}
$$

The local blown-section required lift coefficient is:

$$
C_{L,\mathrm{req},b} = \frac{L_{\mathrm{rem}}}{q_b S_b}
$$

and low-speed lift is feasible if:

$$
C_{L,\mathrm{req},b} \le C_{L,\max,b}
$$

The induced-drag factor is:

$$
k = \frac{1}{\pi e AR}
$$

Unblown-section drag coefficient:

$$
C_{D,u} =
f_{\mathrm{trim}}
\left(
f_{\mathrm{unblown}} C_{D0,\mathrm{flapped}} + k C_{L,u}^2
\right)
$$

Blown-section drag coefficient:

$$
C_{D,b} =
f_{\mathrm{trim}}
\left(
f_{\mathrm{blown}} C_{D0,\mathrm{flapped}} + k C_{L,b}^2
\right)
$$

Low-speed drag:

$$
D_{\mathrm{low}} = q_\infty S_u C_{D,u} + q_b S_b C_{D,b}
$$

## Cruise Lift And Drag

Cruise dynamic pressure:

$$
q_{\mathrm{cruise}} = \frac{1}{2}\rho V_{\infty,\mathrm{cruise}}^2
$$

Cruise lift coefficient:

$$
C_{L,\mathrm{cruise}} = \frac{W}{q_{\mathrm{cruise}} S}
$$

Cruise drag coefficient:

$$
C_{D,\mathrm{cruise}} = C_{D0,\mathrm{clean}} + k C_{L,\mathrm{cruise}}^2
$$

Cruise drag:

$$
D_{\mathrm{cruise}} = q_{\mathrm{cruise}} S C_{D,\mathrm{cruise}}
$$

Cruise is feasible if:

$$
C_{L,\mathrm{cruise}} \le C_{L,\max,\mathrm{clean}}
$$

## RPM Solve Strategy

Stage 1 solves the minimum feasible RPM at each operating point with a bisection method.

At low speed, the required thrust is:

$$
T_{\mathrm{req,low}} =
\max \left(
\gamma_{\mathrm{low}} D_{\mathrm{low}},
T_{\mathrm{req}}(V_{\mathrm{eff,req}})
\right)
$$

with:

$$
\gamma_{\mathrm{low}} = 1.05
$$

The solver finds:

$$
R_{\mathrm{low}}(\mathrm{RPM}) =
T(\mathrm{RPM}) - T_{\mathrm{req,low}} = 0
$$

At cruise:

$$
T_{\mathrm{req,cruise}} = \gamma_{\mathrm{cruise}} D_{\mathrm{cruise}}
$$

with:

$$
\gamma_{\mathrm{cruise}} = 1.02
$$

The solver finds:

$$
R_{\mathrm{cruise}}(\mathrm{RPM}) =
T(\mathrm{RPM}) - T_{\mathrm{req,cruise}} = 0
$$

The low-speed RPM must be at least the cruise RPM:

$$
\mathrm{RPM}_{\mathrm{low}} \ge \mathrm{RPM}_{\mathrm{cruise}}
$$

## Motor Inference And Mass Closure

Stage 1 does not select catalog motors. It infers motor requirements from the prop operating points.

Peak current per motor:

$$
I_{\mathrm{motor}} =
\frac{\max(P_{\mathrm{elec,low}}, P_{\mathrm{elec,cruise}})}{V_{\mathrm{batt}} N_{\mathrm{prop}}}
$$

Required motor \(k_V\):

$$
k_V \approx \frac{\mathrm{RPM}_{\mathrm{low}}}{0.85 V_{\mathrm{batt}}}
$$

Propulsion mass is estimated from CSV-based power-law fits stored in:

- [data/propellers/propeller_mass.csv](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/data/propellers/propeller_mass.csv:1)
- [data/motors/motor_mass.csv](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/data/motors/motor_mass.csv:1)
- [data/motors/esc_mass.csv](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/data/motors/esc_mass.csv:1)

The fitted closures are:

$$
m_{\mathrm{prop}} \sim a_p D^{b_p}(1 + k_p P/D)
$$

$$
m_{\mathrm{motor}} \sim a_m P_{\mathrm{shaft,peak}}^{b_m}
$$

$$
m_{\mathrm{ESC}} \sim a_e I_{\mathrm{motor}}^{b_e}
$$

Battery mass is estimated from mission energy plus reserve:

$$
E_{\mathrm{usable}} =
P_{\mathrm{low}} \frac{t_{\mathrm{climb}}}{60}
+
P_{\mathrm{cruise}} \frac{t_{\mathrm{loiter}}}{60}
$$

$$
E_{\mathrm{pack}} = E_{\mathrm{usable}}(1 + f_{\mathrm{reserve}})
$$

$$
m_{\mathrm{battery}} = \frac{E_{\mathrm{pack}}}{\epsilon_{\mathrm{battery}}}
$$

with:

- \(f_{\mathrm{reserve}} = 0.20\)
- \(\epsilon_{\mathrm{battery}} = 180 \ \mathrm{Wh/kg}\)

Total built mass:

$$
m_{\mathrm{built}} = m_{\mathrm{fixed}} + m_{\mathrm{propulsion}} + m_{\mathrm{battery}}
$$

Feasibility requires:

$$
m_{\mathrm{built}} \le m_{\max}
$$

## Objectives

The current Pareto front is three-dimensional:

1. minimize low-speed electrical power
2. minimize cruise loiter energy
3. minimize propulsion mass

In symbols:

$$
\min P_{\mathrm{low}}
$$

$$
\min E_{\mathrm{loiter}}
$$

$$
\min m_{\mathrm{propulsion}}
$$

where

$$
E_{\mathrm{loiter}} = P_{\mathrm{cruise}} \frac{t_{\mathrm{loiter}}}{60}
$$

## Reported Metrics

Stage 1 currently reports:

- solved low-speed RPM
- solved cruise RPM
- low-speed and cruise electrical power
- loiter energy
- low-speed and cruise thrust
- low-speed and cruise drag
- effective blown velocity and required blown velocity
- blown span fraction
- reference low-speed \(C_L\) requirement
- cruise \(C_L\) requirement
- blown-section \(C_{L,\max}\)
- thrust-to-drag ratio
- disk loading:
  $$
  \frac{T}{A_{\mathrm{disk}}}
  $$
- momentum coefficient \(C_\mu\)
- torque, shaft power, current, and inferred \(k_V\)
- blade Reynolds number and Reynolds penalty factor
- propulsion mass, battery mass, total built mass
- packing, thrust, lift, disk-loading, and mass margins

## Feasibility Conditions

A Stage 1 concept is feasible only if all of the following are satisfied:

- packing margin \(\ge 0\)
- low-speed lift feasible
- cruise lift feasible
- low-speed \(V_{\mathrm{eff}} \ge V_{\mathrm{eff,req}}\)
- low-speed thrust margin \(\ge 0\)
- cruise thrust margin \(\ge 0\)
- low-speed tip Mach margin \(\ge 0\)
- cruise tip Mach margin \(\ge 0\)
- RPM schedule margin \(\ge 0\)
- disk loading within:
  $$
  25 \le \frac{T}{A_{\mathrm{disk}}} \le 500 \ \mathrm{N/m^2}
  $$
- mass budget margin \(\ge 0\)

## Stage 2 And Stage 3 Handoff

Stage 2 currently converts the Stage 1 Pareto front into a prop-span layout report with:

- prop center locations
- packing margins
- blown span fraction
- per-prop thrust, power, and torque at low speed and cruise

Stage 3 now performs a fixed-span AeroSandbox refinement pass for every Stage 2 shortlisted concept.

The current Stage 3 geometry variables are:

- root chord
- taper ratio
- washout
- flap span fraction
- flap chord fraction
- aileron span fraction
- aileron chord fraction
- horizontal-tail volume coefficient
- vertical-tail volume coefficient

Stage 3 uses `asb.Opti` to minimize a weighted geometry score built from:

- low-speed power proxy, scaled from Stage 1 using the refined required blown velocity
- cruise power proxy, scaled from Stage 1 using a refined cruise-drag proxy
- wing area penalty
- control-surface area penalty
- tail-sizing penalties

subject to:

- fixed span
- cruise \(C_L\) limit
- required blown velocity not exceeding the selected propulsor architecture's Stage 1 capability
- minimum horizontal-tail and vertical-tail sizing
- geometric separation between flaps and ailerons

After solving the continuous geometry variables, Stage 3 builds an AeroSandbox airplane model with:

- a refined trapezoidal main wing using `S1210`
- a horizontal tail with elevator
- a vertical tail with rudder
- a fuselage for geometry / visualization context

Stage 3 then evaluates:

- cruise alpha and cruise drag using `VortexLatticeMethod`
- aileron, elevator, and rudder authority from incremental VLM runs
- clean and flapped section polars for `S1210` using NeuralFoil-backed AeroSandbox airfoil calls

Stage 3 writes:

- `outputs/stage3_aerosandbox_queue.csv`
- `outputs/stage3_aerosandbox_results.csv`
- `outputs/stage3_aerosandbox_top_designs.csv`
- `outputs/stage3_visuals/` with planform, three-view, wireframe, polar, mesh, and trade-space artifacts

## Known Limitations

The current Stage 1 / Stage 3 stack is still approximate:

- propeller aerodynamics are surrogate-based, not UIUC-backed yet
- slipstream is treated with a span-averaged actuator-disk model
- no full spanwise prop-slipstream interaction model is used yet inside AeroSandbox
- Stage 3 optimizes around a weighted proxy objective rather than a full mission optimization
- low-speed blowing is enforced through required-\(V_{\mathrm{eff}}\) compatibility with the Stage 1 propulsor architecture, not through a fully coupled prop-wing CFD/VLM solve
- no explicit structural sizing or aeroelastic model is included yet

## Update Rule

Whenever the Stage 1 equations, constraints, output schema, or mass model change, this file should be updated in the same edit pass as the code.
