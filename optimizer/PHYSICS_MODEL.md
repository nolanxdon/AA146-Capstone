# Physics Model Notes

This file is the living reference for the optimizer physics, constraints, and output definitions.

It currently documents the implemented **Stage 1 V2** model in:

- [optimizer/core/data_models.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/data_models.py:1)
- [optimizer/core/physics.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/physics.py:1)
- [optimizer/core/mass_model.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/mass_model.py:1)
- [optimizer/stages/stage1_screen.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/stages/stage1_screen.py:1)

For the blown-wing refinements and control-surface studies discussed later in this project, this note also records which assumptions are:

- `paper-derived`
- `paper-informed`
- `heuristic / calibration parameter`

That distinction is especially important for the motor-height model.

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
  e = 0.6591
  $$
  This is now treated as an effective drag-polar Oswald factor for the current
  DAE51 rectangular wing, obtained by fitting
  `CD_total ≈ CD0 + CL^2 / (pi e AR)` to a clean-wing AeroSandbox VLM +
  NeuralFoil drag polar over the `7.5-10 m/s` cruise/loiter range. The pure
  VLM span-efficiency result was near unity; the lower fitted value is used
  here because the Stage 1 drag model folds lift-dependent profile drag into
  the same quadratic term.
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

## Motor-Height Model Provenance

The current rectangular-wing and motor-height studies use a low-order jet-immersion surrogate in:

- [optimizer/core/high_lift_model.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/high_lift_model.py:1)
- [optimizer/core/control_surface_sizing.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/control_surface_sizing.py:1)
- [optimizer/core/motor_height_trade.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/motor_height_trade.py:1)

The model is intentionally split into three evidence levels.

### Paper-derived anchors

From Hawkswell et al., the strongest directly usable geometric anchors are:

- the wing must remain inside the propeller jet across the operating range, otherwise stall can occur abruptly
- a representative ideal no-nacelle optimum near `0.21c` below the wing centerline
- a representative practical nacelle-included optimum near `0.12c` below the wing centerline

These are the only motor-height scales in the present repository that should be treated as directly paper-derived.

### Paper-informed structure

The present immersion model is paper-informed in the following sense:

- Hawkswell et al. justify treating the blown portion as a nominally uniform 2D jet-covered strip for first-order reasoning.
- Agrawal et al. justify treating blown flap performance as a function of `alpha`, blowing level, flap deflection, and jet-height ratio `h/c`, while also noting that propulsor position and motor-axis angle affect the real flow even when they are not in the simple theory.
- Agrawal et al. also show that the more aggressive motor-axis-angle case can stall more suddenly, which motivates the present “over-drop” or “unfavorable jet geometry” penalty in the surrogate.

So the current motor-height structure is:

- `paper-informed` for the existence of an immersion threshold
- `paper-informed` for the expectation that flap and no-flap cases should not share identical sensitivity
- not yet paper-calibrated for exact constants
- `paper-informed` for the existence of a second, near-stall penalty once motor drop becomes excessive

### Heuristic / calibration parameters

The current constants used in the low-order immersion model,

$$
\beta_0 = 0.08,\qquad
\beta_\alpha = 0.55,\qquad
\beta_f = 0.18,\qquad
\ell_\chi = 0.02c
$$

should be treated as heuristic surrogate parameters, not as paper-derived values.

Their role is:

- `\beta_0`: baseline streamline-height offset
- `\beta_\alpha`: alpha sensitivity of the required jet submergence
- `\beta_f`: additional submergence required by flap deflection and flap chord
- `\ell_\chi`: smoothing width of the immersion transition

These constants were chosen to produce reasonable concept-level trends around the paper-derived motor-height scales above. They are placeholders until a formal calibration is performed.

The current over-drop penalty also uses heuristic calibration parameters:

$$
h_{on,c} = 0.12c,\qquad
h_{on,f} = 0.16c,\qquad
h_{ref} = 0.21c,
$$

$$
p = 1.6,\qquad
q = 0.8,\qquad
K_{\alpha} = 3.0^\circ.
$$

Their interpretation is:

- `h_{on,c}`: clean-wing onset of the “too low” penalty, initialized at the practical Hawkswell motor-height scale
- `h_{on,f}`: slotted-flap onset of the “too low” penalty, initialized above the practical scale but below the idealized scale
- `h_{ref}`: normalization scale for excessive drop, aligned with the idealized Hawkswell height
- `p, q`: shape exponents controlling how strongly the penalty grows with drop and blowing strength
- `K_{\alpha}`: amount by which the apparent peak-lift alpha is shifted left as the penalty grows

These are not paper-derived constants. They are initial calibration values chosen so the clean and flapped branches both show the intended qualitative trend: improvement with increased jet coverage, then degradation once the propulsor geometry becomes too aggressive.

## Recommended Calibration Workflow For Motor Height

The repository should treat the current motor-height constants as an intermediate surrogate and calibrate them in two stages.

### Stage A: Immersion calibration from Hawkswell

Use Hawkswell et al. to anchor the immersion-side geometry:

- maintain the condition that the wing should remain submerged in the jet
- retain the practical placement scale near `0.12c`
- retain the idealized no-nacelle placement scale near `0.21c`

Calibration target:

- as motor height increases from too high to moderately below the wing, the model should show the expected rise in usable blown-wing lift due to improved jet coverage

### Stage B: Stall-shape calibration from Agrawal

Use Agrawal et al. to anchor the post-stall and aggressive-stall behavior:

- preserve the section-level dependence on `alpha`, `\Delta c_J`, `\delta_f`, and `h/c`
- use the observed difference between motor-axis-angle cases as qualitative evidence that unfavorable jet geometry can produce more abrupt stall

Calibration target:

- clean blown wing and flapped blown wing should both show:
  - lift improvement as jet placement becomes favorable
  - a peak or plateau
  - then a sharper post-stall deterioration for overly aggressive geometry

This second stage is now implemented as a bounded engineering surrogate in the current repository, but it is still only qualitatively calibrated.

## Current Over-Drop Stall Penalty

In addition to the Cambridge-style immersion factor, the current code applies a second penalty when the motor is dropped excessively far below the wing. This is intended to represent the Agrawal-style observation that unfavorable jet geometry can sharpen stall even when the wing is still nominally immersed in the jet.

Define the motor height ratio

$$
h = \frac{\Delta z_p}{c}
$$

and the blowing-strength ratio

$$
r_V = \frac{V_{eff}}{V_\infty}.
$$

The current severity surrogate is

$$
S_{od}
=
\frac{
\left[\max\left(0,\frac{h-h_{on}}{h_{ref}-h_{on}}\right)\right]^p
\left[\max(0,r_V-1)\right]^q
}{
1 +
\left[\max\left(0,\frac{h-h_{on}}{h_{ref}-h_{on}}\right)\right]^p
\left[\max(0,r_V-1)\right]^q
}.
$$

Then the blown-strip polar is modified so that:

- low-alpha behavior is changed only weakly
- the apparent stall onset shifts to lower alpha
- the post-stall drop becomes steeper
- drag and nose-down moment both increase in the same region

This is not a CFD model of the lower-surface pressure field. It is a bounded surrogate chosen to reproduce the qualitative trend that the clean blown wing becomes sensitive to excessive drop earlier than the slotted-flap blown wing.

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
  c_f = 0.5 \ \mathrm{in}
  $$
- inter-prop edge clearance:
  $$
  c_p = 0.4 \ \mathrm{in}
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

## Rectangular-Wing Control-Surface Sizing

The current rectangular-wing sizing workflow is implemented in:

- [optimizer/core/control_surface_sizing.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/control_surface_sizing.py:1)
- [optimizer/stages/control_surface_sizing.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/stages/control_surface_sizing.py:1)

For a selected Stage 3 rank, the workflow:

- freezes the wing to a rectangular planform with fixed span and chord
- sizes an inboard single-slotted flap while enforcing overlap with the inboard propeller array
- sizes an outboard aileron with both cruise and low-speed blown-wing authority proxies
- assembles whole-wing common-\(\alpha\) curves for:
  - \(C_L(\alpha)\)
  - \(C_M(\alpha)\)

The low-speed aileron proxy is intentionally simple. It is not a fully coupled roll-dynamics solution. Instead, it scales the cruise aileron geometry by:

- the fraction of the aileron span overlapped by the blown intervals
- the local blown dynamic-pressure ratio on that span
- a roll-damping proxy already used in the rectangular-wing sizing workflow

The current outputs now include:

- `total_cl_curve.png`
- `total_cm_curve.png`
- `low_speed_aileron_curves.png`
- `control_surface_summary.csv`
- `control_surface_summary.md`

The key wing-only control-surface metrics written to the summary file are:

- equivalent whole-wing \(C_{L,\max}\)
- equivalent stall speed
- alpha at \(C_{L,\max}\)
- post-stall drop metric over the next \(5^\circ\)
- whole-wing \(C_M\) at the peak high-lift point
- blown-overlap fraction on the aileron span
- low-speed aileron roll-rate proxy

These should be interpreted as concept-level wing metrics, not as final aircraft handling-quality predictions.

## Representative Motor Targeting

The repository now includes a representative motor-target sheet generator in:

- [optimizer/core/motor_targeting.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/motor_targeting.py:1)
- [optimizer/stages/motor_targeting.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/stages/motor_targeting.py:1)

This stage does not yet select a final catalog motor. Instead, it converts the downselected propeller operating point into a procurement-style target band:

- required low-speed and cruise RPM
- per-motor torque and shaft power
- required Kv
- target Kv band
- required and target electrical current
- representative motor and ESC classes from the local sizing datasets

The representative motor class search is currently based on local datasets in:

- `data/motors/motor_mass.csv`
- `data/motors/esc_mass.csv`

and should therefore be interpreted as a class match, not a final hardware recommendation. A live vendor-backed downselection remains future work.

## Known Limitations

The current Stage 1 / Stage 3 stack is still approximate:

- propeller aerodynamics are surrogate-based, not UIUC-backed yet
- slipstream is treated with a span-averaged actuator-disk model
- no full spanwise prop-slipstream interaction model is used yet inside AeroSandbox
- Stage 3 optimizes around a weighted proxy objective rather than a full mission optimization
- low-speed blowing is enforced through required-\(V_{\mathrm{eff}}\) compatibility with the Stage 1 propulsor architecture, not through a fully coupled prop-wing CFD/VLM solve
- low-speed aileron authority is still a wing-only blown-dynamic-pressure proxy, not a full lateral-directional trim solution
- representative motor targeting is based on local class datasets and heuristics, not a live hardware database
- no explicit structural sizing or aeroelastic model is included yet

## Update Rule

Whenever the Stage 1 equations, constraints, output schema, or mass model change, this file should be updated in the same edit pass as the code.
