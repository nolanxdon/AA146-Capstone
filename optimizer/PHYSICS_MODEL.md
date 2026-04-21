# Physics Model Notes

This is the living reference for the physics, equations, assumptions, and modeling choices used in the blown-wing concept optimizer.

It is intended to be updated as the project evolves from:

1. Stage 1 coarse Pareto screening
2. Stage 2 prop-span sizing
3. Stage 3 AeroSandbox refinement

The current document reflects the **implemented Stage 1 model** in:

- [optimizer/core/data_models.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/data_models.py:1)
- [optimizer/core/physics.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/physics.py:1)
- [optimizer/stages/stage1_screen.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/stages/stage1_screen.py:1)

## Purpose

The current optimizer is a **coarse concept-selection tool** for distributed propulsion on a blown wing. It is currently built to downselect:

- number of propellers
- propeller diameter
- propeller pitch ratio
- coarse prop family

In the current Phase 1 implementation, low-speed RPM and cruise RPM are **solved operating points**, not outer-loop design variables.

The present Stage 1 model is **not** a final propeller-sizing method. It is a fast screening model to identify promising architectures before higher-fidelity analysis.

## Current Mission Assumptions

The default mission assumptions currently implemented are:

- Gross mass: $m = 5.0 \ \mathrm{kg}$
- Max mass: $m_{\max} = 5.0 \ \mathrm{kg}$
- Span: $b = 2.0 \ \mathrm{m}$
- Chord: $c = 0.35 \ \mathrm{m}$
- Battery voltage: $V_{\mathrm{batt}} = 14.8 \ \mathrm{V}$
- Low-speed flight point: $V_{\infty,\mathrm{low}} = 4.0 \ \mathrm{m/s}$
- Cruise / loiter flight point: $V_{\infty,\mathrm{cruise}} = 10.0 \ \mathrm{m/s}$
- Loiter time: $t_{\mathrm{loiter}} = 18 \ \mathrm{min}$

Atmospheric constants:

- Air density: $\rho = 1.225 \ \mathrm{kg/m^3}$
- Gravity: $g = 9.80665 \ \mathrm{m/s^2}$
- Speed of sound: $a = 343.0 \ \mathrm{m/s}$

Aerodynamic constants:

- Oswald efficiency factor: $e = 0.8$
- Flapped section $C_{L,\max}$: $C_{L,\max,\mathrm{flapped}} = 2.2$
- Clean section $C_{L,\max}$: $C_{L,\max,\mathrm{clean}} = 1.4$
- Maximum blowing lift increment parameter: $\Delta C_{L,\mu,\max} = 0.8$
- Flapped zero-lift drag parameter: $C_{D0,\mathrm{flapped}} = 0.11$
- Clean zero-lift drag parameter: $C_{D0,\mathrm{clean}} = 0.05$
- Trim drag factor: $f_{\mathrm{trim}} = 1.05$

Propulsion and system assumptions:

- Electrical chain efficiency:
  $$
  \eta_{\mathrm{elec}} = 0.72
  $$
- Avionics hotel load:
  $$
  P_{\mathrm{avionics}} = 10 \ \mathrm{W}
  $$
- Tip Mach limit:
  $$
  M_{\mathrm{tip,max}} = 0.55
  $$

## Stage 1 Design Variables

The current Stage 1 screen varies:

- Number of props:
  $$
  N_{\mathrm{prop}} \in \{4,6,8,10,12\}
  $$
- Prop diameter:
  $$
  D \in \{4.5, 5.0, 5.5, 6.0, 6.5, 7.0\} \ \mathrm{in}
  $$
- Prop pitch ratio:
  $$
  \frac{P}{D} \in \{0.4,0.5,0.6,0.7,0.8,0.9\}
  $$
- Prop family:
  $$
  \{\mathrm{high\_thrust}, \mathrm{balanced}, \mathrm{cruise}\}
  $$

For each architecture, the solver computes:

- low-speed RPM
- cruise RPM

by solving the required operating-point conditions internally.

## Wing Geometry Model

The wing is currently treated as a fixed rectangular planform.

Wing area:

$$
S = bc
$$

With the current assumptions:

$$
S = 2.0 \times 0.35 = 0.70 \ \mathrm{m^2}
$$

Aspect ratio:

$$
\mathrm{AR} = \frac{b^2}{S}
$$

## Dynamic Pressure

The current model uses:

$$
q = \frac{1}{2} \rho V^2
$$

This is used at both low speed and cruise, and again for the blown region using the local blown velocity.

## Packing Model

The current prop placement model is symmetric about the fuselage.

Each half-wing must satisfy:

- fuselage keep-out
- inboard prop clearance from fuselage
- inter-prop edge clearance
- wingtip margin

The currently implemented geometric margins are:

- fuselage width:
  $$
  w_f = 0.20 \ \mathrm{m}
  $$
- fuselage-to-inboard-prop clearance:
  $$
  c_f = 1.0 \ \mathrm{in}
  $$
- adjacent prop clearance:
  $$
  c_p = 1.0 \ \mathrm{in}
  $$
- wingtip margin:
  $$
  c_t = 1.5 \ \mathrm{in}
  $$

This is implemented explicitly through placed prop centers, not average spacing.

## Blown Span Fraction

The current model estimates the span fraction influenced by prop slipstream using:

$$
w_{\mathrm{blown}} = k_{\mathrm{exp}} D
$$

where:

$$
k_{\mathrm{exp}} = 0.8
$$

Each prop creates a spanwise interval of influence centered on its span location. Overlapping intervals are merged, and the blown span fraction is:

$$
\eta_b = \frac{\text{total merged blown span}}{b}
$$

This is more realistic than the earlier crude assumption:

$$
\eta_b \approx \frac{N_{\mathrm{prop}} k_{\mathrm{exp}} D}{b}
$$

but it is still a coarse span-coverage model.

## Propeller Model

### Important Note

The current Stage 1 propeller model is a **surrogate placeholder**.

It is **not** based on real UIUC prop data yet.

The current model defines three coarse prop families, each with:

- a baseline static thrust coefficient
- a baseline static power coefficient
- a characteristic advance-ratio scale

These are hand-built engineering placeholders for screening only.

### Advance Ratio

For a prop running at rotational speed $n$ rev/s:

$$
n = \frac{\mathrm{RPM}}{60}
$$

Advance ratio is:

$$
J = \frac{V}{nD}
$$

### Surrogate Coefficients

The current Stage 1 model forms coarse static coefficients from pitch ratio:

$$
C_{T,\mathrm{static}} = C_{T,\mathrm{base}}
\left(
\frac{(P/D)}{(P/D)_{\mathrm{ref}}}
\right)^{0.35}
$$

$$
C_{P,\mathrm{static}} = C_{P,\mathrm{base}}
\left(
\frac{(P/D)}{(P/D)_{\mathrm{ref}}}
\right)^{0.80}
$$

Then a hand-built decay with $J$ is used to compute operating $C_T$ and $C_P$.

This gives:

$$
C_T = C_{T,\mathrm{static}} \cdot f_T(J)
$$

$$
C_P = C_{P,\mathrm{static}} \cdot f_P(J)
$$

where $f_T$ and $f_P$ are surrogate decay functions defined in code.

## Propeller Thrust and Power

The current propeller equations are:

### Per-prop thrust

$$
T_{\mathrm{per}} = C_T \rho n^2 D^4
$$

### Per-prop shaft power

$$
P_{\mathrm{shaft,per}} = C_P \rho n^3 D^5
$$

### Total thrust

$$
T_{\mathrm{total}} = N_{\mathrm{prop}} T_{\mathrm{per}}
$$

### Total shaft power

$$
P_{\mathrm{shaft,total}} = N_{\mathrm{prop}} P_{\mathrm{shaft,per}}
$$

### Total electrical power

$$
P_{\mathrm{elec,total}} =
\frac{P_{\mathrm{shaft,total}}}{\eta_{\mathrm{elec}}}
+
P_{\mathrm{avionics}}
$$

### Per-prop torque

$$
\tau_{\mathrm{per}} =
\frac{P_{\mathrm{shaft,per}}}{2\pi n}
$$

### Tip speed and tip Mach

Tip speed is approximated as:

$$
V_{\mathrm{tip}} =
\sqrt{(\pi D n)^2 + V^2}
$$

and tip Mach is:

$$
M_{\mathrm{tip}} = \frac{V_{\mathrm{tip}}}{a}
$$

## RPM Solve Strategy

Phase 1 no longer sweeps RPM as a grid variable. Instead, it solves for the **minimum feasible RPM** at each flight condition.

### Low-speed RPM solve

At low speed, the required thrust is taken as the maximum of:

1. thrust needed to exceed the drag requirement with margin
2. thrust needed to generate the required blown velocity

$$
T_{\mathrm{req,low}} =
\max
\left(
\gamma_{\mathrm{low}} D_{\mathrm{low}},
T_{\mathrm{req}}(V_{\mathrm{eff,req}})
\right)
$$

The solver then finds RPM such that:

$$
R_{\mathrm{low}}(\mathrm{RPM}) =
T(\mathrm{RPM}) - T_{\mathrm{req,low}} = 0
$$

using a bisection root solve over the configured RPM bounds.

### Cruise RPM solve

At cruise, the required thrust is:

$$
T_{\mathrm{req,cruise}} = \gamma_{\mathrm{cruise}} D_{\mathrm{cruise}}
$$

and the solver finds RPM such that:

$$
R_{\mathrm{cruise}}(\mathrm{RPM}) =
T(\mathrm{RPM}) - T_{\mathrm{req,cruise}} = 0
$$

again using a bisection root solve.

## Slipstream Velocity Model

The current model uses an ideal actuator-disk estimate to convert thrust into an effective local blown velocity.

Total prop disk area:

$$
A_{\mathrm{disk}} = N_{\mathrm{prop}} \pi \left( \frac{D}{2} \right)^2
$$

Induced velocity:

$$
v_i = \sqrt{\frac{T_{\mathrm{total}}}{2 \rho A_{\mathrm{disk}}}}
$$

Effective blown velocity:

$$
V_{\mathrm{eff}} = V_{\infty} + 2v_i
$$

This is currently used as a span-averaged representative blown velocity.

## Low-Speed Lift Model

The low-speed model uses a two-zone wing:

- unblown section
- blown section

Wing areas:

$$
S_u = (1-\eta_b)S
$$

$$
S_b = \eta_b S
$$

Dynamic pressures:

$$
q_\infty = \frac{1}{2} \rho V_\infty^2
$$

$$
q_b = \frac{1}{2} \rho V_{\mathrm{eff}}^2
$$

Reference required lift coefficient:

$$
C_{L,\mathrm{req,ref}} = \frac{W}{q_\infty S}
$$

where:

$$
W = mg
$$

### Blowing lift increment

The current Stage 1 model uses a simple saturating blown-lift increment:

$$
\Delta C_{L,\mu}
=
\Delta C_{L,\mu,\max}
\left(
1 - e^{-((V_{\mathrm{eff}}/V_\infty)^2 - 1)}
\right)
$$

and the blown-section local max lift is:

$$
C_{L,\max,b} =
C_{L,\max,\mathrm{flapped}} + \Delta C_{L,\mu}
$$

### Lift allocation

The unblown section is assumed to operate up to its local max first:

$$
L_{u,\max} = q_\infty S_u C_{L,\max,\mathrm{flapped}}
$$

Remaining lift:

$$
L_{\mathrm{rem}} = W - L_{u,\max}
$$

If:

$$
L_{\mathrm{rem}} \le 0
$$

then the blown region is not needed.

Otherwise the blown section must satisfy:

$$
C_{L,\mathrm{req},b} = \frac{L_{\mathrm{rem}}}{q_b S_b}
$$

and feasibility requires:

$$
C_{L,\mathrm{req},b} \le C_{L,\max,b}
$$

## Drag Model

### Induced drag factor

The current model uses:

$$
k = \frac{1}{\pi e \mathrm{AR}}
$$

### Low-speed two-zone drag

The current Stage 1 model assigns separate drag coefficients to the unblown and blown regions:

$$
C_{D,u}
=
f_{\mathrm{trim}}
\left(
f_u C_{D0,\mathrm{flapped}} + k C_{L,u}^2
\right)
$$

$$
C_{D,b}
=
f_{\mathrm{trim}}
\left(
f_b C_{D0,\mathrm{flapped}} + k C_{L,b}^2
\right)
$$

with:

- $f_u = 1.0$
- $f_b = 1.05$

Low-speed drag:

$$
D_u = q_\infty S_u C_{D,u}
$$

$$
D_b = q_b S_b C_{D,b}
$$

$$
D_{\mathrm{low}} = D_u + D_b
$$

### Cruise drag

Cruise uses a simpler single-zone drag model:

$$
C_{D,\mathrm{cruise}} = C_{D0,\mathrm{clean}} + k C_{L,\mathrm{cruise}}^2
$$

$$
D_{\mathrm{cruise}} = q_{\mathrm{cruise}} S C_{D,\mathrm{cruise}}
$$

where:

$$
C_{L,\mathrm{cruise}} = \frac{W}{q_{\mathrm{cruise}} S}
$$

## Mission Energy Model

The current loiter-energy objective is:

$$
E_{\mathrm{loiter}} = P_{\mathrm{elec,cruise}} \cdot \frac{t_{\mathrm{loiter}}}{60}
$$

where:

- $P_{\mathrm{elec,cruise}}$ is in watts
- $t_{\mathrm{loiter}}$ is in minutes
- $E_{\mathrm{loiter}}$ is in watt-hours

## Current Optimization Objectives

Stage 1 currently minimizes:

1. Low-speed electrical power
   $$
   \min P_{\mathrm{elec,low}}
   $$
2. Loiter energy at cruise
   $$
   \min E_{\mathrm{loiter}}
   $$

These are Pareto-sorted after the full grid search.

## Current Constraints

A Stage 1 concept is currently feasible only if all of the following hold:

### Geometry / packing

$$
\text{packing margin} \ge 0
$$

including:

- fuselage clearance
- inter-prop clearance
- wingtip clearance

### Low-speed lift feasibility

Blown-wing lift must close at the low-speed point.

### Cruise lift feasibility

$$
C_{L,\mathrm{cruise}} \le C_{L,\max,\mathrm{clean}}
$$

### Thrust margins

Low speed:

$$
T_{\mathrm{low}} \ge
\max
\left(
1.05 \, D_{\mathrm{low}},
T_{\mathrm{req}}(V_{\mathrm{eff,req}})
\right)
$$

Cruise:

$$
T_{\mathrm{cruise}} \ge 1.02 \, D_{\mathrm{cruise}}
$$

### Tip Mach

$$
M_{\mathrm{tip}} \le 0.55
$$

### RPM ordering

$$
\mathrm{RPM}_{\mathrm{low,solved}} \ge \mathrm{RPM}_{\mathrm{cruise,solved}}
$$

## Known Stage 1 Biases and Limitations

The current Stage 1 model has known biases.

### Large-diameter bias

The current first-pass model tends to favor larger diameter because:

- thrust scales with $D^4$
- larger diameter usually reduces power for a required thrust in the surrogate
- larger diameter also increases blown span fraction through:
  $$
  \eta_b \uparrow \ \text{as} \ D \uparrow
  $$

### Surrogate prop data

The current $C_T$ and $C_P$ values are placeholder surrogates, not measured prop data.

### Coarse blown-lift model

The current $\Delta C_{L,\mu}$ model is a simple saturation curve and is not yet calibrated to real blown-wing data.

### Drag is not a full buildup

The current model does **not** yet include detailed:

- fuselage drag
- tail drag
- landing gear drag
- flap system drag buildup beyond coarse $C_{D0}$ terms
- interference drag
- detailed spanwise induced effects

### Average blown velocity

The current model uses a representative $V_{\mathrm{eff}}$, not a resolved spanwise slipstream field.

## Planned Stage 2 Improvements

Stage 2 is expected to replace or improve:

- surrogate $C_T/C_P$ with real prop data
- average blown velocity with per-prop spanwise influence
- coarse motor inference with explicit motor requirements
- coarse lift increment with a more defensible blown-lift model

## Planned Stage 3 Improvements

Stage 3 is expected to add AeroSandbox-based refinement for:

- wing geometry
- twist / washout
- flap geometry
- higher-fidelity aerodynamic closure

## Update Log

### Current

- Added Stage 1 Pareto screener
- Added explicit symmetric prop placement and packing
- Added 18-minute loiter mission
- Added current coarse thrust, drag, lift, and energy formulations
- Replaced outer-loop RPM sweeps with internal bisection solves for low-speed and cruise operating points

### Next Recommended Update

- Replace surrogate prop families with real UIUC-backed prop interpolation
- Add a separate momentum or disk-loading metric to better study small-diameter / high-slipstream concepts

---

## V2 Upgrade Notes (current implementation)

Stage 1 has been upgraded with a propulsion-mass penalty, a Cμ-based blown-lift model, a blade Reynolds penalty on prop coefficients, disk-loading bounds, and a 5 kg mass-budget closure. The Pareto front is now three-dimensional.

### Propulsion-mass penalty

Commercial component masses are fit by power laws against three CSV databases (APC Electric-E propellers, brushless outrunner motors, BLHeli ESCs). See [data/propellers/propeller_mass.csv](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/data/propellers/propeller_mass.csv), [data/motors/motor_mass.csv](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/data/motors/motor_mass.csv), [data/motors/esc_mass.csv](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/data/motors/esc_mass.csv) for data and source URLs. The fits are done at import in [optimizer/core/mass_model.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/mass_model.py:1):

$$
m_{\mathrm{prop}}[\mathrm{g}] = 0.1029 \, D_{\mathrm{in}}^{2.504} \, \left( 1 + 0.014 \, \tfrac{P}{D} \right)
$$

$$
m_{\mathrm{motor}}[\mathrm{g}] = 0.3955 \, P_{\mathrm{peak}}^{0.860}
$$

$$
m_{\mathrm{esc}}[\mathrm{g}] = 0.3553 \, I_{\mathrm{cont}}^{1.178}
$$

Total propulsion mass is $N_{\mathrm{prop}} \cdot (m_{\mathrm{prop}} + m_{\mathrm{motor}} + m_{\mathrm{esc}})$. Peak shaft power per motor is taken as the maximum of the low-speed and cruise operating points. Peak continuous current per motor is the peak electrical power divided by pack voltage and motor count.

### Battery mass closure

Battery mass is sized from mission energy plus reserve:

$$
E_{\mathrm{usable}} = P_{\mathrm{low}} \, \tfrac{t_{\mathrm{climb}}}{60} + P_{\mathrm{cruise}} \, \tfrac{t_{\mathrm{loiter}}}{60}
$$

$$
m_{\mathrm{batt}} = \frac{(1 + f_{\mathrm{res}}) E_{\mathrm{usable}}}{e_{\mathrm{batt}}}
$$

with $e_{\mathrm{batt}} = 180 \ \mathrm{Wh/kg}$ (pack-level LiPo), $f_{\mathrm{res}} = 0.20$, and $t_{\mathrm{climb}} = 1.5 \ \mathrm{min}$.

### Mass-budget constraint

The total built mass is

$$
m_{\mathrm{tot}} = m_{\mathrm{fixed}} + m_{\mathrm{prop,tot}} + m_{\mathrm{batt}}
$$

with $m_{\mathrm{fixed}} = 2.0 \ \mathrm{kg}$ covering airframe, avionics, payload, and servos. Feasibility requires $m_{\mathrm{tot}} \le 5.0 \ \mathrm{kg}$.

### Cμ-based blown lift

The saturating exponential ΔCL model is replaced with a momentum-coefficient formulation. The blown section lift increment is

$$
\Delta C_{L,\mu} = k_{C\mu} \sqrt{C_\mu}, \qquad C_\mu = \frac{T_{\mathrm{total}}}{q_\infty S}
$$

with $k_{C\mu} = 1.8$ (literature range 1.5–2.5) and a deadband $C_\mu \ge 0.02$ below which blowing is treated as inactive. The section ceiling was raised to $C_{L,\mathrm{ceil}} = 3.2$ so the $\sqrt{C_\mu}$ scaling can actually express high-Cμ designs.

### Blade Reynolds penalty

CT and CP are multiplied by a blade-Reynolds derate

$$
f_{Re} = \max\left( f_{\min}, \ \min\left(1, \left(\tfrac{Re}{Re_{\mathrm{ref}}}\right)^{0.30}\right) \right)
$$

with $Re_{\mathrm{ref}} = 80000$, floor $f_{\min} = 0.60$. The blade Reynolds number is evaluated at $0.75 R$ using the resultant velocity (rotational + freestream) and $c = 0.10 D$.

### Decoupled j_critical

$J_{\mathrm{critical}}$ is now set directly by the prop family (not multiplied by pitch ratio), so thrust-and-power roll-off with advance ratio is a prop-family property rather than a geometry artifact.

### Disk-loading bounds

Feasibility now also requires

$$
25 \ \mathrm{N/m^2} \le T_{\mathrm{low}} / A_{\mathrm{disk}} \le 500 \ \mathrm{N/m^2}
$$

to reject under-loaded concepts (where slipstream is meaningless) and over-loaded concepts (where slipstream assumptions break).

### Three-dimensional Pareto front

The screen now filters on three objectives:

1. $\min P_{\mathrm{elec,low}}$
2. $\min E_{\mathrm{loiter}}$
3. $\min m_{\mathrm{prop,tot}}$

A candidate is Pareto-optimal if no feasible peer is better-or-equal in all three and strictly better in at least one. See [optimizer/core/pareto.py](/Users/nolannguyen/Documents/GitHub/AA146-Capstone/optimizer/core/pareto.py:1).

### Sweep grid (V2)

- $N_{\mathrm{prop}} \in \{4, 6, 8, 10, 12, 14, 16\}$
- $D \in \{4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 9.0, 10.0\}$ in
- $P/D \in \{0.40, 0.50, 0.60, 0.70, 0.80, 0.90\}$
- RPM brackets: low $\in [4000, 14000]$, cruise $\in [3000, 11000]$

