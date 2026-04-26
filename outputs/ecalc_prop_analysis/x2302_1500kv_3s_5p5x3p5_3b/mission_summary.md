# eCalc / Blown-Wing Mission Analysis

## Configuration
- Motor: `SunnySky X2302-1500 V3`
- Battery: `3S 5000 mAh`
- Propeller: `5.5 x 3.5 in`, `3-blade`
- Propulsors: `10`
- Dynamic eCalc design point used: `37 km/h`

## Static Coefficient Calibration
- Mean static `C_T` from the partial-load table: `0.1406`
- Mean static `C_P` from the partial-load table: `0.0703`
- Dynamic full-throttle point at `37 km/h`: `J = 0.355`, `C_T = 0.1164`, `C_P = 0.0706`

## Low-Speed Drag Budget
### Before package drop (`5.0 kg`, `4 m/s`)
- Required whole-wing `C_L`: `7.148`
- Stage 1 steady low-speed drag: `11.061 N`
- Thrust required including the 5% margin / blown-velocity closure: `11.614 N`
- Required effective blown velocity: `9.471 m/s`
- Solver low-speed RPM for this prop family: `9427 rpm`
- eCalc static-thrust RPM lower bound from the same thrust level: `7980 rpm`
- Interpretation: this is the hard low-speed mission point; blowing is still doing the real work here.

### After package drop (`3.866 kg`, `4 m/s`)
- Required whole-wing `C_L`: `5.527`
- Current DAE51 all-high-lift `C_L,max`: `5.810`
- Stage 1 steady low-speed drag: `9.079 N`
- Thrust required including blown-velocity closure: `9.535 N`
- Required effective blown velocity: `8.154 m/s`
- eCalc static-thrust RPM lower bound from the same thrust level: `7243 rpm`
- Interpretation: after the drop, the `4 m/s` condition becomes much closer to the current DAE51 whole-wing capability, so landing is materially easier than the outbound low-speed segment.

## Cruise / Loiter Operating Points
### Outbound loiter (`5.0 kg`, `10 m/s`)
- Required steady drag: `6.883 N`
- Thrust required with the Stage 1 cruise margin: `7.017 N`
- Static-thrust RPM lower bound from thrust alone: `6203 rpm`
- Pitch-speed-margin RPM using `1.15 x V_cruise`: `7744 rpm`
- Recommendation: use the pitch-speed-based number; it is the cleaner cruise input for the solver.

### Return loiter after drop (`3.866 kg`, `9.0 m/s`)
- Required steady drag: `5.234 N`
- Thrust required with the Stage 1 cruise margin: `5.339 N`
- Static-thrust RPM lower bound from thrust alone: `5410 rpm`
- Pitch-speed-margin RPM using `1.15 x V_loiter`: `7052 rpm`
- Recommendation: a lighter post-drop endurance loiter around `8.5-9.0 m/s` is a better mission speed than holding `10 m/s` all the way home.

## eCalc Flight Speeds To Run
- `0 km/h`: cleanest static `C_T` / `C_P` calibration table.
- `14.4 km/h`: direct low-speed blown-flight calibration (`4.0 m/s`).
- `36.0 km/h`: direct nominal cruise calibration (`10.0 m/s`).
- `31-32 km/h`: post-drop endurance loiter calibration (`8.5-9.0 m/s`).
- If you only want one dynamic eCalc run beyond static, use `36 km/h`, because the solver already treats the low-speed point with a separate blown-velocity closure.

## Conservative Mission Energy Check
- Assumption: use the static eCalc partial-load powers as conservative stand-ins for segment power.
- Usable battery energy at `85%` of a `55.5 Wh` pack: `47.17 Wh`
- `endurance_lean` schedule: `43.25 Wh` total, margin `3.93 Wh`
- `margin_rich` schedule: `61.24 Wh` total, margin `-14.07 Wh`
- Interpretation: the mission can close only if the loiter phases are run near the lower-RPM schedule; the higher-margin schedule is too energy-expensive for this pack.
