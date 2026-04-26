# Unified Wing Workflow | DAE51

## Workflow

1. Multi-airfoil front-end section analysis
2. DAE51 rectangular-wing control-surface sizing at the low-speed design point
3. DAE51 motor-height trade on the frozen control geometry
4. Frozen-geometry DAE51 speed sweep from 5 to 15 m/s with RPM re-solved at each speed

## Key outputs

- Airfoil front-end summary: [README.md](airfoil_frontend/README.md)
- DAE51 control summary: [control_surface_summary.md](dae51/control_surface_sizing/rank01_n10_d5p5_p2p2_balanced_b3/dae51/control_surface_summary.md)
- DAE51 motor-height summary: [motor_height_trade_summary.md](dae51/motor_height_trade/rank01_n10_d5p5_p2p2_balanced_b3/dae51/motor_height_trade_summary.md)
- DAE51 speed-sweep summary: [speed_sweep_summary.md](dae51/speed_sweep/rank01_n10_d5p5_p2p2_balanced_b3/dae51/speed_sweep_summary.md)

## Frozen design passed downstream

- Flap geometry: `span=0.65` semispan, `c_f/c=0.34`
- Aileron geometry: `span=0.28` semispan, `c_a/c=0.28`
- Optimized motor drop used for the speed sweep: `0.120 c`
- Speed sweep flap states: `0 deg, 20 deg, 40 deg`
- Speed sweep speeds: `5.0` to `15.0 m/s` in `0.5 m/s` increments
