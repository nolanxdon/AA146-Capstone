# Motor Height Trade Study: Rank 6

- Prop concept: `10 x 5.5 x 2.2 in`

## Key takeaways

- Lowest equivalent stall speed in the sweep occurred at `84 mm` drop: `4.365 m/s`.
- The lift benefit effectively saturated by about `28 mm` drop (`0.08 c`), after which further lowering changed `CLmax` only negligibly.
- Highest 14 deg roll-rate estimate occurred at `0 mm` drop: `30.9 deg/s`.

## Artifacts

- Metric plot: ![](motor_height_trade_metrics.png)

- Whole-wing CL overlay: ![](motor_height_trade_cl_overlay.png)

- Geometry/immersion plot: ![](motor_height_trade_geometry.png)

## Notes

- This trade keeps the selected propulsion architecture and the selected rectangular slotted-flap/aileron geometry fixed while sweeping only motor vertical drop.
- The vertical-drop effect now enters through a Cambridge-style jet-immersion criterion rather than through a purely empirical bonus term.
- The blown benefit remains strong only while the wing stays submerged in the uniform 2D jet; once the jet rides above the wing, the effective blown contribution collapses rapidly.
- These results remain concept-level sensitivity trends, not a CFD-calibrated vertical-placement optimum.