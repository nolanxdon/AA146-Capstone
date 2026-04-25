# Rank-6 Airfoil Trade Analysis

## Scope

This note compares four airfoils within the same blown-wing architecture and rectangular-wing control-surface workflow:

- `S1210`
- `E423`
- `DAE51`
- `NACA 0012`

The propulsion concept is held fixed at the current rank-6 downselect:

- `10 x 5.5 x 2.2 in`
- `balanced`
- `3-blade` metadata
- low-speed blown velocity `V_eff = 14.62 m/s`
- low-speed induced velocity `v_i = 5.31 m/s`
- low-speed target `V_inf = 4.0 m/s`

The flap and aileron layout are also held fixed by the same sizing framework:

- flap span `0.65` semispan
- flap chord ratio `0.34`
- flap deflection `40 deg`
- aileron span `0.28` semispan
- aileron chord ratio `0.28`

This therefore isolates the airfoil trade itself rather than mixing airfoil effects with different propulsor layouts or different planforms.

## Method

Each airfoil was passed through the same control-surface sizing pipeline:

1. Generate clean and flap-deflected section polars with NeuralFoil.
2. Apply the project’s slotted-flap correction model.
3. Apply the Cambridge-style uniform-jet immersion model for the blown strips.
4. Assemble a common-alpha whole-wing lift and moment curve from blown and unblown regions.
5. Extract wing-level performance metrics:
   - `CLmax`
   - equivalent stall speed
   - low-speed lift margin at `4 m/s`
   - `alpha` at peak lift
   - post-stall behavior
   - whole-wing pitching moment at peak lift
   - cruise trim alpha
   - cruise and low-speed roll-response proxies

The comparison is therefore based on the same wing geometry, the same prop placement, and the same flap geometry for all airfoils.

## Summary Table

| Airfoil | `CLmax` | `Vstall` [m/s] | Lift margin at `4 m/s` | `alpha` at `CLmax` [deg] | `CM` at `CLmax` | Post-stall drop over `+5 deg` | Cruise trim alpha [deg] | Roll rate @ `14 deg` [deg/s] |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `S1210` | 6.003 | 4.365 | -16.0% | 13.27 | -0.898 | 0.234 | 6.00 | 30.9 |
| `E423` | 5.897 | 4.404 | -17.5% | 13.55 | -0.792 | 0.199 | 5.38 | 30.1 |
| `DAE51` | 5.810 | 4.437 | -18.7% | 12.98 | -0.684 | 0.255 | 12.00 | 31.8 |
| `NACA 0012` | 5.586 | 4.525 | -21.8% | 13.83 | -0.484 | 0.437 | 16.12 | 33.0 |

## Increment Decomposition at Peak Lift

The whole-wing peak-lift result can also be decomposed into:

- baseline clean-wing contribution
- additional blowing contribution
- additional flap-on-top-of-blowing contribution

| Airfoil | Blow increment over clean | Flap increment over blow | Flap-only increment over clean |
| --- | ---: | ---: | ---: |
| `S1210` | 3.922 | 0.155 | 0.290 |
| `E423` | 4.312 | 0.353 | 0.710 |
| `DAE51` | 4.114 | 0.314 | 0.491 |
| `NACA 0012` | 4.386 | 0.554 | 0.839 |

This table is important because it separates two different questions:

- which airfoil is intrinsically strong in the blown configuration,
- and which airfoil benefits most from adding the slotted flap on top of the blown strip.

## Interpretation

### `S1210`

`S1210` remains the strongest airfoil in absolute low-speed lift. It gives the highest whole-wing `CLmax`, the lowest equivalent stall speed, and the smallest lift deficit relative to the `4 m/s` requirement.

However, it also carries the largest pitching-moment penalty:

- `CM at CLmax ≈ -0.898`

This is the most nose-down case in the present trade, so it is the most demanding option from a wing-only trim perspective. Its flap increment on top of blowing is also relatively small, which indicates that `S1210` is already doing much of the work through the blown clean-strip response rather than through additional flap leverage. In other words, `S1210` is the most direct route to high lift, but it is also the most aggressive from the standpoint of wing pitching moment.

### `E423`

`E423` is the closest competitor to `S1210` in low-speed lift:

- `CLmax` is lower by only about `0.106`
- `Vstall` increases by only about `0.039 m/s`

At the same time, it reduces the pitching-moment burden:

- `CM at CLmax ≈ -0.792`

This is still strongly nose-down, but measurably less severe than `S1210`. `E423` also shows the gentlest post-stall drop of the four airfoils in this study, with the smallest `+5 deg` post-peak lift loss. Within the present solver, that makes `E423` the most balanced high-lift option: it retains nearly all of the low-speed capability of `S1210`, improves the wing-only moment burden, and softens the stall shape slightly.

### `DAE51`

`DAE51` lands in the middle of the trade. It gives less lift than either `S1210` or `E423`, but it also eases the pitching-moment penalty relative to both:

- `CLmax ≈ 5.810`
- `CM at CLmax ≈ -0.684`

The important distinction is that `DAE51` does not behave like a low-moment, low-lift outlier in the same way that `NACA 0012` does. It remains a credible blown-wing candidate, but it shifts the design point toward a more conservative trim burden while accepting a moderate low-speed penalty. The cost of this compromise is visible in the trim condition:

- cruise trim alpha rises to about `12 deg`

That is a significant increase relative to `S1210` and `E423`, and it suggests that `DAE51` is less comfortable in the present cruise sizing point even though it is more moderate in pitching moment at maximum lift.

### `NACA 0012`

`NACA 0012` is the weakest airfoil in absolute low-speed performance in the present architecture:

- lowest `CLmax`
- highest equivalent stall speed
- largest lift deficit relative to the `4 m/s` target

Its principal advantage is the smallest nose-down pitching moment:

- `CM at CLmax ≈ -0.484`

This is much easier to trim than the higher-lift airfoils. It also shows the largest flap increment on top of blowing, which means the flap system is doing more of the work here than on the cambered low-speed sections. But this does not rescue the airfoil in absolute terms. Even after that stronger flap increment, the total `CLmax` remains the lowest of the set. `NACA 0012` therefore reads as a trim-friendly but low-authority choice: acceptable only if the low-speed target is relaxed or if pitching-moment burden is weighted far more heavily than raw lift capability.

## Trade-Off Structure

The trade is not one-dimensional. Three coupled patterns appear:

### 1. Low-speed lift versus pitching moment

There is a clear monotonic trend:

- more low-speed lift tends to come with more negative wing pitching moment

In the current rank-6 architecture, the ordering is:

- best lift: `S1210`
- best moment: `NACA 0012`

with `E423` and `DAE51` in between. This is the dominant trade.

### 2. Absolute lift versus flap leverage

The airfoils with the strongest absolute `CLmax` are not the same airfoils with the largest incremental flap benefit. `NACA 0012` gains the most from flap augmentation, but still remains the weakest overall because its baseline blown response is lower. Conversely, `S1210` gains relatively little from the flap on top of blowing because its blown clean-strip performance is already strong.

### 3. Low-speed performance versus cruise trim convenience

`E423` has the lowest cruise trim alpha in the set, followed closely by `S1210`. `DAE51` and especially `NACA 0012` require much larger cruise trim alpha in the present rectangular-wing sizing. This means the airfoils that look easier from a maximum-lift pitching-moment standpoint are not automatically easier to live with in cruise.

## Design Implications

For the present mission and the present propulsor layout, the airfoils can be grouped as follows.

### Preferred high-lift candidates

- `S1210`
- `E423`

These are the only two cases that remain close enough to the `4 m/s` target to be considered strong primary candidates. `S1210` is the pure low-speed winner. `E423` is the better-balanced design if some lift can be traded for reduced pitching-moment burden and slightly gentler post-stall behavior.

### Middle-ground compromise

- `DAE51`

`DAE51` is viable as a compromise airfoil if the design emphasis shifts from pure low-speed lift toward reduced wing-only moment burden. It is not as effective as `E423` in total lift, but it remains much closer to the high-lift group than `NACA 0012`.

### Low-priority option for the present mission

- `NACA 0012`

`NACA 0012` is not competitive for the current `4 m/s` target. It could still be useful if the project eventually prioritizes:

- a larger wing area,
- a relaxed low-speed requirement,
- or a strong preference for reduced wing pitching moment.

Under the current architecture, however, it is the weakest match to the mission.

## Recommended Interpretation

If the design objective remains:

- maximize blown low-speed capability while staying close to the present mass and wing area,

then the practical ranking is:

1. `S1210` for maximum low-speed authority
2. `E423` for the best overall balance
3. `DAE51` as a moderate-moment compromise
4. `NACA 0012` only if trim burden dominates the decision

If the project’s next step is to reduce overall design risk rather than maximize raw lift, `E423` is the most attractive next airfoil to carry forward alongside `S1210`.

## Key Artifacts

### `S1210`

- Whole-wing `C_L`: [total_cl_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/total_cl_curve.png)
- Whole-wing `C_M`: [total_cm_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/total_cm_curve.png)
- Summary: [control_surface_summary.md](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/control_surface_summary.md)

### `E423`

- Whole-wing `C_L`: [total_cl_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/e423/total_cl_curve.png)
- Whole-wing `C_M`: [total_cm_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/e423/total_cm_curve.png)
- Summary: [control_surface_summary.md](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/e423/control_surface_summary.md)

### `DAE51`

- Whole-wing `C_L`: [total_cl_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/dae51/total_cl_curve.png)
- Whole-wing `C_M`: [total_cm_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/dae51/total_cm_curve.png)
- Summary: [control_surface_summary.md](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/dae51/control_surface_summary.md)

### `NACA 0012`

- Whole-wing `C_L`: [total_cl_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/naca0012/total_cl_curve.png)
- Whole-wing `C_M`: [total_cm_curve.png](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/naca0012/total_cm_curve.png)
- Summary: [control_surface_summary.md](control_surface_sizing/rank06_n10_d5p5_p2p2_balanced_b3/naca0012/control_surface_summary.md)
