from __future__ import annotations

from typing import List, Sequence

from .data_models import Stage1Result


def _objective_tuple(result: Stage1Result) -> tuple[float, float, float]:
    return (
        result.low_speed_power_w,
        result.loiter_energy_wh,
        result.propulsion_mass_kg_total,
    )


def pareto_front(results: Sequence[Stage1Result]) -> List[Stage1Result]:
    """Return the nondominated feasible set using three objectives:

    1. Minimize low-speed electrical power (slow-flight efficiency)
    2. Minimize loiter energy at cruise (mission endurance)
    3. Minimize total propulsion mass (hardware cost / weight budget)
    """

    feasible = [r for r in results if r.is_feasible]
    front: List[Stage1Result] = []

    for candidate in feasible:
        c = _objective_tuple(candidate)
        dominated = False
        for other in feasible:
            if other is candidate:
                continue
            o = _objective_tuple(other)
            better_or_equal = all(o[i] <= c[i] for i in range(3))
            strictly_better = any(o[i] < c[i] for i in range(3))
            if better_or_equal and strictly_better:
                dominated = True
                break
        if not dominated:
            front.append(candidate)

    return sorted(front, key=_objective_tuple)
