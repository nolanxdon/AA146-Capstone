"""Component mass models for the distributed propulsion concept optimizer.

Loads three CSV databases of representative commercially-available components
and exposes either direct table lookups or fitted power-law closures for use
in the Stage 1 screener:

- Propeller mass   m_prop  ~ a_p * D^b_p * (1 + k_p * P/D)
- Motor mass       m_motor ~ a_m * P_peak^b_m
- ESC mass         m_esc   ~ a_e * I_cont^b_e

The power-law coefficients are fit once at import from the CSVs. They are
re-fit automatically whenever the CSVs are edited on disk because the loader
is run inside the module-level initializer.

All outputs are in SI:
- propeller mass returned in kilograms
- motor mass returned in kilograms
- ESC mass returned in kilograms
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple


DATA_ROOT = Path(__file__).resolve().parents[2] / "data"
PROP_MASS_CSV = DATA_ROOT / "propellers" / "propeller_mass.csv"
MOTOR_MASS_CSV = DATA_ROOT / "motors" / "motor_mass.csv"
ESC_MASS_CSV = DATA_ROOT / "motors" / "esc_mass.csv"


@dataclass(frozen=True)
class PropMassRecord:
    diameter_in: float
    pitch_in: float
    mass_g: float
    blades: int


@dataclass(frozen=True)
class MotorMassRecord:
    peak_power_w: float
    mass_g: float
    example: str


@dataclass(frozen=True)
class ESCMassRecord:
    current_a: float
    mass_g: float
    example: str


@dataclass(frozen=True)
class PropMassFit:
    """Power-law propeller mass model.

    m_g = a * D_in**b * (1.0 + k * pitch_ratio)
    """

    a: float
    b: float
    k: float

    def mass_g(self, diameter_in: float, pitch_in: float) -> float:
        pitch_ratio = max(pitch_in, 0.0) / max(diameter_in, 1e-9)
        return self.a * diameter_in**self.b * (1.0 + self.k * pitch_ratio)


@dataclass(frozen=True)
class PowerLawFit:
    """Generic single-variable power-law fit: y = a * x^b."""

    a: float
    b: float

    def evaluate(self, x: float) -> float:
        return self.a * max(x, 1e-9) ** self.b


def _load_csv(path: Path) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    with path.open(newline="", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            break
        else:
            return rows
        header = [h.strip() for h in line.split(",")]
        reader = csv.reader(f)
        for parts in reader:
            parts = [p.strip() for p in parts]
            if not parts or all(p == "" for p in parts):
                continue
            if parts[0].startswith("#"):
                continue
            if len(parts) < len(header):
                parts = parts + [""] * (len(header) - len(parts))
            row = {header[i]: parts[i] for i in range(len(header))}
            rows.append(row)
    return rows


def load_prop_mass_records(path: Path = PROP_MASS_CSV) -> list[PropMassRecord]:
    records: list[PropMassRecord] = []
    for row in _load_csv(path):
        records.append(
            PropMassRecord(
                diameter_in=float(row["diameter_in"]),
                pitch_in=float(row["pitch_in"]),
                mass_g=float(row["mass_g"]),
                blades=int(float(row["blades"])),
            )
        )
    return records


def load_motor_mass_records(path: Path = MOTOR_MASS_CSV) -> list[MotorMassRecord]:
    records: list[MotorMassRecord] = []
    for row in _load_csv(path):
        records.append(
            MotorMassRecord(
                peak_power_w=float(row["peak_power_w"]),
                mass_g=float(row["mass_g"]),
                example=row.get("example", ""),
            )
        )
    return records


def load_esc_mass_records(path: Path = ESC_MASS_CSV) -> list[ESCMassRecord]:
    records: list[ESCMassRecord] = []
    for row in _load_csv(path):
        records.append(
            ESCMassRecord(
                current_a=float(row["current_a"]),
                mass_g=float(row["mass_g"]),
                example=row.get("example", ""),
            )
        )
    return records


def _least_squares(x: Sequence[float], y: Sequence[float]) -> Tuple[float, float]:
    """Simple ordinary least squares for y = m*x + c in log space."""

    n = len(x)
    if n < 2:
        raise ValueError("Need at least two points for a fit.")
    sum_x = sum(x)
    sum_y = sum(y)
    sum_xx = sum(xi * xi for xi in x)
    sum_xy = sum(xi * yi for xi, yi in zip(x, y))
    denom = n * sum_xx - sum_x * sum_x
    if abs(denom) < 1e-18:
        raise ValueError("Degenerate fit: all x values identical.")
    m = (n * sum_xy - sum_x * sum_y) / denom
    c = (sum_y - m * sum_x) / n
    return m, c


def fit_power_law(xs: Sequence[float], ys: Sequence[float]) -> PowerLawFit:
    # Build consistent paired log data
    pairs = [(math.log(xi), math.log(yi)) for xi, yi in zip(xs, ys) if xi > 0 and yi > 0]
    log_x = [p[0] for p in pairs]
    log_y = [p[1] for p in pairs]
    slope, intercept = _least_squares(log_x, log_y)
    return PowerLawFit(a=math.exp(intercept), b=slope)


def fit_prop_mass(records: List[PropMassRecord]) -> PropMassFit:
    """Fit m = a * D^b * (1 + k * P/D).

    Uses two-stage regression. First stage fits log(m) = log(a) + b*log(D) using
    average mass at each diameter. Second stage fits the pitch-ratio correction
    on the residual.
    """

    if len(records) < 3:
        raise ValueError("Need at least three prop records to fit power-law mass.")

    # Base D^b fit (ignore pitch effect at first)
    diameters = [r.diameter_in for r in records]
    masses = [r.mass_g for r in records]
    base_fit = fit_power_law(diameters, masses)

    # Second-stage residual fit vs pitch ratio
    pitch_ratios = [r.pitch_in / r.diameter_in for r in records]
    residuals = [r.mass_g / base_fit.evaluate(r.diameter_in) - 1.0 for r in records]

    # Solve k via least squares: residual = k * pitch_ratio
    num = sum(pr * rr for pr, rr in zip(pitch_ratios, residuals))
    den = sum(pr * pr for pr in pitch_ratios)
    k_fit = num / den if den > 1e-18 else 0.0
    return PropMassFit(a=base_fit.a, b=base_fit.b, k=k_fit)


class MassModel:
    """Aggregate propulsion-mass model backed by CSV databases."""

    def __init__(
        self,
        prop_records: List[PropMassRecord] | None = None,
        motor_records: List[MotorMassRecord] | None = None,
        esc_records: List[ESCMassRecord] | None = None,
    ) -> None:
        self.prop_records = prop_records if prop_records is not None else load_prop_mass_records()
        self.motor_records = motor_records if motor_records is not None else load_motor_mass_records()
        self.esc_records = esc_records if esc_records is not None else load_esc_mass_records()

        self.prop_fit = fit_prop_mass(self.prop_records)
        self.motor_fit = fit_power_law(
            [r.peak_power_w for r in self.motor_records],
            [r.mass_g for r in self.motor_records],
        )
        self.esc_fit = fit_power_law(
            [r.current_a for r in self.esc_records],
            [r.mass_g for r in self.esc_records],
        )

    def prop_mass_kg(self, diameter_in: float, pitch_in: float) -> float:
        return 1e-3 * self.prop_fit.mass_g(diameter_in, pitch_in)

    def motor_mass_kg(self, peak_power_w: float) -> float:
        return 1e-3 * self.motor_fit.evaluate(max(peak_power_w, 1e-6))

    def esc_mass_kg(self, current_a: float) -> float:
        return 1e-3 * self.esc_fit.evaluate(max(current_a, 1e-6))

    def propulsion_mass_breakdown_kg(
        self,
        n_props: int,
        diameter_in: float,
        pitch_in: float,
        peak_shaft_power_per_prop_w: float,
        peak_current_per_motor_a: float,
    ) -> dict[str, float]:
        m_prop = self.prop_mass_kg(diameter_in, pitch_in)
        m_motor = self.motor_mass_kg(peak_shaft_power_per_prop_w)
        m_esc = self.esc_mass_kg(peak_current_per_motor_a)
        m_group = m_prop + m_motor + m_esc
        return {
            "prop_mass_kg_per_unit": m_prop,
            "motor_mass_kg_per_unit": m_motor,
            "esc_mass_kg_per_unit": m_esc,
            "propulsion_mass_kg_per_unit": m_group,
            "prop_mass_kg_total": n_props * m_prop,
            "motor_mass_kg_total": n_props * m_motor,
            "esc_mass_kg_total": n_props * m_esc,
            "propulsion_mass_kg_total": n_props * m_group,
        }


# Module-level default model — loaded once at import.
MASS_MODEL = MassModel()


def describe_fits() -> str:
    """Human-readable summary of the current fitted coefficients."""

    return (
        f"prop:  m[g] = {MASS_MODEL.prop_fit.a:.4f} * D^{MASS_MODEL.prop_fit.b:.3f} "
        f"* (1 + {MASS_MODEL.prop_fit.k:.3f} * P/D)\n"
        f"motor: m[g] = {MASS_MODEL.motor_fit.a:.4f} * P_peak^{MASS_MODEL.motor_fit.b:.3f}\n"
        f"esc:   m[g] = {MASS_MODEL.esc_fit.a:.4f} * I_cont^{MASS_MODEL.esc_fit.b:.3f}"
    )


if __name__ == "__main__":
    print(describe_fits())
    mm = MASS_MODEL
    print()
    for D in (5.0, 7.0, 10.0, 13.0):
        for pitch in (D * 0.5, D * 0.8):
            print(f"D={D} in, P={pitch:.2f} in -> m_prop = {mm.prop_mass_kg(D, pitch)*1000:6.2f} g")
    for P in (100, 300, 800, 2000):
        print(f"P_peak={P} W -> m_motor = {mm.motor_mass_kg(P)*1000:6.2f} g")
    for I in (10, 30, 60, 100):
        print(f"I_cont={I} A -> m_esc = {mm.esc_mass_kg(I)*1000:6.2f} g")
